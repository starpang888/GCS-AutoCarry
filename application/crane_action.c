#include "crane_action.h"
#include <stddef.h>

static bool PositionValid(int64_t value)
{ return value >= -4294967295LL && value <= 4294967295LL; }

static bool DurationValid(uint32_t value)
{ return value > 0U && value < 0x80000000U; }

static bool PulseValid(const ServoInstance *servo, uint16_t value)
{
    return servo != NULL && servo->initialized && value >= servo->config.min_pulse_us &&
           value <= servo->config.max_pulse_us;
}

bool CraneActionIsRunning(const CraneActionInstance *action)
{ return action != NULL && action->initialized && action->status.state == CRANE_ACTION_RUNNING; }

bool CraneActionReady(const CraneActionInstance *action, CraneActionKind kind)
{
    if (action == NULL || !action->initialized ||
        (kind != CRANE_ACTION_PICK && kind != CRANE_ACTION_PLACE)) return false;
    const CraneActionConfig *config = &action->config;
    const CraneActionPoint *point = kind == CRANE_ACTION_PICK ? &config->pick : &config->place;
    return point->configured && PositionValid(config->safe_z_units) &&
        PositionValid(point->yaw_units) && PositionValid(point->z_units) &&
        PulseValid(action->rack, point->rack_pulse_us) &&
        PulseValid(action->gripper, config->open_pulse_us) &&
        PulseValid(action->gripper, config->closed_pulse_us) &&
        DurationValid(config->rack_wait_ms) && DurationValid(config->gripper_wait_ms) &&
        DurationValid(config->step_timeout_ms) && config->step_timeout_ms > config->rack_wait_ms &&
        config->step_timeout_ms > config->gripper_wait_ms;
}

bool CraneActionInit(CraneActionInstance *action, XMotorInstance *yaw, XMotorInstance *z,
                     ServoInstance *rack, ServoInstance *gripper, const CraneActionConfig *config)
{
    if (action == NULL || yaw == NULL || z == NULL || yaw == z || !yaw->initialized ||
        !z->initialized || rack == NULL || gripper == NULL || rack == gripper ||
        !rack->initialized || !gripper->initialized || config == NULL) return false;
    *action = (CraneActionInstance){.axes = {yaw, z}, .rack = rack, .gripper = gripper,
        .config = *config, .initialized = true, .status.fault_axis = UINT8_MAX};
    return true;
}

static void Terminate(CraneActionInstance *action, CraneActionFault fault, bool failed)
{
    action->status.fault = fault;
    action->status.state = failed ? CRANE_ACTION_FAILED : CRANE_ACTION_CANCELLED;
    for (size_t i = 0; i < 2U; ++i) XMotorStop(action->axes[i], X_FAULT_NONE);
}

void CraneActionCancel(CraneActionInstance *action, CraneActionFault reason)
{
    if (CraneActionIsRunning(action)) Terminate(action, reason, false);
}

static bool EnterStep(CraneActionInstance *action, CraneActionStep step, uint32_t now_ms)
{
    action->status.step = step;
    action->status.step_elapsed_ms = 0U;
    action->step_started_ms = now_ms;
    size_t axis = 1U;
    int64_t target = action->active_config.safe_z_units;
    if (step == CRANE_STEP_RACK)
    {
        if (!ServoSetPulseUs(action->rack, action->point.rack_pulse_us)) return false;
        /* 夹取前张开；放下保持当前夹紧输出，到下降完成才张开。 */
        return action->status.action != CRANE_ACTION_PICK ||
            ServoSetPulseUs(action->gripper, action->active_config.open_pulse_us);
    }
    if (step == CRANE_STEP_GRIP)
        return ServoSetPulseUs(action->gripper, action->status.action == CRANE_ACTION_PICK ?
            action->active_config.closed_pulse_us : action->active_config.open_pulse_us);
    if (step == CRANE_STEP_YAW) { axis = 0U; target = action->point.yaw_units; }
    else if (step == CRANE_STEP_LOWER) target = action->point.z_units;
    XMotorInstance *motor = action->axes[axis];
    action->step_position_replies = motor->diagnostics.position_replies;
    action->step_status_replies = motor->diagnostics.status_replies;
    if (!XMotorSetTarget(motor, target, &action->profiles[axis], action->status.sequence, now_ms))
        return false;
    action->axis_owned[axis] = true;
    action->axis_target[axis] = target;
    action->axis_generation[axis] = motor->motion.generation;
    return true;
}

bool CraneActionStart(CraneActionInstance *action, CraneActionKind kind,
                      const XMotorMotionProfile profiles[2], uint32_t sequence, uint32_t now_ms)
{
    if (action == NULL || !action->initialized) return false;
    bool valid_kind = kind == CRANE_ACTION_PICK || kind == CRANE_ACTION_PLACE;
    size_t index = kind == CRANE_ACTION_PLACE ? 1U : 0U;
    if (valid_kind && action->has_sequence[index] && action->last_sequence[index] == sequence)
        return true; /* 完成/取消后重复旧帧同样不重启。 */
    if (CraneActionIsRunning(action) || !CraneActionReady(action, kind) || profiles == NULL ||
        !XMotorProfileValid(&profiles[0]) || !XMotorProfileValid(&profiles[1])) goto reject;
    for (size_t i = 0U; i < 2U; ++i)
    {
        XMotorFeedback feedback;
        XMotorMotionStatus motion;
        if (!XMotorGetFeedback(action->axes[i], now_ms, &feedback) ||
            !XMotorGetMotion(action->axes[i], now_ms, &motion) ||
            !feedback.position_valid || !feedback.status_valid || feedback.stalled ||
            feedback.stall_protection || motion.stop_pending ||
            (motion.goal_active && motion.state != X_MOTION_REACHED)) goto reject;
        if (motion.goal_active)
        {
            int64_t error = motion.target_units - feedback.position_units;
            if (error < 0) error = -error;
            if (!motion.command_accepted || !feedback.enabled || !feedback.position_reached ||
                (uint64_t)error > action->axes[i]->config.position_tolerance_units) goto reject;
        }
    }
    action->active_config = action->config;
    action->point = kind == CRANE_ACTION_PICK ? action->config.pick : action->config.place;
    action->profiles[0] = profiles[0]; action->profiles[1] = profiles[1];
    action->axis_owned[0] = false; action->axis_owned[1] = false;
    action->has_sequence[index] = true; action->last_sequence[index] = sequence;
    ++action->status.generation;
    action->status.state = CRANE_ACTION_RUNNING;
    action->status.action = kind;
    action->status.sequence = sequence;
    action->status.fault = CRANE_ACTION_OK;
    action->status.axis_fault = X_FAULT_NONE;
    action->status.fault_axis = UINT8_MAX;
    if (!EnterStep(action, CRANE_STEP_SAFE_Z, now_ms))
    { Terminate(action, CRANE_ACTION_OUTPUT, true); return false; }
    return true;
reject:
    ++action->status.rejected_commands;
    return false;
}

void CraneActionUpdate(CraneActionInstance *action, uint32_t now_ms)
{
    if (!CraneActionIsRunning(action)) return;
    action->status.step_elapsed_ms = (uint32_t)(now_ms - action->step_started_ms);
    bool held_reached = true;
    for (size_t i = 0U; i < 2U; ++i)
    {
        XMotorFeedback feedback;
        XMotorMotionStatus motion;
        (void)XMotorGetFeedback(action->axes[i], now_ms, &feedback);
        (void)XMotorGetMotion(action->axes[i], now_ms, &motion);
        XMotorFault fault = motion.fault;
        if (!feedback.position_valid || !feedback.status_valid) fault = X_FAULT_FEEDBACK_LOST;
        else if (feedback.stalled || feedback.stall_protection) fault = X_FAULT_DRIVER;
        else if (motion.command_accepted && !feedback.enabled) fault = X_FAULT_DISABLED;
        /* 尚未给该轴目标时允许既有已恢复的历史fault；停机/保护总是终止。 */
        if (motion.stop_pending || !feedback.position_valid || !feedback.status_valid ||
            feedback.stalled || feedback.stall_protection ||
            (motion.command_accepted && !feedback.enabled) ||
            (action->axis_owned[i] && (fault != X_FAULT_NONE || !motion.goal_active)))
        {
            action->status.axis_fault = fault;
            action->status.fault_axis = (uint8_t)i;
            Terminate(action, CRANE_ACTION_AXIS, true);
            return;
        }
        if (!action->axis_owned[i]) continue;
        if (motion.generation != action->axis_generation[i] ||
            motion.target_units != action->axis_target[i])
        { action->status.fault_axis = (uint8_t)i; Terminate(action, CRANE_ACTION_TARGET_CHANGED, true); return; }
        int64_t error = motion.target_units - feedback.position_units;
        if (error < 0) error = -error;
        held_reached &= motion.state == X_MOTION_REACHED && motion.command_accepted &&
            feedback.enabled && feedback.position_reached &&
            (uint64_t)error <= action->axes[i]->config.position_tolerance_units;
    }
    if (action->status.step_elapsed_ms >= action->active_config.step_timeout_ms)
    { Terminate(action, CRANE_ACTION_TIMEOUT, true); return; }
    bool done = false;
    if (action->status.step == CRANE_STEP_RACK)
    {
        uint32_t wait = action->active_config.rack_wait_ms;
        if (action->status.action == CRANE_ACTION_PICK && wait < action->active_config.gripper_wait_ms)
            wait = action->active_config.gripper_wait_ms;
        done = action->status.step_elapsed_ms >= wait;
    }
    else if (action->status.step == CRANE_STEP_GRIP)
        done = action->status.step_elapsed_ms >= action->active_config.gripper_wait_ms;
    else
    {
        size_t axis = action->status.step == CRANE_STEP_YAW ? 0U : 1U;
        const XMotorDiagnostics *diagnostics = &action->axes[axis]->diagnostics;
        /* 即使同目标沿用了已有FD，也必须读取步骤开始后的两类新反馈。 */
        done = diagnostics->position_replies != action->step_position_replies &&
               diagnostics->status_replies != action->step_status_replies;
    }
    if (!done || !held_reached) return;
    if (action->status.step == CRANE_STEP_LIFT) action->status.state = CRANE_ACTION_DONE;
    else if (!EnterStep(action, (CraneActionStep)(action->status.step + 1U), now_ms))
        Terminate(action, CRANE_ACTION_OUTPUT, true);
}
