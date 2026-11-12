/* 应用仲裁：USB速度或位置目标 → 唯一底盘输出；所有调用均位于motorTask。 */
#include "robot.h"
#include <stddef.h>
#include <math.h>

#define ROBOT_POSITION_PERIOD_MS 50U

bool RobotCraneInit(RobotInstance *robot, XMotorInstance *yaw, XMotorInstance *z,
                    const XMotorMotionProfile profiles[ROBOT_CRANE_AXIS_COUNT])
{
    if (robot == NULL || yaw == NULL || z == NULL || yaw == z ||
        !yaw->initialized || !z->initialized || profiles == NULL ||
        !XMotorProfileValid(&profiles[0]) || !XMotorProfileValid(&profiles[1])) return false;
    robot->crane_axes[0] = yaw;
    robot->crane_axes[1] = z;
    robot->crane_profiles[0] = profiles[0];
    robot->crane_profiles[1] = profiles[1];
    return true;
}

bool RobotRackServoInit(RobotInstance *robot, ServoInstance *servo)
{
    if (robot == NULL || servo == NULL || !servo->initialized) return false;
    robot->rack_servo = servo;
    return true;
}

bool RobotGripperServoInit(RobotInstance *robot, ServoInstance *servo)
{
    if (robot == NULL || servo == NULL || !servo->initialized) return false;
    robot->gripper_servo = servo;
    return true;
}

bool RobotCraneIsBusy(const RobotInstance *robot)
{
    if (robot == NULL) return false;
    if (CraneActionIsRunning(&robot->crane_action)) return true;
    for (size_t i = 0; i < ROBOT_CRANE_AXIS_COUNT; ++i)
        if (XMotorIsBusy(robot->crane_axes[i])) return true;
    return false;
}

void RobotCraneStop(RobotInstance *robot, XMotorFault fault, bool only_busy)
{
    if (robot == NULL) return;
    CraneActionCancel(&robot->crane_action, fault == X_FAULT_LINK ? CRANE_ACTION_LINK : CRANE_ACTION_STOP);
    for (size_t i = 0; i < ROBOT_CRANE_AXIS_COUNT; ++i)
        if (!only_busy || XMotorIsBusy(robot->crane_axes[i])) XMotorStop(robot->crane_axes[i], fault);
}

bool RobotCraneActionInit(RobotInstance *robot, const CraneActionConfig *config)
{
    return robot != NULL && CraneActionInit(&robot->crane_action, robot->crane_axes[0],
        robot->crane_axes[1], robot->rack_servo, robot->gripper_servo, config);
}

/* 只清除到达判定，不停止、不改变输出速度。 */
static void ResetArrival(RobotInstance *robot, RobotArrivalState state)
{
    robot->arrival = (RobotArrivalStatus){.state = state};
    robot->arrival_has_sample = false;
}

/* 取消运动来源，但保留最后目标坐标，便于查看停车前的请求。 */
static void CancelPosition(RobotInstance *robot)
{
    robot->mode = ROBOT_IDLE;
    ResetArrival(robot, ROBOT_ARRIVAL_NONE);
    if (robot->position_control != NULL)
        robot->position_control->result = (ChassisPositionControlResult){0};
}

/* 位置闭环需要知道当前在哪里；检查的是定位输入，不增加主机发帧期限。 */
static bool PoseReady(const RobotInstance *robot, uint32_t now_ms)
{
    return ChassisLocalizationReady(robot->localization, now_ms);
}

bool RobotInit(RobotInstance *robot, ChassisControlInstance *control)
{
    if (robot == NULL || control == NULL || !control->initialized) return false;
    *robot = (RobotInstance){
        .control = control,
        .kinematics = control->config.kinematics,
        .mode = ROBOT_IDLE,
    };
    return true;
}

bool RobotPositionInit(RobotInstance *robot, ChassisLocalizationFeedback *localization,
                       ChassisPositionControlInstance *position_control,
                       const RobotArrival_Init_Config_s *arrival_config)
{
    /* 定位尚未收到第一帧时仍允许组装；真正的位置命令在PoseReady处检查。 */
    if (robot == NULL || robot->control == NULL || localization == NULL ||
        position_control == NULL || !position_control->initialized || arrival_config == NULL ||
        !isfinite(arrival_config->linear_speed_m_s) || arrival_config->linear_speed_m_s <= 0.0f ||
        !isfinite(arrival_config->angular_speed_rad_s) || arrival_config->angular_speed_rad_s <= 0.0f ||
        arrival_config->hold_ms == 0U || arrival_config->hold_ms >= 0x80000000U ||
        arrival_config->feedback_timeout_ms == 0U || arrival_config->feedback_timeout_ms >= 0x80000000U ||
        arrival_config->max_sample_span_ms >= arrival_config->feedback_timeout_ms) return false;
    robot->localization = localization;
    robot->position_control = position_control;
    robot->arrival_config = *arrival_config;
    return true;
}

void RobotStop(RobotInstance *robot, ChassisControlFault fault, uint32_t now_ms)
{
    if (robot == NULL || robot->control == NULL) return;
    /* 取消必须发生在停止之前，否则位置环会在下一周期再次发出速度。 */
    CancelPosition(robot);
    ChassisControlStop(robot->control, fault, now_ms);
}

void RobotHostCommand(RobotInstance *robot, const HostCommand *command, uint32_t now_ms)
{
    if (robot == NULL || robot->control == NULL || command == NULL) return;

    if (command->type == HOST_STOP)
    {
        RobotCraneStop(robot, X_FAULT_NONE, false);
        RobotStop(robot, CHASSIS_OK, now_ms);
        return;
    }

    if (command->type == HOST_CRANE_DISABLE)
    {
        if (command->crane_axis >= ROBOT_CRANE_AXIS_COUNT ||
            robot->crane_axes[command->crane_axis] == NULL)
        { ++robot->control->status.rejected_commands; return; }
        CraneActionCancel(&robot->crane_action, CRANE_ACTION_STOP);
        XMotorInstance *axis = robot->crane_axes[command->crane_axis];
        axis->motion.sequence = command->sequence;
        XMotorDisable(axis);
        return;
    }

    if (command->type == HOST_CRANE_PICK || command->type == HOST_CRANE_PLACE)
    {
        if (!CraneActionStart(&robot->crane_action,
            command->type == HOST_CRANE_PICK ? CRANE_ACTION_PICK : CRANE_ACTION_PLACE,
            robot->crane_profiles, command->sequence, now_ms))
            ++robot->control->status.rejected_commands;
        return;
    }

    if (CraneActionIsRunning(&robot->crane_action))
    {
        if (command->type == HOST_CRANE_STOP && command->crane_axis < ROBOT_CRANE_AXIS_COUNT)
        {
            RobotCraneStop(robot, X_FAULT_NONE, false);
            return;
        }
        if (command->type == HOST_CRANE_TARGET || command->type == HOST_CRANE_CONFIG ||
            command->type == HOST_SERVO_PULSE || command->type == HOST_GRIPPER_SERVO_PULSE)
        {
            ++robot->control->status.rejected_commands;
            return;
        }
    }

    if (command->type == HOST_SERVO_PULSE)
    {
        if (robot->rack_servo == NULL ||
            !ServoSetPulseUs(robot->rack_servo, command->servo_pulse_us))
            ++robot->control->status.rejected_commands;
        return;
    }

    if (command->type == HOST_GRIPPER_SERVO_PULSE)
    {
        if (robot->gripper_servo == NULL ||
            !ServoSetPulseUs(robot->gripper_servo, command->gripper_servo_pulse_us))
            ++robot->control->status.rejected_commands;
        return;
    }

    if (command->type == HOST_CRANE_TARGET || command->type == HOST_CRANE_CONFIG ||
        command->type == HOST_CRANE_STOP)
    {
        if (command->crane_axis >= ROBOT_CRANE_AXIS_COUNT ||
            robot->crane_axes[command->crane_axis] == NULL)
        { ++robot->control->status.rejected_commands; return; }
        XMotorInstance *axis = robot->crane_axes[command->crane_axis];
        if (command->crane_reserved != 0U)
        { ++axis->motion.rejected_commands; return; }
        if (command->type == HOST_CRANE_STOP)
        { axis->motion.sequence = command->sequence; XMotorStop(axis, X_FAULT_NONE); }
        else if (command->type == HOST_CRANE_CONFIG)
        {
            XMotorMotionProfile profile = {command->crane_speed_deci_rpm,
                command->crane_acceleration_rpm_s, command->crane_deceleration_rpm_s};
            if (!XMotorProfileValid(&profile)) ++axis->motion.rejected_commands;
            else robot->crane_profiles[command->crane_axis] = profile;
        }
        else (void)XMotorSetTarget(axis, command->crane_target_units,
            &robot->crane_profiles[command->crane_axis], command->sequence, now_ms);
        return;
    }

    if (command->type == HOST_POSITION)
    {
        /* 位置命令须等OPS定位就绪；停止确认期间的新位置命令需要稍后重发。
         * 既有速度命令的等待反馈启动行为保持原样。
         */
        if (robot->position_control == NULL || !PoseReady(robot, now_ms) ||
            robot->control->status.state == CHASSIS_STOPPING)
        {
            ++robot->control->status.rejected_commands;
            return;
        }

        /* OPS失效后已恢复，也必须先取消旧代次目标。
         * 本条命令不在停止确认之前重新使能；主机随后发新命令才能重新运行。
         */
        if (robot->mode == ROBOT_POSITION &&
            robot->goal_pose_epoch != robot->localization->continuity_epoch)
        {
            RobotStop(robot, CHASSIS_NOT_READY, now_ms);
            ++robot->control->status.rejected_commands;
            return;
        }

        ChassisPose target = {
            .x_m = command->target_x_mm * 0.001f,
            .y_m = command->target_y_mm * 0.001f,
            .yaw_rad = command->target_yaw_mrad * 0.001f,
        };

        /* 新位置按绝对坐标覆盖旧目标，不重置里程计，也不追加位移。 */
        if (!ChassisPositionControlCalculate(robot->position_control,
                                             &robot->localization->pose, &target) ||
            !ChassisControlSetVelocity(robot->control,
                                      &robot->position_control->result.velocity, now_ms))
        {
            RobotStop(robot, CHASSIS_INVALID_COMMAND, now_ms);
            return;
        }

        /* 比较原始整数目标：相同坐标重复发帧不重新计时，也不累计位移。
         * sequence即便变化也只是回显；上位机可结合generation区分不同目标。
         */
        bool changed = robot->mode != ROBOT_POSITION ||
            robot->position_target_milli[0] != command->target_x_mm ||
            robot->position_target_milli[1] != command->target_y_mm ||
            robot->position_target_milli[2] != command->target_yaw_mrad;
        if (changed)
        {
            ++robot->goal_generation;
            ResetArrival(robot, ROBOT_ARRIVAL_APPROACHING);
        }
        robot->position_sequence = command->sequence;
        robot->position_target_milli[0] = command->target_x_mm;
        robot->position_target_milli[1] = command->target_y_mm;
        robot->position_target_milli[2] = command->target_yaw_mrad;
        robot->target_pose = target;
        robot->goal_pose_epoch = robot->localization->continuity_epoch;
        robot->mode = ROBOT_POSITION;
        /* 相同目标连续发帧不能推迟50ms更新，否则会饿死到达检测。 */
        if (changed) robot->last_position_update_ms = now_ms;
        /* 只有外部新命令更新接受标记；位置环内部计算不伪造主机消息。 */
        robot->control->status.last_sequence = command->sequence;
        robot->control->status.last_command_ms = command->received_ms;
        return;
    }

    if (command->type == HOST_VELOCITY)
    {
        /* 包括零速度：切到速度模式，旧位置目标不再参与输出。 */
        CancelPosition(robot);
        robot->mode = ROBOT_VELOCITY;
        ChassisControlCommand(robot->control, command, false, now_ms);
        return;
    }

    if (command->type == HOST_ARM)
    {
        /* ARM仅用于可选零速预使能，位置控制中忽略重复ARM。 */
        if (robot->mode != ROBOT_POSITION)
            ChassisControlCommand(robot->control, command, false, now_ms);
        return;
    }

    ++robot->control->status.rejected_commands;
}

/* 只在位置环周期内观察实际反馈。底盘状态和停止逻辑仍由原控制器负责。 */
static void UpdateArrival(RobotInstance *robot, uint32_t now_ms)
{
    const RobotArrival_Init_Config_s *config = &robot->arrival_config;
    float wheel_speeds[OMNI_WHEEL_COUNT];
    uint32_t stamps[OMNI_WHEEL_COUNT];
    uint32_t youngest_age = UINT32_MAX, oldest_age = 0U;
    bool valid = true, all_new = true;
    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
    {
        EmmMotorFeedback feedback;
        if (!EmmMotorGetFeedback(robot->control->config.motors[i], now_ms, &feedback))
        {
            ResetArrival(robot, ROBOT_ARRIVAL_APPROACHING);
            return;
        }
        uint32_t age = now_ms - feedback.position_ms;
        if (age < youngest_age) youngest_age = age;
        if (age > oldest_age) oldest_age = age;
        valid &= feedback.position_valid && feedback.speed_valid &&
            age < config->feedback_timeout_ms;
        wheel_speeds[i] = feedback.wheel_speed_rad_s;
        stamps[i] = feedback.position_ms;
        all_new &= !robot->arrival_has_sample || stamps[i] != robot->arrival_sample_ms[i];
    }
    OmniBodyVelocity measured;
    valid = valid && oldest_age - youngest_age <= config->max_sample_span_ms &&
        OmniKinematicsForward(robot->kinematics, wheel_speeds, &measured);
    if (!valid)
    {
        /* 无法测量时只撤销“到达”，不因这个低速判断新增停机条件。 */
        ResetArrival(robot, ROBOT_ARRIVAL_APPROACHING);
        return;
    }
    bool eligible = robot->control->status.state == CHASSIS_ACTIVE &&
        robot->position_control->result.valid && robot->position_control->result.within_tolerance &&
        hypotf(measured.vx_m_s, measured.vy_m_s) <= config->linear_speed_m_s &&
        fabsf(measured.wz_rad_s) <= config->angular_speed_rad_s;
    if (!eligible) ResetArrival(robot, ROBOT_ARRIVAL_APPROACHING);
    robot->arrival.measured = measured;
    robot->arrival.measured_valid = true;
    if (!eligible || !all_new) return;

    /* 用最旧轮采样的时间表示这一组证据的时刻。只有四轮均更新才增加时间；
     * 主循环反复读取同一组“零速”不能凑满300ms。
     */
    uint32_t sample_ms = now_ms - oldest_age;
    if (!robot->arrival_has_sample)
    {
        /* 条件首次被应用观察到之前的缓存时间不计入稳定时长。 */
        robot->arrival_start_sample_ms = now_ms;
        robot->arrival_has_sample = true;
        robot->arrival.state = ROBOT_ARRIVAL_SETTLING;
    }
    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i) robot->arrival_sample_ms[i] = stamps[i];
    uint32_t elapsed = (int32_t)(sample_ms - robot->arrival_start_sample_ms) >= 0 ?
        sample_ms - robot->arrival_start_sample_ms : 0U;
    robot->arrival.stable_ms = elapsed < config->hold_ms ? elapsed : config->hold_ms;
    if (elapsed >= config->hold_ms) robot->arrival.state = ROBOT_ARRIVAL_REACHED;
}

void RobotUpdate(RobotInstance *robot, uint32_t now_ms)
{
    if (robot == NULL || robot->control == NULL) return;
    CraneActionUpdate(&robot->crane_action, now_ms);

    if (robot->mode != ROBOT_POSITION)
    {
        ChassisControlUpdate(robot->control, now_ms);
        return;
    }

    if (!PoseReady(robot, now_ms) ||
        robot->goal_pose_epoch != robot->localization->continuity_epoch)
    {
        /* fault=7在位置模式下表示定位不可用；取消目标，恢复后须新命令。 */
        RobotStop(robot, CHASSIS_NOT_READY, now_ms);
        return;
    }

    /* 先推进底盘设备检查，再允许位置环更新目标。
     * 若底层已经因堵转/离线等故障停车，不允许内部速度请求重新启动。
     */
    ChassisControlUpdate(robot->control, now_ms);
    if (robot->control->status.state == CHASSIS_STOPPING ||
        (robot->control->status.state == CHASSIS_FAULT && !robot->control->start_pending))
    {
        CancelPosition(robot);
        return;
    }

    if ((uint32_t)(now_ms - robot->last_position_update_ms) < ROBOT_POSITION_PERIOD_MS)
        return;
    robot->last_position_update_ms = now_ms;

    if (!ChassisPositionControlCalculate(robot->position_control,
                                         &robot->localization->pose, &robot->target_pose) ||
        !ChassisControlSetVelocity(robot->control,
                                  &robot->position_control->result.velocity, now_ms))
    {
        RobotStop(robot, CHASSIS_INVALID_COMMAND, now_ms);
        return;
    }
    /* 到达只影响反馈，不失能；偏离目标后继续位置保持并重新确认到达。 */
    UpdateArrival(robot, now_ms);
}
