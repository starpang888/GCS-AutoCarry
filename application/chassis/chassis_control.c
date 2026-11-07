/* 底盘持续控制：速度命令自动使能，保持最新目标，STOP/设备故障停止。 */
#include "chassis_control.h"

/** @brief 判断停止/失能事务及错误隔离是否尚未结束。 */
static bool SafetyBusy(const ChassisControlInstance *control)
{
    const EmmBusInstance *bus = control->config.bus;
    return bus->stop_pending || bus->disable_pending || bus->quarantine_active ||
        (bus->active && (bus->current.kind == EMM_STOP_ALL || bus->current.kind == EMM_DISABLE_ALL));
}

/** @brief 核对四轮反馈新鲜且无堵转；enabled 为 -1 时不要求统一使能状态。 */
static bool Ready(const ChassisControlInstance *control, uint32_t now_ms, int enabled)
{
    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
    {
        EmmMotorFeedback feedback;
        if (!EmmMotorGetFeedback(control->config.motors[i], now_ms, &feedback) ||
            !feedback.position_valid || !feedback.enabled_valid || !feedback.stall_valid ||
            feedback.stalled || feedback.stall_protection ||
            (enabled >= 0 && feedback.enabled != (enabled == 1))) return false;
    }
    return true;
}

/** @brief 校验依赖和四轮顺序，不发送使能或速度指令。 */
bool ChassisControlInit(ChassisControlInstance *control, const ChassisControl_Init_Config_s *config)
{
    if (control == NULL || config == NULL || config->bus == NULL || !config->bus->initialized ||
        config->bus->motor_count != OMNI_WHEEL_COUNT || config->kinematics == NULL ||
        !config->kinematics->initialized) return false;
    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
        if (config->motors[i] == NULL || config->motors[i] != config->bus->motors[i] ||
            config->motors[i]->config.wheel_rad_s_per_speed_unit <= 0.0f) return false;
    *control = (ChassisControlInstance){.config = *config, .initialized = true};
    return true;
}

/** @brief 判断是否存在待启动命令或处于使能、运行、停止阶段。 */
bool ChassisControlIsBusy(const ChassisControlInstance *control)
{
    return control != NULL && control->initialized &&
        (control->start_pending || control->status.state == CHASSIS_ENABLING || control->status.state == CHASSIS_ACTIVE ||
         control->status.state == CHASSIS_STOPPING);
}

/** @brief 清零目标及待启动请求，向总线提交停止失能请求。 */
void ChassisControlStop(ChassisControlInstance *control, ChassisControlFault fault, uint32_t now_ms)
{
    if (control == NULL || !control->initialized) return;
    control->start_pending = false;
    control->status.target = (OmniKinematicsTarget){0};
    if (control->status.state == CHASSIS_STOPPING)
    {
        if (control->status.fault == CHASSIS_OK) control->status.fault = fault;
        return; /* 重复 STOP 不延长停止确认期限。 */
    }
    control->status.state = CHASSIS_STOPPING;
    control->status.fault = fault;
    control->phase_ms = now_ms;
    EmmBusStop(control->config.bus, true, now_ms);
}

/** @brief 共用SI速度入口，避免位置环将浮点速度转换成整数USB载荷再解码。 */
bool ChassisControlSetVelocity(ChassisControlInstance *control,
                               const OmniBodyVelocity *velocity, uint32_t now_ms)
{
    if (control == NULL || !control->initialized || velocity == NULL) return false;
    if (!OmniKinematicsInverse(control->config.kinematics, velocity, &control->status.target))
    {
        ChassisControlStop(control, CHASSIS_INVALID_COMMAND, now_ms);
        return false;
    }
    if (control->status.state != CHASSIS_ACTIVE && control->status.state != CHASSIS_ENABLING)
        control->start_pending = true;
    return true;
}

/** @brief 最新速度覆盖旧目标；ARM 仅作为可选的零速预使能命令。 */
void ChassisControlCommand(ChassisControlInstance *control, const HostCommand *command,
                            bool external_busy, uint32_t now_ms)
{
    if (control == NULL || !control->initialized || command == NULL) return;
    if (command->type == HOST_STOP)
    {
        ChassisControlStop(control, CHASSIS_OK, now_ms);
        return;
    }
    if (external_busy || (command->type != HOST_ARM && command->type != HOST_VELOCITY))
    {
        ++control->status.rejected_commands;
        return;
    }
    if (command->type == HOST_ARM && ChassisControlIsBusy(control)) return;
    /* 接收顺序就是覆盖顺序，sequence 仅回传；按几何直接计算四轮速度。 */
    OmniBodyVelocity velocity = {0};
    if (command->type == HOST_VELOCITY)
        velocity = (OmniBodyVelocity){command->vx_mm_s * 0.001f, command->vy_mm_s * 0.001f,
            command->wz_mrad_s * 0.001f};
    if (!ChassisControlSetVelocity(control, &velocity, now_ms)) return;
    control->status.last_sequence = command->sequence;
    control->status.last_command_ms = command->received_ms;
}

/** @brief 首条速度可先于反馈到达；保留目标，待总线就绪后自动开始使能。 */
static void StartPending(ChassisControlInstance *control, uint32_t now_ms)
{
    if (!control->start_pending || SafetyBusy(control) || !Ready(control, now_ms, 0)) return;
    control->start_pending = false;
    control->status.state = CHASSIS_ENABLING;
    control->status.fault = CHASSIS_OK;
    control->phase_ms = now_ms;
    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
        if (!EmmMotorSetEnabled(control->config.motors[i], true, now_ms))
        {
            ChassisControlStop(control, CHASSIS_MOTOR_ERROR, now_ms);
            return;
        }
}

/** @brief 按轮序提交 SI 轮速，四轮任一失败均由调用者触发全体停止。 */
static bool Submit(ChassisControlInstance *control, uint32_t now_ms)
{
    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
        if (!EmmMotorSetSpeed(control->config.motors[i], control->status.target.wheel_rad_s[i],
                               0U, now_ms)) return false;
    control->last_output_ms = now_ms;
    return true;
}

/** @brief 持续刷新已保存目标，只检查设备状态和使能/停止确认期限。 */
void ChassisControlUpdate(ChassisControlInstance *control, uint32_t now_ms)
{
    if (control == NULL || !control->initialized || !ChassisControlIsBusy(control)) return;
    if (control->status.state == CHASSIS_STOPPING)
    {
        /* 必须收到本次 STOP 之后的失能反馈，旧缓存不能证明已停止。 */
        bool disabled = !SafetyBusy(control);
        for (size_t i = 0; disabled && i < OMNI_WHEEL_COUNT; ++i)
        {
            EmmMotorFeedback feedback;
            disabled = EmmMotorGetFeedback(control->config.motors[i], now_ms, &feedback) &&
                feedback.enabled_valid && !feedback.enabled &&
                (int32_t)(feedback.enabled_ms - control->phase_ms) > 0;
        }
        if (disabled) control->status.state = control->status.fault == CHASSIS_OK ? CHASSIS_DISABLED : CHASSIS_FAULT;
        else if ((uint32_t)(now_ms - control->phase_ms) >= 1000U)
        {
            control->status.state = CHASSIS_FAULT;
            control->status.fault = CHASSIS_STOP_TIMEOUT;
            EmmBusStop(control->config.bus, true, now_ms);
        }
        return;
    }
    if (control->status.state == CHASSIS_DISABLED || control->status.state == CHASSIS_FAULT)
    {
        StartPending(control, now_ms);
        return;
    }
    if (!Ready(control, now_ms, control->status.state == CHASSIS_ACTIVE ? 1 : -1))
    {
        ChassisControlStop(control, CHASSIS_MOTOR_ERROR, now_ms);
        return;
    }
    if (control->status.state == CHASSIS_ENABLING)
    {
        if ((uint32_t)(now_ms - control->phase_ms) >= 500U)
        {
            ChassisControlStop(control, CHASSIS_ENABLE_TIMEOUT, now_ms);
            return;
        }
        /* 四轮均确认本次使能后再输出已保存的目标，期间允许更新非零目标。 */
        for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
        {
            const EmmMotorFeedback *feedback = &control->config.motors[i]->feedback;
            if (!feedback->enabled || (int32_t)(feedback->enabled_ms - control->phase_ms) <= 0) return;
        }
        if (!EmmBusArm(control->config.bus, now_ms)) return;
        control->status.state = CHASSIS_ACTIVE;
        if (!Submit(control, now_ms)) ChassisControlStop(control, CHASSIS_MOTOR_ERROR, now_ms);
    }
    else if (!control->config.bus->motion_allowed)
        ChassisControlStop(control, CHASSIS_MOTOR_ERROR, now_ms);
    else if ((uint32_t)(now_ms - control->last_output_ms) >= 50U && !Submit(control, now_ms))
        ChassisControlStop(control, CHASSIS_MOTOR_ERROR, now_ms);
}
