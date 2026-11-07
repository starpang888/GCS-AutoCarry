/* EMM V5.0 功能码协议：编码控制指令，解析累计位置和状态应答。 */
#include "emm_internal.h"
#include <math.h>
#include <string.h>

/* 暂按 65536 位置计数/转，独立于驱动器细分数；完整圈数比例仍待标定。 */
#define EMM_RAD_PER_COUNT (6.2831853071795864769f / 65536.0f)

/** @brief 检查实例是否已经成功注册。 */
static bool Ready(const EmmMotorInstance *motor)
{
    return motor != NULL && motor->registered;
}

/** @brief 校验资源与比例后注册，初始化过程不发送运动命令。 */
bool EmmMotorInit(EmmMotorInstance *motor, const EmmMotor_Init_Config_s *config)
{
    if (motor == NULL || config == NULL || config->bus == NULL ||
        !config->bus->initialized || config->bus->motion_allowed ||
        config->address == 0U || config->address > 247U ||
        (config->direction != 1 && config->direction != -1) ||
        !isfinite(config->gear_ratio) || config->gear_ratio <= 0.0f ||
        !isfinite(config->wheel_rad_s_per_speed_unit) || config->wheel_rad_s_per_speed_unit < 0.0f)
    {
        return false;
    }
    /* 清零实例之前检查冲突，避免重复注册破坏已经使用的对象。 */
    for (uint8_t i = 0; i < config->bus->motor_count; ++i)
    {
        if (config->bus->motors[i] == motor ||
            config->bus->motors[i]->config.address == config->address)
        {
            return false;
        }
    }
    if (config->bus->motor_count >= EMM_MOTOR_CAPACITY)
    {
        return false;
    }
    memset(motor, 0, sizeof(*motor));
    motor->config = *config;
    motor->registered = EmmBusRegister(config->bus, motor);
    return motor->registered;
}

/** @brief 构造原生速度帧，成功入队后刷新总线和本轮目标期限。 */
bool EmmMotorSetSpeedUnits(EmmMotorInstance *motor, int16_t signed_units,
                         uint8_t acceleration, uint32_t now_ms)
{
    if (!Ready(motor) || !EmmBusMotorIsArmed(motor->config.bus, motor) ||
        signed_units < -EMM_MAX_SPEED_UNITS || signed_units > EMM_MAX_SPEED_UNITS)
    {
        return false;
    }
    uint16_t units = (uint16_t)(signed_units < 0 ? -signed_units : signed_units);
    /* 速度帧 8 字节：地址/F6/方向/速度高低字节/加速度/同步标志/6B。
     * 同步标志为 0，当前总线顺序下发四轮，未使用多机同步触发。 */
    EmmTransaction transaction = {
        .motor = motor, .kind = EMM_COMMAND_SPEED, .length = 8U, .response_length = 4U,
        .frame = {motor->config.address, 0xF6U,
                  (uint8_t)(signed_units < 0 ? 1U : 0U), (uint8_t)(units >> 8U),
                  (uint8_t)units, acceleration, 0U, EMM_CHECK_BYTE},
    };
    if (!EmmBusSubmit(motor->config.bus, &transaction))
    {
        return false;
    }
    motor->config.bus->last_command_ms = now_ms;
    motor->last_target_ms = now_ms;
    return true;
}

/** @brief 轮轴 rad/s 换算为有符号协议单位并四舍五入。 */
bool EmmMotorSetSpeed(EmmMotorInstance *motor, float wheel_rad_s,
                     uint8_t acceleration, uint32_t now_ms)
{
    if (!Ready(motor) || !isfinite(wheel_rad_s) ||
        motor->config.wheel_rad_s_per_speed_unit <= 0.0f)
    {
        return false;
    }
    /* 板级比例已经是轮轴 rad/s/线协议单位，这里只再应用安装方向。 */
    float units = wheel_rad_s * motor->config.direction / motor->config.wheel_rad_s_per_speed_unit;
    if (!isfinite(units) || units < -EMM_MAX_SPEED_UNITS || units > EMM_MAX_SPEED_UNITS)
    {
        return false;
    }
    return EmmMotorSetSpeedUnits(motor, (int16_t)(units + (units < 0.0f ? -0.5f : 0.5f)),
                               acceleration, now_ms);
}

/** @brief 使能按地址排队，失能走总线一致停止路径。 */
bool EmmMotorSetEnabled(EmmMotorInstance *motor, bool enabled, uint32_t now_ms)
{
    if (!Ready(motor))
    {
        return false;
    }
    if (!enabled)
    {
        /* 任一轮要求失能时整车一起停止，不能让另外三轮继续驱动。 */
        EmmBusStop(motor->config.bus, true, now_ms);
        return true;
    }
    EmmTransaction transaction = {
        .motor = motor, .kind = EMM_COMMAND_ENABLE, .length = 6U, .response_length = 4U,
        .frame = {motor->config.address, 0xF3U, 0xABU, 1U, 0U, EMM_CHECK_BYTE},
    };
    bool submitted = EmmBusSubmit(motor->config.bus, &transaction);
    if (submitted)
    {
        /* 留出一个周期停止间隔来确认使能并显式取得运动许可。
         * 该准备阶段只发送使能，不能绕过授权直接输出速度。
         */
        motor->config.bus->stop_repeat_ms = now_ms;
    }
    return submitted;
}

/** @brief 按功能码选择固定回复长度：位置 8 字节、状态 4 字节。 */
bool EmmMotorQuery(EmmMotorInstance *motor, EmmTransactionKind query)
{
    if (!Ready(motor) || (query != EMM_QUERY_POSITION &&
        query != EMM_QUERY_STATUS))
    {
        return false;
    }
    uint8_t function = query == EMM_QUERY_POSITION ? 0x36U : 0x3AU;
    EmmTransaction transaction = {
        .motor = motor, .kind = query, .length = 3U,
        .response_length = query == EMM_QUERY_POSITION ? 8U : 4U,
        .frame = {motor->config.address, function, EMM_CHECK_BYTE},
    };
    return EmmBusSubmit(motor->config.bus, &transaction);
}

/** @brief 作废有效标志，避免传输异常后的旧值继续参与控制。 */
void EmmMotorInvalidate(EmmMotorInstance *motor)
{
    motor->feedback.position_valid = false;
    motor->feedback.speed_valid = false;
    motor->feedback.enabled_valid = false;
    motor->feedback.stall_valid = false;
}

/** @brief 提供按当前时刻判定有效性的副本，不修改实例原始缓存。 */
bool EmmMotorGetFeedback(const EmmMotorInstance *motor, uint32_t now_ms,
                         EmmMotorFeedback *feedback)
{
    if (!Ready(motor) || feedback == NULL)
    {
        return false;
    }
    *feedback = motor->feedback;
    uint32_t timeout = motor->config.bus->config.feedback_timeout_ms;
    feedback->position_valid &= (uint32_t)(now_ms - feedback->position_ms) < timeout;
    feedback->speed_valid &= feedback->position_valid;
    feedback->enabled_valid &= (uint32_t)(now_ms - feedback->enabled_ms) < timeout;
    feedback->stall_valid &= (uint32_t)(now_ms - feedback->stall_ms) < timeout;
    return true;
}

/** @brief 检查运动许可所需的反馈和驱动状态。 */
bool EmmMotorHealthy(const EmmMotorInstance *motor, uint32_t now_ms)
{
    EmmMotorFeedback feedback;
    return EmmMotorGetFeedback(motor, now_ms, &feedback) && feedback.position_valid &&
           feedback.enabled_valid && feedback.enabled && feedback.stall_valid &&
           !feedback.stalled && !feedback.stall_protection;
}

/** @brief 请求总线停止，影响所有电机，不仅是调用的单轮。 */
void EmmMotorStop(EmmMotorInstance *motor, uint32_t now_ms)
{
    if (Ready(motor))
    {
        EmmBusStop(motor->config.bus, false, now_ms);
    }
}

/** @brief 解析有效查询回复，按实际采样间隔差分估算轮速。 */
void EmmMotorResponse(EmmMotorInstance *motor, EmmTransactionKind kind,
                      const uint8_t *frame, uint32_t now_ms)
{
    motor->last_reply_ms = now_ms;
    EmmMotorFeedback *feedback = &motor->feedback;
    if (kind == EMM_QUERY_POSITION)
    {
        uint32_t magnitude = ((uint32_t)frame[3] << 24U) | ((uint32_t)frame[4] << 16U) |
                             ((uint32_t)frame[5] << 8U) | frame[6];
        /* 协议使用独立符号和 uint32 幅值，合并到 int64，避免 int32 范围不足。 */
        int64_t position = frame[2] == 0U ? (int64_t)magnitude : -(int64_t)magnitude;
        int64_t delta = position - feedback->position_raw;
        uint32_t dt_ms = now_ms - feedback->position_ms;
        feedback->speed_valid = feedback->position_valid && dt_ms > 0U &&
            dt_ms < motor->config.bus->config.feedback_timeout_ms;
        float conversion = EMM_RAD_PER_COUNT * motor->config.direction / motor->config.gear_ratio;
        /* 按该指令族 5000 RPM 的上界拒绝不连续位置、复位或溢出跳变；
         * 不擅自假设位置回绕规则。此上界用于有效性判断，不是底盘运行上限。
         */
        float max_delta = (5000.0f * 65536.0f / 60000.0f) * dt_ms + 2.0f;
        feedback->speed_valid &= fabsf((float)delta) <= max_delta;
        if (feedback->speed_valid)
        {
            feedback->wheel_speed_rad_s = (float)delta * conversion * 1000.0f / dt_ms;
        }
        feedback->position_raw = position;
        feedback->wheel_position_rad = (float)position * conversion;
        feedback->position_ms = now_ms;
        feedback->position_valid = true;
    }
    else if (kind == EMM_QUERY_STATUS)
    {
        /* 状态字 bit0=使能、bit2=堵转、bit3=堵转保护；二者共用本次时间。 */
        feedback->status_raw = frame[2];
        feedback->enabled = (frame[2] & 0x01U) != 0U;
        feedback->stalled = (frame[2] & 0x04U) != 0U;
        feedback->stall_protection = (frame[2] & 0x08U) != 0U;
        feedback->enabled_ms = now_ms;
        feedback->enabled_valid = true;
        feedback->stall_ms = now_ms;
        feedback->stall_valid = true;
    }
    /* 写命令成功应答只代表驱动接受，不凭此生成位置或速度测量。 */
}
