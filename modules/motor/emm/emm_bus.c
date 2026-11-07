/* EMM 总线实现：停止/失能优先于普通事务，一次只等待一个地址的回复。 */
#include "emm_internal.h"
#include <string.h>

/** @brief 区分只读查询与控制写命令。 */
static bool IsQuery(EmmTransactionKind kind)
{
    return kind == EMM_QUERY_POSITION || kind == EMM_QUERY_STATUS;
}

/** @brief 识别广播停止和广播失能这两类安全事务。 */
static bool IsSafety(EmmTransactionKind kind)
{
    return kind == EMM_STOP_ALL || kind == EMM_DISABLE_ALL;
}

/** @brief 只有每个已注册电机的新鲜反馈均失能才返回 true。 */
static bool AllMotorsDisabled(const EmmBusInstance *bus, uint32_t now_ms)
{
    if (bus->motor_count == 0U)
    {
        return false;
    }
    for (uint8_t i = 0; i < bus->motor_count; ++i)
    {
        EmmMotorFeedback feedback;
        if (!EmmMotorGetFeedback(bus->motors[i], now_ms, &feedback) ||
            !feedback.enabled_valid || feedback.enabled)
        {
            return false;
        }
    }
    return true;
}

/** @brief 检查时间边界并设置上电停止；硬件发送由后续 Update 执行。 */
bool EmmBusInit(EmmBusInstance *bus, const EmmBus_Init_Config_s *config, uint32_t now_ms)
{
    if (bus == NULL || config == NULL || config->send == NULL || config->tx_busy == NULL ||
        config->response_timeout_ms == 0U || config->response_timeout_ms > 1000U ||
        config->quiet_ms == 0U || config->quiet_ms > 1000U ||
        config->quarantine_ms < config->response_timeout_ms || config->quarantine_ms > 1000U ||
        config->command_timeout_ms == 0U || config->command_timeout_ms > 60000U ||
        config->feedback_timeout_ms == 0U || config->feedback_timeout_ms > 60000U)
    {
        return false;
    }
    memset(bus, 0, sizeof(*bus));
    bus->config = *config;
    bus->initialized = true;
    bus->last_rx_ms = now_ms;
    bus->guard_started_ms = now_ms;
    bus->guard_duration_ms = config->quiet_ms;
    EmmBusStop(bus, true, now_ms);
    return true;
}

/** @brief 固定实例表注册，运行许可开放后拒绝追加电机。 */
bool EmmBusRegister(EmmBusInstance *bus, EmmMotorInstance *motor)
{
    if (bus == NULL || !bus->initialized || motor == NULL || bus->motion_allowed ||
        bus->motor_count >= EMM_MOTOR_CAPACITY)
    {
        return false;
    }
    for (uint8_t i = 0; i < bus->motor_count; ++i)
    {
        if (bus->motors[i] == motor || bus->motors[i]->config.address == motor->config.address)
        {
            return false;
        }
    }
    bus->motors[bus->motor_count++] = motor;
    return true;
}

/** @brief 验证归属和授权，合并同轮同类排队项，满队列拒绝。 */
bool EmmBusSubmit(EmmBusInstance *bus, const EmmTransaction *transaction)
{
    if (bus == NULL || !bus->initialized || transaction == NULL ||
        transaction->motor == NULL || !transaction->motor->registered ||
        transaction->motor->config.bus != bus || transaction->length < 3U ||
        transaction->length > sizeof(transaction->frame) ||
        (transaction->response_length != 4U && transaction->response_length != 8U) ||
        IsSafety(transaction->kind) ||
        (transaction->kind == EMM_COMMAND_SPEED && !EmmBusMotorIsArmed(bus, transaction->motor)) ||
        (transaction->kind == EMM_COMMAND_ENABLE && (bus->stop_pending || bus->disable_pending ||
            (bus->active && IsSafety(bus->current.kind)))))
    {
        return false;
    }
    if (IsQuery(transaction->kind) && bus->active && bus->current.motor == transaction->motor &&
        bus->current.kind == transaction->kind)
    {
        return true;
    }
    for (uint8_t i = 0; i < bus->queue_count; ++i)
    {
        if (bus->queue[i].motor == transaction->motor && bus->queue[i].kind == transaction->kind)
        {
            /* 同轮同类只保留最新项，避免积压历史速度；不改写已在途事务。 */
            bus->queue[i] = *transaction;
            return true;
        }
    }
    if (bus->queue_count >= EMM_QUEUE_CAPACITY)
    {
        ++bus->queue_full_count;
        if (transaction->kind == EMM_COMMAND_SPEED)
        {
            EmmBusStop(bus, true, bus->last_command_ms);
        }
        return false;
    }
    /* 保存完整事务副本，调用者的局部 transaction 随后可释放。 */
    bus->queue[bus->queue_count++] = *transaction;
    return true;
}

/** @brief 立即撤销软件许可和普通排队目标，异步安排物理停止。 */
void EmmBusStop(EmmBusInstance *bus, bool disable, uint32_t now_ms)
{
    if (bus == NULL || !bus->initialized)
    {
        return;
    }
    bus->motion_allowed = false;
    bus->armed_motor_mask = 0U;
    bus->queue_count = 0U;
    bus->stop_pending = true;
    bus->disable_pending |= disable;
    bus->stop_repeat_ms = now_ms;
    for (uint8_t i = 0; i < bus->motor_count; ++i)
    {
        if (disable)
        {
            bus->motors[i]->feedback.enabled_valid = false;
        }
    }
    /* 当前帧可能仍被 TX DMA 使用，不能在这里覆盖；物理发送结束后
     * Update 会优先放弃旧普通事务的回复等待，再发送广播停止。
     */
}

/** @brief 按实例注册序号查询许可位，拒绝非本总线设备。 */
bool EmmBusMotorIsArmed(const EmmBusInstance *bus, const EmmMotorInstance *motor)
{
    if (bus == NULL || !bus->motion_allowed || motor == NULL)
    {
        return false;
    }
    for (uint8_t i = 0; i < bus->motor_count; ++i)
    {
        if (bus->motors[i] == motor)
        {
            return (bus->armed_motor_mask & (1U << i)) != 0U;
        }
    }
    return false;
}

/** @brief 许可轮须健康已使能，未许可轮须健康失能。 */
static bool MotorMatchesPermit(const EmmBusInstance *bus, uint8_t index,
                               uint8_t mask, uint32_t now_ms)
{
    if ((mask & (1U << index)) != 0U)
    {
        return EmmMotorHealthy(bus->motors[index], now_ms);
    }
    EmmMotorFeedback feedback;
    return EmmMotorGetFeedback(bus->motors[index], now_ms, &feedback) &&
        feedback.position_valid && feedback.enabled_valid && !feedback.enabled &&
        feedback.stall_valid && !feedback.stalled && !feedback.stall_protection;
}

/** @brief 检查全部反馈后一次性建立许可掩码，初始化各轮目标期限。 */
static bool ArmMask(EmmBusInstance *bus, uint8_t mask, uint32_t now_ms)
{
    if (bus == NULL || !bus->initialized || bus->motion_allowed || bus->motor_count == 0U ||
        mask == 0U || (mask >> bus->motor_count) != 0U ||
        bus->stop_pending || bus->disable_pending ||
        (bus->active && IsSafety(bus->current.kind)))
    {
        return false;
    }
    for (uint8_t i = 0; i < bus->motor_count; ++i)
    {
        if (!MotorMatchesPermit(bus, i, mask, now_ms))
        {
            return false;
        }
    }
    for (uint8_t i = 0; i < bus->motor_count; ++i)
    {
        bus->motors[i]->last_target_ms = now_ms;
    }
    bus->motion_allowed = true;
    bus->armed_motor_mask = mask;
    bus->last_command_ms = now_ms;
    return true;
}

/** @brief 显式开放全部已注册电机的运动许可。 */
bool EmmBusArm(EmmBusInstance *bus, uint32_t now_ms)
{
    return bus != NULL && bus->initialized && bus->motor_count <= EMM_MOTOR_CAPACITY &&
        ArmMask(bus, (uint8_t)((1U << bus->motor_count) - 1U), now_ms);
}

/** @brief 只开放指定轮的底层许可，不发送速度或改变其他轮使能。 */
bool EmmBusArmMotor(EmmBusInstance *bus, EmmMotorInstance *motor, uint32_t now_ms)
{
    if (bus == NULL || !bus->initialized || motor == NULL)
    {
        return false;
    }
    for (uint8_t i = 0; i < bus->motor_count; ++i)
    {
        if (bus->motors[i] == motor)
        {
            return ArmMask(bus, (uint8_t)(1U << i), now_ms);
        }
    }
    return false;
}

/** @brief 物理链路故障时全体反馈失效，停止失能并隔离迟到应答。 */
void EmmBusTransportFault(EmmBusInstance *bus, uint32_t now_ms)
{
    if (bus == NULL || !bus->initialized)
    {
        return;
    }
    ++bus->transport_error_count;
    for (uint8_t i = 0; i < bus->motor_count; ++i)
    {
        EmmMotorInvalidate(bus->motors[i]);
    }
    EmmBusStop(bus, true, now_ms);
    bus->response_count = 0U;
    bus->quarantine_active = true;
    bus->quarantine_started_ms = now_ms;
}

/** @brief 查询偶发超时保留尚新鲜反馈；控制失败或明确拒绝仍停止。 */
static void Finish(EmmBusInstance *bus, uint32_t now_ms, bool success)
{
    if (!success)
    {
        bus->quarantine_active = true;
        bus->quarantine_started_ms = now_ms;
    }
    if (!success && bus->current.motor != NULL)
    {
        ++bus->current.motor->command_error_count;
        if (!IsQuery(bus->current.kind) || bus->response_failed)
        {
            EmmMotorInvalidate(bus->current.motor);
            EmmBusStop(bus, true, now_ms);
        }
    }
    bus->active = false;
    bus->response_count = 0U;
    bus->guard_started_ms = now_ms;
    bus->guard_duration_ms = success ? bus->config.quiet_ms : bus->config.quarantine_ms;
}

/** @brief 错帧时丢弃首字节，保留后缀继续同步。 */
static void DiscardFirstByte(EmmBusInstance *bus)
{
    --bus->response_count;
    memmove(bus->response, bus->response + 1, bus->response_count);
}

/** @brief 在任务内按当前事务的地址/功能码/长度校验并分发回复。 */
void EmmBusReceive(EmmBusInstance *bus, const uint8_t *data, size_t length, uint32_t now_ms)
{
    if (bus == NULL || !bus->initialized || data == NULL || length == 0U)
    {
        return;
    }
    bus->last_rx_ms = now_ms;
    for (size_t i = 0; i < length; ++i)
    {
        if (!bus->active || bus->response_received || bus->response_failed ||
            (uint32_t)(now_ms - bus->started_ms) >= bus->config.response_timeout_ms)
        {
            continue;
        }
        /* 广播安全帧由 1 号回执，实际四轮状态仍须逐地址查询确认。 */
        uint8_t address = IsSafety(bus->current.kind) ? 1U : bus->current.motor->config.address;
        if (bus->response_count == 0U && data[i] != address)
        {
            continue;
        }
        bus->response[bus->response_count++] = data[i];
        /* 按功能码选固定长度，不扫描 6B 截帧，因为位置数据中也可能包含 6B。
         * 通用错误 00 EE 始终为 4 字节，即使当前等待的是 8 字节位置回复。
         */
        while (bus->response_count != 0U)
        {
            if (bus->response[0] != address)
            {
                DiscardFirstByte(bus);
                continue;
            }
            if (bus->response_count < 2U)
            {
                break;
            }
            bool generic_error = bus->response[1] == 0U;
            if (!generic_error && bus->response[1] != bus->current.frame[1])
            {
                ++bus->protocol_error_count;
                DiscardFirstByte(bus);
                continue;
            }
            uint8_t expected = generic_error ? 4U : bus->current.response_length;
            if (bus->response_count < expected)
            {
                break;
            }
            if (bus->response[expected - 1U] != EMM_CHECK_BYTE ||
                (generic_error && bus->response[2] != 0xEEU) ||
                (!generic_error && bus->current.kind == EMM_QUERY_POSITION && bus->response[2] > 1U))
            {
                ++bus->protocol_error_count;
                DiscardFirstByte(bus);
                continue;
            }
            /* 驱动拒绝计数与帧格式错误分开，记录地址/功能码/状态供排错。 */
            if (generic_error || (!IsQuery(bus->current.kind) && bus->response[2] != 0x02U))
            {
                ++bus->rejected_command_count;
                bus->last_rejected_address = address;
                bus->last_rejected_function = bus->current.frame[1];
                bus->last_rejected_status = bus->response[2];
                bus->response_failed = true;
            }
            else
            {
                bus->response_received = true;
                if (bus->current.motor != NULL)
                {
                    EmmMotorResponse(bus->current.motor, bus->current.kind, bus->response, now_ms);
                }
            }
            break;
        }
    }
}

/** @brief 按反馈、目标期限及安全优先级推进总线，不阻塞等待回复。 */
void EmmBusUpdate(EmmBusInstance *bus, uint32_t now_ms)
{
    if (bus == NULL || !bus->initialized)
    {
        return;
    }
    /* 每次更新都检查所有轮的状态和各授权轮目标期限；任一异常全体停止。 */
    if (bus->motion_allowed)
    {
        for (uint8_t i = 0; i < bus->motor_count; ++i)
        {
            if (!MotorMatchesPermit(bus, i, bus->armed_motor_mask, now_ms) ||
                (EmmBusMotorIsArmed(bus, bus->motors[i]) &&
                 (uint32_t)(now_ms - bus->motors[i]->last_target_ms) >= bus->config.command_timeout_ms))
            {
                EmmBusStop(bus, true, now_ms);
                break;
            }
        }
    }
    bool busy = bus->config.tx_busy(bus->config.context);
    if (bus->active)
    {
        if (busy)
        {
            return; /* 物理发送超时由 BSP 的独立期限处理。 */
        }
        if (bus->stop_pending && !IsSafety(bus->current.kind))
        {
            bus->quarantine_active = true;
            bus->quarantine_started_ms = now_ms;
            Finish(bus, now_ms, true); /* 放弃旧运动事务和等待，停止优先。 */
        }
        else if (bus->response_failed)
        {
            Finish(bus, now_ms, false);
        }
        else if (bus->response_received)
        {
            Finish(bus, now_ms, true);
        }
        else if ((uint32_t)(now_ms - bus->started_ms) >= bus->config.response_timeout_ms)
        {
            ++bus->timeout_count;
            Finish(bus, now_ms, false);
        }
        else
        {
            return;
        }
    }
    if (busy)
    {
        return;
    }
    if (!bus->motion_allowed && (uint32_t)(now_ms - bus->stop_repeat_ms) >= 250U)
    {
        /* V5 对已经失能的电机重复发送 FE 可能返回 E2。只有每轮都有新鲜的
         * 失能反馈，才能省略周期重复停止；未知、过期或仍使能时仍要发送。
         * 此优化绝不清除已经挂起的显式停止或故障停止请求。
         */
        if (!AllMotorsDisabled(bus, now_ms))
        {
            bus->stop_pending = true;
        }
        bus->stop_repeat_ms = now_ms;
    }
    /* 调度优先级：广播停止 > 广播失能 > 普通 FIFO；隔离期阻止旧回复误配。 */
    EmmTransaction transaction = {0};
    if (bus->stop_pending)
    {
        transaction = (EmmTransaction){
            .kind = EMM_STOP_ALL, .length = 5U, .response_length = 4U,
            .frame = {0U, 0xFEU, 0x98U, 0U, EMM_CHECK_BYTE},
        };
    }
    else
    {
        if (bus->quarantine_active &&
            (uint32_t)(now_ms - bus->quarantine_started_ms) >= bus->config.quarantine_ms)
        {
            bus->quarantine_active = false;
        }
        if ((uint32_t)(now_ms - bus->last_rx_ms) < bus->config.quiet_ms ||
            (uint32_t)(now_ms - bus->guard_started_ms) < bus->guard_duration_ms ||
            (bus->quarantine_active && !bus->disable_pending))
        {
            return;
        }
        if (bus->disable_pending)
        {
            transaction = (EmmTransaction){
                .kind = EMM_DISABLE_ALL, .length = 6U, .response_length = 4U,
                .frame = {0U, 0xF3U, 0xABU, 0U, 0U, EMM_CHECK_BYTE},
            };
        }
        else if (bus->queue_count != 0U)
        {
            transaction = bus->queue[0];
        }
        else
        {
            return;
        }
    }
    /* 只有确认旧发送不忙后才替换 current，保持异步发送数据的生命周期。 */
    bus->current = transaction;
    bus->response_count = 0U;
    bus->response_received = false;
    bus->response_failed = false;
    bus->started_ms = now_ms;
    if (!bus->config.send(bus->config.context, bus->current.frame, transaction.length, now_ms))
    {
        EmmBusTransportFault(bus, now_ms);
        return;
    }
    bus->active = true;
    if (transaction.kind == EMM_STOP_ALL)
    {
        bus->stop_pending = false;
    }
    else if (transaction.kind == EMM_DISABLE_ALL)
    {
        bus->disable_pending = false;
    }
    else
    {
        --bus->queue_count;
        memmove(bus->queue, bus->queue + 1, bus->queue_count * sizeof(bus->queue[0]));
    }
}
