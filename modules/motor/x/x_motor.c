#include "x_motor.h"
#include <string.h>

#define X_POSITION 0x36U
#define X_STATUS 0x3AU
#define X_CHECK 0x6BU
#define X_ENABLE 0xF3U
#define X_TARGET 0xFDU
#define X_STOP 0xFEU

static bool IsQuery(uint8_t function)
{
    return function == X_POSITION || function == X_STATUS;
}

bool XMotorProfileValid(const XMotorMotionProfile *profile)
{
    return profile != NULL && profile->speed_deci_rpm > 0U &&
        profile->speed_deci_rpm <= 30000U && profile->acceleration_rpm_s > 0U &&
        profile->deceleration_rpm_s > 0U;
}

bool XMotorIsBusy(const XMotorInstance *motor)
{
    return motor != NULL && motor->initialized &&
        (motor->motion.goal_active || motor->motion.stop_pending);
}

void XMotorStop(XMotorInstance *motor, XMotorFault fault)
{
    if (motor == NULL || !motor->initialized) return;
    if (!motor->motion.stop_pending || motor->motion.disable_requested) motor->stop_probe = false;
    motor->motion.disable_requested = false;
    motor->disable_probe = false;
    motor->motion.goal_active = false;
    motor->motion.command_accepted = false;
    motor->motion.stop_pending = true;
    motor->motion.state = X_MOTION_STOPPING;
    if (fault != X_FAULT_NONE || motor->motion.fault == X_FAULT_NONE)
        motor->motion.fault = fault;
    motor->enable_sent = false;
}

void XMotorDisable(XMotorInstance *motor)
{
    if (motor == NULL || !motor->initialized) return;
    /* 重复请求不能打断已经在等新状态的失能事务。 */
    if (motor->motion.disable_requested && motor->motion.stop_pending) return;
    motor->motion.goal_active = false;
    motor->motion.command_accepted = false;
    motor->motion.stop_pending = true;
    motor->motion.disable_requested = true;
    motor->motion.state = X_MOTION_DISABLING;
    motor->motion.fault = X_FAULT_NONE;
    motor->enable_sent = false;
    motor->stop_probe = false;
    motor->disable_probe = false;
}

bool XMotorSetTarget(XMotorInstance *motor, int64_t target_units,
                     const XMotorMotionProfile *profile, uint32_t sequence, uint32_t now_ms)
{
    if (motor == NULL || !motor->initialized) return false;
    XMotorFeedback feedback;
    (void)XMotorGetFeedback(motor, now_ms, &feedback);
    if (!XMotorProfileValid(profile) || target_units < -4294967295LL ||
        target_units > 4294967295LL || motor->motion.stop_pending ||
        !feedback.position_valid || !feedback.status_valid ||
        feedback.stalled || feedback.stall_protection)
    {
        ++motor->motion.rejected_commands;
        return false;
    }
    motor->motion.sequence = sequence;
    if (motor->motion.goal_active && motor->motion.target_units == target_units) return true;
    motor->motion.target_units = target_units;
    motor->motion.profile = *profile;
    ++motor->motion.generation;
    /* 换目标不能重发已ACK但还在等状态确认的使能事务。 */
    if (!motor->motion.goal_active) motor->enable_sent = false;
    motor->motion.goal_active = true;
    motor->motion.command_accepted = false;
    motor->motion.state = X_MOTION_PENDING;
    motor->motion.fault = X_FAULT_NONE;
    motor->motion.disable_requested = false;
    motor->disable_probe = false;
    return true;
}

bool XMotorGetMotion(const XMotorInstance *motor, uint32_t now_ms, XMotorMotionStatus *motion)
{
    if (motor == NULL || !motor->initialized || motion == NULL) return false;
    *motion = motor->motion;
    XMotorFeedback feedback;
    (void)XMotorGetFeedback(motor, now_ms, &feedback);
    if (motion->state == X_MOTION_REACHED &&
        (!feedback.position_valid || !feedback.status_valid)) motion->state = X_MOTION_MOVING;
    return true;
}

bool XMotorInit(XMotorInstance *motor, const XMotor_Init_Config_s *config, uint32_t now_ms)
{
    if (motor == NULL || config == NULL || config->send == NULL || config->tx_busy == NULL ||
        config->address == 0U || config->address > 247U ||
        config->query_period_ms == 0U || config->query_period_ms > 60000U ||
        config->response_timeout_ms == 0U || config->response_timeout_ms > 1000U ||
        config->quiet_ms == 0U || config->quiet_ms > 1000U ||
        config->quarantine_ms < config->response_timeout_ms || config->quarantine_ms > 60000U ||
        config->feedback_timeout_ms == 0U || config->feedback_timeout_ms > 60000U)
        return false;

    *motor = (XMotorInstance){
        .config = *config,
        .next_function = X_POSITION,
        .last_query_ms = now_ms - config->query_period_ms,
        .last_rx_ms = now_ms,
        .guard_started_ms = now_ms,
        .guard_duration_ms = config->quiet_ms,
        .initialized = true,
    };
    return true;
}

bool XMotorGetFeedback(const XMotorInstance *motor, uint32_t now_ms, XMotorFeedback *feedback)
{
    if (motor == NULL || !motor->initialized || feedback == NULL) return false;
    *feedback = motor->feedback;
    feedback->position_valid &= (uint32_t)(now_ms - feedback->position_ms) < motor->config.feedback_timeout_ms;
    feedback->status_valid &= (uint32_t)(now_ms - feedback->status_ms) < motor->config.feedback_timeout_ms;
    return true;
}

void XMotorTransportFault(XMotorInstance *motor, uint32_t now_ms)
{
    if (motor == NULL || !motor->initialized) return;
    ++motor->diagnostics.transport_errors;
    if (motor->motion.disable_requested && motor->motion.stop_pending)
    {
        motor->motion.fault = X_FAULT_TRANSPORT;
        motor->disable_probe = false;
    }
    else if (XMotorIsBusy(motor)) XMotorStop(motor, X_FAULT_TRANSPORT);
    motor->feedback.position_valid = false;
    motor->feedback.status_valid = false;
    motor->active = false;
    motor->response_count = 0U;
    motor->response_received = false;
    motor->response_failed = false;
    motor->guard_started_ms = now_ms;
    motor->guard_duration_ms = motor->config.quarantine_ms;
    /* request可能仍被DMA引用，在tx_busy=false之前不改写。 */
}

static void DiscardFirstByte(XMotorInstance *motor)
{
    --motor->response_count;
    memmove(motor->response, motor->response + 1, motor->response_count);
}

static void AcceptFeedback(XMotorInstance *motor, uint32_t now_ms)
{
    XMotorFeedback *feedback = &motor->feedback;
    const uint8_t *frame = motor->response;
    if (motor->request[1] == X_POSITION)
    {
        uint32_t magnitude = (uint32_t)frame[3] << 24 | (uint32_t)frame[4] << 16 |
                             (uint32_t)frame[5] << 8 | (uint32_t)frame[6];
        feedback->position_units = frame[2] == 0U ? (int64_t)magnitude : -(int64_t)magnitude;
        feedback->position_ms = now_ms;
        feedback->position_valid = true;
        ++motor->diagnostics.position_replies;
    }
    else
    {
        feedback->status_raw = frame[2];
        feedback->enabled = (frame[2] & 0x01U) != 0U;
        feedback->position_reached = (frame[2] & 0x02U) != 0U;
        feedback->stalled = (frame[2] & 0x04U) != 0U;
        feedback->stall_protection = (frame[2] & 0x08U) != 0U;
        feedback->left_limit_high = (frame[2] & 0x10U) != 0U;
        feedback->right_limit_high = (frame[2] & 0x20U) != 0U;
        feedback->power_loss = (frame[2] & 0x80U) != 0U;
        feedback->status_ms = now_ms;
        feedback->status_valid = true;
        ++motor->diagnostics.status_replies;
    }
}

/* Unsolicited FD/9F can arrive between query bytes/frames. It is a separate
 * fixed 4-byte frame and never completes a command or asserts target arrival. */
void XMotorReceive(XMotorInstance *motor, const uint8_t *data, size_t length, uint32_t now_ms)
{
    if (motor == NULL || !motor->initialized || data == NULL || length == 0U) return;
    motor->last_rx_ms = now_ms;
    motor->diagnostics.rx_bytes += (uint32_t)length;
    for (size_t i = 0U; i < length; ++i)
    {
        if (!motor->active || motor->response_received || motor->response_failed ||
            (uint32_t)(now_ms - motor->started_ms) >= motor->config.response_timeout_ms)
            continue;
        motor->response[motor->response_count++] = data[i];
        while (motor->response_count != 0U)
        {
            if (motor->response[0] != motor->config.address)
            { DiscardFirstByte(motor); continue; }
            if (motor->response_count < 2U) break;
            uint8_t function = motor->response[1];
            bool generic_error = function == 0U;
            bool async_candidate = function == X_TARGET;
            if (!generic_error && !async_candidate && function != motor->request[1])
            {
                ++motor->diagnostics.protocol_errors;
                DiscardFirstByte(motor);
                continue;
            }
            uint8_t expected = function == X_POSITION ? 8U : 4U;
            if (motor->response_count < expected) break;
            if (motor->response[expected - 1U] != X_CHECK ||
                (generic_error && motor->response[2] != 0xEEU) ||
                (function == X_POSITION && motor->response[2] > 1U))
            {
                ++motor->diagnostics.protocol_errors;
                DiscardFirstByte(motor);
                continue;
            }
            if (async_candidate && motor->response[2] == 0x9FU)
            {
                ++motor->diagnostics.asynchronous_arrivals;
                motor->response_count = 0U;
                break;
            }
            if (!generic_error && function != motor->request[1])
            { motor->response_count = 0U; break; }
            bool query = IsQuery(motor->request[1]);
            if (generic_error || (!query && motor->response[2] != 0x02U))
            {
                if (query) ++motor->diagnostics.rejected_queries;
                else ++motor->diagnostics.rejected_controls;
                motor->diagnostics.last_rejected_function = motor->request[1];
                motor->diagnostics.last_rejected_status = motor->response[2];
                if (motor->request[1] == X_POSITION) motor->feedback.position_valid = false;
                if (motor->request[1] == X_STATUS) motor->feedback.status_valid = false;
                motor->response_failed = true;
            }
            else
            {
                if (query) AcceptFeedback(motor, now_ms);
                motor->response_received = true;
            }
            break;
        }
    }
}

static void UpdateMotion(XMotorInstance *motor)
{
    XMotorMotionStatus *motion = &motor->motion;
    const XMotorFeedback *feedback = &motor->feedback;
    if (!motion->goal_active) return;
    if (!feedback->position_valid || !feedback->status_valid)
        XMotorStop(motor, X_FAULT_FEEDBACK_LOST);
    else if (feedback->stalled || feedback->stall_protection)
        XMotorStop(motor, X_FAULT_DRIVER);
    else if (motion->command_accepted)
    {
        if (!feedback->enabled) { XMotorStop(motor, X_FAULT_DISABLED); return; }
        int64_t error = motion->target_units - feedback->position_units;
        if (error < 0) error = -error;
        bool fresh = motor->diagnostics.position_replies != motor->accepted_position_replies &&
                     motor->diagnostics.status_replies != motor->accepted_status_replies;
        motion->state = fresh && feedback->position_reached &&
            (uint64_t)error <= motor->config.position_tolerance_units ?
            X_MOTION_REACHED : X_MOTION_MOVING;
    }
}

static void FinishControl(XMotorInstance *motor, bool success)
{
    uint8_t function = motor->request[1];
    if (function == X_ENABLE && motor->request[3] == 0U)
    {
        /* 即使ACK丢失或被拒绝，也先查询新状态，再决定是否重试F3。 */
        if (motor->motion.disable_requested) motor->disable_probe = true;
        return;
    }
    /* 失能覆盖了在途FD、使能或FE；它们的迟到结果不能恢复旧目标或取消失能。 */
    if (motor->motion.disable_requested) return;
    if (function == X_STOP)
    {
        if (success)
        {
            motor->motion.stop_pending = false;
            motor->motion.state = motor->motion.fault == X_FAULT_NONE ? X_MOTION_IDLE : X_MOTION_FAULT;
        }
        else motor->stop_probe = true;
        return; /* Failed stop remains pending, retries after quarantine. */
    }
    if (!success)
    {
        XMotorStop(motor, motor->response_failed ? X_FAULT_REJECTED : X_FAULT_COMMAND_TIMEOUT);
        return;
    }
    if (!motor->motion.goal_active) return;
    if (function == X_ENABLE)
    {
        motor->enable_sent = true;
        motor->enable_status_replies = motor->diagnostics.status_replies;
    }
    else if (function == X_TARGET && motor->request_generation == motor->motion.generation)
    {
        motor->motion.command_accepted = true;
        motor->motion.state = X_MOTION_MOVING;
        motor->accepted_position_replies = motor->diagnostics.position_replies;
        motor->accepted_status_replies = motor->diagnostics.status_replies;
    }
}

static void PutBE16(uint8_t *out, uint16_t value)
{ out[0] = (uint8_t)(value >> 8); out[1] = (uint8_t)value; }

void XMotorUpdate(XMotorInstance *motor, uint32_t now_ms)
{
    if (motor == NULL || !motor->initialized) return;
    (void)XMotorGetFeedback(motor, now_ms, &motor->feedback);
    UpdateMotion(motor);
    /* UART BSP owns physical DMA timeout; never modify its source while busy. */
    if (motor->config.tx_busy(motor->config.context)) return;
    if (motor->active)
    {
        bool timed_out = (uint32_t)(now_ms - motor->started_ms) >= motor->config.response_timeout_ms;
        if (!motor->response_received && !motor->response_failed && !timed_out) return;
        if (!motor->response_received && !motor->response_failed) ++motor->diagnostics.timeouts;
        if (IsQuery(motor->request[1]))
        {
            motor->next_function = motor->request[1] == X_POSITION ? X_STATUS : X_POSITION;
            if (motor->motion.stop_pending && motor->disable_probe && motor->request[1] == X_STATUS)
            {
                if (motor->response_received && !motor->feedback.enabled)
                {
                    motor->motion.stop_pending = false;
                    motor->motion.fault = X_FAULT_NONE;
                    motor->motion.state = X_MOTION_IDLE;
                }
                motor->disable_probe = false;
            }
            else if (motor->motion.stop_pending && motor->stop_probe && motor->request[1] == X_STATUS)
            {
                if (motor->response_received && !motor->feedback.enabled)
                {
                    motor->motion.stop_pending = false;
                    motor->motion.state = motor->motion.fault == X_FAULT_NONE ? X_MOTION_IDLE : X_MOTION_FAULT;
                }
                motor->stop_probe = false;
            }
        }
        else FinishControl(motor, motor->response_received);
        motor->guard_started_ms = now_ms;
        motor->guard_duration_ms = motor->response_received ? motor->config.quiet_ms : motor->config.quarantine_ms;
        motor->active = false;
        motor->response_count = 0U;
    }
    if ((uint32_t)(now_ms - motor->last_rx_ms) < motor->config.quiet_ms ||
        (uint32_t)(now_ms - motor->guard_started_ms) < motor->guard_duration_ms) return;

    uint8_t function = motor->next_function;
    bool pending = motor->motion.goal_active && !motor->motion.command_accepted;
    if (motor->motion.stop_pending)
        function = motor->motion.disable_requested ?
            (motor->disable_probe ? X_STATUS : X_ENABLE) : (motor->stop_probe ? X_STATUS : X_STOP);
    else if (pending)
    {
        if (motor->feedback.enabled) function = X_TARGET;
        else if (!motor->enable_sent) function = X_ENABLE;
        else if (motor->diagnostics.status_replies != motor->enable_status_replies)
        { XMotorStop(motor, X_FAULT_DISABLED); function = X_STOP; }
        /* Otherwise query until enable is evidenced by fresh status. */
    }
    bool query = IsQuery(function);
    if (query && (uint32_t)(now_ms - motor->last_query_ms) < motor->config.query_period_ms) return;
    uint8_t *frame = motor->request;
    frame[0] = motor->config.address;
    frame[1] = function;
    size_t length = 3U;
    if (function == X_TARGET)
    {
        const XMotorMotionStatus *motion = &motor->motion;
        uint32_t magnitude = (uint32_t)(motion->target_units < 0 ? -motion->target_units : motion->target_units);
        frame[2] = motion->target_units < 0 ? 1U : 0U;
        PutBE16(frame + 3, motion->profile.acceleration_rpm_s);
        PutBE16(frame + 5, motion->profile.deceleration_rpm_s);
        PutBE16(frame + 7, motion->profile.speed_deci_rpm);
        for (size_t i = 0U; i < 4U; ++i) frame[9U + i] = (uint8_t)(magnitude >> (24U - 8U * i));
        frame[13] = 1U; /* Absolute coordinate, no modulo or relative delta. */
        frame[14] = 0U; /* Independent UARTs, no multi-motor sync. */
        length = 16U;
    }
    else if (function == X_ENABLE)
    { frame[2] = 0xABU; frame[3] = motor->motion.disable_requested ? 0U : 1U;
      frame[4] = 0U; length = 6U; }
    else if (function == X_STOP)
    { frame[2] = 0x98U; frame[3] = 0U; length = 5U; }
    frame[length - 1U] = X_CHECK;
    motor->response_count = 0U;
    motor->response_received = false;
    motor->response_failed = false;
    motor->started_ms = now_ms;
    motor->request_generation = motor->motion.generation;
    if (query) motor->last_query_ms = now_ms;
    if (!motor->config.send(motor->config.context, frame, length, now_ms))
    { XMotorTransportFault(motor, now_ms); return; }
    motor->active = true;
    if (query) ++motor->diagnostics.queries_sent;
    else ++motor->diagnostics.controls_sent;
}
