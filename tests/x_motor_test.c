/* 离线回归；真实X解析器、板级组装和UART BSP，HAL只模拟缓冲与回调。 */
#include "x_motor.h"
#include "board_crane.h"
#include "bsp_uart.h"
#include <string.h>

#define CHECK(c) do { if (!(c)) return __LINE__; } while (0)

typedef struct
{
    const uint8_t *pending;
    uint32_t sent;
    bool busy;
    bool fail_send;
    bool control_seen;
    size_t length;
} TestLink;

static bool Send(void *context, const uint8_t *data, size_t length, uint32_t now_ms)
{
    (void)now_ms;
    TestLink *link = context;
    if (length != 3U || data[0] != 1U || data[2] != 0x6BU ||
        (data[1] != 0x36U && data[1] != 0x3AU)) link->control_seen = true;
    if (link->fail_send) return false;
    link->pending = data;
    link->length = length;
    link->busy = true;
    ++link->sent;
    return true;
}

static bool Busy(void *context) { return ((TestLink *)context)->busy; }

static bool Setup(XMotorInstance *motor, TestLink *link, uint32_t now_ms)
{
    *link = (TestLink){0};
    const XMotor_Init_Config_s config = {
        .send = Send, .tx_busy = Busy, .context = link, .address = 1U,
        .query_period_ms = 25U, .response_timeout_ms = 20U,
        .quiet_ms = 2U, .quarantine_ms = 40U, .feedback_timeout_ms = 500U,
    };
    return XMotorInit(motor, &config, now_ms);
}

static void PositionFrame(uint8_t frame[8], int64_t value)
{
    uint32_t magnitude = (uint32_t)(value < 0 ? -value : value);
    const uint8_t reply[8] = {1U, 0x36U, value < 0 ? 1U : 0U,
        (uint8_t)(magnitude >> 24), (uint8_t)(magnitude >> 16),
        (uint8_t)(magnitude >> 8), (uint8_t)magnitude, 0x6BU};
    memcpy(frame, reply, 8U);
}

static int TestProtocol(void)
{
    XMotorInstance motor;
    TestLink link;
    CHECK(Setup(&motor, &link, 0U));
    XMotor_Init_Config_s invalid = motor.config;
    invalid.address = 0U;
    CHECK(!XMotorInit(&motor, &invalid, 0U));
    invalid = motor.config;
    invalid.quarantine_ms = 1U;
    CHECK(!XMotorInit(&motor, &invalid, 0U));
    CHECK(!XMotorGetFeedback(NULL, 0U, &motor.feedback));

    const int64_t values[] = {3600, 72000, -180000, 0x006B016B, 4294967295LL, -4294967295LL};
    for (size_t i = 0U; i < sizeof(values) / sizeof(values[0]); ++i)
    {
        /* 同时覆盖时间回绕、逐字节分片和载荷含6B。 */
        uint32_t base = i % 2U == 0U ? 0U : 0xFFFFFFF0U;
        CHECK(Setup(&motor, &link, base));
        XMotorFeedback feedback;
        CHECK(XMotorGetFeedback(&motor, base, &feedback));
        CHECK(!feedback.position_valid && !feedback.status_valid && link.sent == 0U);
        XMotorUpdate(&motor, base + 1U);
        CHECK(link.sent == 0U);
        XMotorUpdate(&motor, base + 2U);
        CHECK(link.sent == 1U && link.pending[1] == 0x36U);
        uint8_t frame[8];
        PositionFrame(frame, values[i]);
        for (size_t byte = 0U; byte < sizeof(frame); ++byte)
            XMotorReceive(&motor, &frame[byte], 1U, base + 3U);
        CHECK(motor.feedback.position_units == values[i]);
        XMotorUpdate(&motor, base + 27U); /* 应答已到，但DMA仍busy，不能覆盖发送。 */
        CHECK(link.sent == 1U && link.pending[1] == 0x36U);
        link.busy = false;
        XMotorUpdate(&motor, base + 28U);
        XMotorUpdate(&motor, base + 30U);
        CHECK(link.sent == 2U && link.pending[1] == 0x3AU);
        const uint8_t status[] = {1U, 0x3AU, 0xFFU, 0x6BU};
        XMotorReceive(&motor, status, sizeof(status), base + 31U);
        link.busy = false;
        XMotorUpdate(&motor, base + 32U);
        CHECK(XMotorGetFeedback(&motor, base + 502U, &feedback));
        CHECK(feedback.position_valid && feedback.status_valid);
        CHECK(feedback.enabled && feedback.position_reached && feedback.stalled && feedback.stall_protection);
        CHECK(feedback.left_limit_high && feedback.right_limit_high && feedback.power_loss);
        CHECK(feedback.status_raw == 0xFFU);
        CHECK(XMotorGetFeedback(&motor, base + 503U, &feedback));
        CHECK(!feedback.position_valid && feedback.status_valid);
        CHECK(feedback.position_units == values[i]);
        CHECK(XMotorGetFeedback(&motor, base + 531U, &feedback));
        CHECK(!feedback.position_valid && !feedback.status_valid && !link.control_seen);
    }

    CHECK(Setup(&motor, &link, 0U));
    XMotorUpdate(&motor, 2U);
    link.busy = false;
    const uint8_t noise[] = {0x99U, 2U, 0x36U, 0U, 0U, 0U, 0U, 0U, 0x6BU,
                            1U, 0x3AU, 0x83U, 0x6BU};
    XMotorReceive(&motor, noise, sizeof(noise), 3U);
    CHECK(!motor.feedback.position_valid && motor.diagnostics.protocol_errors != 0U);
    const uint8_t bad_sign[] = {1U, 0x36U, 2U, 0U, 0U, 0U, 0U, 0x6BU};
    const uint8_t bad_tail[] = {1U, 0x36U, 0U, 0U, 0U, 0U, 0U, 0xAAU};
    XMotorReceive(&motor, bad_sign, sizeof(bad_sign), 3U);
    XMotorReceive(&motor, bad_tail, sizeof(bad_tail), 3U);
    CHECK(!motor.feedback.position_valid);
    uint8_t good[8];
    PositionFrame(good, -72000);
    XMotorReceive(&motor, good, sizeof(good), 4U);
    CHECK(motor.feedback.position_valid && motor.feedback.position_units == -72000);
    XMotorReceive(&motor, good, sizeof(good), 4U); /* 重复应答不能重复采样。 */
    CHECK(motor.diagnostics.position_replies == 1U);
    XMotorUpdate(&motor, 5U);
    XMotorUpdate(&motor, 27U);
    const uint8_t status_e2[] = {1U, 0x3AU, 0xE2U, 0x6BU};
    XMotorReceive(&motor, status_e2, sizeof(status_e2), 28U);
    CHECK(motor.feedback.status_valid && motor.feedback.status_raw == 0xE2U);
    CHECK(motor.diagnostics.rejected_queries == 0U); /* 状态载荷E2不是命令拒绝。 */

    CHECK(Setup(&motor, &link, 0U));
    XMotorUpdate(&motor, 2U);
    link.busy = false;
    XMotorUpdate(&motor, 22U);
    CHECK(motor.diagnostics.timeouts == 1U);
    XMotorReceive(&motor, good, sizeof(good), 23U); /* 迟到应答不得刷新有效性。 */
    CHECK(!motor.feedback.position_valid);
    XMotorUpdate(&motor, 61U);
    CHECK(link.sent == 1U);
    XMotorUpdate(&motor, 62U);
    CHECK(link.sent == 2U && link.pending[1] == 0x3AU);
    const uint8_t rejected[] = {1U, 0U, 0xEEU, 0x6BU};
    XMotorReceive(&motor, rejected, sizeof(rejected), 63U);
    link.busy = false;
    XMotorUpdate(&motor, 64U);
    CHECK(motor.diagnostics.rejected_queries == 1U && !motor.feedback.status_valid);
    CHECK(motor.diagnostics.last_rejected_function == 0x3AU);
    CHECK(motor.diagnostics.last_rejected_status == 0xEEU);
    XMotorUpdate(&motor, 104U);
    CHECK(link.pending[1] == 0x36U);
    XMotorReceive(&motor, good, sizeof(good), 124U); /* 恰好截止时也拒绝。 */
    CHECK(!motor.feedback.position_valid);
    link.busy = false;
    XMotorUpdate(&motor, 124U);
    CHECK(motor.diagnostics.timeouts == 2U);

    CHECK(Setup(&motor, &link, 0U));
    link.fail_send = true;
    XMotorUpdate(&motor, 2U);
    CHECK(motor.diagnostics.transport_errors == 1U && link.sent == 0U);
    link.fail_send = false;
    XMotorUpdate(&motor, 41U);
    CHECK(link.sent == 0U);
    XMotorUpdate(&motor, 42U);
    CHECK(link.sent == 1U);
    XMotorReceive(&motor, good, sizeof(good), 43U);
    CHECK(motor.feedback.position_valid);
    XMotorTransportFault(&motor, 44U);
    CHECK(!motor.feedback.position_valid && !motor.feedback.status_valid);
    XMotorUpdate(&motor, 100U);
    CHECK(link.sent == 1U && link.pending[1] == 0x36U); /* DMA仍busy。 */
    link.busy = false;
    XMotorUpdate(&motor, 101U);
    CHECK(link.sent == 2U && !link.control_seen);
    return 0;
}

typedef struct
{
    XMotorInstance motor;
    TestLink link;
    uint32_t now;
    int64_t actual;
    uint8_t flags;
    uint8_t drop_function;
    uint8_t reject_function;
    uint8_t last_target[16];
    uint32_t target_sends, enable_sends, disable_sends, stop_sends;
    uint8_t last_disable[6];
    bool ignore_disable;
    bool apply_dropped_disable;
} MotionFixture;

static void MotionStep(MotionFixture *f)
{
    ++f->now;
    if (f->link.busy)
    {
        uint8_t function = f->link.pending[1];
        uint8_t reply[8] = {1U, function, 2U, 0x6BU};
        size_t size = 4U;
        if (function == 0x36U) { PositionFrame(reply, f->actual); size = 8U; }
        else if (function == 0x3AU) reply[2] = f->flags;
        else if (function == 0xFDU)
        { ++f->target_sends; memcpy(f->last_target, f->link.pending, 16U); }
        else if (function == 0xF3U)
        {
            if (f->link.pending[3] != 0U) ++f->enable_sends;
            else { ++f->disable_sends; memcpy(f->last_disable, f->link.pending, 6U); }
        }
        else if (function == 0xFEU) ++f->stop_sends;
        if (function == f->reject_function) reply[2] = 0xE2U;
        else if (function == 0xF3U)
        {
            if (f->link.pending[3] != 0U && function != f->drop_function) f->flags |= 1U;
            else if (f->link.pending[3] == 0U && !f->ignore_disable &&
                     (function != f->drop_function || f->apply_dropped_disable)) f->flags &= (uint8_t)~1U;
        }
        f->link.busy = false;
        if (function != f->drop_function)
            XMotorReceive(&f->motor, reply, size, f->now);
    }
    XMotorUpdate(&f->motor, f->now);
}

static void MotionRun(MotionFixture *f, uint32_t duration)
{ for (uint32_t i = 0U; i < duration; ++i) MotionStep(f); }

static bool MotionSetup(MotionFixture *f, uint8_t flags)
{
    *f = (MotionFixture){.flags = flags};
    if (!Setup(&f->motor, &f->link, 0U)) return false;
    f->motor.config.position_tolerance_units = 10U;
    MotionRun(f, 32U);
    return f->motor.feedback.position_valid && f->motor.feedback.status_valid && !f->link.control_seen;
}

static int TestDisable(void)
{
    MotionFixture f;
    XMotorMotionProfile profile = {100U, 100U, 100U};
    CHECK(MotionSetup(&f, 3U));
    CHECK(XMotorSetTarget(&f.motor, 7200, &profile, 1U, f.now));
    XMotorDisable(&f.motor);
    CHECK(!f.motor.motion.goal_active && !f.motor.motion.command_accepted && XMotorIsBusy(&f.motor));
    CHECK(f.motor.motion.state == X_MOTION_DISABLING);
    CHECK(!XMotorSetTarget(&f.motor, 0, &profile, 2U, f.now));
    while (f.disable_sends == 0U && f.now < 150U) MotionStep(&f);
    CHECK(f.disable_sends == 1U && f.motor.motion.stop_pending && f.motor.disable_probe);
    const uint8_t golden[] = {1U, 0xF3U, 0xABU, 0U, 0U, 0x6BU};
    CHECK(!memcmp(f.last_disable, golden, sizeof(golden)));
    XMotorDisable(&f.motor); /* 重复请求不得丢掉ACK后的新状态确认。 */
    CHECK(f.motor.disable_probe);
    MotionRun(&f, 100U);
    CHECK(!XMotorIsBusy(&f.motor) && f.motor.motion.state == X_MOTION_IDLE);
    CHECK(!f.motor.feedback.enabled && f.motor.feedback.status_valid && f.disable_sends == 1U);
    CHECK(f.target_sends == 0U && f.enable_sends == 0U && f.stop_sends == 0U);
    f.actual = -180000;
    MotionRun(&f, 100U);
    CHECK(f.motor.feedback.position_units == -180000 && f.motor.feedback.position_valid);
    CHECK(f.disable_sends == 1U && f.enable_sends == 0U);
    CHECK(XMotorSetTarget(&f.motor, 7200, &profile, 3U, f.now));
    MotionRun(&f, 100U);
    CHECK(f.enable_sends == 1U && f.target_sends == 1U && !f.motor.motion.disable_requested);

    /* 取消DMA仍在途的FD、F3使能、FE；其结果均不得撤销新失能意图。 */
    for (unsigned kind = 0U; kind < 3U; ++kind)
    {
        CHECK(MotionSetup(&f, kind == 1U ? 2U : 3U));
        if (kind == 2U) XMotorStop(&f.motor, X_FAULT_NONE);
        else CHECK(XMotorSetTarget(&f.motor, 3600, &profile, 1U, f.now));
        MotionRun(&f, 1U);
        CHECK(f.link.busy);
        uint8_t old[16]; size_t length = f.link.length;
        memcpy(old, f.link.pending, length);
        XMotorDisable(&f.motor);
        CHECK(!memcmp(old, f.link.pending, length));
        MotionRun(&f, 140U);
        CHECK(!XMotorIsBusy(&f.motor) && !f.motor.feedback.enabled && f.disable_sends == 1U);
        CHECK(!f.motor.motion.goal_active && f.target_sends == (kind == 0U ? 1U : 0U));
    }

    /* ACK丢失但实际已失能，应由新状态确认；无失能证据则持续待确认并重试。 */
    CHECK(MotionSetup(&f, 3U));
    f.drop_function = 0xF3U; f.apply_dropped_disable = true;
    XMotorDisable(&f.motor); MotionRun(&f, 180U);
    CHECK(!XMotorIsBusy(&f.motor) && f.disable_sends == 1U && !f.motor.feedback.enabled);
    CHECK(MotionSetup(&f, 3U));
    f.reject_function = 0xF3U;
    XMotorDisable(&f.motor); MotionRun(&f, 200U);
    CHECK(XMotorIsBusy(&f.motor) && f.disable_sends > 1U && f.motor.feedback.enabled);
    CHECK(!XMotorSetTarget(&f.motor, 3600, &profile, 1U, f.now));
    f.reject_function = 0U; MotionRun(&f, 100U);
    CHECK(!XMotorIsBusy(&f.motor) && !f.motor.feedback.enabled && f.target_sends == 0U);
    CHECK(MotionSetup(&f, 2U)); /* 发送前已有失能状态也不能提前确认。 */
    f.drop_function = 0x3AU;
    XMotorDisable(&f.motor); MotionRun(&f, 180U);
    CHECK(f.motor.motion.stop_pending && f.disable_sends > 1U);

    CHECK(MotionSetup(&f, 3U));
    f.link.fail_send = true;
    XMotorDisable(&f.motor); MotionRun(&f, 80U);
    CHECK(f.motor.motion.disable_requested && f.motor.motion.stop_pending);
    f.link.fail_send = false; MotionRun(&f, 160U);
    CHECK(!XMotorIsBusy(&f.motor) && !f.motor.feedback.enabled);
    CHECK(MotionSetup(&f, 3U));
    f.ignore_disable = true;
    XMotorDisable(&f.motor); MotionRun(&f, 30U);
    XMotorStop(&f.motor, X_FAULT_LINK); MotionRun(&f, 120U);
    CHECK(!f.motor.motion.disable_requested && !XMotorIsBusy(&f.motor) && f.stop_sends != 0U);
    return 0;
}

static int TestMotion(void)
{
    MotionFixture f;
    XMotorMotionProfile profile = {1000U, 100U, 100U};
    CHECK(MotionSetup(&f, 0x83U)); /* bit7 is not a fault. */
    CHECK(XMotorSetTarget(&f.motor, 7200, &profile, 9U, f.now));
    uint32_t generation = f.motor.motion.generation;
    MotionRun(&f, 10U);
    const uint8_t golden[] = {1U, 0xFDU, 0U, 0U, 100U, 0U, 100U, 3U, 0xE8U,
        0U, 0U, 0x1CU, 0x20U, 1U, 0U, 0x6BU};
    CHECK(f.target_sends == 1U && !memcmp(f.last_target, golden, sizeof(golden)));
    CHECK(f.motor.motion.command_accepted && f.motor.motion.state == X_MOTION_MOVING);
    CHECK(XMotorSetTarget(&f.motor, 7200, &profile, 10U, f.now));
    MotionRun(&f, 55U);
    CHECK(f.target_sends == 1U && f.motor.motion.generation == generation && f.motor.motion.sequence == 10U);
    CHECK(f.motor.motion.state != X_MOTION_REACHED); /* old reached bit, wrong actual. */
    f.actual = 7200;
    MotionRun(&f, 55U);
    CHECK(f.motor.motion.state == X_MOTION_REACHED && f.motor.motion.goal_active);
    f.actual = 7211;
    MotionRun(&f, 55U);
    CHECK(f.motor.motion.state == X_MOTION_MOVING);
    f.actual = 7200;
    MotionRun(&f, 55U);
    CHECK(f.motor.motion.state == X_MOTION_REACHED && f.enable_sends == 0U);
    XMotorMotionStatus snapshot;
    CHECK(XMotorGetMotion(&f.motor, f.now + 500U, &snapshot) && snapshot.state != X_MOTION_REACHED);

    CHECK(!XMotorSetTarget(&f.motor, INT64_MIN, &profile, 1U, f.now));
    CHECK(!XMotorSetTarget(&f.motor, 4294967296LL, &profile, 1U, f.now));
    XMotorMotionProfile bad = {30001U, 100U, 100U};
    CHECK(!XMotorSetTarget(&f.motor, 0, &bad, 1U, f.now));
    bad = (XMotorMotionProfile){100U, 0U, 100U};
    CHECK(!XMotorSetTarget(&f.motor, 0, &bad, 1U, f.now));
    CHECK(f.motor.motion.target_units == 7200 && f.motor.motion.generation == generation);

    /* New absolute signed target replaces an in-flight one without editing DMA. */
    CHECK(MotionSetup(&f, 3U));
    CHECK(XMotorSetTarget(&f.motor, 1000, &profile, 1U, f.now));
    MotionStep(&f);
    CHECK(f.link.busy && f.link.pending[1] == 0xFDU);
    uint8_t old_dma[16]; memcpy(old_dma, f.link.pending, 16U);
    CHECK(XMotorSetTarget(&f.motor, -4294967295LL, &profile, 2U, f.now));
    CHECK(!memcmp(old_dma, f.link.pending, 16U));
    MotionStep(&f); /* ACK of older generation must not mark latest as accepted. */
    CHECK(!f.motor.motion.command_accepted);
    MotionRun(&f, 10U);
    CHECK(f.target_sends == 2U && f.motor.motion.command_accepted);
    CHECK(f.last_target[2] == 1U && f.last_target[9] == 255U && f.last_target[12] == 255U);
    CHECK(f.motor.motion.target_units == -4294967295LL);

    /* Positive smaller absolute target still has sign=0, regardless of travel direction. */
    CHECK(XMotorSetTarget(&f.motor, 1, &profile, 3U, f.now));
    MotionRun(&f, 10U);
    CHECK(f.last_target[2] == 0U && f.last_target[12] == 1U);
    XMotorStop(&f.motor, X_FAULT_NONE);
    CHECK(!XMotorSetTarget(&f.motor, 5000, &profile, 4U, f.now));
    MotionRun(&f, 10U);
    CHECK(!f.motor.motion.goal_active && !f.motor.motion.stop_pending && f.motor.motion.state == X_MOTION_IDLE);
    uint32_t sends = f.target_sends;
    MotionRun(&f, 60U);
    CHECK(f.target_sends == sends && f.stop_sends == 1U);
    CHECK(XMotorSetTarget(&f.motor, 1, &profile, 5U, f.now));
    MotionRun(&f, 10U);
    CHECK(f.target_sends == sends + 1U); /* same coordinate after STOP is a new goal. */

    CHECK(MotionSetup(&f, 2U)); /* Disabled: automatic F3, fresh enabled state, FD. */
    CHECK(XMotorSetTarget(&f.motor, -180000, &profile, 1U, f.now));
    MotionRun(&f, 80U);
    CHECK(f.enable_sends == 1U && f.target_sends == 1U && f.motor.motion.command_accepted);
    CHECK(f.last_target[2] == 1U && f.last_target[10] == 2U && f.last_target[11] == 0xBFU && f.last_target[12] == 0x20U);
    /* Enabled drop during motion cancels goal, does not automatically enable again. */
    f.flags &= (uint8_t)~1U;
    MotionRun(&f, 70U);
    CHECK(!f.motor.motion.goal_active && f.motor.motion.fault == X_FAULT_DISABLED && f.enable_sends == 1U);

    CHECK(MotionSetup(&f, 2U));
    CHECK(XMotorSetTarget(&f.motor, 100, &profile, 1U, f.now));
    MotionRun(&f, 3U); /* F3 accepted, waiting for a new status sample. */
    CHECK(f.motor.enable_sent && !f.motor.feedback.enabled);
    CHECK(XMotorSetTarget(&f.motor, 200, &profile, 2U, f.now));
    MotionRun(&f, 80U);
    CHECK(f.enable_sends == 1U && f.target_sends == 1U && f.motor.motion.target_units == 200);

    CHECK(MotionSetup(&f, 2U));
    f.reject_function = 0xF3U;
    CHECK(XMotorSetTarget(&f.motor, 100, &profile, 1U, f.now));
    MotionRun(&f, 80U);
    CHECK(f.enable_sends == 1U && f.target_sends == 0U && f.stop_sends == 1U);
    CHECK(f.motor.motion.fault == X_FAULT_REJECTED && !f.motor.motion.goal_active);

    CHECK(MotionSetup(&f, 3U));
    f.reject_function = 0xFDU;
    CHECK(XMotorSetTarget(&f.motor, 100, &profile, 1U, f.now));
    MotionRun(&f, 80U);
    CHECK(f.target_sends == 1U && f.stop_sends == 1U && f.motor.motion.state == X_MOTION_FAULT);
    CHECK(f.motor.motion.fault == X_FAULT_REJECTED && f.motor.diagnostics.rejected_controls == 1U);
    f.reject_function = 0U;
    CHECK(XMotorSetTarget(&f.motor, 100, &profile, 2U, f.now));
    MotionRun(&f, 10U);
    CHECK(f.target_sends == 2U);

    CHECK(MotionSetup(&f, 3U));
    f.drop_function = 0xFDU;
    CHECK(XMotorSetTarget(&f.motor, 100, &profile, 1U, f.now));
    MotionRun(&f, 100U);
    CHECK(f.target_sends == 1U && f.stop_sends == 1U && f.motor.motion.fault == X_FAULT_COMMAND_TIMEOUT);
    CHECK(!f.motor.motion.goal_active);

    CHECK(MotionSetup(&f, 3U));
    CHECK(XMotorSetTarget(&f.motor, 100, &profile, 1U, f.now));
    MotionRun(&f, 10U);
    f.motor.feedback.status_ms = f.now - 500U;
    MotionStep(&f);
    CHECK(!f.motor.motion.goal_active && f.motor.motion.fault == X_FAULT_FEEDBACK_LOST);
    MotionRun(&f, 60U);
    CHECK(f.target_sends == 1U && f.stop_sends == 1U);

    CHECK(MotionSetup(&f, 3U));
    CHECK(XMotorSetTarget(&f.motor, 100, &profile, 1U, f.now));
    MotionRun(&f, 10U);
    f.flags |= 4U;
    MotionRun(&f, 70U);
    CHECK(!f.motor.motion.goal_active && f.motor.motion.fault == X_FAULT_DRIVER);
    CHECK(!XMotorSetTarget(&f.motor, 200, &profile, 2U, f.now));

    /* Failed FE must not use old disabled feedback: probe after the FE response. */
    CHECK(MotionSetup(&f, 2U));
    CHECK(XMotorSetTarget(&f.motor, 100, &profile, 1U, f.now));
    MotionStep(&f); /* F3 in flight while cached status is disabled. */
    XMotorStop(&f.motor, X_FAULT_LINK);
    f.reject_function = 0xFEU;
    MotionRun(&f, 10U);
    CHECK(f.target_sends == 0U && f.motor.motion.stop_pending && f.enable_sends == 1U);
    MotionRun(&f, 120U);
    CHECK(f.motor.motion.stop_pending && f.stop_sends >= 2U);
    f.flags &= (uint8_t)~1U;
    MotionRun(&f, 80U);
    CHECK(!f.motor.motion.stop_pending && f.motor.motion.state == X_MOTION_FAULT);

    /* Async FD/9F does not satisfy ACK and can precede a valid query reply. */
    CHECK(MotionSetup(&f, 3U));
    CHECK(XMotorSetTarget(&f.motor, 100, &profile, 1U, f.now));
    MotionStep(&f);
    const uint8_t arrived[] = {1U, 0xFDU, 0x9FU, 0x6BU};
    XMotorReceive(&f.motor, arrived, sizeof(arrived), f.now);
    CHECK(!f.motor.response_received && !f.motor.response_failed);
    MotionRun(&f, 10U);
    CHECK(f.motor.motion.command_accepted && f.motor.motion.state != X_MOTION_REACHED);
    while (!f.link.busy || f.link.pending[1] != 0x36U) MotionStep(&f);
    uint8_t joined[12]; memcpy(joined, arrived, 4U); PositionFrame(joined + 4U, 100);
    f.link.busy = false;
    XMotorReceive(&f.motor, joined, sizeof(joined), f.now + 1U);
    CHECK(f.motor.feedback.position_units == 100 && f.motor.diagnostics.asynchronous_arrivals == 2U);

    CHECK(MotionSetup(&f, 3U));
    CHECK(XMotorSetTarget(&f.motor, 100, &profile, 1U, f.now));
    MotionRun(&f, 10U);
    XMotorTransportFault(&f.motor, f.now);
    CHECK(!f.motor.motion.goal_active && f.motor.motion.stop_pending && f.motor.motion.fault == X_FAULT_TRANSPORT);
    MotionRun(&f, 100U);
    CHECK(f.target_sends == 1U && f.stop_sends == 1U);
    return 0;
}

/* 板级回归：两条旧串口+两条塔吊串口，真实BSP分发HAL回调。 */
static DMA_HandleTypeDef dma3_rx = {.Init.Mode = DMA_CIRCULAR};
static DMA_HandleTypeDef dma3_tx = {.Init.Mode = DMA_NORMAL};
static DMA_HandleTypeDef dma6_rx = {.Init.Mode = DMA_CIRCULAR};
static DMA_HandleTypeDef dma6_tx = {.Init.Mode = DMA_NORMAL};
UART_HandleTypeDef huart3 = {.hdmarx = &dma3_rx, .hdmatx = &dma3_tx};
UART_HandleTypeDef huart6 = {.hdmarx = &dma6_rx, .hdmatx = &dma6_tx};

typedef struct
{
    UART_HandleTypeDef *handle;
    uint8_t *rx;
    uint16_t rx_size;
    uint16_t rx_pos;
    uint8_t tx[16];
    uint16_t tx_length;
    uint32_t sends;
} HALLink;

static HALLink hal_links[2];
static HALLink *FindLink(UART_HandleTypeDef *handle)
{
    return handle == &huart6 ? &hal_links[0] : handle == &huart3 ? &hal_links[1] : NULL;
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *handle, uint8_t *data, uint16_t size)
{
    HALLink *link = FindLink(handle);
    if (link == NULL) return HAL_ERROR;
    link->handle = handle;
    link->rx = data;
    link->rx_size = size;
    link->rx_pos = 0U;
    handle->hdmarx->remaining = size;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *handle, uint8_t *data, uint16_t size)
{
    HALLink *link = FindLink(handle);
    if (link == NULL || size > sizeof(link->tx)) return HAL_ERROR;
    memcpy(link->tx, data, size);
    link->tx_length = size;
    ++link->sends;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *handle) { (void)handle; return HAL_OK; }
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);

static void Inject(HALLink *link, const uint8_t *data, size_t length)
{
    for (size_t i = 0U; i < length; ++i)
    {
        link->rx[link->rx_pos] = data[i];
        link->rx_pos = (uint16_t)((link->rx_pos + 1U) % link->rx_size);
    }
    link->handle->hdmarx->remaining = link->rx_size - link->rx_pos;
    HAL_UART_TxCpltCallback(link->handle);
    HAL_UARTEx_RxEventCallback(link->handle, link->rx_pos);
}

static int TestBoard(void)
{
    static BSPUARTInstance old_uarts[2], fifth, duplicate;
    static DMA_HandleTypeDef old_rx[3] = {{.Init.Mode = DMA_CIRCULAR},
        {.Init.Mode = DMA_CIRCULAR}, {.Init.Mode = DMA_CIRCULAR}};
    static DMA_HandleTypeDef old_tx = {.Init.Mode = DMA_NORMAL};
    static UART_HandleTypeDef old_handles[3];
    old_handles[0] = (UART_HandleTypeDef){.hdmarx = &old_rx[0], .hdmatx = &old_tx};
    old_handles[1] = (UART_HandleTypeDef){.hdmarx = &old_rx[1]};
    old_handles[2] = (UART_HandleTypeDef){.hdmarx = &old_rx[2]};
    BSPUART_Init_Config_s config = {.handle = &old_handles[0], .tx_timeout_ms = 20U};
    CHECK(BSPUARTInit(&old_uarts[0], &config));
    CHECK(!BSPUARTInit(&duplicate, &config));
    config = (BSPUART_Init_Config_s){.handle = &old_handles[1], .receive_only = true};
    CHECK(BSPUARTInit(&old_uarts[1], &config));
    CHECK(BoardCraneInit(0U));
    CHECK(!BoardCraneInit(0U));
    config.handle = &old_handles[2];
    CHECK(!BSPUARTInit(&fifth, &config)); /* 四实例满，仍拒绝超容量注册。 */
    CHECK(BoardCraneStart(0U));
    BoardCraneStatus status;
    BoardCraneGetStatus(&status, 0U);
    CHECK(!status.axes[0].feedback.position_valid && !status.axes[1].feedback.status_valid);
    CHECK(hal_links[0].sends == 0U && hal_links[1].sends == 0U);
    BoardCraneUpdate(2U);
    for (size_t i = 0U; i < 2U; ++i)
        CHECK(hal_links[i].tx_length == 3U && hal_links[i].tx[0] == 1U && hal_links[i].tx[1] == 0x36U);
    uint8_t frame[8];
    PositionFrame(frame, 72000);
    Inject(&hal_links[0], frame, sizeof(frame));
    PositionFrame(frame, -180000);
    Inject(&hal_links[1], frame, sizeof(frame));
    BoardCraneUpdate(3U);
    BoardCraneGetStatus(&status, 3U);
    CHECK(status.axes[BOARD_CRANE_YAW].feedback.position_units == 72000);
    CHECK(status.axes[BOARD_CRANE_Z].feedback.position_units == -180000);
    CHECK(status.axes[0].feedback.position_valid && status.axes[1].feedback.position_valid);
    BoardCraneUpdate(27U);
    const uint8_t yaw_status[] = {1U, 0x3AU, 0x83U, 0x6BU};
    const uint8_t z_status[] = {1U, 0x3AU, 0x01U, 0x6BU};
    Inject(&hal_links[0], yaw_status, sizeof(yaw_status));
    Inject(&hal_links[1], z_status, sizeof(z_status));
    BoardCraneUpdate(28U);
    BoardCraneGetStatus(&status, 28U);
    CHECK(status.axes[0].feedback.status_valid && status.axes[1].feedback.status_valid);
    CHECK(status.axes[0].feedback.position_reached && !status.axes[1].feedback.position_reached);
    HAL_UART_ErrorCallback(&huart6);
    BoardCraneUpdate(29U);
    BoardCraneGetStatus(&status, 29U);
    CHECK(!status.axes[0].feedback.position_valid && !status.axes[0].feedback.status_valid);
    CHECK(status.axes[0].uart_errors == 1U && status.axes[0].diagnostics.transport_errors == 1U);
    CHECK(status.axes[1].feedback.position_valid && status.axes[1].feedback.status_valid);
    BoardCraneGetStatus(&status, 528U);
    CHECK(!status.axes[1].feedback.position_valid && !status.axes[1].feedback.status_valid);
    return 0;
}

int RunXMotorTests(void)
{
    int result = TestProtocol();
    if (result != 0) return result;
    result = TestMotion();
    if (result != 0) return result;
    result = TestDisable();
    return result != 0 ? result : TestBoard();
}
