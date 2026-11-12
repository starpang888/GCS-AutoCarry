/* 离线：真实USB缓存/协议→Robot→X实例，观察命令及反馈，不访问硬件。 */
#include "host_service.h"
#include "bsp_pwm.h"
#include "servo.h"
#include <string.h>

#define CHECK(c) do { if (!(c)) return __LINE__; } while (0)
extern uint32_t test_usb_clock; /* existing host_link_test owns USB HAL shim. */
#define clock_ms test_usb_clock

static bool Send(void *context, const uint8_t *data, size_t length, uint32_t now)
{ (void)context; (void)data; (void)length; (void)now; return true; }
static bool Busy(void *context) { (void)context; return false; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{ (void)timer; (void)channel; return HAL_OK; }

typedef struct
{
    RobotInstance robot;
    ChassisControlInstance control;
    EmmBusInstance bus;
    XMotorInstance axes[2];
    TIM_TypeDef timer_registers;
    TIM_HandleTypeDef timer;
    BSPPWMInstance servo_pwm;
    ServoInstance servo;
    BSPPWMInstance gripper_pwm;
    ServoInstance gripper;
    HostServiceInstance host;
} Fixture;

static bool Setup(Fixture *f)
{
    memset(f, 0, sizeof(*f));
    clock_ms = 100U;
    const EmmBus_Init_Config_s bus_config = {
        .send = Send, .tx_busy = Busy, .response_timeout_ms = 20U, .quiet_ms = 2U,
        .quarantine_ms = 40U, .command_timeout_ms = 200U, .feedback_timeout_ms = 500U,
    };
    if (!EmmBusInit(&f->bus, &bus_config, clock_ms)) return false;
    f->control.initialized = true;
    f->control.config.bus = &f->bus;
    if (!RobotInit(&f->robot, &f->control)) return false;
    f->timer.Instance = &f->timer_registers;
    f->servo_pwm.config = (BSPPWM_Init_Config_s){.timer = &f->timer, .channel = TIM_CHANNEL_1,
        .timer_tick_hz = 1000000U, .period_ticks = 20000U};
    const Servo_Init_Config_s servo_config = {.pwm = &f->servo_pwm, .min_pulse_us = 500U,
        .center_pulse_us = 1500U, .max_pulse_us = 2500U};
    if (!ServoInit(&f->servo, &servo_config) || !RobotRackServoInit(&f->robot, &f->servo)) return false;
    f->gripper_pwm.config = (BSPPWM_Init_Config_s){.timer = &f->timer, .channel = TIM_CHANNEL_2,
        .timer_tick_hz = 1000000U, .period_ticks = 20000U};
    const Servo_Init_Config_s gripper_config = {.pwm = &f->gripper_pwm, .min_pulse_us = 500U,
        .center_pulse_us = 1500U, .max_pulse_us = 2500U};
    if (!ServoInit(&f->gripper, &gripper_config) || !RobotGripperServoInit(&f->robot, &f->gripper)) return false;
    for (unsigned i = 0; i < 2; ++i)
    {
        const XMotor_Init_Config_s config = {
            .send = Send, .tx_busy = Busy, .address = 1U, .query_period_ms = 25U,
            .response_timeout_ms = 20U, .quiet_ms = 2U, .quarantine_ms = 40U,
            .feedback_timeout_ms = 500U, .position_tolerance_units = 10U,
        };
        if (!XMotorInit(&f->axes[i], &config, clock_ms)) return false;
        f->axes[i].feedback = (XMotorFeedback){.position_valid = true, .status_valid = true,
            .enabled = true, .position_ms = clock_ms, .status_ms = clock_ms};
    }
    const XMotorMotionProfile profiles[2] = {{100U, 100U, 100U}, {200U, 300U, 400U}};
    if (!RobotCraneInit(&f->robot, &f->axes[0], &f->axes[1], profiles)) return false;
    HostServiceInit(&f->host, &f->robot);
    BSPUSBConnected(false);
    BSPUSBConnected(true);
    HostServiceReceive(&f->host, clock_ms);
    return true;
}

static void TargetPayload(uint8_t out[12], uint8_t axis, int64_t target)
{
    memset(out, 0, 12U); out[0] = axis;
    uint64_t bits = (uint64_t)target;
    HostPutU32(out + 4U, (uint32_t)bits);
    HostPutU32(out + 8U, (uint32_t)(bits >> 32U));
}

static void Queue(Fixture *f, uint8_t type, uint32_t sequence, const uint8_t *payload, uint16_t size)
{
    (void)f;
    uint8_t frame[24];
    size_t length = HostProtocolEncode(type, sequence, payload, size, frame, sizeof(frame));
    BSPUSBReceive(frame, length, clock_ms);
}

static void Dispatch(Fixture *f)
{ HostServiceReceive(&f->host, clock_ms); }

static int GripperTests(void)
{
    Fixture f;
    CHECK(Setup(&f));
    CHECK(f.timer_registers.compare == 1500U && f.timer_registers.compare2 == 1500U);
    uint8_t pulse[2] = {0x40U, 0x06U}; /* 1600 us, CH2 only. */
    Queue(&f, HOST_GRIPPER_SERVO_PULSE, 1U, pulse, 2U); Dispatch(&f);
    CHECK(ServoGetPulseUs(&f.gripper) == 1600U && f.timer_registers.compare2 == 1600U);
    CHECK(ServoGetPulseUs(&f.servo) == 1500U && f.timer_registers.compare == 1500U);
    pulse[0] = 0x0EU; pulse[1] = 0x06U; /* 1550 us, CH1 only. */
    Queue(&f, HOST_SERVO_PULSE, 2U, pulse, 2U); Dispatch(&f);
    CHECK(f.timer_registers.compare == 1550U && f.timer_registers.compare2 == 1600U);
    const uint16_t invalid[] = {0U, 499U, 2501U, UINT16_MAX};
    for (unsigned i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        pulse[0] = (uint8_t)invalid[i]; pulse[1] = (uint8_t)(invalid[i] >> 8U);
        Queue(&f, HOST_GRIPPER_SERVO_PULSE, 3U, pulse, 2U); Dispatch(&f);
    }
    CHECK(f.control.status.rejected_commands == 4U && f.timer_registers.compare2 == 1600U);
    uint32_t valid = f.host.status.received_frames;
    Queue(&f, HOST_GRIPPER_SERVO_PULSE, 4U, pulse, 1U); Dispatch(&f);
    CHECK(f.host.status.received_frames == valid && f.timer_registers.compare2 == 1600U);
    Queue(&f, HOST_STOP, 5U, NULL, 0U); Dispatch(&f);
    CHECK(f.timer_registers.compare == 1550U && f.timer_registers.compare2 == 1600U);
    BSPUSBConnected(false); Dispatch(&f);
    CHECK(f.timer_registers.compare2 == 1600U);
    return 0;
}

static int DisableTests(void)
{
    Fixture f;
    CHECK(Setup(&f));
    uint8_t target[12]; TargetPayload(target, 0U, 3600);
    Queue(&f, HOST_CRANE_TARGET, 1U, target, 12U); Dispatch(&f);
    uint8_t axis = 0U;
    Queue(&f, HOST_CRANE_DISABLE, 2U, &axis, 1U);
    Queue(&f, HOST_CRANE_TARGET, 3U, target, 12U);
    Queue(&f, HOST_CRANE_PICK, 3U, NULL, 0U); Dispatch(&f);
    CHECK(!f.axes[0].motion.goal_active && f.axes[0].motion.disable_requested);
    CHECK(f.axes[0].motion.stop_pending && f.axes[0].motion.sequence == 2U);
    CHECK(!f.axes[1].motion.stop_pending && f.control.status.state == CHASSIS_DISABLED);
    CHECK(f.robot.crane_action.status.rejected_commands == 0U);
    axis = 1U;
    Queue(&f, HOST_CRANE_DISABLE, 3U, &axis, 1U); Dispatch(&f);
    CHECK(f.axes[1].motion.disable_requested && f.axes[1].motion.stop_pending);
    axis = 2U;
    Queue(&f, HOST_CRANE_DISABLE, 4U, &axis, 1U); Dispatch(&f);
    CHECK(f.control.status.rejected_commands == 1U);
    uint32_t received = f.host.status.received_frames;
    Queue(&f, HOST_CRANE_DISABLE, 4U, NULL, 0U); Dispatch(&f);
    CHECK(f.host.status.received_frames == received);
    uint8_t invalid[2] = {0U, 0U};
    Queue(&f, HOST_CRANE_DISABLE, 4U, invalid, 2U); Dispatch(&f);
    CHECK(f.host.status.received_frames == received);
    Queue(&f, HOST_STOP, 5U, NULL, 0U); Dispatch(&f);
    CHECK(!f.axes[0].motion.disable_requested && !f.axes[1].motion.disable_requested);
    CHECK(f.axes[0].motion.stop_pending && f.axes[1].motion.stop_pending);
    CHECK(Setup(&f));
    const uint8_t yaw[] = {0xA5U, 0x5AU, 1U, 0x0CU, 1U, 0U, 1U, 0U, 0U, 0U, 0U, 0x38U, 0xE4U};
    const uint8_t z[] = {0xA5U, 0x5AU, 1U, 0x0CU, 1U, 0U, 1U, 0U, 0U, 0U, 1U, 0x19U, 0xF4U};
    BSPUSBReceive(yaw, sizeof(yaw), clock_ms); Dispatch(&f);
    BSPUSBReceive(z, sizeof(z), clock_ms); Dispatch(&f);
    CHECK(f.axes[0].motion.disable_requested && f.axes[1].motion.disable_requested);
    return 0;
}

int RunCraneHostTests(void)
{
    int RunHostLinkTests(void);
    int RunEmmTests(void);
    int RunUARTTests(void);
    int RunCraneActionTests(void);
    int regression = RunHostLinkTests();
    if (regression != 0) return 10000 + regression;
    regression = RunEmmTests();
    if (regression != 0) return 20000 + regression;
    regression = RunUARTTests();
    if (regression != 0) return 30000 + regression;
    regression = GripperTests();
    if (regression != 0) return regression;
    regression = DisableTests();
    if (regression != 0) return regression;
    regression = RunCraneActionTests();
    if (regression != 0) return 40000 + regression;
    Fixture f;
    CHECK(Setup(&f));
    CHECK(!f.axes[0].motion.goal_active && !f.axes[1].motion.stop_pending);
    uint8_t payload[12];
    TargetPayload(payload, 0U, -4294967295LL);
    Queue(&f, HOST_CRANE_TARGET, 7U, payload, 12U); Dispatch(&f);
    CHECK(f.axes[0].motion.target_units == -4294967295LL && f.axes[0].motion.goal_active);
    CHECK(f.axes[0].motion.sequence == 7U && f.axes[0].motion.generation == 1U);
    CHECK(!f.axes[1].motion.goal_active && f.robot.mode == ROBOT_IDLE);
    CHECK(f.control.status.state == CHASSIS_DISABLED);
    uint8_t pulse_payload[2] = {0x40U, 0x06U}; /* 1600 us */
    Queue(&f, HOST_SERVO_PULSE, 1U, pulse_payload, sizeof(pulse_payload)); Dispatch(&f);
    CHECK(ServoGetPulseUs(&f.servo) == 1600U && f.timer_registers.compare == 1600U);
    pulse_payload[0] = 0xB8U; pulse_payload[1] = 0x0BU; /* 3000 us, outside board range */
    Queue(&f, HOST_SERVO_PULSE, 2U, pulse_payload, sizeof(pulse_payload)); Dispatch(&f);
    CHECK(ServoGetPulseUs(&f.servo) == 1600U && f.timer_registers.compare == 1600U);
    CHECK(f.control.status.rejected_commands == 1U);
    CHECK(!HostServiceSelectFeedback(&f.host, HOST_FEEDBACK_FIREWATER));
    Queue(&f, HOST_CRANE_TARGET, 0U, payload, 12U); Dispatch(&f);
    CHECK(f.axes[0].motion.generation == 1U && f.axes[0].motion.sequence == 0U);
    TargetPayload(payload, 1U, 4294967295LL);
    Queue(&f, HOST_CRANE_TARGET, UINT32_MAX, payload, 12U); Dispatch(&f);
    CHECK(f.axes[1].motion.target_units == 4294967295LL && f.axes[1].motion.goal_active);
    CHECK(f.axes[1].motion.profile.speed_deci_rpm == 200U);

    /* Profile is injected per axis; changes affect a new goal, never restart one. */
    uint8_t config[12] = {0U, 0U, 0U, 0U, 0xE8U, 3U, 0x64U, 0U, 0xC8U, 0U, 0U, 0U};
    Queue(&f, HOST_CRANE_CONFIG, 1U, config, 12U); Dispatch(&f);
    CHECK(f.robot.crane_profiles[0].speed_deci_rpm == 1000U);
    CHECK(f.robot.crane_profiles[0].deceleration_rpm_s == 200U);
    CHECK(f.axes[0].motion.profile.speed_deci_rpm == 100U);
    TargetPayload(payload, 0U, -4294967295LL);
    Queue(&f, HOST_CRANE_TARGET, 3U, payload, 12U); Dispatch(&f);
    CHECK(f.axes[0].motion.profile.speed_deci_rpm == 100U && f.axes[0].motion.generation == 1U);
    TargetPayload(payload, 0U, 7200);
    Queue(&f, HOST_CRANE_TARGET, 3U, payload, 12U); Dispatch(&f);
    CHECK(f.axes[0].motion.profile.speed_deci_rpm == 1000U && f.axes[0].motion.generation == 2U);
    uint32_t rejected = f.axes[0].motion.rejected_commands;
    config[10] = 1U;
    Queue(&f, HOST_CRANE_CONFIG, 1U, config, 12U); Dispatch(&f);
    config[10] = 0U; config[4] = 0U; config[5] = 0U;
    Queue(&f, HOST_CRANE_CONFIG, 1U, config, 12U); Dispatch(&f);
    TargetPayload(payload, 0U, INT64_MIN);
    Queue(&f, HOST_CRANE_TARGET, 1U, payload, 12U); Dispatch(&f);
    TargetPayload(payload, 0U, INT64_MAX);
    Queue(&f, HOST_CRANE_TARGET, 1U, payload, 12U); Dispatch(&f);
    TargetPayload(payload, 0U, 9000); payload[2] = 1U;
    Queue(&f, HOST_CRANE_TARGET, 1U, payload, 12U); Dispatch(&f);
    CHECK(f.axes[0].motion.rejected_commands == rejected + 5U && f.axes[0].motion.target_units == 7200);
    TargetPayload(payload, 2U, 100);
    Queue(&f, HOST_CRANE_TARGET, 1U, payload, 12U); Dispatch(&f);
    CHECK(f.control.status.rejected_commands == 2U);
    uint32_t valid = f.host.status.received_frames;
    Queue(&f, HOST_CRANE_TARGET, 1U, payload, 11U); Dispatch(&f);
    CHECK(f.host.status.received_frames == valid && f.host.status.format_errors > 0U);

    uint8_t axis = 0U;
    Queue(&f, HOST_CRANE_STOP, 10U, &axis, 1U);
    TargetPayload(payload, 0U, 10000);
    Queue(&f, HOST_CRANE_TARGET, 11U, payload, 12U); /* same batch cannot restart yaw. */
    TargetPayload(payload, 1U, -72000);
    Queue(&f, HOST_CRANE_TARGET, 12U, payload, 12U); Dispatch(&f);
    CHECK(!f.axes[0].motion.goal_active && f.axes[0].motion.stop_pending);
    CHECK(f.axes[0].motion.target_units == 7200 && f.axes[0].motion.sequence == 10U);
    CHECK(f.axes[1].motion.goal_active && f.axes[1].motion.target_units == -72000);
    Queue(&f, HOST_STOP, 13U, NULL, 0U);
    Queue(&f, HOST_CRANE_TARGET, 14U, payload, 12U); Dispatch(&f);
    CHECK(!f.axes[1].motion.goal_active && f.axes[1].motion.stop_pending);
    CHECK(f.control.status.state == CHASSIS_STOPPING && f.bus.stop_pending);

    CHECK(Setup(&f));
    TargetPayload(payload, 1U, -72000);
    Queue(&f, HOST_CRANE_TARGET, 1U, payload, 12U); Dispatch(&f);
    BSPUSBConnected(false); Dispatch(&f);
    CHECK(!f.axes[1].motion.goal_active && f.axes[1].motion.stop_pending && f.axes[1].motion.fault == X_FAULT_LINK);
    CHECK(!f.axes[0].motion.stop_pending); /* idle axis has no startup/reconnect stop. */
    BSPUSBConnected(true); Dispatch(&f);
    CHECK(!f.axes[1].motion.goal_active);
    CHECK(Setup(&f));
    TargetPayload(payload, 0U, 72000);
    Queue(&f, HOST_CRANE_TARGET, 1U, payload, 12U); Dispatch(&f);
    uint8_t noise[512] = {0};
    BSPUSBReceive(noise, sizeof(noise), clock_ms);
    BSPUSBReceive(noise, 1U, clock_ms); Dispatch(&f);
    CHECK(f.axes[0].motion.stop_pending && f.axes[0].motion.fault == X_FAULT_LINK);

    /* Binary crane output keeps 64-bit actual/target and original STATUS/POSE. */
    CHECK(Setup(&f));
    f.axes[0].feedback.position_units = -4294967295LL;
    f.axes[1].feedback.position_units = 4294967295LL;
    TargetPayload(payload, 0U, 7200);
    Queue(&f, HOST_CRANE_TARGET, 19U, payload, 12U); Dispatch(&f);
    BoardMotorStatus motors = {0};
    clock_ms = 150U; HostServiceTransmit(&f.host, &motors, clock_ms);
    CHECK(f.host.tx[3] == HOST_STATUS && f.host.tx[4] == 196U);
    BSPUSBTxComplete();
    clock_ms = 151U; HostServiceTransmit(&f.host, &motors, clock_ms);
    CHECK(f.host.tx[3] == HOST_CRANE && f.host.tx[4] == 116U);
    const uint8_t *out = f.host.tx + 10U;
    CHECK(HostGetU32(out) == 151U);
    out += 4U;
    CHECK(HostGetU32(out) == 1U && HostGetU32(out + 4U) == UINT32_MAX);
    CHECK(HostGetU32(out + 8U) == 7200U && HostGetU32(out + 12U) == 0U);
    CHECK(HostGetU32(out + 16U) == 19U && HostGetU32(out + 20U) == 1U);
    CHECK(HostGetU32(out + 36U) == 151U && out[40] == X_MOTION_PENDING);
    CHECK(HostGetU32(out + 44U) == 100U && HostGetU32(out + 56U) == UINT32_MAX);
    CHECK(HostGetU32(out + 60U) == 0U);
    CHECK(HostProtocolCRC(f.host.tx, 126U) == ((uint16_t)f.host.tx[126] | (uint16_t)f.host.tx[127] << 8U));
    BSPUSBTxComplete();
    clock_ms = 175U; HostServiceTransmit(&f.host, &motors, clock_ms);
    CHECK(f.host.tx[3] == HOST_POSE && f.host.tx[4] == 64U);
    BSPUSBTxComplete();
    f.axes[0].motion.goal_active = false;
    CHECK(HostServiceSelectFeedback(&f.host, HOST_FEEDBACK_FIREWATER));
    uint32_t old_crane_ms = f.host.last_crane_tx_ms;
    clock_ms = 226U; HostServiceTransmit(&f.host, &motors, clock_ms);
    CHECK(f.host.last_crane_tx_ms == old_crane_ms && f.host.tx[0] != 0xA5U);
    BSPUSBTxComplete();
    return 0;
}
