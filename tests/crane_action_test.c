/* 离线联动：真实USB/Robot/X/PWM；仅HAL和驱动反馈使用模拟。 */
#include "host_service.h"
#include "board_crane_action.h"
#include <string.h>

#define CHECK(c) do { if (!(c)) return __LINE__; } while (0)
extern uint32_t test_usb_clock;

typedef struct
{
    RobotInstance robot;
    ChassisControlInstance control;
    EmmBusInstance bus;
    XMotorInstance axes[2];
    TIM_TypeDef registers;
    TIM_HandleTypeDef timer;
    BSPPWMInstance pwm[2];
    ServoInstance servos[2];
    HostServiceInstance host;
    uint8_t tx[2][16];
    size_t tx_length[2];
} ActionFixture;

static bool BusSend(void *context, const uint8_t *data, size_t length, uint32_t now_ms)
{ (void)context; (void)data; (void)length; (void)now_ms; return true; }
static bool Busy(void *context) { (void)context; return false; }
typedef struct { ActionFixture *fixture; size_t axis; } AxisContext;
static AxisContext contexts[2];
static bool AxisSend(void *context, const uint8_t *data, size_t length, uint32_t now_ms)
{
    (void)now_ms;
    AxisContext *axis = context;
    memcpy(axis->fixture->tx[axis->axis], data, length);
    axis->fixture->tx_length[axis->axis] = length;
    return true;
}

static void Fresh(ActionFixture *f)
{
    for (size_t i = 0; i < 2U; ++i)
    {
        f->axes[i].feedback.position_ms = test_usb_clock;
        f->axes[i].feedback.status_ms = test_usb_clock;
    }
}

static bool Setup(ActionFixture *f)
{
    memset(f, 0, sizeof(*f));
    test_usb_clock = 100U;
    const EmmBus_Init_Config_s bus_config = {.send = BusSend, .tx_busy = Busy,
        .response_timeout_ms = 20U, .quiet_ms = 2U, .quarantine_ms = 40U,
        .command_timeout_ms = 200U, .feedback_timeout_ms = 500U};
    if (!EmmBusInit(&f->bus, &bus_config, test_usb_clock)) return false;
    f->control.initialized = true; f->control.config.bus = &f->bus;
    if (!RobotInit(&f->robot, &f->control)) return false;
    f->timer.Instance = &f->registers;
    for (size_t i = 0; i < 2U; ++i)
    {
        f->pwm[i].config = (BSPPWM_Init_Config_s){.timer = &f->timer,
            .channel = i == 0U ? TIM_CHANNEL_1 : TIM_CHANNEL_2,
            .timer_tick_hz = 1000000U, .period_ticks = 20000U};
        const Servo_Init_Config_s servo_config = {.pwm = &f->pwm[i], .min_pulse_us = 500U,
            .center_pulse_us = i == 0U ? 2400U : 1000U, .max_pulse_us = 2500U};
        if (!ServoInit(&f->servos[i], &servo_config)) return false;
        contexts[i] = (AxisContext){f, i};
        const XMotor_Init_Config_s config = {.send = AxisSend, .tx_busy = Busy,
            .context = &contexts[i], .address = 1U, .query_period_ms = 25U,
            .response_timeout_ms = 20U, .quiet_ms = 2U, .quarantine_ms = 40U,
            .feedback_timeout_ms = 500U, .position_tolerance_units = 10U};
        if (!XMotorInit(&f->axes[i], &config, test_usb_clock)) return false;
        f->axes[i].feedback = (XMotorFeedback){.position_valid = true, .status_valid = true,
            .enabled = true, .position_ms = test_usb_clock, .status_ms = test_usb_clock};
    }
    const XMotorMotionProfile profiles[2] = {{100U, 100U, 100U}, {200U, 100U, 100U}};
    if (!RobotCraneInit(&f->robot, &f->axes[0], &f->axes[1], profiles) ||
        !RobotRackServoInit(&f->robot, &f->servos[0]) ||
        !RobotGripperServoInit(&f->robot, &f->servos[1])) return false;
    const CraneActionConfig config = {
        .pick = {.yaw_units = -7200, .z_units = 15000, .rack_pulse_us = 1800U, .configured = true},
        .place = {.yaw_units = 10800, .z_units = 12000, .rack_pulse_us = 2000U, .configured = true},
        .safe_z_units = 3600, .open_pulse_us = 1000U, .closed_pulse_us = 550U,
        .rack_wait_ms = 100U, .gripper_wait_ms = 50U, .step_timeout_ms = 5000U,
    };
    if (!RobotCraneActionInit(&f->robot, &config)) return false;
    HostServiceInit(&f->host, &f->robot);
    BSPUSBConnected(false); BSPUSBConnected(true);
    HostServiceReceive(&f->host, test_usb_clock);
    return true;
}

static void Queue(ActionFixture *f, uint8_t type, uint32_t sequence,
                  const uint8_t *payload, uint16_t size)
{
    (void)f;
    uint8_t frame[24];
    size_t length = HostProtocolEncode(type, sequence, payload, size, frame, sizeof(frame));
    BSPUSBReceive(frame, length, test_usb_clock);
}

static void Dispatch(ActionFixture *f)
{ HostServiceReceive(&f->host, test_usb_clock); }

/* 经过真实FD ACK、真实36/3A解析及X目标关联，模拟当前目标到达。 */
static bool Arrive(ActionFixture *f, size_t axis)
{
    XMotorInstance *motor = &f->axes[axis];
    test_usb_clock += 3U; Fresh(f);
    XMotorUpdate(motor, test_usb_clock);
    if (f->tx_length[axis] != 16U || f->tx[axis][1] != 0xFDU) return false;
    const uint8_t ack[] = {1U, 0xFDU, 2U, 0x6BU};
    XMotorReceive(motor, ack, sizeof(ack), test_usb_clock);
    XMotorUpdate(motor, test_usb_clock);
    for (unsigned i = 0U; i < 2U; ++i)
    {
        test_usb_clock += 25U; Fresh(f);
        XMotorUpdate(motor, test_usb_clock);
        if (!motor->active) return false;
        if (motor->request[1] == 0x36U)
        {
            int64_t target = motor->motion.target_units;
            uint32_t magnitude = (uint32_t)(target < 0 ? -target : target);
            uint8_t reply[] = {1U, 0x36U, target < 0 ? 1U : 0U,
                (uint8_t)(magnitude >> 24U), (uint8_t)(magnitude >> 16U),
                (uint8_t)(magnitude >> 8U), (uint8_t)magnitude, 0x6BU};
            XMotorReceive(motor, reply, sizeof(reply), test_usb_clock);
        }
        else if (motor->request[1] == 0x3AU)
        {
            const uint8_t reply[] = {1U, 0x3AU, 3U, 0x6BU};
            XMotorReceive(motor, reply, sizeof(reply), test_usb_clock);
        }
        else return false;
        XMotorUpdate(motor, test_usb_clock);
    }
    return motor->motion.state == X_MOTION_REACHED;
}

static bool ToStep(ActionFixture *f, CraneActionStep step)
{
    for (unsigned count = 0U; count < 7U; ++count)
    {
        CraneActionInstance *action = &f->robot.crane_action;
        if (action->status.step == step) return true;
        if (!CraneActionIsRunning(action)) return false;
        if (action->status.step == CRANE_STEP_RACK || action->status.step == CRANE_STEP_GRIP)
        {
            uint32_t duration = action->active_config.gripper_wait_ms;
            if (action->status.step == CRANE_STEP_RACK && action->active_config.rack_wait_ms > duration)
                duration = action->active_config.rack_wait_ms;
            test_usb_clock += duration;
        }
        else if (!Arrive(f, action->status.step == CRANE_STEP_YAW ? 0U : 1U)) return false;
        Fresh(f);
        RobotUpdate(&f->robot, test_usb_clock);
    }
    return false;
}

static int NormalAndOwnership(void)
{
    ActionFixture f;
    CHECK(Setup(&f));
    Queue(&f, HOST_CRANE_PICK, 41U, NULL, 0U); Dispatch(&f);
    CraneActionInstance *action = &f.robot.crane_action;
    CHECK(action->status.step == CRANE_STEP_SAFE_Z && f.axes[1].motion.target_units == 3600);
    CHECK(!f.axes[0].motion.goal_active && f.registers.compare == 2400U);
    uint32_t generation = action->status.generation;
    Queue(&f, HOST_CRANE_PICK, 41U, NULL, 0U); Dispatch(&f);
    CHECK(action->status.generation == generation && action->status.rejected_commands == 0U);
    Queue(&f, HOST_CRANE_PLACE, 42U, NULL, 0U); Dispatch(&f);
    CHECK(action->status.rejected_commands == 1U && action->status.action == CRANE_ACTION_PICK);
    uint8_t pulse[2] = {0xE8U, 3U};
    Queue(&f, HOST_SERVO_PULSE, 1U, pulse, 2U);
    Queue(&f, HOST_GRIPPER_SERVO_PULSE, 1U, pulse, 2U);
    uint8_t target[12] = {0};
    Queue(&f, HOST_CRANE_TARGET, 1U, target, 12U);
    Queue(&f, HOST_CRANE_CONFIG, 1U, target, 12U); Dispatch(&f);
    CHECK(f.control.status.rejected_commands == 5U && f.registers.compare == 2400U);
    CHECK(!HostServiceSelectFeedback(&f.host, HOST_FEEDBACK_FIREWATER));
    /* 实际ACK之前的原始到达位、旧反馈不得推进。 */
    f.axes[1].feedback.position_reached = true;
    f.axes[1].feedback.position_units = 3600;
    RobotUpdate(&f.robot, test_usb_clock);
    CHECK(action->status.step == CRANE_STEP_SAFE_Z);
    CHECK(ToStep(&f, CRANE_STEP_RACK));
    CHECK(f.axes[0].motion.target_units == -7200 && f.registers.compare == 1800U);
    CHECK(f.registers.compare2 == 1000U);
    /* 快照不受后续配置改变影响。 */
    action->config.pick.z_units = -999;
    f.robot.crane_profiles[1].speed_deci_rpm = 900U;
    test_usb_clock += 99U; Fresh(&f); RobotUpdate(&f.robot, test_usb_clock);
    CHECK(action->status.step == CRANE_STEP_RACK);
    test_usb_clock += 1U; Fresh(&f); RobotUpdate(&f.robot, test_usb_clock);
    CHECK(action->status.step == CRANE_STEP_LOWER && f.axes[1].motion.target_units == 15000);
    CHECK(f.axes[1].motion.profile.speed_deci_rpm == 200U);
    CHECK(ToStep(&f, CRANE_STEP_GRIP));
    CHECK(f.registers.compare2 == 550U);
    CHECK(ToStep(&f, CRANE_STEP_LIFT));
    CHECK(f.axes[1].motion.target_units == 3600);
    CHECK(Arrive(&f, 1U)); RobotUpdate(&f.robot, test_usb_clock);
    CHECK(action->status.state == CRANE_ACTION_DONE && f.registers.compare2 == 550U);
    Queue(&f, HOST_CRANE_PICK, 41U, NULL, 0U); Dispatch(&f);
    CHECK(action->status.state == CRANE_ACTION_DONE && action->status.generation == generation);
    Queue(&f, HOST_CRANE_PLACE, 42U, NULL, 0U); Dispatch(&f);
    CHECK(CraneActionIsRunning(action) && action->status.generation == generation + 1U);
    /* 新步骤目标和上次安全高度相同，也要等两类新反馈。 */
    RobotUpdate(&f.robot, test_usb_clock);
    CHECK(action->status.step == CRANE_STEP_SAFE_Z);
    ++f.axes[1].diagnostics.position_replies; RobotUpdate(&f.robot, test_usb_clock);
    CHECK(action->status.step == CRANE_STEP_SAFE_Z);
    ++f.axes[1].diagnostics.status_replies; RobotUpdate(&f.robot, test_usb_clock);
    CHECK(action->status.step == CRANE_STEP_YAW);
    CHECK(ToStep(&f, CRANE_STEP_LOWER));
    CHECK(f.registers.compare == 2000U && f.registers.compare2 == 550U);
    CHECK(f.axes[1].motion.target_units == 12000);
    CHECK(ToStep(&f, CRANE_STEP_GRIP) && f.registers.compare2 == 1000U);
    CHECK(ToStep(&f, CRANE_STEP_LIFT));
    CHECK(Arrive(&f, 1U)); RobotUpdate(&f.robot, test_usb_clock);
    CHECK(action->status.state == CRANE_ACTION_DONE && f.registers.compare2 == 1000U);
    return 0;
}

static int EveryStageCancellation(void)
{
    for (unsigned step = CRANE_STEP_SAFE_Z; step <= CRANE_STEP_LIFT; ++step)
    {
        for (unsigned event = 0U; event < 8U; ++event)
        {
            ActionFixture f;
            CHECK(Setup(&f));
            Queue(&f, HOST_CRANE_PICK, 1U, NULL, 0U); Dispatch(&f);
            CHECK(ToStep(&f, (CraneActionStep)step));
            uint32_t rack = f.registers.compare, grip = f.registers.compare2;
            uint8_t axis = 0U;
            switch (event)
            {
            case 0U: Queue(&f, HOST_STOP, 2U, NULL, 0U); Dispatch(&f); break;
            case 1U:
                Queue(&f, HOST_CRANE_STOP, 2U, &axis, 1U);
                Queue(&f, HOST_CRANE_PLACE, 3U, NULL, 0U); Dispatch(&f); break;
            case 2U: BSPUSBConnected(false); Dispatch(&f); break;
            case 3U: f.axes[0].feedback.stalled = true; RobotUpdate(&f.robot, test_usb_clock); break;
            case 4U: f.axes[1].feedback.status_valid = false; RobotUpdate(&f.robot, test_usb_clock); break;
            case 5U:
                test_usb_clock += 5000U; Fresh(&f); RobotUpdate(&f.robot, test_usb_clock); break;
            case 7U:
                Queue(&f, HOST_CRANE_DISABLE, 2U, &axis, 1U);
                Queue(&f, HOST_CRANE_PLACE, 3U, NULL, 0U); Dispatch(&f);
                CHECK(f.axes[0].motion.disable_requested && !f.axes[1].motion.disable_requested);
                break;
            default:
            {
                uint8_t noise[512] = {0};
                BSPUSBReceive(noise, sizeof(noise), test_usb_clock);
                BSPUSBReceive(noise, 1U, test_usb_clock); Dispatch(&f); break;
            }
            }
            CHECK(!CraneActionIsRunning(&f.robot.crane_action));
            CHECK(f.axes[0].motion.stop_pending && f.axes[1].motion.stop_pending);
            CHECK(f.registers.compare == rack && f.registers.compare2 == grip);
            CHECK(f.robot.crane_action.status.state ==
                ((event >= 3U && event <= 5U) ? CRANE_ACTION_FAILED : CRANE_ACTION_CANCELLED));
            BSPUSBConnected(true); Dispatch(&f);
            test_usb_clock += 1000U; Fresh(&f); RobotUpdate(&f.robot, test_usb_clock);
            CHECK(f.registers.compare2 == grip && !f.axes[1].motion.goal_active);
            Queue(&f, HOST_CRANE_PICK, 1U, NULL, 0U); Dispatch(&f);
            CHECK(!CraneActionIsRunning(&f.robot.crane_action));
        }
    }
    return 0;
}

static int ValidationAndFeedback(void)
{
    ActionFixture f;
    CHECK(Setup(&f));
    CHECK(BoardCraneActionRobotInit(&f.robot)); /* 核对真实板级标定参数及动作输出。 */
    CHECK(CraneActionReady(&f.robot.crane_action, CRANE_ACTION_PICK));
    CHECK(CraneActionReady(&f.robot.crane_action, CRANE_ACTION_PLACE));
    CHECK(f.robot.crane_action.config.safe_z_units == 0);
    CHECK(f.robot.crane_action.config.pick.yaw_units == 153);
    CHECK(f.robot.crane_action.config.pick.z_units == 37121);
    CHECK(f.robot.crane_action.config.pick.rack_pulse_us == 2400U);
    CHECK(f.robot.crane_action.config.place.yaw_units == 2816);
    CHECK(f.robot.crane_action.config.place.z_units == 22315);
    CHECK(f.robot.crane_action.config.place.rack_pulse_us == 2400U);
    CHECK(f.robot.crane_action.config.step_timeout_ms == 120000U);
    f.axes[1].feedback.position_units = 37121;
    Queue(&f, HOST_CRANE_PICK, 1U, NULL, 0U); Dispatch(&f);
    CHECK(f.axes[1].motion.target_units == 0);
    /* 当前10 RPM最长z行程超过原30秒，未到达时应继续等，不得发yaw。 */
    test_usb_clock += 62000U; Fresh(&f); RobotUpdate(&f.robot, test_usb_clock);
    CHECK(CraneActionIsRunning(&f.robot.crane_action));
    CHECK(f.robot.crane_action.status.step == CRANE_STEP_SAFE_Z);
    CHECK(!f.axes[0].motion.goal_active);
    CHECK(ToStep(&f, CRANE_STEP_YAW) && f.axes[0].motion.target_units == 153);
    CHECK(ToStep(&f, CRANE_STEP_LOWER) && f.axes[1].motion.target_units == 37121);
    CHECK(f.registers.compare == 2400U && f.registers.compare2 == 1000U);
    CHECK(ToStep(&f, CRANE_STEP_LIFT) && f.axes[1].motion.target_units == 0);
    CHECK(f.registers.compare2 == 550U);
    CHECK(Setup(&f));
    CHECK(BoardCraneActionRobotInit(&f.robot));
    Queue(&f, HOST_CRANE_PLACE, 1U, NULL, 0U); Dispatch(&f);
    CHECK(f.axes[1].motion.target_units == 0);
    CHECK(ToStep(&f, CRANE_STEP_YAW) && f.axes[0].motion.target_units == 2816);
    CHECK(ToStep(&f, CRANE_STEP_LOWER) && f.axes[1].motion.target_units == 22315);
    CHECK(f.registers.compare == 2400U);
    CHECK(ToStep(&f, CRANE_STEP_LIFT) && f.axes[1].motion.target_units == 0);
    CHECK(f.registers.compare2 == 1000U);
    CHECK(Setup(&f));
    CHECK(BoardCraneActionRobotInit(&f.robot));
    Queue(&f, HOST_CRANE_PLACE, 1U, NULL, 0U); Dispatch(&f);
    test_usb_clock += 120000U; Fresh(&f); RobotUpdate(&f.robot, test_usb_clock);
    CHECK(f.robot.crane_action.status.state == CRANE_ACTION_FAILED);
    CHECK(f.robot.crane_action.status.fault == CRANE_ACTION_TIMEOUT);
    CHECK(f.axes[0].motion.stop_pending && f.axes[1].motion.stop_pending);
    CHECK(Setup(&f));
    /* 仍保留未配置拒绝的回归。 */
    f.robot.crane_action.config.pick.configured = false;
    f.robot.crane_action.config.place.configured = false;
    CHECK(!CraneActionReady(&f.robot.crane_action, CRANE_ACTION_PICK));
    CHECK(!CraneActionReady(&f.robot.crane_action, CRANE_ACTION_PLACE));
    Queue(&f, HOST_CRANE_PICK, 1U, NULL, 0U); Dispatch(&f);
    CHECK(f.robot.crane_action.status.rejected_commands == 1U && !f.axes[1].motion.goal_active);
    CHECK(Setup(&f));
    f.robot.crane_action.config.pick.yaw_units = INT64_MAX;
    Queue(&f, HOST_CRANE_PICK, 1U, NULL, 0U); Dispatch(&f);
    CHECK(!CraneActionIsRunning(&f.robot.crane_action));
    CHECK(CraneActionReady(&f.robot.crane_action, CRANE_ACTION_PLACE));
    CHECK(Setup(&f));
    f.robot.crane_action.config.pick.rack_pulse_us = 499U;
    CHECK(!CraneActionReady(&f.robot.crane_action, CRANE_ACTION_PICK));
    f.robot.crane_action.config.pick.rack_pulse_us = 1800U;
    f.robot.crane_action.config.gripper_wait_ms = 0U;
    CHECK(!CraneActionReady(&f.robot.crane_action, CRANE_ACTION_PICK));
    CHECK(Setup(&f));
    const uint8_t invalid_payload = 1U;
    Queue(&f, HOST_CRANE_PICK, 1U, &invalid_payload, 1U); Dispatch(&f);
    CHECK(f.host.status.received_frames == 0U && f.host.status.format_errors != 0U);
    CHECK(!f.axes[1].motion.goal_active);
    CHECK(Setup(&f));
    f.axes[0].motion.stop_pending = true;
    Queue(&f, HOST_CRANE_PICK, 1U, NULL, 0U); Dispatch(&f);
    CHECK(!CraneActionIsRunning(&f.robot.crane_action));
    CHECK(Setup(&f));
    f.axes[0].feedback.position_valid = false;
    Queue(&f, HOST_CRANE_PICK, 1U, NULL, 0U); Dispatch(&f);
    CHECK(!CraneActionIsRunning(&f.robot.crane_action));
    CHECK(Setup(&f));
    /* 已恢复且停止已确认的历史故障不阻止新的外部事件。 */
    f.axes[0].motion.state = X_MOTION_FAULT;
    f.axes[0].motion.fault = X_FAULT_DRIVER;
    Queue(&f, HOST_CRANE_PICK, 1U, NULL, 0U); Dispatch(&f);
    RobotUpdate(&f.robot, test_usb_clock);
    CHECK(CraneActionIsRunning(&f.robot.crane_action));
    CHECK(ToStep(&f, CRANE_STEP_RACK) && f.axes[0].motion.fault == X_FAULT_NONE);
    CHECK(Setup(&f));
    CHECK(ServoSetPulseUs(&f.servos[1], 550U));
    Queue(&f, HOST_CRANE_PLACE, 0xAABBCCDDU, NULL, 0U); Dispatch(&f);
    CHECK(ToStep(&f, CRANE_STEP_RACK));
    BoardMotorStatus motors = {0};
    uint32_t tx_time = test_usb_clock + 100U;
    HostServiceTransmit(&f.host, &motors, tx_time); /* STATUS */
    CHECK(f.host.tx[3] == HOST_STATUS); BSPUSBTxComplete();
    HostServiceTransmit(&f.host, &motors, tx_time + 1U); /* CRANE */
    CHECK(f.host.tx[3] == HOST_CRANE); BSPUSBTxComplete();
    HostServiceTransmit(&f.host, &motors, tx_time + 2U); /* ACTION */
    CHECK(f.host.tx[3] == HOST_CRANE_ACTION && f.host.tx[4] == 40U);
    const uint8_t *out = f.host.tx + 10U;
    CHECK(HostGetU32(out + 4U) == 0xAABBCCDDU && HostGetU32(out + 8U) == 1U);
    CHECK(out[20] == CRANE_ACTION_RUNNING && out[21] == CRANE_ACTION_PLACE);
    CHECK(out[22] == CRANE_STEP_RACK && HostGetU32(out + 24U) == 15U);
    CHECK(((uint16_t)out[28] | (uint16_t)out[29] << 8U) == 2000U);
    CHECK(((uint16_t)out[30] | (uint16_t)out[31] << 8U) == 550U);
    CHECK(out[32] == UINT8_MAX && out[34] == 0U && out[39] == 0U);
    CHECK(HostProtocolCRC(f.host.tx, 50U) == ((uint16_t)f.host.tx[50] | (uint16_t)f.host.tx[51] << 8U));
    BSPUSBTxComplete();
    /* 运行中模式切换拒绝；停止后的FireWater不混发ACTION。 */
    CHECK(!HostServiceSelectFeedback(&f.host, HOST_FEEDBACK_FIREWATER));
    Queue(&f, HOST_STOP, 9U, NULL, 0U); Dispatch(&f);
    f.axes[0].motion.stop_pending = false; f.axes[1].motion.stop_pending = false;
    f.control.status.state = CHASSIS_DISABLED; f.control.start_pending = false;
    CHECK(HostServiceSelectFeedback(&f.host, HOST_FEEDBACK_FIREWATER));
    uint32_t action_ms = f.host.last_action_tx_ms;
    HostServiceTransmit(&f.host, &motors, tx_time + 200U);
    CHECK(f.host.tx[0] != 0xA5U && f.host.last_action_tx_ms == action_ms); BSPUSBTxComplete();
    CHECK(Setup(&f));
    Queue(&f, HOST_CRANE_PICK, 1U, NULL, 0U); Dispatch(&f);
    ++f.axes[1].motion.generation; RobotUpdate(&f.robot, test_usb_clock);
    CHECK(f.robot.crane_action.status.fault == CRANE_ACTION_TARGET_CHANGED);
    CHECK(Setup(&f));
    test_usb_clock = UINT32_MAX - 20U; Fresh(&f);
    Queue(&f, HOST_CRANE_PICK, 1U, NULL, 0U); Dispatch(&f);
    CHECK(ToStep(&f, CRANE_STEP_RACK)); /* 正常流程跨毫秒回绕。 */
    CHECK(test_usb_clock < 200U);
    CHECK(ToStep(&f, CRANE_STEP_LOWER));
    return 0;
}

int RunCraneActionTests(void)
{
    int result = NormalAndOwnership(); if (result != 0) return result;
    result = EveryStageCancellation(); if (result != 0) return result;
    return ValidationAndFeedback();
}
