#include "robot.h"
#include "host_service.h"
#include <math.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

typedef struct
{
    EmmBusInstance bus;
    EmmMotorInstance motors[4];
    RobotInstance robot;
    ChassisOdometryInstance odometry;
    ChassisPositionControlInstance position_control;
    OmniKinematicsInstance chassis_kinematics;
    ChassisControlInstance control;
    uint8_t tx[8];
    size_t tx_length;
    uint32_t now;
    uint32_t enable_count;
    uint8_t slot;
    bool tx_busy;
    bool enabled[4];
    bool drove[4];
    int16_t speed[4];
    int64_t position[4];
    int32_t fraction[4];
    bool reject_enable;
    bool reject_speed;
    bool hold_last_enable;
    bool stalled[4];
    bool skip_application;
    bool drop_last_motor;
    bool drop_next_query;
} Fixture;

static bool Send(void *context, const uint8_t *data, size_t length, uint32_t now_ms)
{
    (void)now_ms;
    Fixture *f = context;
    if (f->tx_busy || length > sizeof(f->tx))
    {
        return false;
    }
    memcpy(f->tx, data, length);
    f->tx_length = length;
    f->tx_busy = true;
    return true;
}

static bool TxBusy(void *context)
{
    return ((Fixture *)context)->tx_busy;
}

static void Reply(Fixture *f)
{
    if (!f->tx_busy)
    {
        return;
    }
    f->tx_busy = false;
    if (f->drop_next_query && (f->tx[1] == 0x36U || f->tx[1] == 0x3AU))
    {
        f->drop_next_query = false;
        return;
    }
    uint8_t address = f->tx[0] == 0U ? 1U : f->tx[0];
    uint8_t index = address - 1U;
    uint8_t response[8] = {address, f->tx[1], 2U, 0x6BU};
    size_t length = 4U;
    switch (f->tx[1])
    {
    case 0xFE:
        response[2] = f->enabled[0] ? 2U : 0xE2U;
        for (uint8_t i = 0; i < 4U; ++i) f->speed[i] = 0;
        break;
    case 0xF3:
        if (f->tx[3] != 0U && f->reject_enable)
        {
            response[2] = 0xE2U;
        }
        else if (f->tx[0] == 0U)
        {
            for (uint8_t i = 0; i < 4U; ++i) f->enabled[i] = false;
        }
        else
        {
            if (!(f->hold_last_enable && index == 3U && f->tx[3] != 0U))
            {
                f->enabled[index] = f->tx[3] != 0U;
            }
            ++f->enable_count;
        }
        break;
    case 0xF6:
        if (!f->enabled[index] || f->reject_speed)
        {
            response[2] = 0xE2U;
        }
        else
        {
            int16_t speed = (int16_t)(((uint16_t)f->tx[3] << 8U) | f->tx[4]);
            f->speed[index] = f->tx[2] != 0U ? -speed : speed;
            f->drove[index] |= speed != 0;
        }
        break;
    case 0x36:
    {
        uint32_t magnitude = (uint32_t)(f->position[index] < 0 ? -f->position[index] : f->position[index]);
        response[2] = f->position[index] < 0 ? 1U : 0U;
        response[3] = (uint8_t)(magnitude >> 24U);
        response[4] = (uint8_t)(magnitude >> 16U);
        response[5] = (uint8_t)(magnitude >> 8U);
        response[6] = (uint8_t)magnitude;
        response[7] = 0x6BU;
        length = 8U;
        break;
    }
    case 0x3A:
        response[2] = (f->enabled[index] ? 0x83U : 0x82U) | (f->stalled[index] ? 4U : 0U);
        break;
    default:
        response[1] = 0U;
        response[2] = 0xEEU;
        break;
    }
    if (!(f->drop_last_motor && address == 4U))
    {
        EmmBusReceive(&f->bus, response, length, f->now);
    }
}

static void Advance(Fixture *f, uint32_t milliseconds)
{
    for (uint32_t n = 0; n < milliseconds; ++n)
    {
        ++f->now;
        for (uint8_t i = 0; i < 4U; ++i)
        {
            if (f->enabled[i])
            {
                f->fraction[i] += (int32_t)f->speed[i] * 65536;
                f->position[i] += f->fraction[i] / 60000;
                f->fraction[i] %= 60000;
            }
        }
        Reply(f);
        if (f->now % 5U == 0U)
        {
            (void)EmmMotorQuery(&f->motors[f->slot % 4U],
                f->slot < 4U ? EMM_QUERY_POSITION : EMM_QUERY_STATUS);
            f->slot = (uint8_t)((f->slot + 1U) % 8U);
        }
        EmmBusUpdate(&f->bus, f->now);
        if (!f->skip_application)
        {
            if (f->robot.odometry != NULL)
            {
                EmmMotorFeedback wheels[4];
                for (unsigned i = 0; i < 4; ++i)
                    EmmMotorGetFeedback(&f->motors[i], f->now, &wheels[i]);
                ChassisOdometryUpdate(&f->odometry, wheels, f->now);
            }
            RobotUpdate(&f->robot, f->now);
        }
    }
}

static bool Init(Fixture *f, uint32_t now)
{
    memset(f, 0, sizeof(*f));
    f->now = now;
    const EmmBus_Init_Config_s config = {
        .send = Send, .tx_busy = TxBusy, .context = f,
        .response_timeout_ms = 20U, .quiet_ms = 2U, .quarantine_ms = 40U,
        .command_timeout_ms = 200U, .feedback_timeout_ms = 500U,
    };
    if (!EmmBusInit(&f->bus, &config, now)) return false;
    for (uint8_t i = 0; i < 4U; ++i)
    {
        const EmmMotor_Init_Config_s motor_config = {
            .bus = &f->bus, .address = i + 1U, .direction = 1, .gear_ratio = 1.0f,
        };
        if (!EmmMotorInit(&f->motors[i], &motor_config)) return false;
    }
    return true;
}

static bool InitControl(Fixture *f, HostServiceInstance *service, uint32_t now)
{
    if (!Init(f, now)) return false;
    const OmniKinematics_Init_Config_s kinematics_config = {
        .wheel_radius_m = 0.0375f, .wheelbase_m = 0.240f,
        .track_width_m = 0.240f,
    };
    if (!OmniKinematicsInit(&f->chassis_kinematics, &kinematics_config)) return false;
    for (unsigned i = 0; i < 4; ++i)
    {
        f->motors[i].config.direction = -1;
        f->motors[i].config.wheel_rad_s_per_speed_unit = 0.10471975512f;
    }
    const ChassisControl_Init_Config_s config = {
        .bus = &f->bus, .motors = {&f->motors[0], &f->motors[1], &f->motors[2], &f->motors[3]},
        .kinematics = &f->chassis_kinematics,
    };
    if (!ChassisControlInit(&f->control, &config) || !RobotInit(&f->robot, &f->control)) return false;
    HostServiceInit(service, &f->robot);
    BSPUSBConnected(true);
    HostServiceReceive(service, now); /* Acknowledge initial configuration epoch. */
    Advance(f, 200);
    return true;
}

extern uint32_t test_usb_clock;
static void HostSend(Fixture *f, HostServiceInstance *service, uint8_t type, uint32_t seq, int32_t vx)
{
    uint8_t frame[24], payload[12] = {0};
    HostPutU32(payload, (uint32_t)vx);
    size_t length = HostProtocolEncode(type, seq, payload, type == HOST_VELOCITY ? 12 : 0, frame, 24);
    test_usb_clock = f->now;
    BSPUSBReceive(frame, length, test_usb_clock);
    HostServiceReceive(service, f->now);
}
static bool ArmControl(Fixture *f, HostServiceInstance *service)
{
    HostSend(f, service, HOST_ARM, 0xFFFFFFFE, 0);
    Advance(f, 150);
    return f->control.status.state == CHASSIS_ACTIVE;
}
static int RunPositionIntegrationTests(void);
static int RunArrivalTests(void);

int RunHostControlTests(void)
{
    Fixture f;
    HostServiceInstance service;
    /* Startup remains idle until a command; one VELOCITY starts without ARM. */
    CHECK(InitControl(&f, &service, 0));
    Advance(&f, 1200);
    CHECK(f.enable_count == 0 && !f.bus.motion_allowed);
    HostSend(&f, &service, HOST_VELOCITY, 1, 50);
    Advance(&f, 150);
    CHECK(f.control.status.state == CHASSIS_ACTIVE);
    CHECK(!HostServiceSelectFeedback(&service, HOST_FEEDBACK_FIREWATER));
    CHECK(service.status.feedback_mode == HOST_FEEDBACK_BINARY);
    /* A single command remains active for ten seconds without host refresh. */
    Advance(&f, 10000);
    CHECK(f.control.status.state == CHASSIS_ACTIVE);
    CHECK(f.speed[0] == 9 && f.speed[1] == -9 && f.speed[2] == -9 && f.speed[3] == 9);
    CHECK(f.motors[0].feedback.wheel_speed_rad_s < 0 && f.motors[1].feedback.wheel_speed_rad_s > 0);
    BoardMotorStatus snapshot = {.motion_allowed = f.bus.motion_allowed};
    for (unsigned i = 0; i < 4; ++i) EmmMotorGetFeedback(&f.motors[i], f.now, &snapshot.wheels[i]);
    BSPUSBTxComplete();
    HostServiceTransmit(&service, &snapshot, f.now);
    CHECK(HostGetU32(service.payload + 76) == 1);
    CHECK((int32_t)HostGetU32(service.payload + 64) > 40);
    CHECK((int32_t)HostGetU32(service.payload + 64) < 55);
    /* Repeated and decreasing sequence values still apply the latest target. */
    HostSend(&f, &service, HOST_VELOCITY, 1, -50);
    Advance(&f, 100);
    CHECK(f.speed[0] == -9 && f.speed[1] == 9 && f.control.status.last_sequence == 1);
    HostSend(&f, &service, HOST_VELOCITY, 0, 50);
    Advance(&f, 100);
    CHECK(f.speed[0] == 9 && f.control.status.last_sequence == 0);
    CHECK(f.control.status.rejected_commands == 0);
    /* ARM remains compatible and cannot replace a running target with zero. */
    HostSend(&f, &service, HOST_ARM, 123, 0);
    Advance(&f, 100);
    CHECK(f.speed[0] == 9);
    /* 0.5 m/s directly reaches about 90 RPM per wheel, above the old ceiling. */
    HostSend(&f, &service, HOST_VELOCITY, 0, 500);
    Advance(&f, 100);
    CHECK(f.control.status.state == CHASSIS_ACTIVE);
    CHECK(f.speed[0] == 90 && f.speed[1] == -90 && f.speed[2] == -90 && f.speed[3] == 90);
    CHECK(f.control.status.target.applied.vx_m_s == 0.5f);
    HostSend(&f, &service, HOST_VELOCITY, 0, -500);
    Advance(&f, 100);
    CHECK(f.speed[0] == -90 && f.speed[1] == 90 && f.speed[2] == 90 && f.speed[3] == -90);
    HostSend(&f, &service, HOST_VELOCITY, 0, 0);
    Advance(&f, 100);
    for (unsigned i = 0; i < 4; ++i) CHECK(f.enabled[i] && f.speed[i] == 0);
    /* Diagnostic parser error counts alone must not stop healthy motors. */
    ++f.bus.protocol_error_count;
    Advance(&f, 1);
    CHECK(f.control.status.state == CHASSIS_ACTIVE);
    uint32_t timeouts = f.bus.timeout_count;
    f.drop_next_query = true;
    Advance(&f, 150);
    CHECK(!f.drop_next_query && f.bus.timeout_count == timeouts + 1);
    CHECK(f.control.status.state == CHASSIS_ACTIVE);
    HostSend(&f, &service, HOST_STOP, 0, 0);
    uint32_t stop_ms = f.control.phase_ms;
    HostSend(&f, &service, HOST_STOP, 0, 0);
    CHECK(f.control.phase_ms == stop_ms);
    Advance(&f, 300);
    CHECK(f.control.status.state == CHASSIS_DISABLED && !f.bus.motion_allowed);
    for (unsigned i = 0; i < 4; ++i) CHECK(!f.enabled[i] && f.speed[i] == 0);

    /* Impossible driver commands remain rejected instead of overflowing fields. */
    CHECK(InitControl(&f, &service, 0));
    HostCommand large = {.type = HOST_VELOCITY, .sequence = 1,
        .vx_mm_s = INT32_MIN, .vy_mm_s = INT32_MAX, .wz_mrad_s = INT32_MIN};
    RobotHostCommand(&f.robot, &large, f.now);
    Advance(&f, 500);
    CHECK(f.control.status.state == CHASSIS_FAULT && f.control.status.fault == CHASSIS_MOTOR_ERROR);
    for (unsigned i = 0; i < 4; ++i) CHECK(!f.drove[i] && !f.enabled[i]);

    /* Compatibility ARM may be followed immediately by a nonzero target. */
    CHECK(InitControl(&f, &service, 0));
    HostSend(&f, &service, HOST_ARM, 1, 0);
    Advance(&f, 1);
    CHECK(f.control.status.state == CHASSIS_ENABLING);
    HostSend(&f, &service, HOST_VELOCITY, 1, 50);
    Advance(&f, 200);
    CHECK(f.control.status.state == CHASSIS_ACTIVE && f.speed[0] == 9);
    /* Device stall still stops all wheels and clears the held target. */
    f.stalled[2] = true;
    Advance(&f, 500);
    CHECK(f.control.status.state == CHASSIS_FAULT && !f.bus.motion_allowed);
    CHECK(f.control.status.fault == CHASSIS_MOTOR_ERROR);
    CHECK(!f.control.start_pending && !f.control.status.target.valid);
    f.stalled[2] = false;
    Advance(&f, 300);
    CHECK(f.control.status.state == CHASSIS_FAULT && !f.bus.motion_allowed);
    HostSend(&f, &service, HOST_VELOCITY, 0, 50);
    Advance(&f, 200);
    CHECK(f.control.status.state == CHASSIS_ACTIVE && f.speed[0] == 9);

    /* USB disconnect stops; reconnect never resumes the previous target. */
    CHECK(InitControl(&f, &service, 0xFFFFFE00));
    HostSend(&f, &service, HOST_VELOCITY, 0, -50);
    Advance(&f, 200);
    CHECK(f.control.status.state == CHASSIS_ACTIVE);
    BSPUSBConnected(false);
    HostServiceReceive(&service, f.now);
    CHECK(f.control.status.state == CHASSIS_STOPPING && f.control.status.fault == CHASSIS_LINK_LOST);
    Advance(&f, 300);
    BSPUSBConnected(true);
    HostSend(&f, &service, HOST_VELOCITY, 0, 50); /* Connection-change batch discarded. */
    CHECK(f.control.status.state == CHASSIS_FAULT && !f.control.start_pending);
    Advance(&f, 300);
    CHECK(!f.bus.motion_allowed);
    HostSend(&f, &service, HOST_VELOCITY, 0, 50);
    Advance(&f, 200);
    CHECK(f.control.status.state == CHASSIS_ACTIVE);
    /* STOP takes precedence over a velocity following it in the same batch. */
    uint8_t frames[36], payload[12] = {0};
    HostPutU32(payload, 50);
    CHECK(HostProtocolEncode(HOST_STOP, 0, NULL, 0, frames, 12) == 12);
    CHECK(HostProtocolEncode(HOST_VELOCITY, 0, payload, 12, frames + 12, 24) == 24);
    test_usb_clock = f.now;
    BSPUSBReceive(frames, sizeof(frames), f.now);
    HostServiceReceive(&service, f.now);
    Advance(&f, 300);
    CHECK(f.control.status.state == CHASSIS_DISABLED && !f.control.start_pending);

    /* First command can wait for initial device feedback without retransmit. */
    CHECK(InitControl(&f, &service, 0));
    for (unsigned i = 0; i < 4; ++i) f.motors[i].feedback.position_valid = false;
    HostSend(&f, &service, HOST_VELOCITY, 0, 50);
    Advance(&f, 200);
    CHECK(f.control.status.state == CHASSIS_ACTIVE && f.speed[0] == 9);
    /* A pending start is cancelled if the link disappears before readiness. */
    CHECK(InitControl(&f, &service, 0));
    for (unsigned i = 0; i < 4; ++i) f.motors[i].feedback.position_valid = false;
    HostSend(&f, &service, HOST_VELOCITY, 0, 50);
    CHECK(f.control.start_pending);
    BSPUSBConnected(false);
    HostServiceReceive(&service, f.now);
    Advance(&f, 300);
    CHECK(!f.control.start_pending && !f.bus.motion_allowed);

    CHECK(InitControl(&f, &service, 0));
    CHECK(ArmControl(&f, &service));
    uint8_t flood[512] = {0};
    BSPUSBReceive(flood, sizeof(flood), f.now);
    BSPUSBReceive(flood, 1, f.now);
    HostServiceReceive(&service, f.now);
    CHECK(f.control.status.state == CHASSIS_STOPPING && f.control.status.fault == CHASSIS_LINK_LOST);
    /* Host scheduling latency alone no longer discards a complete command. */
    CHECK(InitControl(&f, &service, 0));
    uint8_t delayed_frame[24], delayed_payload[12] = {0};
    HostPutU32(delayed_payload, 50);
    CHECK(HostProtocolEncode(HOST_VELOCITY, 0, delayed_payload, 12, delayed_frame, 24) == 24);
    BSPUSBReceive(delayed_frame, sizeof(delayed_frame), f.now - 500);
    test_usb_clock = f.now;
    HostServiceReceive(&service, f.now);
    Advance(&f, 200);
    CHECK(f.control.status.state == CHASSIS_ACTIVE && f.speed[0] == 9);
    /* Failed enable or device offline remains a genuine device fault. */
    CHECK(InitControl(&f, &service, 0));
    f.reject_enable = true;
    HostSend(&f, &service, HOST_VELOCITY, 0, 50);
    Advance(&f, 500);
    CHECK(f.control.status.state == CHASSIS_FAULT && !f.bus.motion_allowed);
    for (unsigned i = 0; i < 4; ++i) CHECK(!f.drove[i]);
    CHECK(InitControl(&f, &service, 0));
    f.hold_last_enable = true;
    HostSend(&f, &service, HOST_VELOCITY, 0, 50);
    Advance(&f, 900);
    CHECK(f.control.status.state == CHASSIS_FAULT && f.control.status.fault == CHASSIS_ENABLE_TIMEOUT);
    CHECK(InitControl(&f, &service, 0));
    HostSend(&f, &service, HOST_VELOCITY, 0, 50);
    Advance(&f, 200);
    f.drop_last_motor = true;
    Advance(&f, 400);
    CHECK(!f.bus.motion_allowed && f.control.status.state != CHASSIS_ACTIVE);

    CHECK(InitControl(&f, &service, 0));
    CHECK(!HostServiceSelectFeedback(&service, 2));
    CHECK(HostServiceSelectFeedback(&service, HOST_FEEDBACK_FIREWATER));
    BoardMotorStatus vofa_snapshot = {0};
    for (unsigned i = 0; i < 4; ++i)
        EmmMotorGetFeedback(&f.motors[i], f.now, &vofa_snapshot.wheels[i]);
    vofa_snapshot.wheels[0].position_raw = -21636;
    vofa_snapshot.wheels[0].wheel_speed_rad_s = -0.942f;
    vofa_snapshot.wheels[0].speed_valid = true;
    vofa_snapshot.wheels[1].wheel_speed_rad_s = 99.0f;
    vofa_snapshot.wheels[1].speed_valid = false;
    BSPUSBTxComplete();
    HostServiceTransmit(&service, &vofa_snapshot, f.now);
    CHECK(!memcmp(service.tx, "0,0,0,-0.942,0.000,", 18));
    CHECK(service.vofa_channels[7] == -21636 && service.vofa_channels[16] == 15);
    CHECK((service.vofa_channels[17] & 2) == 0 && (service.vofa_channels[17] & 1) == 1);
    unsigned commas = 0, cursor = 0;
    while (cursor < sizeof(service.tx) && service.tx[cursor] != '\r')
    {
        CHECK(service.tx[cursor] >= ' ' && service.tx[cursor] <= '~');
        commas += service.tx[cursor++] == ',';
    }
    CHECK(commas == 21 && cursor + 1 < sizeof(service.tx) && service.tx[cursor + 1] == '\n');
    /* Largest raw signed positions, counters and finite wire speed values
     * must fit the same 240-byte USB-owned buffer without truncation.
     */
    for (unsigned i = 0; i < 4; ++i)
    {
        vofa_snapshot.wheels[i].position_raw = INT64_MIN;
        vofa_snapshot.wheels[i].wheel_speed_rad_s = -2147482.0f;
        vofa_snapshot.wheels[i].speed_valid = true;
    }
    vofa_snapshot.transaction_timeouts = vofa_snapshot.protocol_errors =
        vofa_snapshot.rejected_commands = vofa_snapshot.transport_errors = vofa_snapshot.rx_errors = UINT32_MAX;
    BSPUSBTxComplete();
    uint32_t skipped = service.status.tx_skipped;
    HostServiceTransmit(&service, &vofa_snapshot, UINT32_MAX);
    CHECK(service.status.tx_skipped == skipped && service.tx[0] == '0');
    CHECK(HostServiceSelectFeedback(&service, HOST_FEEDBACK_BINARY));
    int result = RunPositionIntegrationTests();
    return result ? result : RunArrivalTests();
}


/* Offline only: actual protocol/application/odometry with simulated EMM drivers.
 * No host executable or firmware motion entry point is introduced.
 */
static bool InitPosition(Fixture *f, HostServiceInstance *service)
{
    if (!InitControl(f, service, 0)) return false;
    ChassisOdometry_Init_Config_s odometry_config = {
        .kinematics = &f->chassis_kinematics,
        .feedback_timeout_ms = 500,
        .max_sample_span_ms = 80,
    };
    for (unsigned i = 0; i < 4; ++i)
        odometry_config.wheel_rad_per_count[i] = -6.28318530718f / 65536.0f;
    ChassisPositionControl_Init_Config_s position_config = {
        .kp_xy = 0.5f, .kp_yaw = 1.0f,
        .position_tolerance_m = 0.02f, .yaw_tolerance_rad = 0.0174533f,
    };
    const RobotArrival_Init_Config_s arrival_config = {
        .linear_speed_m_s = 0.01f, .angular_speed_rad_s = 0.02f,
        .hold_ms = 300, .feedback_timeout_ms = 200, .max_sample_span_ms = 80,
    };
    if (!ChassisOdometryInit(&f->odometry, &odometry_config) ||
        !ChassisPositionControlInit(&f->position_control, &position_config) ||
        !RobotPositionInit(&f->robot, &f->odometry, &f->position_control, &arrival_config)) return false;
    Advance(f, 100);
    return f->odometry.pose_valid && !f->odometry.tracking_lost;
}

static void HostPosition(Fixture *f, HostServiceInstance *service,
                         uint32_t sequence, int32_t x, int32_t y, int32_t yaw)
{
    uint8_t frame[24], payload[12];
    HostPutU32(payload, (uint32_t)x);
    HostPutU32(payload + 4, (uint32_t)y);
    HostPutU32(payload + 8, (uint32_t)yaw);
    size_t length = HostProtocolEncode(HOST_POSITION, sequence, payload, 12, frame, 24);
    test_usb_clock = f->now;
    BSPUSBReceive(frame, length, f->now);
    HostServiceReceive(service, f->now);
}

static int RunPositionIntegrationTests(void)
{
    Fixture f;
    HostServiceInstance service;
    CHECK(InitPosition(&f, &service));
    CHECK(f.enable_count == 0 && f.robot.mode == ROBOT_IDLE);
    /* One USB goal converges with actual encoder feedback and no host refresh. */
    HostPosition(&f, &service, 12, 200, 0, 0);
    CHECK(f.robot.mode == ROBOT_POSITION);
    CHECK(fabsf(f.position_control.result.velocity.vx_m_s - 0.1f) < 0.00001f);
    Advance(&f, 7000);
    CHECK(f.robot.mode == ROBOT_POSITION && f.control.status.state == CHASSIS_ACTIVE);
    CHECK(f.position_control.result.within_tolerance);
    CHECK(fabsf(f.odometry.pose.x_m - 0.2f) < 0.021f);
    CHECK(fabsf(f.odometry.pose.y_m) < 0.005f);
    CHECK(f.control.status.last_sequence == 12);
    CHECK(f.robot.arrival.state == ROBOT_ARRIVAL_REACHED && f.robot.arrival.stable_ms == 300);
    uint32_t accepted_ms = f.control.status.last_command_ms;
    Advance(&f, 1000);
    CHECK(f.control.status.last_command_ms == accepted_ms);
    float previous_x = f.odometry.pose.x_m;
    /* Repeated absolute coordinate does not add another 0.2m to the trip. */
    HostPosition(&f, &service, 12, 200, 0, 0);
    Advance(&f, 500);
    CHECK(fabsf(f.odometry.pose.x_m - previous_x) < 0.005f);
    for (unsigned i = 0; i < 4; ++i) CHECK(f.enabled[i] && f.speed[i] == 0);

    /* STOP cancels goal; no new enable even after stop confirmation completes. */
    HostPosition(&f, &service, 2, 500, 0, 0);
    Advance(&f, 150);
    HostSend(&f, &service, HOST_STOP, 0, 0);
    CHECK(f.robot.mode == ROBOT_IDLE);
    CHECK(f.robot.arrival.state == ROBOT_ARRIVAL_NONE && f.robot.arrival.stable_ms == 0);
    Advance(&f, 500);
    uint32_t enables = f.enable_count;
    Advance(&f, 1000);
    CHECK(f.enable_count == enables && f.control.status.state == CHASSIS_DISABLED);
    for (unsigned i = 0; i < 4; ++i) CHECK(!f.enabled[i]);

    /* Zero VELOCITY changes source and cancels position corrections. */
    CHECK(InitPosition(&f, &service));
    HostPosition(&f, &service, 1, 200, 0, 0);
    Advance(&f, 200);
    HostSend(&f, &service, HOST_VELOCITY, 3, 0);
    Advance(&f, 300);
    CHECK(f.robot.mode == ROBOT_VELOCITY);
    for (unsigned i = 0; i < 4; ++i) CHECK(f.speed[i] == 0);
    CHECK(f.control.status.last_sequence == 3);

    /* Link fault clears application goal; reconnect alone cannot restart. */
    CHECK(InitPosition(&f, &service));
    HostPosition(&f, &service, 1, 200, 0, 0);
    Advance(&f, 200);
    BSPUSBConnected(false);
    HostServiceReceive(&service, f.now);
    CHECK(f.robot.mode == ROBOT_IDLE);
    Advance(&f, 500);
    enables = f.enable_count;
    BSPUSBConnected(true);
    HostServiceReceive(&service, f.now);
    Advance(&f, 800);
    CHECK(f.robot.mode == ROBOT_IDLE && !f.bus.motion_allowed && f.enable_count == enables);
    HostPosition(&f, &service, 4, 200, 0, 0);
    Advance(&f, 200);
    CHECK(f.robot.mode == ROBOT_POSITION && f.control.status.state == CHASSIS_ACTIVE);

    /* A device fault cancels goal and cannot be renewed by internal control. */
    f.stalled[2] = true;
    Advance(&f, 500);
    CHECK(f.robot.mode == ROBOT_IDLE && f.control.status.state == CHASSIS_FAULT);
    CHECK(f.control.status.fault == CHASSIS_MOTOR_ERROR);
    f.stalled[2] = false;
    enables = f.enable_count;
    Advance(&f, 800);
    CHECK(f.enable_count == enables && !f.bus.motion_allowed);

    /* Loss of pose cancels goal, rather than holding the old nonzero velocity. */
    CHECK(InitPosition(&f, &service));
    HostPosition(&f, &service, 1, 200, 0, 0);
    Advance(&f, 200);
    f.odometry.tracking_lost = true;
    Advance(&f, 500);
    CHECK(f.robot.mode == ROBOT_IDLE && f.control.status.fault == CHASSIS_NOT_READY);
    CHECK(!f.bus.motion_allowed);

    /* Translation and yaw can converge together, using the rotating frame. */
    CHECK(InitPosition(&f, &service));
    HostPosition(&f, &service, 7, 0, 200, 1571);
    Advance(&f, 8000);
    CHECK(f.robot.mode == ROBOT_POSITION && f.position_control.result.within_tolerance);
    CHECK(fabsf(f.odometry.pose.x_m) < 0.022f);
    CHECK(fabsf(f.odometry.pose.y_m - 0.2f) < 0.022f);
    CHECK(fabsf(f.odometry.pose.yaw_rad - 1.571f) < 0.018f);

    /* Shortest yaw and a ninety-degree body/world conversion. */
    ChassisPose current = {.yaw_rad = 1.570796327f};
    ChassisPose target = {.x_m = 0.4f, .yaw_rad = 1.570796327f};
    CHECK(ChassisPositionControlCalculate(&f.position_control, &current, &target));
    CHECK(fabsf(f.position_control.result.velocity.vx_m_s) < 0.00001f);
    CHECK(fabsf(f.position_control.result.velocity.vy_m_s + 0.2f) < 0.00001f);
    current = (ChassisPose){.yaw_rad = 3.12413936f};
    target = (ChassisPose){.yaw_rad = -3.12413936f};
    CHECK(ChassisPositionControlCalculate(&f.position_control, &current, &target));
    CHECK(fabsf(f.position_control.result.velocity.wz_rad_s - 0.0349066f) < 0.00001f);
    return 0;
}

/* 离线注入采样：用于区分“目标零速”和“真实仍在运动”，不属于固件操作入口。
 * 底盘实际仲裁仍走RobotUpdate，协议反馈仍走HostServiceTransmit。
 */
static void ArrivalSample(Fixture *f, float vx, float wz, bool speed_valid, bool fresh)
{
    f->now += 50;
    f->odometry.last_update_ms = f->now;
    OmniBodyVelocity body = {.vx_m_s = vx, .wz_rad_s = wz};
    OmniKinematicsTarget target;
    (void)OmniKinematicsInverse(&f->chassis_kinematics, &body, &target);
    for (unsigned i = 0; i < 4; ++i)
    {
        EmmMotorFeedback *feedback = &f->motors[i].feedback;
        feedback->enabled_ms = feedback->stall_ms = f->now;
        if (fresh) feedback->position_ms = f->now;
        feedback->speed_valid = speed_valid;
        feedback->wheel_speed_rad_s = target.wheel_rad_s[i];
    }
    RobotUpdate(&f->robot, f->now);
}

static int RunArrivalTests(void)
{
    Fixture f;
    HostServiceInstance service;
    CHECK(InitPosition(&f, &service));
    HostPosition(&f, &service, 21, 0, 0, 0);
    Advance(&f, 1000);
    CHECK(f.robot.arrival.state == ROBOT_ARRIVAL_REACHED);
    uint32_t generation = f.robot.goal_generation;
    /* 高频重发同坐标不应饿死50ms更新，也不能清除已到达状态。 */
    for (unsigned i = 0; i < 60; ++i)
    {
        HostPosition(&f, &service, 22, 0, 0, 0);
        Advance(&f, 10);
    }
    CHECK(f.robot.arrival.state == ROBOT_ARRIVAL_REACHED);
    CHECK(f.robot.goal_generation == generation && f.robot.position_sequence == 22);
    HostPosition(&f, &service, 22, 1, 0, 0);
    CHECK(f.robot.goal_generation == generation + 1);
    CHECK(f.robot.arrival.state == ROBOT_ARRIVAL_APPROACHING && f.robot.arrival.stable_ms == 0);
    /* 新坐标虽已在容差内，也必须从头确认稳定。 */
    for (unsigned i = 0; i < 6; ++i) ArrivalSample(&f, 0, 0, true, true);
    CHECK(f.position_control.result.within_tolerance);
    CHECK(f.robot.arrival.state == ROBOT_ARRIVAL_SETTLING && f.robot.arrival.stable_ms == 250);
    ArrivalSample(&f, 0, 0, true, true);
    CHECK(f.robot.arrival.state == ROBOT_ARRIVAL_REACHED);
    ArrivalSample(&f, 0.02f, 0, true, true);
    CHECK(f.robot.arrival.state == ROBOT_ARRIVAL_APPROACHING);
    CHECK(f.control.status.state == CHASSIS_ACTIVE && f.robot.arrival.measured_valid);
    CHECK(f.position_control.result.velocity.vx_m_s == 0); /* 目标零速不代表已停稳。 */
    ArrivalSample(&f, 0, 0.03f, true, true);
    CHECK(f.robot.arrival.state == ROBOT_ARRIVAL_APPROACHING);
    ArrivalSample(&f, 0, 0, true, true);
    CHECK(f.robot.arrival.state == ROBOT_ARRIVAL_SETTLING && f.robot.arrival.stable_ms == 0);
    for (unsigned i = 0; i < 3; ++i) ArrivalSample(&f, 0, 0, true, false);
    CHECK(f.robot.arrival.stable_ms == 0); /* 重复缓存不能计时。 */
    ArrivalSample(&f, 0, 0, true, false);
    CHECK(!f.robot.arrival.measured_valid && f.robot.arrival.state == ROBOT_ARRIVAL_APPROACHING);
    for (unsigned i = 0; i < 7; ++i) ArrivalSample(&f, 0, 0, true, true);
    CHECK(f.robot.arrival.state == ROBOT_ARRIVAL_REACHED);
    ArrivalSample(&f, 0, 0, false, true);
    CHECK(!f.robot.arrival.measured_valid && f.robot.arrival.stable_ms == 0);
    CHECK(f.control.status.state == CHASSIS_ACTIVE); /* 判定失效不新增停车动作。 */
    for (unsigned i = 0; i < 7; ++i) ArrivalSample(&f, 0, 0, true, true);
    CHECK(f.robot.arrival.state == ROBOT_ARRIVAL_REACHED);
    f.odometry.pose.x_m = 0.1f;
    ArrivalSample(&f, 0, 0, true, true);
    CHECK(!f.position_control.result.within_tolerance);
    CHECK(f.robot.arrival.state == ROBOT_ARRIVAL_APPROACHING);
    f.odometry.pose.x_m = 0;
    /* 毫秒计数回绕时仍按实际采样支持的300ms确认。 */
    f.now = UINT32_MAX - 125;
    f.robot.last_position_update_ms = f.now;
    for (unsigned i = 0; i < 7; ++i) ArrivalSample(&f, 0, 0, true, true);
    CHECK(f.robot.arrival.state == ROBOT_ARRIVAL_REACHED && f.robot.arrival.stable_ms == 300);

    BoardMotorStatus snapshot = {0};
    for (unsigned i = 0; i < 4; ++i)
        EmmMotorGetFeedback(&f.motors[i], f.now, &snapshot.wheels[i]);
    BSPUSBTxComplete();
    service.last_tx_ms = f.now - 25;
    HostServiceTransmit(&service, &snapshot, f.now);
    CHECK(service.tx[3] == HOST_STATUS && service.tx[4] == 196 && service.next_pose);
    CHECK(HostProtocolCRC(service.tx, 206) == ((uint16_t)service.tx[206] | (uint16_t)service.tx[207] << 8));
    /* 不完成上次IN传输，下一次只能失败，不能跳过POSE或递增成功序号。 */
    uint32_t skipped = service.status.tx_skipped, sequence = service.tx_sequence;
    HostServiceTransmit(&service, &snapshot, f.now + 25);
    CHECK(service.next_pose && service.status.tx_skipped == skipped + 1 && service.tx_sequence == sequence);
    BSPUSBTxComplete();
    HostServiceTransmit(&service, &snapshot, f.now + 50);
    CHECK(service.tx[3] == HOST_POSE && service.tx[4] == 64 && !service.next_pose);
    CHECK(HostGetU32(service.tx + 6) == sequence);
    const uint8_t *p = service.tx + 10;
    CHECK(HostGetU32(p + 8) == 22 && HostGetU32(p + 12) == f.robot.goal_generation);
    CHECK(p[16] == ROBOT_POSITION && p[17] == ROBOT_ARRIVAL_REACHED && p[18] == 15);
    CHECK((int32_t)HostGetU32(p + 20) == 0 && (int32_t)HostGetU32(p + 32) == 1);
    CHECK(HostGetU32(p + 56) == 300 && HostGetU32(p + 60) == 300);
    CHECK(HostProtocolCRC(service.tx, 74) == ((uint16_t)service.tx[74] | (uint16_t)service.tx[75] << 8));
    /* 无效位姿必须给出明确标记，不发送旧坐标冒充有效定位。 */
    f.odometry.tracking_lost = true;
    BSPUSBTxComplete();
    HostServiceTransmit(&service, &snapshot, f.now + 75);
    BSPUSBTxComplete();
    HostServiceTransmit(&service, &snapshot, f.now + 100);
    p = service.tx + 10;
    CHECK(service.tx[3] == HOST_POSE && !(p[18] & 1));
    CHECK((int32_t)HostGetU32(p + 20) == INT32_MIN);
    RobotStop(&f.robot, CHASSIS_OK, f.now);
    CHECK(f.robot.arrival.state == ROBOT_ARRIVAL_NONE);
    CHECK(f.robot.arrival.stable_ms == 0 && !f.robot.arrival.measured_valid);
    return 0;
}
