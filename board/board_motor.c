#include "board_motor.h"
#include "bsp_uart.h"
#include "usart.h"

static BSPUARTInstance motor_uart;
static EmmBusInstance motor_bus;
static EmmMotorInstance motors[BOARD_WHEEL_COUNT];
static bool initialized;
static uint32_t last_query_ms;
static uint8_t query_slot;

/* 240 mm dimensions are user-specified placeholders, not measurements.
 * Installation signs were verified with bounded forward, left and yaw tests.
 * Apply the same signs to SI targets AND measured wheel angles/speeds.
 */
static const BoardChassisGeometry geometry = {
    .wheel_radius_m = 0.0375f,
    .wheelbase_m = 0.240f,
    .track_width_m = 0.240f,
    .dimensions_measured = false,
    .directions_calibrated = true, /* User verified forward, left, positive yaw. */
};
static const uint8_t addresses[BOARD_WHEEL_COUNT] = {1U, 2U, 3U, 4U};
static const int8_t directions[BOARD_WHEEL_COUNT] = {-1, -1, -1, -1};

_Static_assert((int)BOARD_WHEEL_FRONT_LEFT == (int)OMNI_FRONT_LEFT &&
               (int)BOARD_WHEEL_FRONT_RIGHT == (int)OMNI_FRONT_RIGHT &&
               (int)BOARD_WHEEL_REAR_RIGHT == (int)OMNI_REAR_RIGHT &&
               (int)BOARD_WHEEL_REAR_LEFT == (int)OMNI_REAR_LEFT &&
               (int)BOARD_WHEEL_COUNT == (int)OMNI_WHEEL_COUNT, "Chassis wheel order mismatch");

bool BoardChassisKinematicsInit(OmniKinematicsInstance *instance)
{
    const OmniKinematics_Init_Config_s config = {
        .wheel_radius_m = geometry.wheel_radius_m,
        .wheelbase_m = geometry.wheelbase_m,
        .track_width_m = geometry.track_width_m,
    };
    return OmniKinematicsInit(instance, &config);
}

static bool Send(void *context, const uint8_t *data, size_t length, uint32_t now_ms)
{
    return BSPUARTSend(context, data, length, now_ms);
}

static bool TxBusy(void *context)
{
    return BSPUARTTxBusy(context);
}

bool BoardMotorInit(uint32_t now_ms)
{
    if (initialized)
    {
        return false;
    }
    const BSPUART_Init_Config_s uart_config = {
        .handle = &huart5, .tx_timeout_ms = 20U,
    };
    const EmmBus_Init_Config_s bus_config = {
        .send = Send, .tx_busy = TxBusy, .context = &motor_uart,
        .response_timeout_ms = 20U, .quiet_ms = 2U, .quarantine_ms = 40U,
        .command_timeout_ms = 200U, .feedback_timeout_ms = BOARD_MOTOR_FEEDBACK_TIMEOUT_MS,
    };
    if (!BSPUARTInit(&motor_uart, &uart_config) || !EmmBusInit(&motor_bus, &bus_config, now_ms))
    {
        return false;
    }
    for (uint8_t i = 0; i < BOARD_WHEEL_COUNT; ++i)
    {
        const EmmMotor_Init_Config_s config = {
            .bus = &motor_bus, .address = addresses[i], .direction = directions[i],
            .gear_ratio = 1.0f, .wheel_rad_s_per_speed_unit = 0.10471975512f,
        };
        if (!EmmMotorInit(&motors[i], &config))
        {
            return false;
        }
    }
    last_query_ms = now_ms;
    initialized = true;
    return true;
}

bool BoardMotorStart(uint32_t now_ms)
{
    return initialized && BSPUARTStart(&motor_uart, now_ms);
}

bool BoardChassisControlInit(ChassisControlInstance *control, OmniKinematicsInstance *kinematics)
{
    if (!initialized || !geometry.directions_calibrated) return false;
    const ChassisControl_Init_Config_s config = {
        .bus = &motor_bus, .motors = {&motors[0], &motors[1], &motors[2], &motors[3]},
        .kinematics = kinematics,
    };
    return ChassisControlInit(control, &config);
}

void BoardMotorUpdate(uint32_t now_ms)
{
    if (!initialized)
    {
        return;
    }
    if (!BSPUARTService(&motor_uart, now_ms))
    {
        EmmBusTransportFault(&motor_bus, now_ms);
        return;
    }
    /* At most one complete software-ring capacity per iteration: bounded even
     * when unsolicited bytes arrive. ISR never executes protocol parsing.
     */
    uint8_t received[64];
    for (uint8_t i = 0; i < BSP_UART_RX_SIZE / sizeof(received); ++i)
    {
        size_t length = BSPUARTRead(&motor_uart, received, sizeof(received));
        if (motor_uart.fault)
        {
            EmmBusTransportFault(&motor_bus, now_ms);
            return;
        }
        if (length == 0U)
        {
            break;
        }
        EmmBusReceive(&motor_bus, received, length, now_ms);
    }
    /* Nominal query schedule: one request every 5 ms; 8 slots per round.
     * Timeout/quarantine may lengthen it. Actual sample timestamps, not this
     * nominal period, are used for position-difference speed estimates.
     */
    if ((uint32_t)(now_ms - last_query_ms) >= 5U)
    {
        static const EmmTransactionKind queries[] = {
            EMM_QUERY_POSITION, EMM_QUERY_STATUS,
        };
        uint8_t motor_index = query_slot % BOARD_WHEEL_COUNT;
        (void)EmmMotorQuery(&motors[motor_index], queries[query_slot / BOARD_WHEEL_COUNT]);
        query_slot = (uint8_t)((query_slot + 1U) % (BOARD_WHEEL_COUNT * 2U));
        last_query_ms = now_ms;
    }
    EmmBusUpdate(&motor_bus, now_ms);
}

void BoardMotorGetStatus(BoardMotorStatus *status, uint32_t now_ms)
{
    if (status == NULL)
    {
        return;
    }
    *status = (BoardMotorStatus){0};
    status->timestamp_ms = now_ms;
    if (!initialized)
    {
        return;
    }
    for (uint8_t i = 0; i < BOARD_WHEEL_COUNT; ++i)
    {
        (void)EmmMotorGetFeedback(&motors[i], now_ms, &status->wheels[i]);
    }
    status->motion_allowed = motor_bus.motion_allowed;
    status->transaction_timeouts = motor_bus.timeout_count;
    status->protocol_errors = motor_bus.protocol_error_count;
    status->rejected_commands = motor_bus.rejected_command_count;
    status->last_rejected_address = motor_bus.last_rejected_address;
    status->last_rejected_function = motor_bus.last_rejected_function;
    status->last_rejected_status = motor_bus.last_rejected_status;
    status->transport_errors = motor_bus.transport_error_count;
    status->rx_errors = motor_uart.error_count;
}

const BoardChassisGeometry *BoardMotorGetGeometry(void)
{
    return &geometry;
}

bool BoardChassisOdometryInit(
    ChassisOdometryInstance *odometry,
    const OmniKinematicsInstance *kinematics)
{
    if (!initialized)
    {
        return false;
    }

    ChassisOdometry_Init_Config_s config = {
        .kinematics = kinematics,
        .feedback_timeout_ms = BOARD_MOTOR_FEEDBACK_TIMEOUT_MS,
        .max_sample_span_ms = 80U,
    };

    /*
     * 当前驱动模块暂按65536计数/电机轴一圈处理。
     * 必须与emm_motor.c中的位置换算一致。
     * 后续标定每圈比例时，两处需要同步调整。
     */
    const float rad_per_count = 6.2831853071795864769f / 65536.0f;

    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
    {
        /*
         * 使用电机实例中的实际配置。
         *
         * 安装方向目前均为-1，此处已经应用一次。
         * 后面里程计解算时不要再次反转。
         */
        config.wheel_rad_per_count[i] = rad_per_count *
            motors[i].config.direction /
            motors[i].config.gear_ratio;
    }

    return ChassisOdometryInit(odometry, &config);
}
/* 初始调试参数：不代表实际定位精度，后续按运行结果调整。 */
bool BoardChassisPositionControlInit(ChassisPositionControlInstance *position_control)
{
    const ChassisPositionControl_Init_Config_s config = {
        .kp_xy = 0.5f,                 /* 0.2m位置误差产生0.1m/s平移速度。 */
        .kp_yaw = 1.0f,                /* 0.1rad角度误差产生0.1rad/s角速度。 */
        .position_tolerance_m = 0.005f, /* 当前源码为5mm容差，按平面距离判断；本步保留。 */
        .yaw_tolerance_rad = 0.0174533f, /* 朝向容差约1度。 */
    };
    return ChassisPositionControlInit(position_control, &config);
}

bool BoardRobotPositionInit(RobotInstance *robot, ChassisOdometryInstance *odometry,
                            ChassisPositionControlInstance *position_control)
{
    const RobotArrival_Init_Config_s config = {
        .linear_speed_m_s = 0.01f,    /* 实测平移速率<=10mm/s才可确认到达。 */
        .angular_speed_rad_s = 0.02f, /* 实测角速度绝对值<=20mrad/s。 */
        .hold_ms = 300U,             /* 连续稳定300ms；不是命令超时。 */
        .feedback_timeout_ms = 200U, /* 旧轮速不能用作静稳证据。 */
        .max_sample_span_ms = 80U,   /* 与现有四轮速度反馈时间差约定一致。 */
    };
    return RobotPositionInit(robot, odometry, position_control, &config);
}
