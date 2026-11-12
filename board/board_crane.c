#include "board_crane.h"
#include "bsp_uart.h"
#include "usart.h"

typedef struct
{
    BSPUARTInstance uart; /* 普通SRAM静态存储，DMA不可放入CCMRAM。 */
    XMotorInstance motor;
    bool initialized;
} BoardCraneAxisInstance;

static BoardCraneAxisInstance axes[BOARD_CRANE_AXIS_COUNT];
static bool init_attempted;

static bool Send(void *context, const uint8_t *data, size_t length, uint32_t now_ms)
{
    return BSPUARTSend((BSPUARTInstance *)context, data, length, now_ms);
}

static bool TxBusy(void *context)
{
    return BSPUARTTxBusy((const BSPUARTInstance *)context);
}

bool BoardCraneInit(uint32_t now_ms)
{
    if (init_attempted) return false;
    init_attempted = true;
    UART_HandleTypeDef *const handles[BOARD_CRANE_AXIS_COUNT] = {&huart6, &huart3};
    bool success = true;
    for (size_t i = 0U; i < BOARD_CRANE_AXIS_COUNT; ++i)
    {
        BoardCraneAxisInstance *axis = &axes[i];
        const BSPUART_Init_Config_s uart_config = {
            .handle = handles[i], .tx_timeout_ms = 20U,
        };
        const XMotor_Init_Config_s motor_config = {
            .send = Send, .tx_busy = TxBusy, .context = &axis->uart,
            .address = 1U,
            .query_period_ms = 25U, /* 正常每轴位置/状态各约20Hz。 */
            .response_timeout_ms = 20U,
            .quiet_ms = 2U,
            .quarantine_ms = 40U,
            .feedback_timeout_ms = BOARD_CRANE_FEEDBACK_TIMEOUT_MS,
            .position_tolerance_units = BOARD_CRANE_POSITION_TOLERANCE_UNITS,
        };
        axis->initialized = XMotorInit(&axis->motor, &motor_config, now_ms) &&
                            BSPUARTInit(&axis->uart, &uart_config);
        success &= axis->initialized;
    }
    return success;
}

bool BoardCraneRobotInit(RobotInstance *robot)
{
    const XMotorMotionProfile profiles[BOARD_CRANE_AXIS_COUNT] = {
        {BOARD_CRANE_YAW_SPEED_DECI_RPM, BOARD_CRANE_YAW_ACCELERATION_RPM_S,
         BOARD_CRANE_YAW_DECELERATION_RPM_S},
        {BOARD_CRANE_Z_SPEED_DECI_RPM, BOARD_CRANE_Z_ACCELERATION_RPM_S,
         BOARD_CRANE_Z_DECELERATION_RPM_S},
    };
    return RobotCraneInit(robot, &axes[0].motor, &axes[1].motor, profiles);
}

bool BoardCraneStart(uint32_t now_ms)
{
    bool success = true;
    for (size_t i = 0U; i < BOARD_CRANE_AXIS_COUNT; ++i)
    {
        bool started = axes[i].initialized && BSPUARTStart(&axes[i].uart, now_ms);
        success &= started;
    }
    return success;
}

void BoardCraneUpdate(uint32_t now_ms)
{
    for (size_t i = 0U; i < BOARD_CRANE_AXIS_COUNT; ++i)
    {
        BoardCraneAxisInstance *axis = &axes[i];
        if (!axis->initialized) continue;
        if (!BSPUARTService(&axis->uart, now_ms))
        {
            XMotorTransportFault(&axis->motor, now_ms);
            continue;
        }
        uint8_t received[64];
        for (size_t chunk = 0U; chunk < BSP_UART_RX_SIZE / sizeof(received); ++chunk)
        {
            size_t length = BSPUARTRead(&axis->uart, received, sizeof(received));
            if (axis->uart.fault)
            {
                XMotorTransportFault(&axis->motor, now_ms);
                break;
            }
            if (length == 0U) break;
            XMotorReceive(&axis->motor, received, length, now_ms);
        }
        if (!axis->uart.fault) XMotorUpdate(&axis->motor, now_ms);
    }
}

void BoardCraneGetStatus(BoardCraneStatus *status, uint32_t now_ms)
{
    if (status == NULL) return;
    *status = (BoardCraneStatus){.timestamp_ms = now_ms};
    for (size_t i = 0U; i < BOARD_CRANE_AXIS_COUNT; ++i)
    {
        BoardCraneAxisInstance *axis = &axes[i];
        BoardCraneAxisStatus *snapshot = &status->axes[i];
        snapshot->initialized = axis->initialized;
        if (!axis->initialized) continue;
        (void)XMotorGetFeedback(&axis->motor, now_ms, &snapshot->feedback);
        snapshot->diagnostics = axis->motor.diagnostics;
        (void)XMotorGetMotion(&axis->motor, now_ms, &snapshot->motion);
        snapshot->uart_errors = axis->uart.error_count;
        snapshot->uart_running = axis->uart.running && !axis->uart.fault;
    }
}
