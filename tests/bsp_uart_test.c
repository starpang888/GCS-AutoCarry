#include "bsp_uart.h"
#include <string.h>

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

static bool fail_rx;
static bool fail_tx;
static bool fail_abort;
static uint8_t *active_tx;
static uint16_t active_tx_length;

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *handle, uint8_t *data, uint16_t size)
{
    (void)data;
    handle->hdmarx->remaining = size;
    return fail_rx ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *handle, uint8_t *data, uint16_t size)
{
    (void)handle;
    active_tx = data;
    active_tx_length = size;
    return fail_tx ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *handle)
{
    (void)handle;
    return fail_abort ? HAL_ERROR : HAL_OK;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *handle, uint16_t size);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *handle);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle);

int RunUARTTests(void)
{
    static BSPUARTInstance uart;
    static BSPUARTInstance duplicate;
    static DMA_HandleTypeDef rx_dma = {.Init.Mode = DMA_CIRCULAR};
    static DMA_HandleTypeDef tx_dma = {.Init.Mode = DMA_NORMAL};
    static UART_HandleTypeDef handle = {.hdmarx = &rx_dma, .hdmatx = &tx_dma};
    BSPUART_Init_Config_s config = {.handle = &handle, .tx_timeout_ms = 20U};
    CHECK(BSPUARTInit(&uart, &config));
    CHECK(!BSPUARTInit(&duplicate, &config));
    CHECK(!BSPUARTInit(&uart, &config));
    CHECK(BSPUARTStart(&uart, 0U));
    CHECK(!BSPUARTStart(&uart, 0U));
    CHECK(BSPUARTService(&uart, 0U));

    for (uint16_t i = 0; i < BSP_UART_DMA_SIZE; ++i)
    {
        uart.dma_rx[i] = (uint8_t)i;
    }
    uint8_t data[256];
    rx_dma.remaining = BSP_UART_DMA_SIZE - 64U;
    HAL_UARTEx_RxEventCallback(&handle, 64U);
    CHECK(BSPUARTRead(&uart, data, sizeof(data)) == 64U);
    for (uint16_t i = 0; i < 64U; ++i)
    {
        CHECK(data[i] == (uint8_t)i);
    }
    rx_dma.remaining = BSP_UART_DMA_SIZE; /* TC: DMA has wrapped to position 0. */
    HAL_UARTEx_RxEventCallback(&handle, 128U);
    CHECK(BSPUARTRead(&uart, data, sizeof(data)) == 64U);
    CHECK(data[0] == 64U && data[63] == 127U);
    HAL_UARTEx_RxEventCallback(&handle, 128U); /* Duplicate IDLE/TC: no duplicate bytes. */
    CHECK(BSPUARTRead(&uart, data, sizeof(data)) == 0U);
    rx_dma.remaining = BSP_UART_DMA_SIZE - 12U;
    HAL_UARTEx_RxEventCallback(&handle, 64U); /* Stale HT Size, current DMA pos is 12. */
    CHECK(BSPUARTRead(&uart, data, sizeof(data)) == 12U);
    CHECK(data[0] == 0U && data[11] == 11U);

    uint8_t command[] = {1U, 0x36U, 0x6BU};
    CHECK(BSPUARTSend(&uart, command, sizeof(command), 1U));
    command[1] = 0U;
    CHECK(active_tx_length == 3U && active_tx[1] == 0x36U); /* Owner copy. */
    CHECK(!BSPUARTSend(&uart, command, sizeof(command), 1U));
    CHECK(BSPUARTTxBusy(&uart));
    HAL_UART_TxCpltCallback(&handle);
    CHECK(!BSPUARTTxBusy(&uart));
    CHECK(BSPUARTSend(&uart, command, sizeof(command), 0xFFFFFFF6U));
    CHECK(BSPUARTService(&uart, 9U));
    CHECK(!BSPUARTService(&uart, 10U)); /* 20 ms TX deadline across wrap. */
    CHECK(!BSPUARTTxBusy(&uart) && uart.running);

    /* Four half-buffer chunks fit; fifth must report overflow, not overwrite. */
    for (uint8_t i = 0; i < 5U; ++i)
    {
        rx_dma.remaining = (i % 2U) == 0U ? 64U : 128U;
        HAL_UARTEx_RxEventCallback(&handle, (i % 2U) == 0U ? 64U : 128U);
    }
    CHECK(uart.fault && uart.error_count == 1U);
    CHECK(!BSPUARTService(&uart, 11U));
    CHECK(BSPUARTRead(&uart, data, sizeof(data)) == 0U);
    CHECK(BSPUARTService(&uart, 12U));
    HAL_UART_ErrorCallback(&handle);
    fail_abort = true;
    CHECK(!BSPUARTService(&uart, 13U));
    CHECK(!BSPUARTSend(&uart, command, sizeof(command), 13U));
    fail_abort = false;
    fail_rx = true;
    CHECK(!BSPUARTService(&uart, 14U));
    CHECK(!uart.running && uart.fault);
    fail_rx = false;
    CHECK(!BSPUARTService(&uart, 15U));
    CHECK(BSPUARTService(&uart, 16U));
    fail_tx = true;
    CHECK(!BSPUARTSend(&uart, command, sizeof(command), 17U));
    CHECK(uart.fault);
    fail_tx = false;
    CHECK(!BSPUARTService(&uart, 18U));
    CHECK(BSPUARTService(&uart, 19U));
    return 0;
}

int RunEmmTests(void);
int RunOmniKinematicsTests(void);
int RunHostLinkTests(void);
int RunHostControlTests(void);

int RunMotorTests(void)
{
    int result = RunHostLinkTests();
    if (result) return result + 60000;
    result = RunHostControlTests();
    if (result) return result + 50000;
    result = RunOmniKinematicsTests();
    if (result != 0)
    {
        return result + 30000;
    }
    result = RunEmmTests();
    if (result != 0)
    {
        return result;
    }
    result = RunUARTTests();
    return result == 0 ? 0 : result + 10000;
}
