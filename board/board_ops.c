#include "board_ops.h"
#include "bsp_uart.h"
#include "usart.h"

/* DMA缓冲必须在普通SRAM内，不能放到DMA无法访问的CCMRAM。 */
static BSPUARTInstance ops_uart;
static OPSInstance ops;
static bool initialized;
static uint32_t rx_bytes;

bool BoardOPSInit(void)
{
    if (initialized) return false;
    const OPS_Init_Config_s ops_config = {
        .feedback_timeout_ms = BOARD_OPS_FEEDBACK_TIMEOUT_MS, /* 100Hz定位反馈的新鲜度。 */
    };
    const BSPUART_Init_Config_s uart_config = {
        .handle = &huart4,
        .receive_only = true, /* OPS只需接收，不要求为UART4配置TX DMA。 */
    };
    if (!OPSInit(&ops, &ops_config) || !BSPUARTInit(&ops_uart, &uart_config)) return false;
    initialized = true;
    return true;
}

bool BoardOPSStart(uint32_t now_ms)
{
    return initialized && BSPUARTStart(&ops_uart, now_ms);
}

void BoardOPSUpdate(uint32_t now_ms)
{
    if (!initialized) return;
    /* BSP统一拥有HAL UART回调；本文件不能再定义同名HAL回调。
     * 服务发现错误时，中止/重启DMA由BSP完成，协议层丢弃残留半帧。
     */
    if (!BSPUARTService(&ops_uart, now_ms))
    {
        OPSTransportFault(&ops);
        OPSUpdate(&ops, now_ms);
        return;
    }
    uint8_t received[64];
    /* 每个周期最多取256字节，不随外部连续输入无限循环。
     * 通常1ms只有约12字节；空闲/HT/TC中断负责及时搬运DMA缓冲。
     */
    for (size_t i = 0U; i < BSP_UART_RX_SIZE / sizeof(received); ++i)
    {
        size_t length = BSPUARTRead(&ops_uart, received, sizeof(received));
        if (ops_uart.fault)
        {
            OPSTransportFault(&ops);
            break;
        }
        if (length == 0U) break;
        rx_bytes += (uint32_t)length;
        OPSReceive(&ops, received, length, now_ms);
    }
    OPSUpdate(&ops, now_ms);
}

void BoardOPSGetStatus(BoardOPSStatus *status)
{
    if (status == NULL) return;
    *status = (BoardOPSStatus){
        .feedback = ops.feedback,
        .rx_bytes = rx_bytes,
        .uart_errors = ops_uart.error_count,
        .initialized = initialized,
        .uart_running = ops_uart.running && !ops_uart.fault,
    };
}
