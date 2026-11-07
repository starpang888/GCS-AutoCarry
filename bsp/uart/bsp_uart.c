/* UART 实现：固定实例表、循环接收 DMA、软件环形缓存和任务内恢复。 */
#include "bsp_uart.h"
#include <string.h>

#define BSP_UART_CAPACITY 2U
static BSPUARTInstance *instances[BSP_UART_CAPACITY];

/** @brief 由 HAL 句柄查找已注册实例，供中断回调分发。 */
static BSPUARTInstance *FindUART(UART_HandleTypeDef *handle)
{
    for (size_t i = 0; i < BSP_UART_CAPACITY; ++i)
    {
        if (instances[i] != NULL && instances[i]->config.handle == handle)
        {
            return instances[i];
        }
    }
    return NULL;
}

/** @brief 校验 DMA 模式、重复注册和固定实例池容量。 */
bool BSPUARTInit(BSPUARTInstance *uart, const BSPUART_Init_Config_s *config)
{
    if (uart == NULL || config == NULL || config->handle == NULL ||
        config->tx_timeout_ms == 0U || config->handle->hdmarx == NULL ||
        config->handle->hdmatx == NULL ||
        config->handle->hdmarx->Init.Mode != DMA_CIRCULAR ||
        config->handle->hdmatx->Init.Mode != DMA_NORMAL)
    {
        return false;
    }
    for (size_t i = 0; i < BSP_UART_CAPACITY; ++i)
    {
        if (instances[i] == uart || (instances[i] != NULL &&
            instances[i]->config.handle == config->handle))
        {
            return false;
        }
    }
    for (size_t i = 0; i < BSP_UART_CAPACITY; ++i)
    {
        if (instances[i] == NULL)
        {
            memset(uart, 0, sizeof(*uart));
            uart->config = *config;
            instances[i] = uart;
            return true;
        }
    }
    return false;
}

/** @brief 清空接收计数并启动循环 DMA，失败留下故障标志。 */
bool BSPUARTStart(BSPUARTInstance *uart, uint32_t now_ms)
{
    if (uart == NULL || FindUART(uart->config.handle) != uart || uart->running)
    {
        return false;
    }
    uart->restart_ms = now_ms;
    uart->dma_read_pos = 0U;
    uart->read_count = 0U;
    uart->write_count = 0U;
    uart->running = true;
    if (HAL_UARTEx_ReceiveToIdle_DMA(uart->config.handle, uart->dma_rx,
                                   BSP_UART_DMA_SIZE) != HAL_OK)
    {
        uart->running = false;
        uart->fault = true;
        return false;
    }
    /* 保留半传输 HT 中断，连续输入时必须在 DMA 完整绕回前搬走数据。 */
    return true;
}

/** @brief 检测故障与发送超时，任务内中止并按期限重启接收。 */
bool BSPUARTService(BSPUARTInstance *uart, uint32_t now_ms)
{
    if (uart == NULL)
    {
        return false;
    }
    bool failed = uart->fault || (uart->tx_busy &&
        (uint32_t)(now_ms - uart->tx_started_ms) >= uart->config.tx_timeout_ms);
    if (failed)
    {
        uart->running = false;
        /* HAL 阻塞中止有有限的 HAL tick 超时，只能在任务且中断开启时执行。
         * 中止成功前不允许启动新的发送，避免复用仍被 DMA 使用的缓冲。
         */
        if (HAL_UART_Abort(uart->config.handle) != HAL_OK)
        {
            uart->fault = true;
            return false;
        }
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        uart->fault = false;
        uart->tx_busy = false;
        uart->write_count = 0U;
        uart->read_count = 0U;
        __set_PRIMASK(primask);
    }
    if (!uart->running && (failed || (uint32_t)(now_ms - uart->restart_ms) >= 20U))
    {
        (void)BSPUARTStart(uart, now_ms);
    }
    return !failed && uart->running;
}

/** @brief 按累计计数从环形区读取，DMB 保护发布顺序。 */
size_t BSPUARTRead(BSPUARTInstance *uart, uint8_t *data, size_t capacity)
{
    if (uart == NULL || data == NULL)
    {
        return 0U;
    }
    size_t length = uart->write_count - uart->read_count;
    if (length > capacity)
    {
        length = capacity;
    }
    /* 单生产者/单消费者环形区：中断复制完才发布 write_count；任务读取完
     * 才发布 read_count。配合 DMB 保证访问顺序，避免覆盖未读数据；
     * volatile 仅保证访问可见，不能单独替代此同步约定。
     */
    __DMB();
    uint32_t read_count = uart->read_count;
    for (size_t i = 0; i < length; ++i)
    {
        data[i] = uart->rx[(read_count + i) % BSP_UART_RX_SIZE];
    }
    __DMB();
    uart->read_count = read_count + (uint32_t)length;
    return length;
}

/** @brief 先复制到私有缓冲，再交给 DMA，回调完成前禁止复用。 */
bool BSPUARTSend(BSPUARTInstance *uart, const uint8_t *data, size_t length,
                 uint32_t now_ms)
{
    if (uart == NULL || data == NULL || length == 0U ||
        length > BSP_UART_TX_SIZE || !uart->running || uart->fault || uart->tx_busy)
    {
        return false;
    }
    memcpy(uart->tx, data, length);
    uart->tx_started_ms = now_ms;
    uart->tx_busy = true;
    if (HAL_UART_Transmit_DMA(uart->config.handle, uart->tx, (uint16_t)length) != HAL_OK)
    {
        uart->tx_busy = false;
        uart->fault = true;
        return false;
    }
    return true;
}

/** @brief 提供总线事务调度需要的物理发送忙状态。 */
bool BSPUARTTxBusy(const BSPUARTInstance *uart)
{
    return uart != NULL && uart->tx_busy;
}

/** @brief 中断入口：按 DMA 实时位置搬运新增数据，不解析协议。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *handle, uint16_t size)
{
    (void)size;
    BSPUARTInstance *uart = FindUART(handle);
    if (uart == NULL || !uart->running || uart->fault)
    {
        return;
    }
    /* 读取实时 NDTR，而非可能已经滞后的 HT/TC 回调 Size。
     * UART 与 RX DMA 中断必须具有相同抢占优先级，避免生产者嵌套。
     */
    uint16_t pos = (uint16_t)(BSP_UART_DMA_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx));
    pos %= BSP_UART_DMA_SIZE;
    uint16_t length = (uint16_t)((pos + BSP_UART_DMA_SIZE - uart->dma_read_pos) % BSP_UART_DMA_SIZE);
    uint32_t write_count = uart->write_count;
    /* 整段放不下时记录故障，不部分覆盖；任务稍后中止并重建链路。 */
    if (length > BSP_UART_RX_SIZE - (write_count - uart->read_count))
    {
        ++uart->error_count;
        uart->fault = true;
        return;
    }
    __DMB();
    for (uint16_t i = 0; i < length; ++i)
    {
        uart->rx[(write_count + i) % BSP_UART_RX_SIZE] =
            uart->dma_rx[(uart->dma_read_pos + i) % BSP_UART_DMA_SIZE];
    }
    uart->dma_read_pos = pos;
    __DMB();
    uart->write_count = write_count + length;
}

/** @brief 发送完成中断释放 TX 缓冲占用。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *handle)
{
    BSPUARTInstance *uart = FindUART(handle);
    if (uart != NULL)
    {
        uart->tx_busy = false;
    }
}

/** @brief 错误中断只记数并置故障，由任务执行恢复。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle)
{
    BSPUARTInstance *uart = FindUART(handle);
    if (uart != NULL)
    {
        ++uart->error_count;
        uart->fault = true;
    }
}
