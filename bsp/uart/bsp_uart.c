#include "bsp_uart.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* 单次通信允许的最大字节数。 */
#define UART5_BUFFER_SIZE 64U

/* 任务通知的事件位，可以同时记录发送完成和接收完成。 */
#define UART5_EVENT_TX_DONE (1UL << 0)
#define UART5_EVENT_RX_DONE (1UL << 1)
#define UART5_EVENT_ERROR   (1UL << 2)

#define UART5_EVENT_COMPLETE \
    (UART5_EVENT_TX_DONE | UART5_EVENT_RX_DONE)

/* DMA 使用静态缓冲区，保证传输期间内存始终有效。
 * 普通静态变量位于当前链接脚本的 SRAM 中，不放入 DMA 无法访问的 CCM。
 */
static uint8_t tx_dma_buffer[UART5_BUFFER_SIZE];
static uint8_t rx_dma_buffer[UART5_BUFFER_SIZE];

/* 当前等待通信结果的任务，由任务设置，由中断读取。
 * UART5 只能由一个通信任务使用。
 */
static TaskHandle_t volatile waiting_task;

/**
 * @brief 在中断中通知通信任务。
 * @note UART5 和对应 DMA 中断优先级必须允许调用 FreeRTOS FromISR 接口。
 *       当前工程这些中断的抢占优先级均为 5，符合现有配置。
 */
static void NotifyCommunicationTask(uint32_t event)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    TaskHandle_t task = waiting_task;

    if (task != NULL)
    {
        xTaskNotifyFromISR(
            task,
            event,
            eSetBits,
            &higher_priority_task_woken);

        /* 若唤醒了更高优先级任务，在退出中断时及时切换。 */
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

HAL_StatusTypeDef BSPUart5Exchange(
    const uint8_t *tx_data,
    uint16_t tx_length,
    uint8_t *rx_data,
    uint16_t rx_length,
    uint32_t timeout_ms)
{
    HAL_StatusTypeDef result;
    TickType_t timeout_ticks;
    TickType_t start_tick;
    TickType_t elapsed_ticks;
    uint32_t events;
    uint32_t completed_events = 0U;

    if (tx_data == NULL || rx_data == NULL ||
        tx_length == 0U || tx_length > UART5_BUFFER_SIZE ||
        rx_length == 0U || rx_length > UART5_BUFFER_SIZE ||
        timeout_ms == 0U)
    {
        return HAL_ERROR;
    }

    /* 此检查用于发现误用；接口本身不支持多个任务并发调用。 */
    if (waiting_task != NULL)
    {
        return HAL_BUSY;
    }

    timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    /* 防止较低节拍频率下，较短超时时间换算成零个节拍。 */
    if (timeout_ticks == 0U)
    {
        timeout_ticks = 1U;
    }

    /* 结束可能遗留的 UART5 传输，重新建立一次完整事务。 */
    result = HAL_UART_Abort(&huart5);
    if (result != HAL_OK)
    {
        return result;
    }

    /* 清除上一次残留的通知。
     * 因此本任务的默认通知必须专用于本串口通信。
     */
    (void)xTaskNotifyWait(0U, UINT32_MAX, &events, 0U);

    memcpy(tx_dma_buffer, tx_data, tx_length);

    /* 清除遗留的接收数据和溢出状态，再开始新的接收。 */
    __HAL_UART_CLEAR_OREFLAG(&huart5);

    waiting_task = xTaskGetCurrentTaskHandle();
    start_tick = xTaskGetTickCount();

    /* 先接收后发送，避免漏掉驱动器快速返回的数据。 */
    result = HAL_UART_Receive_DMA(
        &huart5,
        rx_dma_buffer,
        rx_length);

    if (result != HAL_OK)
    {
        goto exchange_finished;
    }

    /* 固定长度回复只需完整接收通知，关闭半传输中断。 */
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);

    result = HAL_UART_Transmit_DMA(
        &huart5,
        tx_dma_buffer,
        tx_length);

    if (result != HAL_OK)
    {
        goto exchange_finished;
    }

    __HAL_DMA_DISABLE_IT(huart5.hdmatx, DMA_IT_HT);

    for (;;)
    {
        elapsed_ticks = xTaskGetTickCount() - start_tick;

        if (elapsed_ticks >= timeout_ticks)
        {
            result = HAL_TIMEOUT;
            break;
        }

        /* 没有中断通知时，任务进入阻塞态，不反复检查标志。
         * CPU 可以执行 LED 和其他就绪任务。
         */
        if (xTaskNotifyWait(
                0U,
                UINT32_MAX,
                &events,
                timeout_ticks - elapsed_ticks) != pdTRUE)
        {
            result = HAL_TIMEOUT;
            break;
        }

        if ((events & UART5_EVENT_ERROR) != 0U)
        {
            result = HAL_ERROR;
            break;
        }

        completed_events |= events;

        /* 收发完成顺序可能不同，必须等两者都完成。 */
        if ((completed_events & UART5_EVENT_COMPLETE) ==
            UART5_EVENT_COMPLETE)
        {
            result = HAL_OK;
            break;
        }
    }

exchange_finished:
    /* 当前 RX DMA 为循环模式，因此即使收齐回复，也要主动结束接收。
     * 使用同步终止接口，保证返回前 DMA 不再修改缓冲区。
     * 同时清理发送状态，方便下一次请求。
     */
    if (HAL_UART_Abort(&huart5) != HAL_OK)
    {
        result = HAL_ERROR;
    }

    waiting_task = NULL;

    if (result == HAL_OK)
    {
        /* 仅在收发成功后向调用者交付回复数据。 */
        memcpy(rx_data, rx_dma_buffer, rx_length);
    }

    return result;
}

/* DMA 收齐预期长度的回复后，由 HAL 调用。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        NotifyCommunicationTask(UART5_EVENT_RX_DONE);
    }
}

/* UART 最后一个字节发送完成后，由 HAL 调用。
 * 此路径需要已有的 UART5 全局中断。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        NotifyCommunicationTask(UART5_EVENT_TX_DONE);
    }
}

/* 串口发生帧错误、溢出等异常时，唤醒任务处理。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        NotifyCommunicationTask(UART5_EVENT_ERROR);
    }
}