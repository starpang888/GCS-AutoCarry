/* UART DMA 服务：中断搬运字节，任务负责协议解析、发送及故障恢复。 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#define BSP_UART_DMA_SIZE 128U
#define BSP_UART_RX_SIZE 256U
#define BSP_UART_TX_SIZE 16U

/* Start/Service/Read/Send 由唯一任务调用，HAL 回调只搬运字节和记录标志。
 * 实例须长期有效，放在 DMA 可访问的 SRAM，不能放进 CCM。Send 复制数据
 * 到实例私有缓冲，发送完成前该缓冲由 DMA 占用，调用者无需保留输入缓冲。
 */
typedef struct
{
    UART_HandleTypeDef *handle;
    uint32_t tx_timeout_ms; /* DMA 发送完成期限，单位 ms。 */
} BSPUART_Init_Config_s;

typedef struct
{
    BSPUART_Init_Config_s config;
    uint8_t dma_rx[BSP_UART_DMA_SIZE]; /* 循环 DMA 原始接收区。 */
    uint8_t rx[BSP_UART_RX_SIZE]; /* 中断生产、任务消费的软件环形区。 */
    uint8_t tx[BSP_UART_TX_SIZE];
    volatile uint32_t write_count; /* 累计写入量，可自然回绕，不是数组索引。 */
    volatile uint32_t read_count; /* 累计读出量，与 write_count 差值为积压字节数。 */
    volatile uint32_t error_count;
    volatile bool fault;
    volatile bool running;
    volatile bool tx_busy;
    uint16_t dma_read_pos; /* 上次搬运结束的 DMA 环位置。 */
    uint32_t tx_started_ms;
    uint32_t restart_ms;
} BSPUARTInstance;

/** @brief 注册实例和 HAL 句柄；要求 RX 循环 DMA、TX 普通 DMA。 */
bool BSPUARTInit(BSPUARTInstance *uart, const BSPUART_Init_Config_s *config);
/** @brief 启动 ReceiveToIdle DMA，并重置接收游标；不允许重复启动。 */
bool BSPUARTStart(BSPUARTInstance *uart, uint32_t now_ms);
/* 任务周期服务：发生传输故障时返回 false，并尝试有界恢复。
 * 即使重启成功，本次故障也不能被掩盖，拥有者应作废当前协议事务。
 */
bool BSPUARTService(BSPUARTInstance *uart, uint32_t now_ms);
/** @brief 从软件环形区取出最多 capacity 字节，返回实际长度。 */
size_t BSPUARTRead(BSPUARTInstance *uart, uint8_t *data, size_t capacity);
/** @brief 复制并启动异步 DMA 发送；未运行、故障、忙或超长时返回 false。 */
bool BSPUARTSend(BSPUARTInstance *uart, const uint8_t *data, size_t length,
                 uint32_t now_ms);
/** @brief 查询发送缓冲是否仍被 DMA 占用。 */
bool BSPUARTTxBusy(const BSPUARTInstance *uart);

#endif
