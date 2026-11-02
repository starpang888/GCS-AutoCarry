#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/**
 * @brief 通过 UART5 DMA 发送请求并接收固定长度回复。
 * @param tx_data 待发送数据。
 * @param tx_length 发送长度，范围为 1～64 字节。
 * @param rx_data 回复输出缓冲区。
 * @param rx_length 预期回复长度，范围为 1～64 字节。
 * @param timeout_ms 整次通信的超时时间，单位毫秒。
 * @return HAL_OK 成功；HAL_TIMEOUT 超时；其他值表示忙或通信错误。
 * @note 仅由一个普通通信任务调用，不可在中断中调用。
 *       本接口独占调用任务的默认任务通知，不能与其他通知用途混用。
 *       当前适用于已知回复长度的查询，后续控制指令需指定对应回复长度。
 */
HAL_StatusTypeDef BSPUart5Exchange(
    const uint8_t *tx_data,
    uint16_t tx_length,
    uint8_t *rx_data,
    uint16_t rx_length,
    uint32_t timeout_ms);

#endif