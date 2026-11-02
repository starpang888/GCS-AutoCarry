/* 状态灯板级接口：把逻辑亮灭与实际 GPIO 电平分开。 */
#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdint.h>

/**
 * @brief 初始化状态灯为熄灭状态。
 * @note 必须先调用 MX_GPIO_Init()；当前硬件为 PB2、低电平点亮。
 */
void BSPLedInit(void);

/**
 * @brief 设置状态灯亮灭。
 * @param on 0 表示熄灭，非 0 表示点亮；调用者无需关心有效电平。
 */
void BSPLedWrite(uint8_t on);

#endif
