/* GPIO 状态灯接口：板级注入引脚和有效电平，上层只使用亮/灭语义。 */
#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState active_level; /* 点亮时输出电平，本板 PB2 为高电平点亮。 */
} BSPLED_Init_Config_s;

typedef struct
{
    BSPLED_Init_Config_s config;
    bool initialized;
} BSPLEDInstance;

/* 调用前 CubeMX 必须完成 GPIO 模式和时钟初始化；本接口初始化后灯保持熄灭。 */
/** @brief 检查单引脚和有效电平配置，初始化实例并熄灯。 */
bool BSPLEDInit(BSPLEDInstance *instance, const BSPLED_Init_Config_s *config);
/** @brief 把逻辑亮灭转换为物理电平，未初始化时忽略操作。 */
void BSPLEDSet(BSPLEDInstance *instance, bool on);

#endif
