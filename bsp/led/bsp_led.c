/* 板级实现：仅在这一层操作 HAL GPIO，供上层灯效模块调用。 */
#include "bsp_led.h"
#include "main.h"

/* 板级极性配置：1 表示低电平点亮，0 表示高电平点亮。 */
#define STATUS_LED_ACTIVE_LOW 1U

void BSPLedInit(void)
{
    /* GPIO 模式已由 CubeMX 配置，这里只设置初始亮灭状态。 */
    BSPLedWrite(0U);
}

void BSPLedWrite(uint8_t on)
{
    GPIO_PinState level;

    /* 将上层的“亮/灭”转换为硬件需要的“低/高”电平。 */
    if (STATUS_LED_ACTIVE_LOW)
    {
        level = on ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }
    else
    {
        level = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }

    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, level);
}
