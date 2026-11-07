/* 状态灯硬件适配：只输出 GPIO 电平，闪烁节奏由任务监测模块决定。 */
#include "bsp_led.h"
#include <stddef.h>

/** @brief 拒绝空资源、多引脚掩码和非法电平。 */
bool BSPLEDInit(BSPLEDInstance *instance, const BSPLED_Init_Config_s *config)
{
    if (instance == NULL || config == NULL || config->port == NULL ||
        config->pin == 0U || (config->pin & (config->pin - 1U)) != 0U ||
        (config->active_level != GPIO_PIN_SET && config->active_level != GPIO_PIN_RESET))
    {
        return false;
    }

    instance->config = *config;
    instance->initialized = true;
    BSPLEDSet(instance, false);
    return true;
}

/** @brief 按 active_level 映射亮灭，兼容高有效或低有效状态灯。 */
void BSPLEDSet(BSPLEDInstance *instance, bool on)
{
    if (instance != NULL && instance->initialized)
    {
        GPIO_PinState level = instance->config.active_level;
        if (!on)
        {
            level = (level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        }
        HAL_GPIO_WritePin(instance->config.port, instance->config.pin, level);
    }
}
