#include "bsp_pwm.h"
#include <stddef.h>

static bool ValidChannel(uint32_t channel)
{
    return channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2 || channel == TIM_CHANNEL_3 || channel == TIM_CHANNEL_4;
}

bool BSPPWMInit(BSPPWMInstance *instance, const BSPPWM_Init_Config_s *config, uint32_t initial_pulse_us)
{
    if (instance == NULL || config == NULL || config->timer == NULL || !ValidChannel(config->channel) ||
        config->timer_tick_hz == 0U || config->period_ticks == 0U) return false;
    instance->config = *config;
    instance->initialized = true;
    instance->pulse_ticks = 0U;
    if (!BSPPWMSetPulseUs(instance, initial_pulse_us) || HAL_TIM_PWM_Start(config->timer, config->channel) != HAL_OK)
    {
        instance->initialized = false;
        return false;
    }
    return true;
}

bool BSPPWMSetPulseUs(BSPPWMInstance *instance, uint32_t pulse_us)
{
    if (instance == NULL || !instance->initialized) return false;
    uint64_t period_us = ((uint64_t)instance->config.period_ticks * 1000000U) /
                         instance->config.timer_tick_hz;
    if ((uint64_t)pulse_us >= period_us) return false;
    uint64_t ticks = ((uint64_t)pulse_us * instance->config.timer_tick_hz + 500000U) / 1000000U;
    if (ticks >= instance->config.period_ticks) return false;
    __HAL_TIM_SET_COMPARE(instance->config.timer, instance->config.channel, (uint32_t)ticks);
    instance->pulse_ticks = (uint32_t)ticks;
    return true;
}

uint32_t BSPPWMGetPulseUs(const BSPPWMInstance *instance)
{
    if (instance == NULL || !instance->initialized || instance->config.timer_tick_hz == 0U) return 0U;
    return (uint32_t)(((uint64_t)instance->pulse_ticks * 1000000U + instance->config.timer_tick_hz / 2U) / instance->config.timer_tick_hz);
}
