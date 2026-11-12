#include "servo.h"
#include <stddef.h>

bool ServoInit(ServoInstance *instance, const Servo_Init_Config_s *config)
{
    if (instance == NULL || config == NULL || config->pwm == NULL || config->min_pulse_us >= config->center_pulse_us || config->center_pulse_us >= config->max_pulse_us) return false;
    if (!BSPPWMInit(config->pwm, &config->pwm->config, config->center_pulse_us)) return false;
    instance->config = *config;
    instance->pulse_us = config->center_pulse_us;
    instance->initialized = true;
    return true;
}

bool ServoSetPulseUs(ServoInstance *instance, uint16_t pulse_us)
{
    if (instance == NULL || !instance->initialized || pulse_us < instance->config.min_pulse_us || pulse_us > instance->config.max_pulse_us || !BSPPWMSetPulseUs(instance->config.pwm, pulse_us)) return false;
    instance->pulse_us = pulse_us;
    return true;
}

bool ServoSetPositionPermille(ServoInstance *instance, int16_t position)
{
    if (instance == NULL || !instance->initialized || position < -1000 || position > 1000) return false;
    int32_t span = (int32_t)instance->config.max_pulse_us - instance->config.min_pulse_us;
    uint16_t pulse = (uint16_t)(instance->config.min_pulse_us + ((int32_t)(position + 1000) * span + 1000) / 2000);
    return ServoSetPulseUs(instance, pulse);
}

uint16_t ServoGetPulseUs(const ServoInstance *instance)
{
    return instance != NULL && instance->initialized ? instance->pulse_us : 0U;
}
