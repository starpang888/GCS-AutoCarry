#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct { TIM_HandleTypeDef *timer; uint32_t channel; uint32_t timer_tick_hz; uint32_t period_ticks; } BSPPWM_Init_Config_s;
typedef struct { BSPPWM_Init_Config_s config; bool initialized; uint32_t pulse_ticks; } BSPPWMInstance;

bool BSPPWMInit(BSPPWMInstance *instance, const BSPPWM_Init_Config_s *config, uint32_t initial_pulse_us);
bool BSPPWMSetPulseUs(BSPPWMInstance *instance, uint32_t pulse_us);
uint32_t BSPPWMGetPulseUs(const BSPPWMInstance *instance);

#endif
