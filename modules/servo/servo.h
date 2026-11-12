#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_pwm.h"

typedef struct { BSPPWMInstance *pwm; uint16_t min_pulse_us; uint16_t max_pulse_us; uint16_t center_pulse_us; } Servo_Init_Config_s;
typedef struct { Servo_Init_Config_s config; uint16_t pulse_us; bool initialized; } ServoInstance;

bool ServoInit(ServoInstance *instance, const Servo_Init_Config_s *config);
bool ServoSetPulseUs(ServoInstance *instance, uint16_t pulse_us);
bool ServoSetPositionPermille(ServoInstance *instance, int16_t position);
uint16_t ServoGetPulseUs(const ServoInstance *instance);

#endif
