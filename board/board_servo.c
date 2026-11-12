#include "board_servo.h"
#include "tim.h"

static BSPPWMInstance rack_pwm;
static ServoInstance rack_servo;
static BSPPWMInstance gripper_pwm;
static ServoInstance gripper_servo;

bool BoardRackServoInit(void)
{
    rack_pwm.config = (BSPPWM_Init_Config_s){ .timer = &htim9, .channel = TIM_CHANNEL_1, .timer_tick_hz = 1000000U, .period_ticks = 20000U };
    const Servo_Init_Config_s config = { .pwm = &rack_pwm, .min_pulse_us = BOARD_RACK_SERVO_MIN_PULSE_US, .center_pulse_us = BOARD_RACK_SERVO_CENTER_PULSE_US, .max_pulse_us = BOARD_RACK_SERVO_MAX_PULSE_US };
    return ServoInit(&rack_servo, &config);
}

bool BoardRackServoSetPulseUs(uint16_t pulse_us) { return ServoSetPulseUs(&rack_servo, pulse_us); }
bool BoardRackServoSetPositionPermille(int16_t position) { return ServoSetPositionPermille(&rack_servo, position); }
uint16_t BoardRackServoGetPulseUs(void) { return ServoGetPulseUs(&rack_servo); }
bool BoardRackServoRobotInit(RobotInstance *robot) { return RobotRackServoInit(robot, &rack_servo); }

bool BoardGripperServoInit(void)
{
    gripper_pwm.config = (BSPPWM_Init_Config_s){ .timer = &htim9, .channel = TIM_CHANNEL_2,
        .timer_tick_hz = 1000000U, .period_ticks = 20000U };
    const Servo_Init_Config_s config = { .pwm = &gripper_pwm,
        .min_pulse_us = BOARD_GRIPPER_SERVO_MIN_PULSE_US,
        .center_pulse_us = BOARD_GRIPPER_SERVO_CENTER_PULSE_US,
        .max_pulse_us = BOARD_GRIPPER_SERVO_MAX_PULSE_US };
    return ServoInit(&gripper_servo, &config);
}

bool BoardGripperServoSetPulseUs(uint16_t pulse_us)
{ return ServoSetPulseUs(&gripper_servo, pulse_us); }

bool BoardGripperServoSetPositionPermille(int16_t position)
{ return ServoSetPositionPermille(&gripper_servo, position); }

uint16_t BoardGripperServoGetPulseUs(void)
{ return ServoGetPulseUs(&gripper_servo); }

bool BoardGripperServoRobotInit(RobotInstance *robot)
{ return RobotGripperServoInit(robot, &gripper_servo); }
