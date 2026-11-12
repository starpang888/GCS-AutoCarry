#ifndef BOARD_SERVO_H
#define BOARD_SERVO_H

#include <stdbool.h>
#include <stdint.h>
#include "robot.h"
#include "servo.h"

#define BOARD_RACK_SERVO_MIN_PULSE_US 500U
#define BOARD_RACK_SERVO_CENTER_PULSE_US 2400U
#define BOARD_RACK_SERVO_MAX_PULSE_US 2500U
#define BOARD_GRIPPER_SERVO_MIN_PULSE_US 500U
#define BOARD_GRIPPER_SERVO_OPEN_PULSE_US 1000U
#define BOARD_GRIPPER_SERVO_CLOSED_PULSE_US 550U
/* 上电输出：当前为标定后的张开位置。开合位置不等同于安全行程上下限。 */
#define BOARD_GRIPPER_SERVO_CENTER_PULSE_US 1000U
#define BOARD_GRIPPER_SERVO_MAX_PULSE_US 2500U

bool BoardRackServoInit(void);
bool BoardRackServoRobotInit(RobotInstance *robot);
bool BoardRackServoSetPulseUs(uint16_t pulse_us);
bool BoardRackServoSetPositionPermille(int16_t position);
uint16_t BoardRackServoGetPulseUs(void);

bool BoardGripperServoInit(void);
bool BoardGripperServoRobotInit(RobotInstance *robot);
bool BoardGripperServoSetPulseUs(uint16_t pulse_us);
bool BoardGripperServoSetPositionPermille(int16_t position);
uint16_t BoardGripperServoGetPulseUs(void);

#endif
