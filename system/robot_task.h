#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include <stdbool.h>
#include "board_motor.h"
#include "board_crane.h"
#include "robot.h"

/* One-time startup from MX_FREERTOS_Init, before osKernelStart. */
bool RobotTaskInit(void);

/* Task context only; called after each completed defaultTask work cycle. */
void RobotTaskHeartbeat(void);

/* Task context only: synchronized copy, false until the first motor cycle.
 * Feedback timestamps/validity must be checked by the caller.
 */
bool RobotMotorGetStatus(BoardMotorStatus *status);

/* 一致快照，反馈有效性在读取时再次按时间检查。 */
bool RobotCraneGetStatus(BoardCraneStatus *status);

#endif
