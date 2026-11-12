#include "board_crane_action.h"
#include "board_servo.h"

bool BoardCraneActionRobotInit(RobotInstance *robot)
{
    const CraneActionConfig config = {
        .pick = {.yaw_units = BOARD_CRANE_PICK_YAW_UNITS, .z_units = BOARD_CRANE_PICK_Z_UNITS,
            .rack_pulse_us = BOARD_CRANE_PICK_RACK_PULSE_US, .configured = BOARD_CRANE_PICK_CONFIGURED != 0},
        .place = {.yaw_units = BOARD_CRANE_PLACE_YAW_UNITS, .z_units = BOARD_CRANE_PLACE_Z_UNITS,
            .rack_pulse_us = BOARD_CRANE_PLACE_RACK_PULSE_US, .configured = BOARD_CRANE_PLACE_CONFIGURED != 0},
        .safe_z_units = BOARD_CRANE_ACTION_SAFE_Z_UNITS,
        .open_pulse_us = BOARD_GRIPPER_SERVO_OPEN_PULSE_US,
        .closed_pulse_us = BOARD_GRIPPER_SERVO_CLOSED_PULSE_US,
        .rack_wait_ms = BOARD_CRANE_ACTION_RACK_WAIT_MS,
        .gripper_wait_ms = BOARD_CRANE_ACTION_GRIPPER_WAIT_MS,
        .step_timeout_ms = BOARD_CRANE_ACTION_STEP_TIMEOUT_MS,
    };
    return RobotCraneActionInit(robot, &config);
}
