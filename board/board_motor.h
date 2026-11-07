#ifndef BOARD_MOTOR_H
#define BOARD_MOTOR_H

#include "emm_motor.h"
#include "omni_kinematics.h"
#include "chassis_control.h"
#include "chassis_position_control.h"
#include "chassis_odometry.h"
#include "robot.h"

#define BOARD_MOTOR_FEEDBACK_TIMEOUT_MS 500U

typedef enum
{
    BOARD_WHEEL_FRONT_LEFT,
    BOARD_WHEEL_FRONT_RIGHT,
    BOARD_WHEEL_REAR_RIGHT,
    BOARD_WHEEL_REAR_LEFT,
    BOARD_WHEEL_COUNT
} BoardWheel;

typedef struct
{
    float wheel_radius_m;
    float wheelbase_m;
    float track_width_m;
    bool dimensions_measured;
    bool directions_calibrated;
} BoardChassisGeometry;

typedef struct
{
    EmmMotorFeedback wheels[BOARD_WHEEL_COUNT];
    uint32_t timestamp_ms;
    uint32_t transaction_timeouts;
    uint32_t protocol_errors;
    uint32_t rejected_commands;
    uint8_t last_rejected_address;
    uint8_t last_rejected_function;
    uint8_t last_rejected_status;
    uint32_t transport_errors;
    uint32_t rx_errors;
    bool motion_allowed;
} BoardMotorStatus;

/* Init before scheduler; other APIs only in motorTask. Board owns all handles,
 * addresses and motor instances; application code must not access huart5.
 */
bool BoardMotorInit(uint32_t now_ms);
bool BoardMotorStart(uint32_t now_ms);
/* Mathematical model only; does not enable motors or calibrate their signs. */
bool BoardChassisKinematicsInit(OmniKinematicsInstance *instance);
bool BoardChassisControlInit(ChassisControlInstance *control, OmniKinematicsInstance *kinematics);
void BoardMotorUpdate(uint32_t now_ms);
void BoardMotorGetStatus(BoardMotorStatus *status, uint32_t now_ms);
const BoardChassisGeometry *BoardMotorGetGeometry(void);

/* 使用实际电机安装方向、减速比和底盘模型初始化里程计。 */
bool BoardChassisOdometryInit(
    ChassisOdometryInstance *odometry,
    const OmniKinematicsInstance *kinematics);

/* 位置环初始参数集中在板级配置，不在通用控制器中写死。 */
bool BoardChassisPositionControlInit(ChassisPositionControlInstance *position_control);

/* 板级集中注入稳定到达阈值，初始化不发送命令、不使能电机。 */
bool BoardRobotPositionInit(RobotInstance *robot, ChassisOdometryInstance *odometry,
                            ChassisPositionControlInstance *position_control);

#endif
