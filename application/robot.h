#ifndef ROBOT_H
#define ROBOT_H

#include "chassis_control.h"
#include "chassis_odometry.h"
#include "chassis_position_control.h"

/* 这是运动来源，不是电机使能状态；使能状态仍看底盘控制和真实反馈。 */
typedef enum
{
    ROBOT_IDLE = 0,
    ROBOT_VELOCITY = 1,
    ROBOT_POSITION = 2,
} RobotMode;

/* 到达是当前状态：离开容差或反馈失效后撤销，不是锁存的任务完成事件。 */
typedef enum
{
    ROBOT_ARRIVAL_NONE = 0,       /* 没有正在执行的位置目标。 */
    ROBOT_ARRIVAL_APPROACHING = 1,/* 尚未满足位置、低速或使能条件。 */
    ROBOT_ARRIVAL_SETTLING = 2,   /* 已满足条件，正在累计稳定时间。 */
    ROBOT_ARRIVAL_REACHED = 3,    /* 连续稳定时间已达到配置值。 */
} RobotArrivalState;

typedef struct
{
    float linear_speed_m_s;     /* 判定静稳的实测平移速率阈值，不是输出限速。 */
    float angular_speed_rad_s;  /* 判定静稳的实测角速度绝对值阈值。 */
    uint32_t hold_ms;           /* 需要连续满足条件的时间。 */
    uint32_t feedback_timeout_ms;
    uint32_t max_sample_span_ms;
} RobotArrival_Init_Config_s;

typedef struct
{
    RobotArrivalState state;
    OmniBodyVelocity measured;  /* 四轮编码器差分速度正解，m/s、rad/s。 */
    uint32_t stable_ms;         /* 有新实测采样支持的稳定时间，最多为hold_ms。 */
    bool measured_valid;        /* false时不得使用measured的旧值。 */
} RobotArrivalStatus;

typedef struct
{
    ChassisControlInstance *control;
    OmniKinematicsInstance *kinematics;
    ChassisOdometryInstance *odometry;
    ChassisPositionControlInstance *position_control;

    RobotMode mode;
    ChassisPose target_pose; /* 相对初始原点的绝对目标，m、rad；取消后保留供观察。 */
    uint32_t last_position_update_ms;
    RobotArrival_Init_Config_s arrival_config;
    RobotArrivalStatus arrival;
    uint32_t position_sequence; /* 最后接受的位置命令序号；允许重复。 */
    uint32_t goal_generation;   /* 进入位置模式或坐标变化时+1，允许自然回绕。 */
    int32_t position_target_milli[3]; /* 原始mm/mm/mrad，避免回传时浮点取整。 */
    uint32_t arrival_start_sample_ms;
    uint32_t arrival_sample_ms[OMNI_WHEEL_COUNT];
    bool arrival_has_sample;
} RobotInstance;

/* 先绑定底盘，再绑定位置控制依赖；所有实例均须长期存活。初始化不产生运动。 */
bool RobotInit(RobotInstance *robot, ChassisControlInstance *control);
bool RobotPositionInit(RobotInstance *robot, ChassisOdometryInstance *odometry,
                       ChassisPositionControlInstance *position_control,
                       const RobotArrival_Init_Config_s *arrival_config);

/* 以下入口只由motorTask调用。位置目标只从USB命令进入，不提供手动运动入口。 */
void RobotHostCommand(RobotInstance *robot, const HostCommand *command, uint32_t now_ms);
/* STOP/链路异常先取消位置任务，再停止底盘，防止下次更新恢复旧目标。 */
void RobotStop(RobotInstance *robot, ChassisControlFault fault, uint32_t now_ms);
void RobotUpdate(RobotInstance *robot, uint32_t now_ms);

#endif
