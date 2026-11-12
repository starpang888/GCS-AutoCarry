#ifndef ROBOT_H
#define ROBOT_H

#include "chassis_control.h"
#include "chassis_localization.h"
#include "chassis_position_control.h"
#include "x_motor.h"
#include "servo.h"
#include "crane_action.h"

#define ROBOT_CRANE_AXIS_COUNT 2U

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
    ChassisLocalizationFeedback *localization; /* 统一定位输入，实际由OPS提供。 */
    uint32_t goal_pose_epoch; /* 接受当前位置目标时的定位连续性代次。 */
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
    XMotorInstance *crane_axes[ROBOT_CRANE_AXIS_COUNT]; /* 0=yaw，1=z。 */
    XMotorMotionProfile crane_profiles[ROBOT_CRANE_AXIS_COUNT]; /* 下一条不同目标使用。 */
    ServoInstance *rack_servo;
    ServoInstance *gripper_servo;
    CraneActionInstance crane_action;
} RobotInstance;

/* 先绑定底盘，再绑定位置控制依赖；所有实例均须长期存活。初始化不产生运动。 */
bool RobotInit(RobotInstance *robot, ChassisControlInstance *control);
bool RobotPositionInit(RobotInstance *robot, ChassisLocalizationFeedback *localization,
                       ChassisPositionControlInstance *position_control,
                       const RobotArrival_Init_Config_s *arrival_config);
bool RobotCraneInit(RobotInstance *robot, XMotorInstance *yaw, XMotorInstance *z,
                    const XMotorMotionProfile profiles[ROBOT_CRANE_AXIS_COUNT]);
bool RobotRackServoInit(RobotInstance *robot, ServoInstance *servo);
bool RobotGripperServoInit(RobotInstance *robot, ServoInstance *servo);
bool RobotCraneActionInit(RobotInstance *robot, const CraneActionConfig *config);
void RobotCraneStop(RobotInstance *robot, XMotorFault fault, bool only_busy);
bool RobotCraneIsBusy(const RobotInstance *robot);

/* 以下入口只由motorTask调用。USB与可选Debug邮箱复用此命令仲裁，邮箱不直接输出电机。 */
void RobotHostCommand(RobotInstance *robot, const HostCommand *command, uint32_t now_ms);
/* 底盘停止；全局HOST_STOP另由RobotHostCommand取消两轴，
 * USB链路异常由HostService分别提交底盘/塔吊停止，设备故障保持按机构隔离。 */
void RobotStop(RobotInstance *robot, ChassisControlFault fault, uint32_t now_ms);
void RobotUpdate(RobotInstance *robot, uint32_t now_ms);

#endif
