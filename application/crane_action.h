#ifndef CRANE_ACTION_H
#define CRANE_ACTION_H

#include "x_motor.h"
#include "servo.h"

typedef enum { CRANE_ACTION_NONE = 0, CRANE_ACTION_PICK = 1, CRANE_ACTION_PLACE = 2 } CraneActionKind;
typedef enum
{
    CRANE_ACTION_IDLE = 0, CRANE_ACTION_RUNNING = 1, CRANE_ACTION_DONE = 2,
    CRANE_ACTION_CANCELLED = 3, CRANE_ACTION_FAILED = 4,
} CraneActionState;
typedef enum
{
    CRANE_STEP_NONE = 0, CRANE_STEP_SAFE_Z = 1, CRANE_STEP_YAW = 2,
    CRANE_STEP_RACK = 3, CRANE_STEP_LOWER = 4, CRANE_STEP_GRIP = 5, CRANE_STEP_LIFT = 6,
} CraneActionStep;
typedef enum
{
    CRANE_ACTION_OK = 0, CRANE_ACTION_STOP = 1, CRANE_ACTION_LINK = 2,
    CRANE_ACTION_AXIS = 3, CRANE_ACTION_TIMEOUT = 4, CRANE_ACTION_OUTPUT = 5,
    CRANE_ACTION_TARGET_CHANGED = 6,
} CraneActionFault;

/* 使用已确认的驱动累计坐标/PWM脉宽边界，不换算机构距离和输出角。 */
typedef struct
{
    int64_t yaw_units;
    int64_t z_units;
    uint16_t rack_pulse_us;
    bool configured; /* 未标定的0坐标不能被当作有效工作点。 */
} CraneActionPoint;

typedef struct
{
    CraneActionPoint pick;
    CraneActionPoint place;
    int64_t safe_z_units;
    uint16_t open_pulse_us;
    uint16_t closed_pulse_us;
    uint32_t rack_wait_ms;
    uint32_t gripper_wait_ms;
    uint32_t step_timeout_ms;
} CraneActionConfig;

typedef struct
{
    CraneActionState state;
    CraneActionKind action;
    CraneActionStep step; /* 结束后保留最后一步/出错步骤。 */
    CraneActionFault fault;
    XMotorFault axis_fault;
    uint8_t fault_axis; /* 0=yaw、1=z，255=无指定轴。 */
    uint32_t sequence;
    uint32_t generation;
    uint32_t rejected_commands;
    uint32_t step_elapsed_ms;
} CraneActionStatus;

typedef struct
{
    XMotorInstance *axes[2];
    ServoInstance *rack;
    ServoInstance *gripper;
    CraneActionConfig config;
    CraneActionConfig active_config; /* 启动快照，运行中不改变。 */
    CraneActionPoint point;
    XMotorMotionProfile profiles[2];
    CraneActionStatus status;
    uint32_t step_started_ms;
    uint32_t step_position_replies;
    uint32_t step_status_replies;
    uint32_t axis_generation[2];
    int64_t axis_target[2];
    bool axis_owned[2];
    uint32_t last_sequence[2];
    bool has_sequence[2];
    bool initialized;
} CraneActionInstance;

/* 仅motorTask拥有；初始化不输出运动，不要求坐标已经填写。 */
bool CraneActionInit(CraneActionInstance *action, XMotorInstance *yaw, XMotorInstance *z,
                     ServoInstance *rack, ServoInstance *gripper, const CraneActionConfig *config);
bool CraneActionReady(const CraneActionInstance *action, CraneActionKind kind);
bool CraneActionIsRunning(const CraneActionInstance *action);
/* 相同动作最近已接受sequence重复提交为幂等，无需递增，但再次执行需不同值。
 * 运行时拒绝其他启动，不排队。尚在手动运动/停止确认时拒绝。 */
bool CraneActionStart(CraneActionInstance *action, CraneActionKind kind,
                      const XMotorMotionProfile profiles[2], uint32_t sequence, uint32_t now_ms);
void CraneActionUpdate(CraneActionInstance *action, uint32_t now_ms);
/* 取消未来步骤，停止两步进轴且保留使能；保持两舵机当前脉宽。 */
void CraneActionCancel(CraneActionInstance *action, CraneActionFault reason);

#endif
