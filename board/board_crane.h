#ifndef BOARD_CRANE_H
#define BOARD_CRANE_H

#include "x_motor.h"
#include "robot.h"

#define BOARD_CRANE_FEEDBACK_TIMEOUT_MS 500U
/* 初始调试参数；均为电机轴单位，未按齿轮/丝杆换算，需实机整定。
 * USB CRANE_CONFIG可更新下一条不同目标的参数，不写驱动器Flash。 */
#define BOARD_CRANE_YAW_SPEED_DECI_RPM 100U
#define BOARD_CRANE_YAW_ACCELERATION_RPM_S 100U
#define BOARD_CRANE_YAW_DECELERATION_RPM_S 100U
#define BOARD_CRANE_Z_SPEED_DECI_RPM 1000U /* 100 RPM，用户要求原10 RPM的10倍。 */
#define BOARD_CRANE_Z_ACCELERATION_RPM_S 100U
#define BOARD_CRANE_Z_DECELERATION_RPM_S 100U
#define BOARD_CRANE_POSITION_TOLERANCE_UNITS 10U

typedef enum
{
    BOARD_CRANE_YAW = 0,
    BOARD_CRANE_Z = 1,
    BOARD_CRANE_AXIS_COUNT = 2
} BoardCraneAxis;

typedef struct
{
    XMotorFeedback feedback;
    XMotorDiagnostics diagnostics;
    XMotorMotionStatus motion;
    uint32_t uart_errors;
    bool initialized;
    bool uart_running;
} BoardCraneAxisStatus;

typedef struct
{
    BoardCraneAxisStatus axes[BOARD_CRANE_AXIS_COUNT]; /* 0=yaw/USART6，1=z/USART3。 */
    uint32_t timestamp_ms;
} BoardCraneStatus;

/* 任务创建前组装；Start/Update/GetStatus仅由motorTask调用。
 * 两条串口各一个地址1实例，独立事务和错误恢复。 */
bool BoardCraneInit(uint32_t now_ms);
bool BoardCraneRobotInit(RobotInstance *robot);
bool BoardCraneStart(uint32_t now_ms);
void BoardCraneUpdate(uint32_t now_ms);
void BoardCraneGetStatus(BoardCraneStatus *status, uint32_t now_ms);

#endif
