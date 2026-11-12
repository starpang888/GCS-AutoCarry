/* 调度层调试适配：调试器提交一次请求，motorTask复用正式Robot命令接口。 */
#ifndef ROBOT_DEBUG_POSITION_H
#define ROBOT_DEBUG_POSITION_H

#include "board_debug.h"

#if ROBOT_DEBUG_POSITION_ENABLED
#include "robot.h"

typedef enum
{
    ROBOT_DEBUG_NONE = 0,
    ROBOT_DEBUG_POSITION = 1,
    ROBOT_DEBUG_STOP = 2,
} RobotDebugCommand;

typedef enum
{
    ROBOT_DEBUG_RESULT_NONE = 0,
    ROBOT_DEBUG_RESULT_POSITION_ACCEPTED = 1, /* 已接受目标，不表示电机已运动或已到达。 */
    ROBOT_DEBUG_RESULT_REJECTED = 2,          /* 定位/控制状态不满足等，需再次显式提交。 */
    ROBOT_DEBUG_RESULT_STOP_REQUESTED = 3,    /* 已请求停止，真实失能需看四轮反馈。 */
    ROBOT_DEBUG_RESULT_INVALID_COMMAND = 4,
} RobotDebugResult;

/* 固定容量单槽邮箱，仅用于调试器 -> motorTask。
 * 使用契约：
 * 1. command==0时才允许调试器写坐标；x/y/yaw全部写好后，最后写command=1。
 * 2. command非0时，不得改坐标或覆盖command，等待任务消费并清零。
 * 3. 停止时只写command=2，不需要修改坐标。
 * 4. result/completed/last_command/sequence由任务写，调试器只读。
 *
 * 所有字段为32位且按4字节对齐，command是单次发布/完成标志。
 * volatile用于外部调试写入的可见性；它本身不能保护结构体快照。
 * 一致性由“唯一外部生产者、唯一任务消费者、最后发布、完成后才再写”保证，
 * 配合任务端DMB；其他任务/ISR不能把这个邮箱当作通用并发命令队列。
 */
typedef struct
{
    int32_t x_mm;          /* OPS固定坐标系绝对X，mm，不是追加位移。 */
    int32_t y_mm;          /* OPS固定坐标系绝对Y，mm。 */
    int32_t yaw_mrad;      /* 绝对朝向，mrad；约1571对应90度。 */
    uint32_t command;     /* 0空闲、1提交位置目标、2请求停止。 */
    uint32_t result;      /* RobotDebugResult，处理结果。 */
    uint32_t completed;   /* 已消费请求的数量；判断是否处理了这一次提交。 */
    uint32_t last_command;/* 最近处理的command，含未知值。 */
    uint32_t sequence;    /* 调试入口自动生成的标记，不用手动填写。 */
} RobotDebugPositionMailbox;

/* 每周期由motorTask调用一次，在OPS更新之后、USB接收之前。
 * mailbox须长期有效且按4字节对齐；本函数不创建任务，也不直接访问电机总线。
 */
void RobotDebugPositionUpdate(volatile RobotDebugPositionMailbox *mailbox,
                              RobotInstance *robot, uint32_t now_ms);
#endif

#endif
