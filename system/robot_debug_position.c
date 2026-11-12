#include "robot_debug_position.h"

#if ROBOT_DEBUG_POSITION_ENABLED
#include "stm32f4xx_hal.h" /* 仅使用CMSIS内存屏障，不直接驱动外设。 */
#include <stddef.h>

_Static_assert(sizeof(uint32_t) == 4U &&
               offsetof(RobotDebugPositionMailbox, command) % 4U == 0U,
               "Debug mailbox requires aligned 32-bit command");

void RobotDebugPositionUpdate(volatile RobotDebugPositionMailbox *mailbox,
                              RobotInstance *robot, uint32_t now_ms)
{
    if (mailbox == NULL) return;
    uint32_t action = mailbox->command;
    if (action == ROBOT_DEBUG_NONE) return; /* 没有显式提交，绝不自动运动。 */
    __DMB(); /* 先看发布标志，再读取已提交且不得再修改的坐标。 */

    HostCommand command = {
        .received_ms = now_ms,
        .sequence = mailbox->sequence + 1U,
    };
    RobotDebugResult result = ROBOT_DEBUG_RESULT_REJECTED;
    if (action != ROBOT_DEBUG_POSITION && action != ROBOT_DEBUG_STOP)
    {
        result = ROBOT_DEBUG_RESULT_INVALID_COMMAND;
    }
    else if (robot != NULL && robot->control != NULL)
    {
        if (action == ROBOT_DEBUG_STOP)
        {
            /* 与USB STOP走同一取消目标/停止路径，不只把目标速度改成零。 */
            command.type = HOST_STOP;
            RobotHostCommand(robot, &command, now_ms);
            result = ROBOT_DEBUG_RESULT_STOP_REQUESTED;
        }
        else
        {
            /* 任务内复制为局部完整命令后，再交给正式仲裁。
             * 不直接写target_pose，不绕过OPS有效性、停止确认及代次检查。
             */
            command.type = HOST_POSITION;
            command.target_x_mm = mailbox->x_mm;
            command.target_y_mm = mailbox->y_mm;
            command.target_yaw_mrad = mailbox->yaw_mrad;
            uint32_t rejected_before = robot->control->status.rejected_commands;
            RobotHostCommand(robot, &command, now_ms);
            if (robot->control->status.rejected_commands == rejected_before &&
                robot->mode == ROBOT_POSITION &&
                robot->position_sequence == command.sequence &&
                robot->position_target_milli[0] == command.target_x_mm &&
                robot->position_target_milli[1] == command.target_y_mm &&
                robot->position_target_milli[2] == command.target_yaw_mrad)
            {
                result = ROBOT_DEBUG_RESULT_POSITION_ACCEPTED;
            }
        }
    }

    mailbox->sequence = command.sequence;
    mailbox->last_command = action;
    mailbox->result = (uint32_t)result;
    ++mailbox->completed;
    __DMB(); /* 结果发布完成之后，最后清零command，允许调试器准备下一次请求。 */
    mailbox->command = ROBOT_DEBUG_NONE;
}
#endif
