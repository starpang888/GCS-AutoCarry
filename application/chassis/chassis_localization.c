#include "chassis_localization.h"
#include <math.h>
#include <stddef.h>

void ChassisLocalizationFromOPS(ChassisLocalizationFeedback *feedback,
                                const OPSFeedback *ops, uint32_t timeout_ms)
{
    if (feedback == NULL) return;
    if (ops == NULL)
    {
        feedback->valid = false;
        return;
    }
    /* 坐标方向已在OPS积分入口按装车结果修正，这里不能再反号。
     * 原点沿用OPS启动原点；底盘重启不会把第一帧重新设成(0,0,0)。
     */
    *feedback = (ChassisLocalizationFeedback){
        .pose = {.x_m = ops->x_m, .y_m = ops->y_m, .yaw_rad = ops->yaw_rad},
        .last_update_ms = ops->pose_ms,
        .feedback_timeout_ms = timeout_ms,
        .continuity_epoch = ops->continuity_epoch,
        .valid = ops->valid && ops->online && ops->source_valid &&
            ops->continuous && ops->state == 1U && ops->error == 0U,
    };
}

bool ChassisLocalizationReady(const ChassisLocalizationFeedback *feedback,
                              uint32_t now_ms)
{
    return feedback != NULL && feedback->valid &&
        feedback->feedback_timeout_ms != 0U && feedback->feedback_timeout_ms < 0x80000000U &&
        (uint32_t)(now_ms - feedback->last_update_ms) < feedback->feedback_timeout_ms &&
        isfinite(feedback->pose.x_m) && isfinite(feedback->pose.y_m) &&
        isfinite(feedback->pose.yaw_rad);
}
