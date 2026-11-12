/* 应用层定位快照：位置控制与USB POSE共同读取，不绑定电机编码器实例。 */
#ifndef CHASSIS_LOCALIZATION_H
#define CHASSIS_LOCALIZATION_H

#include "chassis_odometry.h" /* 复用ChassisPose的m/rad坐标契约。 */
#include "ops.h"

typedef struct
{
    ChassisPose pose;             /* OPS给出的绝对位姿，不在底盘再次积分。 */
    uint32_t last_update_ms;      /* 本板接受有效新采样的时间，不是OPS的us时钟。 */
    uint32_t feedback_timeout_ms; /* 位姿新鲜度；不要求上位机周期续命令。 */
    uint32_t continuity_epoch;    /* 定位发生中断/重启时改变，用于取消旧目标。 */
    bool valid;                  /* 为false时保留坐标仅供观察，不可闭环。 */
} ChassisLocalizationFeedback;

/* 均由motorTask调用。timeout_ms由板级/任务组装注入，不在算法中写死。 */
void ChassisLocalizationFromOPS(ChassisLocalizationFeedback *feedback,
                                const OPSFeedback *ops, uint32_t timeout_ms);
bool ChassisLocalizationReady(const ChassisLocalizationFeedback *feedback,
                              uint32_t now_ms);

#endif
