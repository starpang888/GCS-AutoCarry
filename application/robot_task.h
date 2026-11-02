/* 应用任务入口和整机状态接口；状态灯是当前接入的第一个应用任务。 */
#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include <stdint.h>

/* 四个电机的目标转速，单位为电机轴 RPM。
 * 下标顺序：左前、右前、右后、左后。
 */
extern volatile int16_t motor_target_rpm[4];

typedef enum
{
    ROBOT_STATUS_STARTUP = 0, /* 本地软件正在初始化。 */
    ROBOT_STATUS_READY,       /* 本地初始化完成，不代表 USB 已连接或电机在线。 */
    ROBOT_STATUS_MOVING,      /* 业务模块报告底盘处于运动状态。 */
    ROBOT_STATUS_FAULT        /* 业务模块报告异常，当前用于故障灯效。 */
} RobotStatus;

/**
 * @brief 创建应用任务及其通信资源，目前创建状态队列和状态灯任务。
 * @note 在 GPIO 初始化后、调度器启动前调用一次。
 * @return 1 表示成功；0 表示队列或任务创建失败。
 */
uint8_t RobotTasksInit(void);

/**
 * @brief 请求切换整机显示状态，新状态覆盖尚未消费的旧状态。
 * @param status 目标状态。
 * @return 1 表示请求成功；0 表示参数非法、服务未初始化或写入失败。
 * @note 初始化成功后仅在普通任务中调用，不可在中断中调用。
 *       当前由调用者协调状态优先级，尚未加入统一的故障锁定与恢复策略。
 */
uint8_t RobotSetStatus(RobotStatus status);

#endif
