#ifndef CHASSIS_POSITION_CONTROL_H
#define CHASSIS_POSITION_CONTROL_H

#include <stdbool.h>

#include "chassis_odometry.h"
#include "omni_kinematics.h"

/**
 * @brief 位置控制参数。
 *
 * 第一版使用P控制，不包含积分和微分项。
 * 后续根据实际运动表现调整参数。
 */
typedef struct
{
    /*
     * 平移比例增益，单位1/s。
     *
     * 例如kp_xy=0.5，位置误差为0.4m：
     *   输出平移速度为0.2m/s。
     *
     * X、Y使用相同增益，使平移方向朝向目标点。
     */
    float kp_xy;

    /*
     * 朝向比例增益，单位1/s。
     *
     * 例如kp_yaw=1.0，角度误差为0.5rad：
     *   输出角速度为0.5rad/s。
     */
    float kp_yaw;

    /*
     * 平面位置容差，单位m。
     *
     * 判断距离使用sqrt(ex²+ey²)，
     * 不是分别判断X、Y是否进入两个独立窗口。
     */
    float position_tolerance_m;

    /* 朝向容差，单位rad。 */
    float yaw_tolerance_rad;
} ChassisPositionControl_Init_Config_s;

/**
 * @brief 一次位置控制计算的结果。
 */
typedef struct
{
    /* 固定坐标系中的位置误差，单位m。 */
    float error_x_world_m;
    float error_y_world_m;

    /* 车体坐标系中的位置误差，单位m。 */
    float error_x_body_m;
    float error_y_body_m;

    /* 到目标点的平面距离，单位m。 */
    float distance_m;

    /* 最短方向的角度误差，单位rad，范围[-π, π]。 */
    float error_yaw_rad;

    /*
     * 车体坐标系速度：
     *   vx：沿当前车头向前，m/s；
     *   vy：沿当前车头向左，m/s；
     *   wz：逆时针旋转，rad/s。
     *
     * 可交给现有底盘逆解使用。
     */
    OmniBodyVelocity velocity;

    /*
     * 只表示当前几何误差进入容差。
     *
     * 不等于“已经稳定停车”。
     * 正式到达判断还需要实际速度和持续稳定时间，
     * 后续在应用层加入。
     */
    bool within_tolerance;

    /* 本次计算是否成功，不能只看velocity是否非零。 */
    bool valid;
} ChassisPositionControlResult;

/**
 * @brief 位置控制器实例，由motorTask单独使用。
 */
typedef struct
{
    ChassisPositionControl_Init_Config_s config;
    ChassisPositionControlResult result;
    bool initialized;
} ChassisPositionControlInstance;

/**
 * @brief 保存控制参数，初始化时不产生运动。
 */
bool ChassisPositionControlInit(
    ChassisPositionControlInstance *instance,
    const ChassisPositionControl_Init_Config_s *config);

/**
 * @brief 根据当前位姿和目标位姿计算车体速度。
 *
 * current和target必须属于同一个固定坐标系。
 * target是绝对目标，不是每次需要追加移动的距离。
 *
 * 本函数只计算，不发送命令，也不直接操作电机。
 * 调用方负责确认当前里程计有效，并决定是否采用计算结果。
 *
 * @return true 计算成功；false 输入或计算结果无效。
 */
bool ChassisPositionControlCalculate(
    ChassisPositionControlInstance *instance,
    const ChassisPose *current,
    const ChassisPose *target);

#endif