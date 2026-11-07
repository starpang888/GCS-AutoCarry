#ifndef CHASSIS_ODOMETRY_H
#define CHASSIS_ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "omni_kinematics.h"
#include "emm_motor.h"

/**
 * @brief 底盘相对启动原点的位姿。
 *
 * 坐标系在启动时确定，之后不随车头旋转：
 *   X正方向：启动时车头向前。
 *   Y正方向：启动时车头向左。
 *   yaw正方向：从上往下看，逆时针。
 *
 * 初始位姿为：
 *   x = 0 m，y = 0 m，yaw = 0 rad。
 */
typedef struct
{
    float x_m;      /* 固定坐标系中的X位置，单位m。 */
    float y_m;      /* 固定坐标系中的Y位置，单位m。 */
    float yaw_rad;  /* 当前朝向，单位rad，范围[-π, π]。 */
} ChassisPose;

/**
 * @brief 一段采样时间内，底盘在自身坐标系中的运动增量。
 *
 * 注意：这些字段是“位移和转角”，不是速度。
 *
 * 后续由四轮实际转角增量解算得到，不能用目标速度代替。
 */
typedef struct
{
    float dx_m;       /* 车体向前移动的距离，单位m。 */
    float dy_m;       /* 车体向左移动的距离，单位m。 */
    float dyaw_rad;   /* 车体逆时针旋转的角度，单位rad。 */
} ChassisBodyDelta;

/**
 * @brief 里程计配置，由板级组装层提供。
 *
 * 里程计不自行猜测轮径、安装方向、编码器比例。
 * 这些参数必须与实际电机和底盘配置一致。
 */
typedef struct
{
    /* 已初始化的底盘模型，其生命周期必须覆盖里程计实例。 */
    const OmniKinematicsInstance *kinematics;

    /*
     * 每个原始位置计数对应的轮轴转角，单位rad/count。
     *
     * 此比例包含：
     *   编码器每圈计数、安装方向、减速比。
     *
     * 四轮顺序：左前、右前、右后、左后。
     */
    float wheel_rad_per_count[OMNI_WHEEL_COUNT];

    /* 位置反馈允许的最大年龄，单位ms。 */
    uint32_t feedback_timeout_ms;

    /* 同一组四轮反馈允许的最大采样时间差，单位ms。 */
    uint32_t max_sample_span_ms;
} ChassisOdometry_Init_Config_s;

/**
 * @brief 里程计实例，只由motorTask更新。
 */
typedef struct
{
    ChassisOdometry_Init_Config_s config;
    ChassisPose pose;

    /*
     * 上一组已经使用过的原始计数和时间。
     * 使用整数保存累计计数，先做差再转float，
     * 避免累计计数很大后，小位移被浮点精度吞掉。
     */
    int64_t previous_position_raw[OMNI_WHEEL_COUNT];
    uint32_t previous_position_ms[OMNI_WHEEL_COUNT];

    /* 最近一次建立基准或完成积分的任务时间，单位ms。 */
    uint32_t last_update_ms;

    bool initialized;
    bool has_baseline;  /* 是否已经取得第一组四轮位置基准。 */
    bool pose_valid;    /* 当前是否有连续、可用的里程计估计。 */

    /*
     * 位置反馈出现跳变或长时间中断后，无法确认中间运动。
     * 此标志保持到重新初始化，避免恢复通信后假装位置仍准确。
     * 它只描述里程计状态，本步骤不据此新增停车动作。
     */
    bool tracking_lost;
} ChassisOdometryInstance;

/**
 * @brief 保存配置并将起点设置为(0, 0, 0)。
 *
 * 初始化后，等待第一组四轮位置反馈建立计数基准。
 */
bool ChassisOdometryInit(
    ChassisOdometryInstance *instance,
    const ChassisOdometry_Init_Config_s *config);

/**
 * @brief 将一次车体运动增量累计到固定坐标系位姿。
 *
 * @param instance 已初始化的里程计实例。
 * @param delta    本次实际运动增量，不是累计总位移。
 *
 * 一份增量只能使用一次，否则会重复累计。
 * 函数不发送电机命令，也不进行位置控制。
 *
 * @return true 累计成功；false 输入或计算结果无效，位姿不更新。
 */
bool ChassisOdometryIntegrate(
    ChassisOdometryInstance *instance,
    const ChassisBodyDelta *delta);

/**
 * @brief 使用四轮实际累计位置更新里程计。
 *
 * 在motorTask中周期调用，输入为同一次状态快照中的四轮反馈。
 *
 * 返回true：本次完成了一次位姿积分。
 * 返回false：建立基准、等待新反馈或反馈不可用。
 *
 * 返回false不一定代表故障，具体看pose_valid、tracking_lost。
 */
bool ChassisOdometryUpdate(
    ChassisOdometryInstance *instance,
    const EmmMotorFeedback wheels[OMNI_WHEEL_COUNT],
    uint32_t now_ms);

#endif