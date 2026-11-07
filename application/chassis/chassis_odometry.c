#include "chassis_odometry.h"

#include <math.h>
#include <stddef.h>

#define CHASSIS_TWO_PI 6.2831853071795864769f

bool ChassisOdometryInit(
    ChassisOdometryInstance *instance,
    const ChassisOdometry_Init_Config_s *config)
{
    if (instance == NULL || config == NULL ||
        config->kinematics == NULL ||
        !config->kinematics->initialized ||
        config->feedback_timeout_ms == 0U ||
        config->max_sample_span_ms >= config->feedback_timeout_ms)
    {
        return false;
    }

    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
    {
        if (!isfinite(config->wheel_rad_per_count[i]) ||
            config->wheel_rad_per_count[i] == 0.0f)
        {
            return false;
        }
    }

    /*
     * 配置按值复制，板级初始化函数中的临时配置可以安全离开作用域。
     * 只有kinematics是指针，指向工程已有的长期存活模型。
     *
     * 其余成员自动清零：
     *   起点为(0, 0, 0)；
     *   尚无计数基准；
     *   尚无有效位置估计。
     */
    *instance = (ChassisOdometryInstance){
        .config = *config,
        .initialized = true,
    };

    return true;
}

bool ChassisOdometryIntegrate(
    ChassisOdometryInstance *instance,
    const ChassisBodyDelta *delta)
{
    if (instance == NULL || delta == NULL || !instance->initialized)
    {
        return false;
    }

    /* 非有限数不能进入累计，否则后续所有位置都会变成无效值。 */
    if (!isfinite(delta->dx_m) ||
        !isfinite(delta->dy_m) ||
        !isfinite(delta->dyaw_rad))
    {
        return false;
    }

    /*
     * 一次采样期间可能既平移又旋转。
     *
     * 使用“这一段运动中间时刻的朝向”进行坐标转换，
     * 比直接使用起始朝向更适合边走边转。
     */
    float half_yaw = delta->dyaw_rad * 0.5f;
    float middle_yaw = instance->pose.yaw_rad + half_yaw;

    /*
     * 转弯时，实际轨迹是一段弧线。
     *
     * 在采样期间车体速度近似不变的条件下，
     * sinc(half_yaw)用于把这段运动正确转换为净位移。
     *
     * sinc(a) = sin(a) / a。
     *
     * 当a接近0时使用展开式，避免除以0：
     *   sin(a)/a ≈ 1 - a²/6。
     *
     * 这属于坐标积分计算，不是速度限幅。
     */
    float arc_scale;

    if (fabsf(half_yaw) < 0.0001f)
    {
        arc_scale = 1.0f - half_yaw * half_yaw / 6.0f;
    }
    else
    {
        arc_scale = sinf(half_yaw) / half_yaw;
    }

    float cos_yaw = cosf(middle_yaw);
    float sin_yaw = sinf(middle_yaw);

    /*
     * 将车体坐标系位移转换到固定坐标系：
     *
     *   dX = cos(yaw) * dx - sin(yaw) * dy
     *   dY = sin(yaw) * dx + cos(yaw) * dy
     *
     * 例：车头已经逆时针转到90°时，
     * 车体向前移动，对应固定坐标系的Y增加。
     */
    float world_dx = arc_scale *
        (cos_yaw * delta->dx_m - sin_yaw * delta->dy_m);

    float world_dy = arc_scale *
        (sin_yaw * delta->dx_m + cos_yaw * delta->dy_m);

    /*
     * 先计算新位姿，确认有效后再一起保存。
     * 计算失败时保留原来的位姿。
     */
    ChassisPose next_pose = {
        .x_m = instance->pose.x_m + world_dx,
        .y_m = instance->pose.y_m + world_dy,
        .yaw_rad = remainderf(
            instance->pose.yaw_rad + delta->dyaw_rad,
            CHASSIS_TWO_PI),
    };

    /*
     * remainderf将累计朝向归一化到[-π, π]。
     * 例如转一整圈后，朝向重新接近0。
     */
    if (!isfinite(next_pose.x_m) ||
        !isfinite(next_pose.y_m) ||
        !isfinite(next_pose.yaw_rad))
    {
        return false;
    }

    instance->pose = next_pose;
    return true;
}

/**
 * @brief 保存已经使用过的一组计数。
 *
 * 只有建立初始基准或成功积分之后才调用，
 * 等待其他轮反馈期间不能提前覆盖旧计数。
 */
static void SaveWheelBaseline(
    ChassisOdometryInstance *instance,
    const EmmMotorFeedback wheels[OMNI_WHEEL_COUNT])
{
    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
    {
        instance->previous_position_raw[i] = wheels[i].position_raw;
        instance->previous_position_ms[i] = wheels[i].position_ms;
    }

    instance->has_baseline = true;
}

bool ChassisOdometryUpdate(
    ChassisOdometryInstance *instance,
    const EmmMotorFeedback wheels[OMNI_WHEEL_COUNT],
    uint32_t now_ms)
{
    if (instance == NULL || wheels == NULL || !instance->initialized)
    {
        return false;
    }

    /*
     * 已丢失运动连续性时，保留最后位置供观察，
     * 不自动清零，也不自动把它重新标记为有效。
     */
    if (instance->tracking_lost)
    {
        instance->pose_valid = false;
        return false;
    }

    uint32_t youngest_age = UINT32_MAX;
    uint32_t oldest_age = 0U;
    bool all_new = true;

    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
    {
        /*
         * 使用无符号时间差，允许毫秒时钟正常回绕。
         * 板级快照已经提供有效标志，这里同时核对实际年龄。
         */
        uint32_t age = now_ms - wheels[i].position_ms;

        if (!wheels[i].position_valid ||
            age >= instance->config.feedback_timeout_ms)
        {
            instance->pose_valid = false;
            return false;
        }

        if (age < youngest_age)
        {
            youngest_age = age;
        }

        if (age > oldest_age)
        {
            oldest_age = age;
        }

        if (instance->has_baseline)
        {
            uint32_t dt_ms = wheels[i].position_ms -
                instance->previous_position_ms[i];

            if (dt_ms == 0U)
            {
                /* 这一轮尚未取得新位置，暂不积分整组数据。 */
                all_new = false;
            }
            else if (dt_ms >= instance->config.feedback_timeout_ms ||
                     !wheels[i].speed_valid)
            {
                /*
                 * 驱动模块的speed_valid也检查相邻位置是否连续。
                 * 此处只借用它判断计数连续性，不使用轮速进行积分。
                 *
                 * 长时间缺少采样、驱动位置复位或计数跳变时，
                 * 不能把这个差值当成正常移动距离。
                 */
                instance->tracking_lost = true;
                instance->pose_valid = false;
                return false;
            }
        }
    }

    /*
     * 四轮由UART依次查询，并不是硬件同步采样。
     * 时间差太大时等待下一组，避免拼接明显不同时刻的位置。
     */
    if (oldest_age - youngest_age > instance->config.max_sample_span_ms)
    {
        instance->pose_valid = false;
        return false;
    }

    /*
     * 第一组只记录计数基准。
     * 即使电机上电时已有非零累计位置，底盘起点仍然是(0, 0, 0)。
     */
    if (!instance->has_baseline)
    {
        SaveWheelBaseline(instance, wheels);
        instance->last_update_ms = now_ms;
        instance->pose_valid = true;
        return false;
    }

    instance->pose_valid = true;

    if (!all_new)
    {
        return false;
    }

    float wheel_delta_rad[OMNI_WHEEL_COUNT];

    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
    {
        /*
         * EMM原始位置范围为带符号的uint32幅值，
         * 两次相减使用int64不会溢出。
         *
         * 必须先用整数相减，再将“小增量”转成float。
         */
        int64_t delta_count = wheels[i].position_raw -
            instance->previous_position_raw[i];

        wheel_delta_rad[i] = (float)delta_count *
            instance->config.wheel_rad_per_count[i];
    }

    /*
     * 四全向轮正解，同现有OmniKinematicsForward的几何关系。
     *
     * 这里输入的是轮轴转角增量rad，
     * 所以输出是车体位移m、转角rad，不是速度。
     *
     * 系数为 r / (2 * sqrt(2))。
     * 位移已经来自位置差，因此这里不再乘时间dt。
     */
    const OmniKinematicsInstance *model = instance->config.kinematics;
    float factor = model->config.wheel_radius_m * 0.3535533905932738f;

    float d0 = wheel_delta_rad[0];
    float d1 = wheel_delta_rad[1];
    float d2 = wheel_delta_rad[2];
    float d3 = wheel_delta_rad[3];

    ChassisBodyDelta delta = {
        .dx_m = factor * (-d0 + d1 + d2 - d3),
        .dy_m = factor * ( d0 + d1 - d2 - d3),
        .dyaw_rad = factor * (d0 + d1 + d2 + d3) /
            model->rotation_arm_m,
    };

    if (!ChassisOdometryIntegrate(instance, &delta))
    {
        instance->tracking_lost = true;
        instance->pose_valid = false;
        return false;
    }

    /* 本组增量已经累计，记录基准，防止下次重复使用。 */
    SaveWheelBaseline(instance, wheels);
    instance->last_update_ms = now_ms;

    return true;
}