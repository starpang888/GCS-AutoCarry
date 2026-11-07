/* 运动学实现：固定轮序和坐标符号，单位统一为 m、s、rad。 */
#include "omni_kinematics.h"
#include <math.h>

#define OMNI_SQRT_HALF 0.7071067811865475244f

/** @brief 拒绝空指针、NaN 与无穷速度。 */
static bool VelocityFinite(const OmniBodyVelocity *velocity)
{
    return velocity != NULL && isfinite(velocity->vx_m_s) &&
        isfinite(velocity->vy_m_s) && isfinite(velocity->wz_rad_s);
}

/** @brief 检查正有限几何参数，预计算旋转几何量。 */
bool OmniKinematicsInit(OmniKinematicsInstance *instance,
                        const OmniKinematics_Init_Config_s *config)
{
    if (instance == NULL || config == NULL)
    {
        return false;
    }
    *instance = (OmniKinematicsInstance){0};
    if (!isfinite(config->wheel_radius_m) || config->wheel_radius_m <= 0.0f ||
        !isfinite(config->wheelbase_m) || config->wheelbase_m <= 0.0f ||
        !isfinite(config->track_width_m) || config->track_width_m <= 0.0f)
    {
        return false;
    }
    float arm = config->wheelbase_m * 0.5f + config->track_width_m * 0.5f;
    if (!isfinite(arm) || arm <= 0.0f ||
        !isfinite(OMNI_SQRT_HALF / config->wheel_radius_m))
    {
        return false;
    }
    instance->config = *config;
    instance->rotation_arm_m = arm;
    instance->initialized = true;
    return true;
}

/** @brief 按几何直接将车体速度逆解为四轮目标，不限幅或缩放。 */
bool OmniKinematicsInverse(const OmniKinematicsInstance *instance,
                           const OmniBodyVelocity *velocity,
                           OmniKinematicsTarget *target)
{
    if (target == NULL)
    {
        return false;
    }
    *target = (OmniKinematicsTarget){0};
    if (instance == NULL || !instance->initialized || !VelocityFinite(velocity))
    {
        return false;
    }
    float vx = velocity->vx_m_s;
    float vy = velocity->vy_m_s;
    float rotation = instance->rotation_arm_m * velocity->wz_rad_s;
    float factor = OMNI_SQRT_HALF / instance->config.wheel_radius_m;
    /* 将车体平移和旋转速度投影到各轮主动滚动方向，再除以轮半径。 */
    float wheels[OMNI_WHEEL_COUNT] = {
        (-vx + vy + rotation) * factor,
        ( vx + vy + rotation) * factor,
        ( vx - vy + rotation) * factor,
        (-vx - vy + rotation) * factor,
    };
    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
    {
        if (!isfinite(wheels[i]))
        {
            return false;
        }
    }
    OmniKinematicsTarget result = {
        .requested = *velocity,
        .applied = *velocity,
        .valid = true,
    };
    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
    {
        result.wheel_rad_s[i] = wheels[i];
    }
    *target = result;
    return true;
}

/** @brief 由实测轮轴速度正解出车体速度，不进行位置积分。 */
bool OmniKinematicsForward(const OmniKinematicsInstance *instance,
                           const float wheel_rad_s[OMNI_WHEEL_COUNT],
                           OmniBodyVelocity *velocity)
{
    if (velocity == NULL)
    {
        return false;
    }
    *velocity = (OmniBodyVelocity){0};
    if (instance == NULL || !instance->initialized || wheel_rad_s == NULL)
    {
        return false;
    }
    /* 先转换并分摊系数再求和，减少大幅但有限观测值的中间溢出。
     * 正解保留测量结果，不施加驱动器或目标速度上限。
     */
    float rolling[OMNI_WHEEL_COUNT];
    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
    {
        if (!isfinite(wheel_rad_s[i]))
        {
            return false;
        }
        rolling[i] = wheel_rad_s[i] * (instance->config.wheel_radius_m * 0.25f / OMNI_SQRT_HALF);
    }
    OmniBodyVelocity result = {
        .vx_m_s = -rolling[0] + rolling[1] + rolling[2] - rolling[3],
        .vy_m_s =  rolling[0] + rolling[1] - rolling[2] - rolling[3],
        .wz_rad_s = (rolling[0] + rolling[1] + rolling[2] + rolling[3]) / instance->rotation_arm_m,
    };
    if (!VelocityFinite(&result))
    {
        return false;
    }
    *velocity = result;
    return true;
}
