#include "chassis_position_control.h"

#include <math.h>
#include <stddef.h>

#define POSITION_TWO_PI 6.2831853071795864769f
#define POSITION_PI     3.1415926535897932385f

/**
 * @brief 检查位姿中的三个数是否可以参与计算。
 *
 * 这里只判断数值是否有限。
 * 数据是否新鲜、里程计是否丢失，由调用方判断。
 */
static bool PoseFinite(const ChassisPose *pose)
{
    return pose != NULL &&
        isfinite(pose->x_m) &&
        isfinite(pose->y_m) &&
        isfinite(pose->yaw_rad);
}

bool ChassisPositionControlInit(
    ChassisPositionControlInstance *instance,
    const ChassisPositionControl_Init_Config_s *config)
{
    if (instance == NULL || config == NULL)
    {
        return false;
    }

    /* 增益和容差都要求为正有限数。 */
    if (!isfinite(config->kp_xy) ||
        !isfinite(config->kp_yaw) ||
        !isfinite(config->position_tolerance_m) ||
        !isfinite(config->yaw_tolerance_rad) ||
        config->kp_xy <= 0.0f ||
        config->kp_yaw <= 0.0f ||
        config->position_tolerance_m <= 0.0f ||
        config->yaw_tolerance_rad <= 0.0f ||
        config->yaw_tolerance_rad >= POSITION_PI)
    {
        return false;
    }

    *instance = (ChassisPositionControlInstance){
        .config = *config,
        .initialized = true,
    };

    return true;
}

bool ChassisPositionControlCalculate(
    ChassisPositionControlInstance *instance,
    const ChassisPose *current,
    const ChassisPose *target)
{
    if (instance == NULL)
    {
        return false;
    }

    /*
     * 每次先清除上一次结果。
     * 如果本次计算失败，不能把上一次速度误当成本次有效输出。
     */
    instance->result = (ChassisPositionControlResult){0};

    if (!instance->initialized ||
        !PoseFinite(current) ||
        !PoseFinite(target))
    {
        return false;
    }

    ChassisPositionControlResult result = {0};

    /*
     * 第一步：计算固定坐标系中的位置误差。
     *
     * 例如：
     *   当前X=0.2m，目标X=0.5m，
     *   则固定坐标系X方向还需要移动+0.3m。
     */
    result.error_x_world_m = target->x_m - current->x_m;
    result.error_y_world_m = target->y_m - current->y_m;

    result.distance_m = hypotf(
        result.error_x_world_m,
        result.error_y_world_m);

    /*
     * 第二步：计算最短方向的朝向误差。
     *
     * 先分别归一化，避免两个很大的有限角度直接相减溢出。
     *
     * 例如：
     *   当前+179°，目标-179°，
     *   最短误差应为+2°，不是-358°。
     */
    float current_yaw = remainderf(current->yaw_rad, POSITION_TWO_PI);
    float target_yaw = remainderf(target->yaw_rad, POSITION_TWO_PI);

    result.error_yaw_rad = remainderf(
        target_yaw - current_yaw,
        POSITION_TWO_PI);

    /*
     * 第三步：固定坐标系误差转到车体坐标系。
     *
     * 电机逆解需要的速度是相对当前车头的，
     * 不能把固定坐标系的X、Y误差直接当作车体误差。
     *
     * 旋转变换：
     *   ex_body =  cos(yaw)*ex_world + sin(yaw)*ey_world
     *   ey_body = -sin(yaw)*ex_world + cos(yaw)*ey_world
     *
     * 例：车头朝向固定坐标系+Y，即yaw=90°，
     * 此时想向固定坐标系+X移动，
     * 应当向车体右侧移动，也就是vy为负。
     */
    float cos_yaw = cosf(current_yaw);
    float sin_yaw = sinf(current_yaw);

    result.error_x_body_m =
        cos_yaw * result.error_x_world_m +
        sin_yaw * result.error_y_world_m;

    result.error_y_body_m =
        -sin_yaw * result.error_x_world_m +
        cos_yaw * result.error_y_world_m;

    if (!isfinite(result.distance_m) ||
        !isfinite(result.error_x_body_m) ||
        !isfinite(result.error_y_body_m) ||
        !isfinite(result.error_yaw_rad))
    {
        return false;
    }

    /*
     * 第四步：P控制生成速度。
     *
     * 位置和朝向分别判断：
     *   位置已到、朝向未到：只旋转；
     *   朝向已到、位置未到：只平移；
     *   两者都未到：同时平移和旋转。
     *
     * result初始化为0，进入对应容差时该部分输出保持为0。
     */
    bool position_ok =
        result.distance_m <= instance->config.position_tolerance_m;

    bool yaw_ok =
        fabsf(result.error_yaw_rad) <= instance->config.yaw_tolerance_rad;

    if (!position_ok)
    {
        result.velocity.vx_m_s =
            instance->config.kp_xy * result.error_x_body_m;

        result.velocity.vy_m_s =
            instance->config.kp_xy * result.error_y_body_m;
    }

    if (!yaw_ok)
    {
        result.velocity.wz_rad_s =
            instance->config.kp_yaw * result.error_yaw_rad;
    }

    /*
     * 本模块不施加速度上限或比例缩放。
     * 数值溢出则本次计算失败，不输出无效浮点数。
     */
    if (!isfinite(result.velocity.vx_m_s) ||
        !isfinite(result.velocity.vy_m_s) ||
        !isfinite(result.velocity.wz_rad_s))
    {
        return false;
    }

    result.within_tolerance = position_ok && yaw_ok;
    result.valid = true;

    instance->result = result;
    return true;
}