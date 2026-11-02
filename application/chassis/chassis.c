#include "chassis.h"
#include "car_def.h"

#if CAR_CHASSIS_DEBUG_ENABLE
/* 临时调试命令，默认保持停止。 */
volatile ChassisDebugCommand chassis_debug_command =
    CHASSIS_DEBUG_STOP;
#endif

/* 电机正方向修正值，顺序为左前、右前、右后、左后。 */
static const int8_t motor_direction[CAR_MOTOR_COUNT] =
{
    CAR_MOTOR_DIRECTION_LEFT_FRONT,
    CAR_MOTOR_DIRECTION_RIGHT_FRONT,
    CAR_MOTOR_DIRECTION_RIGHT_REAR,
    CAR_MOTOR_DIRECTION_LEFT_REAR
};

/* 圆周率，用于速度单位换算。 */
#define CAR_PI_F 3.14159265358979f
/* 当前底盘目标速度。
 * 该变量由动作函数更新，由后续底盘任务读取。
 */
static volatile ChassisTarget chassis_target =
{
    0.0f,
    0.0f,
    0.0f
};

/* 停止底盘。 */
static void ChassisStop(void)
{
    chassis_target.vx_mps = 0.0f;
    chassis_target.vy_mps = 0.0f;
    chassis_target.wz_rad_s = 0.0f;
}

/* 前进：vx 为正。 */
static void ChassisForward(float speed_mps)
{
    chassis_target.vx_mps = speed_mps;
    chassis_target.vy_mps = 0.0f;
    chassis_target.wz_rad_s = 0.0f;
}

/* 后退：vx 为负。 */
static void ChassisBackward(float speed_mps)
{
    chassis_target.vx_mps = -speed_mps;
    chassis_target.vy_mps = 0.0f;
    chassis_target.wz_rad_s = 0.0f;
}

/* 左移：vy 为正。 */
static void ChassisLeft(float speed_mps)
{
    chassis_target.vx_mps = 0.0f;
    chassis_target.vy_mps = speed_mps;
    chassis_target.wz_rad_s = 0.0f;
}

/* 右移：vy 为负。 */
static void ChassisRight(float speed_mps)
{
    chassis_target.vx_mps = 0.0f;
    chassis_target.vy_mps = -speed_mps;
    chassis_target.wz_rad_s = 0.0f;
}

/* 左旋转：wz 为正。 */
static void ChassisRotateLeft(float speed_rad_s)
{
    chassis_target.vx_mps = 0.0f;
    chassis_target.vy_mps = 0.0f;
    chassis_target.wz_rad_s = speed_rad_s;
}

/* 右旋转：wz 为负。 */
static void ChassisRotateRight(float speed_rad_s)
{
    chassis_target.vx_mps = 0.0f;
    chassis_target.vy_mps = 0.0f;
    chassis_target.wz_rad_s = -speed_rad_s;
}

/* 初始化底盘目标为停止状态。 */
void ChassisInit(void)
{
    ChassisStop();
}

/* 读取底盘目标速度。 */
void ChassisGetTarget(ChassisTarget *target)
{
    if (target == 0)
    {
        return;
    }

    target->vx_mps = chassis_target.vx_mps;
    target->vy_mps = chassis_target.vy_mps;
    target->wz_rad_s = chassis_target.wz_rad_s;
}

/* 对外提供的底盘动作函数表。 */
const ChassisActions Chassis =
{
    .stop = ChassisStop,
    .forward = ChassisForward,
    .backward = ChassisBackward,
    .left = ChassisLeft,
    .right = ChassisRight,
    .rotate_left = ChassisRotateLeft,
    .rotate_right = ChassisRotateRight
};


/**
 * @brief 将底盘目标速度换算成四个电机轴的目标转速。
 * @note 目标速度约定：前进 vx 为正，左移 vy 为正，左转 wz 为正。
 */
void ChassisCalculateMotorRpm(
    const ChassisTarget *target,
    int16_t motor_rpm[CAR_MOTOR_COUNT])
{
    float rotation_speed;
    float wheel_speed[CAR_MOTOR_COUNT];
    float wheel_radius_m;

    if (target == 0 || motor_rpm == 0)
    {
        return;
    }

    /* 旋转时轮子的线速度分量：
     * 等效旋转半径为轴距与轮距的一半之和，尺寸从 mm 换算成 m。
     */
    rotation_speed =
        ((float)(CAR_WHEEL_BASE_MM + CAR_TRACK_WIDTH_MM) / 2000.0f)
        * target->wz_rad_s;

    /* X 型四轮全向底盘的轮速公式。
     * 数组顺序：左前、右前、右后、左后。
     */
    wheel_speed[CAR_MOTOR_LEFT_FRONT] =
        target->vx_mps - target->vy_mps - rotation_speed;

    wheel_speed[CAR_MOTOR_RIGHT_FRONT] =
        target->vx_mps + target->vy_mps + rotation_speed;

    wheel_speed[CAR_MOTOR_RIGHT_REAR] =
        target->vx_mps - target->vy_mps + rotation_speed;

    wheel_speed[CAR_MOTOR_LEFT_REAR] =
        target->vx_mps + target->vy_mps - rotation_speed;

    /* 轮子线速度换算为电机轴 RPM。 */
    wheel_radius_m = (float)CAR_WHEEL_RADIUS_MM / 1000.0f;

    for (uint8_t i = 0U; i < CAR_MOTOR_COUNT; ++i)
    {
        float rpm;

        /* v = 2πr × 轮子转速 / 60，再乘传动比得到电机轴转速。 */
        rpm = wheel_speed[i]
            * (float)motor_direction[i]
            * CAR_MOTOR_GEAR_RATIO
            * 60.0f
            / (2.0f * CAR_PI_F * wheel_radius_m);

        /* 四舍五入，并限制在 int16_t 的表示范围内。 */
        if (rpm > 32767.0f)
        {
            motor_rpm[i] = 32767;
        }
        else if (rpm < -32768.0f)
        {
            motor_rpm[i] = -32768;
        }
        else if (rpm >= 0.0f)
        {
            motor_rpm[i] = (int16_t)(rpm + 0.5f);
        }
        else
        {
            motor_rpm[i] = (int16_t)(rpm - 0.5f);
        }
    }
}

#if CAR_CHASSIS_DEBUG_ENABLE
/**
 * @brief 根据调试命令调用底盘动作函数。
 * @note 调试器修改命令变量后，由底盘任务调用本函数更新目标速度。
 */
void ChassisDebugUpdate(void)
{
    switch (chassis_debug_command)
    {
    case CHASSIS_DEBUG_FORWARD:
        Chassis.forward(0.10f);
        break;

    case CHASSIS_DEBUG_BACKWARD:
        Chassis.backward(0.10f);
        break;

    case CHASSIS_DEBUG_LEFT:
        Chassis.left(0.10f);
        break;

    case CHASSIS_DEBUG_RIGHT:
        Chassis.right(0.10f);
        break;

    case CHASSIS_DEBUG_ROTATE_LEFT:
        Chassis.rotate_left(0.50f);
        break;

    case CHASSIS_DEBUG_ROTATE_RIGHT:
        Chassis.rotate_right(0.50f);
        break;

    case CHASSIS_DEBUG_STOP:
    default:
        Chassis.stop();
        break;
    }
}
#endif
