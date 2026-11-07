/* 标准 X 型 45° 四全向轮数学模型：车体速度与轮轴速度的正反解算。 */
#ifndef OMNI_KINEMATICS_H
#define OMNI_KINEMATICS_H

#include <stdbool.h>
#include <stddef.h>

typedef enum
{
    OMNI_FRONT_LEFT,
    OMNI_FRONT_RIGHT,
    OMNI_REAR_RIGHT,
    OMNI_REAR_LEFT,
    OMNI_WHEEL_COUNT
} OmniWheel;

typedef struct
{
    float vx_m_s;  /* 车体 x 向前，单位 m/s。 */
    float vy_m_s;  /* 车体 y 向左，单位 m/s。 */
    float wz_rad_s; /* z 向上，俯视逆时针为正，单位 rad/s。 */
} OmniBodyVelocity;

typedef struct
{
    float wheel_radius_m;
    float wheelbase_m;   /* 完整前后轮心间距，单位 m，并非半轴距。 */
    float track_width_m; /* 完整左右轮心间距，单位 m，并非半轮距。 */
} OmniKinematics_Init_Config_s;

typedef struct
{
    OmniKinematics_Init_Config_s config;
    float rotation_arm_m; /* 旋转几何量：(轴距 + 轮距) / 2，单位 m。 */
    bool initialized;
} OmniKinematicsInstance;

typedef struct
{
    OmniBodyVelocity requested; /* 输入的原始车体目标。 */
    OmniBodyVelocity applied; /* 等于请求目标，保留反馈协议布局；不是实测反馈。 */
    float wheel_rad_s[OMNI_WHEEL_COUNT];
    bool valid;
} OmniKinematicsTarget;

/* 普通全向轮标准 X 型安装：轮轴沿底盘对角线，主动滚动方向与轮轴垂直。
 * 轮序：左前、右前、右后、左后；正滚动单位向量 (x,y) 为：
 * (-s,+s)、(+s,+s)、(+s,-s)、(-s,-s)，其中 s=1/sqrt(2)。
 * 因此四轮数学轮速同为正时产生正 yaw。驱动器 CW/CCW 与数学正方向
 * 的安装映射在电机边界处理。本层纯数学、单拥有者，不操作 HAL、
 * 电机或时间状态，也不进行里程计积分。
 */
bool OmniKinematicsInit(OmniKinematicsInstance *instance,
                        const OmniKinematics_Init_Config_s *config);
/* 逆解：车体 m/s、rad/s 转为四轮 rad/s。非法输入返回 false 并清空 target。
 * target 不得与输入或实例内存重叠。本层不限制或缩放速度。
 */
bool OmniKinematicsInverse(const OmniKinematicsInstance *instance,
                           const OmniBodyVelocity *velocity,
                           OmniKinematicsTarget *target);
/* 正解：调用方提供新鲜且方向已换算的四轮实测速度，不得用目标冒充测量。
 * 本接口不裁剪观测值；非法输入清空 velocity。输出不得与轮速数组重叠。
 */
bool OmniKinematicsForward(const OmniKinematicsInstance *instance,
                           const float wheel_rad_s[OMNI_WHEEL_COUNT],
                           OmniBodyVelocity *velocity);

#endif
