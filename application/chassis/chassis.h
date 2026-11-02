#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>
#include "car_def.h"

/* 底盘当前目标速度。
 * vx：前后平移速度，单位 m/s。
 * vy：左右平移速度，单位 m/s。
 * wz：绕垂直轴旋转速度，单位 rad/s。
 */
typedef struct
{
    float vx_mps;
    float vy_mps;
    float wz_rad_s;
} ChassisTarget;

/* 底盘动作函数集合。
 * 外部模块通过 Chassis.forward() 等函数控制底盘目标。
 */
typedef struct
{
    void (*stop)(void);
    void (*forward)(float speed_mps);
    void (*backward)(float speed_mps);
    void (*left)(float speed_mps);
    void (*right)(float speed_mps);
    void (*rotate_left)(float speed_rad_s);
    void (*rotate_right)(float speed_rad_s);
} ChassisActions;

/* 全局底盘动作接口。 */
extern const ChassisActions Chassis;

/**
 * @brief 初始化底盘目标速度。
 */
void ChassisInit(void);

/**
 * @brief 读取当前底盘目标速度。
 * @param target 输出目标速度。
 */
void ChassisGetTarget(ChassisTarget *target);

/**
 * @brief 将底盘目标速度转换为四个电机目标转速。
 * @param target 底盘目标速度，单位为 m/s 和 rad/s。
 * @param motor_rpm 输出四个电机目标转速，单位为电机轴 RPM。
 * @note X 型全向轮运动学公式由用户填写。
 */
void ChassisCalculateMotorRpm(
    const ChassisTarget *target,
    int16_t motor_rpm[4]);


#if CAR_CHASSIS_DEBUG_ENABLE
/* 调试命令用于通过调试器验证底盘动作和运动学计算。 */
typedef enum
{
    CHASSIS_DEBUG_STOP = 0,
    CHASSIS_DEBUG_FORWARD,
    CHASSIS_DEBUG_BACKWARD,
    CHASSIS_DEBUG_LEFT,
    CHASSIS_DEBUG_RIGHT,
    CHASSIS_DEBUG_ROTATE_LEFT,
    CHASSIS_DEBUG_ROTATE_RIGHT
} ChassisDebugCommand;

/* 调试器修改此变量即可改变底盘目标。 */
extern volatile ChassisDebugCommand chassis_debug_command;

/**
 * @brief 根据调试命令调用对应底盘动作。
 */
void ChassisDebugUpdate(void);
#endif

#endif
