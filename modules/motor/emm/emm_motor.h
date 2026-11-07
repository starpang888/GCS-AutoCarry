/* EMM V5.0 电机实例：原生协议单位与 SI 轮轴单位之间的边界。 */
#ifndef EMM_MOTOR_H
#define EMM_MOTOR_H

#include "emm_bus.h"

typedef struct
{
    EmmBusInstance *bus;
    uint8_t address; /* 单机地址 1..247，0 留给总线广播。 */
    /* +1 表示原生 CW 对应正轮轴方向，-1 表示相反。由板级安装标定注入，
     * 同时作用于 SI 目标和反馈；不改变原始位置或原生速度接口的符号。
     */
    int8_t direction;
    float gear_ratio; /* 转子转数/轮轴转数；用于位置反馈换算，直驱为 1。 */
    /* 每个线协议速度单位对应的轮轴 rad/s，须包含实际减速关系；0 表示未知，
     * 禁用 SI 输出。带功能码版本通常为 1 RPM/单位，也可能配置为 0.1 RPM。
     * 本板使用名义 1 RPM/单位，完整标定尚未完成，不能自动假定任意驱动配置。
     */
    float wheel_rad_s_per_speed_unit;
} EmmMotor_Init_Config_s;

typedef struct
{
    int64_t position_raw; /* 带符号累计计数，保留原生方向，不是世界绝对位姿。 */
    float wheel_position_rad; /* 已应用方向和减速比的轮轴累计角度 rad。 */
    float wheel_speed_rad_s; /* 有效相邻位置差分所得 rad/s，非目标值或直接测速寄存器。 */
    uint32_t position_ms; /* 最近位置应答处理时间，单位 ms。 */
    uint32_t enabled_ms; /* 最近使能状态应答时间。 */
    uint32_t stall_ms; /* 最近堵转/保护状态应答时间。 */
    bool position_valid; /* 数值必须与有效标志一起使用，失效时旧数值可能仍保留。 */
    bool speed_valid; /* 首次位置、过期采样或位置跳变均不产生有效轮速。 */
    bool enabled_valid;
    bool stall_valid;
    bool enabled;
    bool stalled;
    bool stall_protection;
    uint8_t status_raw;
} EmmMotorFeedback;

struct EmmMotorInstance
{
    EmmMotor_Init_Config_s config;
    EmmMotorFeedback feedback;
    uint32_t last_reply_ms;
    uint32_t last_target_ms;
    uint32_t command_error_count;
    bool registered;
};

/** @brief 检查地址、方向、比例及实例/地址冲突，然后注册到总线。 */
bool EmmMotorInit(EmmMotorInstance *motor, const EmmMotor_Init_Config_s *config);
/* 原生接口：有符号线协议单位 [-5000,5000]，正=CW，负=CCW。
 * 通常 1 RPM/单位，实际分辨率须核对；acceleration=0 表示不使用加速斜坡。
 * 要求总线已授权该电机，且需在 command_timeout_ms 内持续刷新。
 * 同电机队列内的旧速度被最新目标覆盖；返回 true 仅表示入队，不代表实物已执行。
 */
bool EmmMotorSetSpeedUnits(EmmMotorInstance *motor, int16_t signed_units,
                         uint8_t acceleration, uint32_t now_ms);
/* SI 轮轴 rad/s 接口：比例未配置或数值越界返回 false，不隐式猜测 RPM。 */
bool EmmMotorSetSpeed(EmmMotorInstance *motor, float wheel_rad_s,
                     uint8_t acceleration, uint32_t now_ms);
/** @brief true 提交本机使能；false 为整车一致停止，触发全总线停止失能。 */
bool EmmMotorSetEnabled(EmmMotorInstance *motor, bool enabled, uint32_t now_ms);
/** @brief 只允许位置 0x36 与状态 0x3A 查询，回复由总线分发。 */
bool EmmMotorQuery(EmmMotorInstance *motor, EmmTransactionKind query);
/** @brief 复制反馈并按当前时间更新有效性；true 仅表示可复制，仍须检查各有效位。 */
bool EmmMotorGetFeedback(const EmmMotorInstance *motor, uint32_t now_ms,
                         EmmMotorFeedback *feedback);
/** @brief 请求整条总线停止，不新增失能要求；后续以真实反馈确认状态。 */
void EmmMotorStop(EmmMotorInstance *motor, uint32_t now_ms);

#endif
