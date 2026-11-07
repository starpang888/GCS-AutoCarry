/* 底盘持续控制：速度帧自动使能并保持目标，STOP 撤销运动许可。 */
#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "emm_motor.h"
#include "omni_kinematics.h"
#include "host_protocol.h"

/* 状态依次为失能、等待使能、运行、等待停止、故障；数值同时用于 USB 反馈。 */
typedef enum { CHASSIS_DISABLED, CHASSIS_ENABLING, CHASSIS_ACTIVE, CHASSIS_STOPPING, CHASSIS_FAULT } ChassisControlState;
/* 故障值对应 USB 协议；NOT_READY 保留，部分命令拒绝仅累加计数。 */
typedef enum
{
    CHASSIS_OK, CHASSIS_COMMAND_TIMEOUT, CHASSIS_LINK_LOST, CHASSIS_MOTOR_ERROR,
    CHASSIS_ENABLE_TIMEOUT, CHASSIS_STOP_TIMEOUT, CHASSIS_INVALID_COMMAND, CHASSIS_NOT_READY
} ChassisControlFault;

typedef struct
{
    EmmBusInstance *bus;
    EmmMotorInstance *motors[OMNI_WHEEL_COUNT];
    OmniKinematicsInstance *kinematics;
} ChassisControl_Init_Config_s;

typedef struct
{
    ChassisControlState state;
    ChassisControlFault fault;
    uint32_t last_sequence; /* 最后接受的 ARM/速度序号，不是通用应答。 */
    uint32_t last_command_ms; /* 最近接受命令的接收时间，仅供诊断，不设主机期限。 */
    uint32_t rejected_commands;
    OmniKinematicsTarget target; /* 请求及解算目标，均不代表实测速度。 */
} ChassisControlStatus;

typedef struct
{
    ChassisControl_Init_Config_s config;
    ChassisControlStatus status;
    uint32_t phase_ms; /* 本次使能或停止阶段的起点，单位 ms。 */
    uint32_t last_output_ms; /* 上次向电机提交四轮速度的任务时间。 */
    bool initialized;
    bool start_pending; /* 已收到命令，等待总线/四轮失能反馈就绪后启动。 */
} ChassisControlInstance;

/* 全部接口由 motorTask 单独拥有。VELOCITY 自动使能，保持最新目标直到新命令或
 * STOP/设备故障/USB断开。ARM 是可选零速预使能，运行中重复 ARM 不改变目标。
 * sequence 仅回传，不去重；主机不需要周期续期。故障清除目标，恢复需新命令。
 * external_busy 表示外部占用。底层设备通信与周期输出故障仍会停止。
 */
/** @brief 检查四轮总线、实例顺序及单位换算配置，初始化为失能状态。 */
bool ChassisControlInit(ChassisControlInstance *control, const ChassisControl_Init_Config_s *config);
/** @brief 等待启动、使能中、运行中或停止确认中返回 true。 */
bool ChassisControlIsBusy(const ChassisControlInstance *control);
/* 应用层SI入口：直接保存m/s、rad/s目标并申请自动使能。
 * 不修改USB接受序号和接收时间；返回true只表示保存成功，不代表实物已执行。
 * 与Command一样仅由motorTask调用；位置应用负责故障后的取消，不能自动续启旧目标。
 */
bool ChassisControlSetVelocity(ChassisControlInstance *control,
                               const OmniBodyVelocity *velocity, uint32_t now_ms);
/** @brief 保存最新速度并请求自动使能；STOP 清零并请求失能。 */
void ChassisControlCommand(ChassisControlInstance *control, const HostCommand *command,
                            bool external_busy, uint32_t now_ms);
/** @brief 撤销目标并请求全总线停止失能，等待实际状态确认。 */
void ChassisControlStop(ChassisControlInstance *control, ChassisControlFault fault, uint32_t now_ms);
/** @brief 周期检查设备反馈，推进使能和停止确认并持续输出目标。 */
void ChassisControlUpdate(ChassisControlInstance *control, uint32_t now_ms);

#endif
