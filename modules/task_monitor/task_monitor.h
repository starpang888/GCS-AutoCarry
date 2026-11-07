/* 任务心跳监测：用工作进度判断任务活性，通过回调控制状态灯。 */
#ifndef TASK_MONITOR_H
#define TASK_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#define TASK_MONITOR_CAPACITY 8U
#define TASK_MONITOR_INVALID_ID UINT8_MAX

typedef enum
{
    TASK_MONITOR_STARTING = 0, /* 等待首次心跳，灯保持熄灭。 */
    TASK_MONITOR_HEALTHY, /* 所有已注册任务心跳新鲜，使用正常闪烁周期。 */
    TASK_MONITOR_FAULT /* 首次心跳超出宽限期或后续心跳超时。 */
} TaskMonitor_State_e;

typedef struct
{
    /* Init/Update 同步调用；可能处于调用者临界区，必须短小且不阻塞。 */
    void (*write_led)(void *context, bool on);
    void *context;
    uint32_t startup_grace_ms; /* 等待首次心跳的启动宽限，单位 ms。 */
    uint32_t healthy_half_period_ms; /* 健康时亮/灭各持续多久，完整周期为两倍。 */
    uint32_t fault_half_period_ms; /* 故障时半周期，板级通常配置为更快闪烁。 */
} TaskMonitor_Init_Config_s;

typedef struct
{
    /* 名称只保存指针，存储必须比监测实例长寿，通常使用字符串常量。 */
    const char *name;
    uint32_t timeout_ms;
    uint32_t last_feed_ms;
    bool has_fed; /* 是否至少完成过一次有效工作心跳。 */
} TaskMonitor_Entry_s;

typedef struct
{
    TaskMonitor_Init_Config_s config;
    TaskMonitor_Entry_s entries[TASK_MONITOR_CAPACITY];
    uint32_t started_at_ms;
    uint32_t last_toggle_ms;
    uint8_t count;
    TaskMonitor_State_e state;
    bool led_on;
    bool initialized;
} TaskMonitorInstance;

/* 所有时间使用同一个可回绕 uint32 毫秒时钟。启动时注册任务，
 * 仅在任务完成实际工作后 Feed，不能用无条件定时喂心跳掩盖工作卡死。
 * Feed/Update 并发时由调用者串行保护，参见 system/robot_task.c。
 * 模块不创建/暂停任务或分配内存；状态灯只表示任务进度，
 * 不能据此认定 USB 在线或所有电机反馈正常，也不替代运动停止逻辑。
 */
/** @brief 检查时长和输出回调，初始化为 STARTING 并熄灯。 */
bool TaskMonitorInit(TaskMonitorInstance *instance,
                     const TaskMonitor_Init_Config_s *config, uint32_t now_ms);
/** @brief 按名称注册任务，失败返回 TASK_MONITOR_INVALID_ID。 */
uint8_t TaskMonitorRegister(TaskMonitorInstance *instance, const char *name,
                            uint32_t timeout_ms);
/** @brief 记录该任务最新完成工作的时间，返回是否有效。 */
bool TaskMonitorFeed(TaskMonitorInstance *instance, uint8_t task_id, uint32_t now_ms);
/** @brief 检查所有心跳并按健康/故障半周期更新灯态。 */
void TaskMonitorUpdate(TaskMonitorInstance *instance, uint32_t now_ms);

#endif
