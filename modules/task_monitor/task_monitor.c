/* 任务监测实现：首次心跳使用启动宽限，后续心跳使用各任务超时。 */
#include "task_monitor.h"
#include <stddef.h>
#include <string.h>

/** @brief 限定为正且不超过 INT32_MAX，使回绕时钟比较保持明确。 */
static bool ValidInterval(uint32_t interval_ms)
{
    return interval_ms > 0U && interval_ms <= INT32_MAX;
}

/** @brief 校验配置后保存起点，通过注入回调熄灯。 */
bool TaskMonitorInit(TaskMonitorInstance *instance,
                     const TaskMonitor_Init_Config_s *config, uint32_t now_ms)
{
    if (instance == NULL || config == NULL || config->write_led == NULL ||
        !ValidInterval(config->startup_grace_ms) ||
        !ValidInterval(config->healthy_half_period_ms) ||
        !ValidInterval(config->fault_half_period_ms))
    {
        return false;
    }

    *instance = (TaskMonitorInstance){0};
    instance->config = *config;
    instance->started_at_ms = now_ms;
    instance->last_toggle_ms = now_ms;
    instance->state = TASK_MONITOR_STARTING;
    instance->initialized = true;
    instance->config.write_led(instance->config.context, false);
    return true;
}

/** @brief 拒绝空名称、重名、非法超时及容量超限。 */
uint8_t TaskMonitorRegister(TaskMonitorInstance *instance, const char *name,
                            uint32_t timeout_ms)
{
    if (instance == NULL || !instance->initialized || name == NULL || name[0] == '\0' ||
        !ValidInterval(timeout_ms) || instance->count >= TASK_MONITOR_CAPACITY)
    {
        return TASK_MONITOR_INVALID_ID;
    }

    for (uint8_t i = 0; i < instance->count; ++i)
    {
        if (strcmp(instance->entries[i].name, name) == 0)
        {
            return TASK_MONITOR_INVALID_ID;
        }
    }

    uint8_t task_id = instance->count++;
    instance->entries[task_id] = (TaskMonitor_Entry_s){
        .name = name,
        .timeout_ms = timeout_ms,
    };
    return task_id;
}

/** @brief 记录有效任务的最近心跳；同步由调用方保证。 */
bool TaskMonitorFeed(TaskMonitorInstance *instance, uint8_t task_id, uint32_t now_ms)
{
    if (instance == NULL || !instance->initialized || task_id >= instance->count)
    {
        return false;
    }

    instance->entries[task_id].last_feed_ms = now_ms;
    instance->entries[task_id].has_fed = true;
    return true;
}

/** @brief 任何一项超时即故障，全部已喂且新鲜才健康。 */
static TaskMonitor_State_e GetState(const TaskMonitorInstance *instance, uint32_t now_ms)
{
    /* 无任务或仍在等首次心跳时保持 STARTING，不直接宣称健康。 */
    bool waiting = (instance->count == 0U);
    for (uint8_t i = 0; i < instance->count; ++i)
    {
        const TaskMonitor_Entry_s *entry = &instance->entries[i];
        if (!entry->has_fed)
        {
            if ((uint32_t)(now_ms - instance->started_at_ms) >= instance->config.startup_grace_ms)
            {
                return TASK_MONITOR_FAULT;
            }
            waiting = true;
        }
        else if ((uint32_t)(now_ms - entry->last_feed_ms) >= entry->timeout_ms)
        {
            return TASK_MONITOR_FAULT;
        }
    }
    return waiting ? TASK_MONITOR_STARTING : TASK_MONITOR_HEALTHY;
}

/** @brief 状态切换后重置灯相位，随后按半周期非阻塞翻转。 */
void TaskMonitorUpdate(TaskMonitorInstance *instance, uint32_t now_ms)
{
    if (instance == NULL || !instance->initialized)
    {
        return;
    }

    TaskMonitor_State_e state = GetState(instance, now_ms);
    /* 状态切换重置闪烁相位；FAULT 可以在所有任务重新新鲜后恢复。 */
    if (state != instance->state)
    {
        instance->state = state;
        instance->last_toggle_ms = now_ms;
        instance->led_on = (state != TASK_MONITOR_STARTING);
        instance->config.write_led(instance->config.context, instance->led_on);
    }

    if (state != TASK_MONITOR_STARTING)
    {
        uint32_t half_period_ms = (state == TASK_MONITOR_HEALTHY)
                                      ? instance->config.healthy_half_period_ms
                                      : instance->config.fault_half_period_ms;
        if ((uint32_t)(now_ms - instance->last_toggle_ms) >= half_period_ms)
        {
            instance->last_toggle_ms = now_ms;
            instance->led_on = !instance->led_on;
            instance->config.write_led(instance->config.context, instance->led_on);
        }
    }
}
