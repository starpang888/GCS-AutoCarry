#include "task_monitor.h"
#include <stddef.h>

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

typedef struct
{
    bool on;
    uint32_t writes;
} TestLED;

static void WriteLED(void *context, bool on)
{
    TestLED *led = (TestLED *)context;
    led->on = on;
    ++led->writes;
}

/* Runs production module code; returns the failed source line, or zero. */
int RunTaskMonitorTests(void)
{
    TaskMonitorInstance monitor;
    TestLED led = {0};
    TaskMonitor_Init_Config_s config = {
        .write_led = WriteLED,
        .context = &led,
        .startup_grace_ms = 2000U,
        .healthy_half_period_ms = 500U,
        .fault_half_period_ms = 100U,
    };
    CHECK(!TaskMonitorInit(NULL, &config, 0U));
    CHECK(!TaskMonitorInit(&monitor, NULL, 0U));
    CHECK(TaskMonitorInit(&monitor, &config, 0U));
    CHECK(!led.on);
    TaskMonitorUpdate(&monitor, 0U);
    CHECK(monitor.state == TASK_MONITOR_STARTING);

    uint8_t robot = TaskMonitorRegister(&monitor, "robot", 200U);
    uint8_t bus = TaskMonitorRegister(&monitor, "bus", 200U);
    CHECK(robot != TASK_MONITOR_INVALID_ID && bus != TASK_MONITOR_INVALID_ID);
    CHECK(TaskMonitorRegister(&monitor, "robot", 200U) == TASK_MONITOR_INVALID_ID);
    CHECK(TaskMonitorRegister(&monitor, "", 200U) == TASK_MONITOR_INVALID_ID);
    CHECK(TaskMonitorRegister(&monitor, "zero", 0U) == TASK_MONITOR_INVALID_ID);
    CHECK(!TaskMonitorFeed(&monitor, TASK_MONITOR_INVALID_ID, 0U));

    /* An unstarted task must never be reported healthy; grace ends at exactly 2 s. */
    TaskMonitorUpdate(&monitor, 1999U);
    CHECK(monitor.state == TASK_MONITOR_STARTING && !led.on);
    TaskMonitorUpdate(&monitor, 2000U);
    CHECK(monitor.state == TASK_MONITOR_FAULT && led.on);
    TaskMonitorUpdate(&monitor, 2099U);
    CHECK(led.on);
    TaskMonitorUpdate(&monitor, 2100U);
    CHECK(!led.on);

    CHECK(TaskMonitorFeed(&monitor, robot, 2100U));
    TaskMonitorUpdate(&monitor, 2100U);
    CHECK(monitor.state == TASK_MONITOR_FAULT);
    CHECK(TaskMonitorFeed(&monitor, bus, 2100U));
    TaskMonitorUpdate(&monitor, 2100U);
    CHECK(monitor.state == TASK_MONITOR_HEALTHY && led.on);

    /* Exercise normal blink edges while both task heartbeats remain fresh. */
    for (uint32_t now = 2200U; now <= 2500U; now += 100U)
    {
        CHECK(TaskMonitorFeed(&monitor, robot, now));
        CHECK(TaskMonitorFeed(&monitor, bus, now));
        TaskMonitorUpdate(&monitor, now);
        CHECK(monitor.state == TASK_MONITOR_HEALTHY && led.on);
    }
    CHECK(TaskMonitorFeed(&monitor, robot, 2599U));
    CHECK(TaskMonitorFeed(&monitor, bus, 2599U));
    TaskMonitorUpdate(&monitor, 2599U);
    CHECK(led.on);
    CHECK(TaskMonitorFeed(&monitor, robot, 2600U));
    CHECK(TaskMonitorFeed(&monitor, bus, 2600U));
    TaskMonitorUpdate(&monitor, 2600U);
    CHECK(!led.on);

    /* A fresh robot cannot hide a stalled bus. Recovering feed resets blink phase. */
    CHECK(TaskMonitorFeed(&monitor, robot, 2799U));
    TaskMonitorUpdate(&monitor, 2799U);
    CHECK(monitor.state == TASK_MONITOR_HEALTHY);
    TaskMonitorUpdate(&monitor, 2800U);
    CHECK(monitor.state == TASK_MONITOR_FAULT && led.on);
    TaskMonitorUpdate(&monitor, 2900U);
    CHECK(!led.on);
    CHECK(TaskMonitorFeed(&monitor, bus, 2900U));
    TaskMonitorUpdate(&monitor, 2900U);
    CHECK(monitor.state == TASK_MONITOR_HEALTHY && led.on);

    /* A task that has run once uses its heartbeat deadline, not startup grace. */
    CHECK(TaskMonitorInit(&monitor, &config, 0U));
    robot = TaskMonitorRegister(&monitor, "robot", 200U);
    CHECK(TaskMonitorRegister(&monitor, "bus", 200U) != TASK_MONITOR_INVALID_ID);
    CHECK(TaskMonitorFeed(&monitor, robot, 0U));
    TaskMonitorUpdate(&monitor, 199U);
    CHECK(monitor.state == TASK_MONITOR_STARTING);
    TaskMonitorUpdate(&monitor, 200U);
    CHECK(monitor.state == TASK_MONITOR_FAULT);

    /* Millisecond overflow must preserve the exact timeout boundary. */
    CHECK(TaskMonitorInit(&monitor, &config, UINT32_MAX - 99U));
    robot = TaskMonitorRegister(&monitor, "wrap", 200U);
    CHECK(TaskMonitorFeed(&monitor, robot, UINT32_MAX - 49U));
    TaskMonitorUpdate(&monitor, 149U);
    CHECK(monitor.state == TASK_MONITOR_HEALTHY);
    TaskMonitorUpdate(&monitor, 150U);
    CHECK(monitor.state == TASK_MONITOR_FAULT);

    /* Blink timing also survives overflow. */
    config.healthy_half_period_ms = 50U;
    CHECK(TaskMonitorInit(&monitor, &config, UINT32_MAX - 20U));
    robot = TaskMonitorRegister(&monitor, "blink-wrap", 200U);
    CHECK(TaskMonitorFeed(&monitor, robot, UINT32_MAX - 20U));
    TaskMonitorUpdate(&monitor, UINT32_MAX - 20U);
    CHECK(led.on);
    TaskMonitorUpdate(&monitor, 28U);
    CHECK(led.on);
    TaskMonitorUpdate(&monitor, 29U);
    CHECK(!led.on && monitor.state == TASK_MONITOR_HEALTHY);

    static const char *const names[TASK_MONITOR_CAPACITY] = {
        "0", "1", "2", "3", "4", "5", "6", "7"
    };
    CHECK(TaskMonitorInit(&monitor, &config, 0U));
    for (uint8_t i = 0; i < TASK_MONITOR_CAPACITY; ++i)
    {
        CHECK(TaskMonitorRegister(&monitor, names[i], 100U) != TASK_MONITOR_INVALID_ID);
    }
    CHECK(TaskMonitorRegister(&monitor, "overflow", 100U) == TASK_MONITOR_INVALID_ID);
    CHECK(monitor.count == TASK_MONITOR_CAPACITY);

    config.healthy_half_period_ms = 0U;
    CHECK(!TaskMonitorInit(&monitor, &config, 0U));
    return 0;
}
