#include "board.h"
#include "bsp_led.h"
#include "main.h"

static BSPLEDInstance status_led;

static void WriteStatusLED(void *context, bool on)
{
    BSPLEDSet((BSPLEDInstance *)context, on);
}

bool BoardTaskMonitorInit(TaskMonitorInstance *monitor, uint32_t now_ms)
{
    const BSPLED_Init_Config_s led_config = {
        .port = STATUS_LED_GPIO_Port,
        .pin = STATUS_LED_Pin,
        .active_level = GPIO_PIN_SET,
    };
    if (!BSPLEDInit(&status_led, &led_config))
    {
        return false;
    }

    const TaskMonitor_Init_Config_s monitor_config = {
        .write_led = WriteStatusLED,
        .context = &status_led,
        .startup_grace_ms = 2000U,
        .healthy_half_period_ms = 500U,
        .fault_half_period_ms = 100U,
    };
    return TaskMonitorInit(monitor, &monitor_config, now_ms);
}
