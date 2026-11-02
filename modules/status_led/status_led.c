/* 用时间差计算灯效阶段，避免长延时阻塞状态切换。 */
#include "status_led.h"
#include "bsp_led.h"

typedef struct
{
    /* 各阶段时长，单位 ms；按“亮、灭、亮、灭……”排列，最多六段。 */
    uint16_t durations_ms[6];
    uint8_t phase_count; /* 当前灯效实际使用的阶段数量。 */
} LedPattern;

/* 灯效参数表：修改时长即可调整节奏，未使用的数组元素不参与计算。 */
static const LedPattern patterns[STATUS_LED_MODE_COUNT] = {
    [STATUS_LED_STARTUP] = {{100U, 100U}, 2U},                         /* 快闪。 */
    [STATUS_LED_READY] = {{100U, 900U}, 2U},                           /* 单闪。 */
    [STATUS_LED_MOVING] = {{100U, 100U, 100U, 700U}, 4U},               /* 双闪。 */
    [STATUS_LED_FAULT] = {{100U, 100U, 100U, 100U, 100U, 500U}, 6U},   /* 三连闪。 */
};

static StatusLedMode current_mode; /* 当前正在显示的模式，由状态任务独占。 */
static uint32_t pattern_start_ms;  /* 当前模式开始显示的毫秒时刻。 */

void StatusLedInit(uint32_t now_ms)
{
    BSPLedInit();
    current_mode = STATUS_LED_STARTUP;
    pattern_start_ms = now_ms;
    StatusLedUpdate(current_mode, now_ms);
}

void StatusLedUpdate(StatusLedMode mode, uint32_t now_ms)
{
    const LedPattern *pattern;
    uint32_t period_ms = 0U;
    uint32_t phase_ms;

    /* 转为无符号数后，负值和超出模式范围的值都会被拦截。 */
    if ((unsigned int)mode >= STATUS_LED_MODE_COUNT)
    {
        mode = STATUS_LED_FAULT;
    }

    /* 只有模式发生变化才重置起点，重复请求同一模式不会打断闪烁。 */
    if (mode != current_mode)
    {
        current_mode = mode;
        pattern_start_ms = now_ms;
    }

    /* 累加各阶段时长，得到当前灯效的一轮总时长。 */
    pattern = &patterns[current_mode];
    for (uint8_t i = 0U; i < pattern->phase_count; ++i)
    {
        period_ms += pattern->durations_ms[i];
    }

    /* 无符号时间差可跨越计数器回绕；取余得到当前位于本轮的第几毫秒。 */
    phase_ms = (now_ms - pattern_start_ms) % period_ms;
    /* 依次扣除已经经过的阶段，定位当前阶段。偶数段亮，奇数段灭。 */
    for (uint8_t i = 0U; i < pattern->phase_count; ++i)
    {
        if (phase_ms < pattern->durations_ms[i])
        {
            BSPLedWrite((i % 2U) == 0U);
            break;
        }
        phase_ms -= pattern->durations_ms[i];
    }
}
