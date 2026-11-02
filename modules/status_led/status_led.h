/* 非阻塞灯效模块：接收显示模式和时间，计算当前应该亮还是灭。 */
#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdint.h>

typedef enum
{
    STATUS_LED_STARTUP = 0, /* 启动：亮 100 ms、灭 100 ms，循环快闪。 */
    STATUS_LED_READY,       /* 就绪：每秒短闪一次。 */
    STATUS_LED_MOVING,      /* 运动：每秒双闪。 */
    STATUS_LED_FAULT,       /* 故障：每秒三连闪。 */
    STATUS_LED_MODE_COUNT  /* 模式总数，仅用于数组长度和参数检查。 */
} StatusLedMode;

/**
 * @brief 初始化灯效并立即显示启动模式。
 * @param now_ms 当前单调递增的毫秒计时值，允许 uint32_t 自然回绕。
 * @note GPIO 初始化后调用一次；启动后由状态任务独占本模块。
 */
void StatusLedInit(uint32_t now_ms);

/**
 * @brief 根据模式和当前时间更新灯效，不执行任何延时等待。
 * @param mode 目标灯效；非法模式按故障灯效处理。
 * @param now_ms 与初始化时相同时间来源的毫秒计时值。
 * @note 仅由状态任务周期调用，切换模式时从新灯效的第一阶段开始。
 */
void StatusLedUpdate(StatusLedMode mode, uint32_t now_ms);

#endif
