#ifndef ZDT_MOTOR_H
#define ZDT_MOTOR_H

#include <stdint.h>

typedef struct
{
    uint8_t negative;       /* 位置符号：0 为正，1 为负。 */
    uint32_t magnitude;     /* 位置绝对值，65536 对应一圈。 */
    float position_deg;     /* 转换后的带符号累计角度，单位为度。 */
} ZdtPosition;

/**
 * @brief 查询指定地址电机的当前位置。
 * @param address 驱动器地址，本次测试使用 0x01。
 * @param position 成功时写入位置；失败时保留原值。
 * @return 1 表示收到并校验了正常回复，0 表示查询失败。
 */
uint8_t ZdtMotorReadPosition(uint8_t address, ZdtPosition *position);

/**
 * @brief 设置电机使能。
 * @param address 电机地址。
 * @param enable 1 使能，0 失能。
 * @return 1 表示驱动器应答成功，0 表示失败。
 */
uint8_t ZdtMotorEnable(uint8_t address, uint8_t enable);

/**
 * @brief 以速度模式控制电机。
 * @param address 电机地址。
 * @param rpm 带符号转速，单位 RPM；正负表示协议中的两个方向。
 * @param acceleration 加速度参数：0 表示不使用加减速，其余按驱动器定义。
 * @return 1 表示驱动器应答成功，0 表示失败。
 * @note 协议方向不等于车轮前进方向，安装方向后续单独配置。
 */
uint8_t ZdtMotorSetSpeed(
    uint8_t address,
    int16_t rpm,
    uint8_t acceleration);

/**
 * @brief 立即停止电机，保持使能状态。
 * @return 1 表示驱动器应答成功，0 表示失败。
 */
uint8_t ZdtMotorStop(uint8_t address);

#endif

