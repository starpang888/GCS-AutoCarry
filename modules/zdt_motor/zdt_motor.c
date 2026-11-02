#include "zdt_motor.h"
#include "bsp_uart.h"

uint8_t ZdtMotorReadPosition(uint8_t address, ZdtPosition *position)
{
    uint8_t request[3] = {address, 0x36U, 0x6BU};
    uint8_t response[8] = {0};
    uint32_t magnitude;
    float angle_deg;

    if (position == NULL)
    {
        return 0U;
    }

    /* 查询当前位置，最多等待 50 ms。
     * 回复缓冲区在函数返回前始终有效。
     */
    if (BSPUart5Exchange(
            request, sizeof(request),
            response, sizeof(response),
            50U) != HAL_OK)
    {
        return 0U;
    }

    /* 检查地址、功能码、符号范围和固定校验字节。 */
    if (response[0] != address ||
        response[1] != 0x36U ||
        response[2] > 1U ||
        response[7] != 0x6BU)
    {
        return 0U;
    }

    /* 四字节位置值按高字节在前的顺序组合。
     * 先转换为 uint32_t，再移位，避免有符号移位问题。
     */
    magnitude = ((uint32_t)response[3] << 24)
              | ((uint32_t)response[4] << 16)
              | ((uint32_t)response[5] << 8)
              |  (uint32_t)response[6];

    angle_deg = (float)magnitude * (360.0f / 65536.0f);

    if (response[2] != 0U)
    {
        angle_deg = -angle_deg;
    }

    /* 只有完整回复校验成功后，才更新输出结果。 */
    position->negative = response[2];
    position->magnitude = magnitude;
    position->position_deg = angle_deg;

    return 1U;
}

/**
 * @brief 发送控制指令，检查四字节执行应答。
 * @note 常见 EMM_V5.0 成功应答：
 *       地址、功能码、0x02、0x6B。
 *       驱动器需配置为返回指令执行应答。
 */
static uint8_t SendControlCommand(
    const uint8_t *command,
    uint16_t length)
{
    uint8_t response[4];

    if (BSPUart5Exchange(
            command, length,
            response, sizeof(response),
            50U) != HAL_OK)
    {
        return 0U;
    }

    /* 核对回复属于本电机、本指令，且执行结果为成功。 */
    return response[0] == command[0] &&
           response[1] == command[1] &&
           response[2] == 0x02U &&
           response[3] == 0x6BU;
}

uint8_t ZdtMotorEnable(uint8_t address, uint8_t enable)
{
    /* 格式：地址、功能码、辅助码、使能状态、同步标志、校验。
     * 同步标志为 0：收到指令后立即执行。
     */
    uint8_t command[6] = {
        address,
        0xF3U,
        0xABU,
        enable ? 1U : 0U,
        0x00U,
        0x6BU
    };

    return SendControlCommand(command, sizeof(command));
}

uint8_t ZdtMotorSetSpeed(
    uint8_t address,
    int16_t rpm,
    uint8_t acceleration)
{
    /* 先提升到 int32_t 再取绝对值，保证负数转换正确。 */
    int32_t signed_speed = rpm;
    uint16_t speed = (uint16_t)(
        signed_speed < 0 ? -signed_speed : signed_speed);

    /* 格式：
     * 地址、功能码、方向、速度高字节、速度低字节、
     * 加速度参数、同步标志、校验。
     */
    uint8_t command[8] = {
        address,
        0xF6U,
        rpm < 0 ? 1U : 0U,
        (uint8_t)(speed >> 8),
        (uint8_t)speed,
        acceleration,
        0x00U,
        0x6BU
    };

    return SendControlCommand(command, sizeof(command));
}

uint8_t ZdtMotorStop(uint8_t address)
{
    /* 立即停止指令不等于失能，停止后仍可保持轴的位置。 */
    uint8_t command[5] = {
        address,
        0xFEU,
        0x98U,
        0x00U,
        0x6BU
    };

    return SendControlCommand(command, sizeof(command));
}