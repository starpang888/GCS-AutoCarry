/* FireWater 格式化实现：用整数运算输出有符号数及固定三位小数。 */
#include "vofa_protocol.h"
#include <stdbool.h>

/** @brief 容量足够时追加一个字符，失败不越界写入。 */
static bool Put(uint8_t *output, size_t capacity, size_t *length, uint8_t byte)
{
    if (*length >= capacity) return false;
    output[(*length)++] = byte;
    return true;
}

/** @brief 编码一个有符号通道，milli 决定是否插入三位小数。 */
static bool Number(int64_t value, bool milli, uint8_t *output, size_t capacity, size_t *length)
{
    /* 先转无符号再做减法，避免直接取 -INT64_MIN 产生有符号溢出。 */
    uint64_t magnitude = value < 0 ? (uint64_t)0 - (uint64_t)value : (uint64_t)value;
    if (value < 0 && !Put(output, capacity, length, '-')) return false;
    uint64_t integer = milli ? magnitude / 1000U : magnitude;
    uint8_t reversed[20]; /* uint64 最大值需要 20 位十进制数字。 */
    size_t digits = 0U;
    do
    {
        reversed[digits++] = (uint8_t)('0' + integer % 10U);
        integer /= 10U;
    } while (integer);
    while (digits)
        if (!Put(output, capacity, length, reversed[--digits])) return false;
    if (milli)
    {
        unsigned fraction = (unsigned)(magnitude % 1000U);
        if (!Put(output, capacity, length, '.') ||
            !Put(output, capacity, length, (uint8_t)('0' + fraction / 100U)) ||
            !Put(output, capacity, length, (uint8_t)('0' + fraction / 10U % 10U)) ||
            !Put(output, capacity, length, (uint8_t)('0' + fraction % 10U))) return false;
    }
    return true;
}

/** @brief 编码整行通道并追加 CRLF；任一步失败返回 0。 */
size_t VofaFireWaterEncode(const int64_t *channels, size_t count, uint32_t milli_mask,
                          uint8_t *output, size_t capacity)
{
    if (channels == NULL || output == NULL || count == 0U || count > 32U) return 0U;
    size_t length = 0U;
    for (size_t i = 0U; i < count; ++i)
    {
        if (i && !Put(output, capacity, &length, ',')) return 0U;
        if (!Number(channels[i], (milli_mask & (1UL << i)) != 0U, output, capacity, &length)) return 0U;
    }
    if (!Put(output, capacity, &length, '\r') || !Put(output, capacity, &length, '\n')) return 0U;
    return length;
}
