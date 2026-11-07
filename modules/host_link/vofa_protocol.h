/* VOFA FireWater 文本编码：仅负责观测格式，不产生运动命令。 */
#ifndef VOFA_PROTOCOL_H
#define VOFA_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>

/* 输出 ASCII 数字，以逗号分隔、CRLF 结尾，无二进制帧头，也不附加 NUL。
 * milli_mask 的第 i 位为 1 时，第 i 通道按整数/1000 输出三位小数。
 * 仅在任务中调用，输入和输出由调用者拥有，无堆分配或浮点 printf。
 * count 为 1..32，返回实际字节数；参数非法或容量不足返回 0，
 * 失败可能留下部分文本，调用者不得发送不完整输出。
 */
size_t VofaFireWaterEncode(const int64_t *channels, size_t count, uint32_t milli_mask,
                          uint8_t *output, size_t capacity);
#endif
