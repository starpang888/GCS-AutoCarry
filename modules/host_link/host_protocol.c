/* 主机协议实现：编码显式写字节，解析失败逐字节重同步。 */
#include "host_protocol.h"
#include <string.h>

/** @brief 将整数按小端展开，避免发送 C 结构体带来的填充问题。 */
void HostPutU32(uint8_t *bytes, uint32_t value)
{
    for (size_t i = 0; i < 4U; ++i) bytes[i] = (uint8_t)(value >> (8U * i));
}

/** @brief 按小端拼接 32 位字段。 */
uint32_t HostGetU32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8U |
        (uint32_t)bytes[2] << 16U | (uint32_t)bytes[3] << 24U;
}

/** @brief 逐位计算 CCITT-FALSE，校验数据不包括 CRC 自身。 */
uint16_t HostProtocolCRC(const uint8_t *bytes, size_t length)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= (uint16_t)bytes[i] << 8U;
        for (size_t bit = 0; bit < 8U; ++bit)
            crc = (uint16_t)((crc << 1U) ^ ((crc & 0x8000U) ? 0x1021U : 0U));
    }
    return crc;
}

/** @brief 检查容量后组装版本 1 帧，返回实际总长度。 */
size_t HostProtocolEncode(uint8_t type, uint32_t sequence, const uint8_t *payload,
                           uint16_t length, uint8_t *frame, size_t capacity)
{
    size_t total = (size_t)length + 12U;
    if (frame == NULL || total > capacity || total > HOST_FRAME_CAPACITY ||
        (length != 0U && payload == NULL)) return 0U;
    frame[0] = 0xA5U; frame[1] = 0x5AU; frame[2] = 1U; frame[3] = type;
    frame[4] = (uint8_t)length; frame[5] = (uint8_t)(length >> 8U);
    HostPutU32(frame + 6U, sequence);
    if (length) memcpy(frame + 10U, payload, length);
    uint16_t crc = HostProtocolCRC(frame, total - 2U);
    frame[total - 2U] = (uint8_t)crc;
    frame[total - 1U] = (uint8_t)(crc >> 8U);
    return total;
}

/** @brief 建立固定容量解析器并绑定同步命令回调。 */
void HostProtocolInit(HostProtocolInstance *instance,
                       void (*command)(void *, const HostCommand *), void *context)
{
    *instance = (HostProtocolInstance){.command = command, .context = context};
}

/** @brief 连接变化等事件发生时丢弃旧半帧，保留错误计数。 */
void HostProtocolReset(HostProtocolInstance *instance)
{
    instance->count = 0U;
}

/** @brief 仅移除首字节，继续在缓存中扫描可能的合法帧头。 */
static void Drop(HostProtocolInstance *instance)
{
    --instance->count;
    memmove(instance->bytes, instance->bytes + 1U, instance->count);
}

/** @brief 先检查字节间超时，再依次校验帧头、类型长度及 CRC。 */
void HostProtocolReceive(HostProtocolInstance *instance, uint8_t byte, uint32_t received_ms)
{
    if (instance->count && (uint32_t)(received_ms - instance->last_byte_ms) >= 100U)
    {
        instance->count = 0U;
        ++instance->format_errors;
    }
    instance->last_byte_ms = received_ms;
    instance->bytes[instance->count++] = byte;
    /* 保留可能成为下一帧头的后缀，错帧时不能直接清空所有数据。 */
    while (instance->count)
    {
        uint8_t *frame = instance->bytes;
        if (frame[0] != 0xA5U || (instance->count >= 2U && frame[1] != 0x5AU) ||
            (instance->count >= 3U && frame[2] != 1U))
        {
            Drop(instance);
            continue;
        }
        if (instance->count < 6U) return;
        uint16_t length = (uint16_t)frame[4] | (uint16_t)frame[5] << 8U;
                /*
         * 速度和位置命令都包含三个int32，载荷均为12字节。
         * ARM和STOP不带载荷。
         *
         * 类型与长度一起核对，防止将不完整数据解释为目标。
         */
        bool format =
            ((frame[3] == HOST_VELOCITY ||
              frame[3] == HOST_POSITION) && length == 12U) ||
            ((frame[3] == HOST_ARM ||
              frame[3] == HOST_STOP) && length == 0U);

        if (!format)
        {
            ++instance->format_errors;
            Drop(instance);
            continue;
        }
        size_t total = (size_t)length + 12U;
        if (instance->count < total) return;
        uint16_t crc = (uint16_t)frame[total - 2U] | (uint16_t)frame[total - 1U] << 8U;
        if (crc != HostProtocolCRC(frame, total - 2U))
        {
            ++instance->crc_errors;
            Drop(instance);
            continue;
        }

               /*
         * 未显式赋值的成员自动清零。
         * 类型决定本次填写速度字段还是位置字段。
         */
        HostCommand command = {
            .type = frame[3],
            .sequence = HostGetU32(frame + 6U),
            .received_ms = received_ms,
        };

        if (command.type == HOST_VELOCITY)
        {
            /* 车体速度：mm/s、mm/s、mrad/s。 */
            command.vx_mm_s =
                (int32_t)HostGetU32(frame + 10U);

            command.vy_mm_s =
                (int32_t)HostGetU32(frame + 14U);

            command.wz_mrad_s =
                (int32_t)HostGetU32(frame + 18U);
        }
        else if (command.type == HOST_POSITION)
        {
            /*
             * 固定坐标系目标：mm、mm、mrad。
             *
             * 这里仅按字节解码。
             * 转换为应用层m、rad，在Robot应用层完成。
             */
            command.target_x_mm =
                (int32_t)HostGetU32(frame + 10U);

            command.target_y_mm =
                (int32_t)HostGetU32(frame + 14U);

            command.target_yaw_mrad =
                (int32_t)HostGetU32(frame + 18U);
        }

        /*
         * 解析通过只表示帧完整且CRC正确。
         * 是否执行、使用哪种控制模式，由Robot应用层决定。
         */
        ++instance->valid_frames;
        instance->count = 0U;
        if (instance->command) instance->command(instance->context, &command);
    }
}
