#include "usb_protocol.h"

/* 协议解析状态。 */
typedef enum
{
    USB_PARSE_HEAD_0 = 0,
    USB_PARSE_HEAD_1,
    USB_PARSE_LENGTH,
    USB_PARSE_CONTENT,
    USB_PARSE_CRC_LOW,
    USB_PARSE_CRC_HIGH
} UsbParseState;

/* 协议解析运行状态。 */
static UsbParseState parse_state;
static uint8_t frame_length;
static uint8_t frame_index;
static uint8_t frame_content[USB_PROTOCOL_MAX_LENGTH];
static uint16_t received_crc;

/* 最近一次有效的底盘命令。 */
static volatile UsbChassisCommand latest_chassis_command;
static volatile uint8_t chassis_command_valid;

/* 最近一次有效的电机使能命令。 */
static volatile uint8_t latest_motor_enable;
static volatile uint8_t motor_enable_valid;

/* 重置协议解析状态。 */
static void UsbProtocolResetParser(void)
{
    parse_state = USB_PARSE_HEAD_0;
    frame_length = 0U;
    frame_index = 0U;
    received_crc = 0U;
}

/* 计算 CRC16-CCITT。 */
static uint16_t UsbProtocolCalculateCrc(
    const uint8_t *data,
    uint8_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint8_t i = 0U; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;

        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

/* 将四个电机的位置反馈打包成一个二进制帧。 */
void UsbProtocolBuildEncoderFrame(
    uint8_t sequence,
    const uint8_t negative[USB_ENCODER_MOTOR_COUNT],
    const uint32_t magnitude[USB_ENCODER_MOTOR_COUNT],
    uint8_t frame[USB_ENCODER_FRAME_SIZE])
{
    uint16_t crc;
    uint8_t payload_index = 5U;

    frame[0] = USB_PROTOCOL_HEAD_0;
    frame[1] = USB_PROTOCOL_HEAD_1;
    frame[2] = 22U;
    frame[3] = USB_FRAME_DATA_ENCODER;
    frame[4] = sequence;

    /* 每个电机写入符号字节和小端序的位置绝对值。 */
    for (uint8_t i = 0U; i < USB_ENCODER_MOTOR_COUNT; ++i)
    {
        frame[payload_index++] = negative[i];
        frame[payload_index++] = (uint8_t)magnitude[i];
        frame[payload_index++] = (uint8_t)(magnitude[i] >> 8);
        frame[payload_index++] = (uint8_t)(magnitude[i] >> 16);
        frame[payload_index++] = (uint8_t)(magnitude[i] >> 24);
    }

    /* CRC 覆盖 LEN、TYPE、SEQ 和 PAYLOAD。 */
    crc = UsbProtocolCalculateCrc(&frame[2], 23U);
    frame[25] = (uint8_t)crc;
    frame[26] = (uint8_t)(crc >> 8);
}

/* 处理一帧已经通过 CRC 校验的数据。 */
static void UsbProtocolHandleFrame(void)
{
    uint8_t frame_type;
    uint16_t payload_length;
    uint16_t calculated_crc;
    /* CRC 输入包含 LEN 字节和后续帧内容。 */
    uint8_t crc_data[USB_PROTOCOL_MAX_LENGTH + 1U];

    /* 按协议约定，CRC 覆盖 LEN、TYPE、SEQ 和 PAYLOAD。 */
    crc_data[0] = frame_length;

    for (uint8_t i = 0U; i < frame_length; ++i)
    {
        crc_data[i + 1U] = frame_content[i];
    }

    calculated_crc = UsbProtocolCalculateCrc(
        crc_data,
        (uint8_t)(frame_length + 1U));

    if (calculated_crc != received_crc)
    {
        return;
    }

    /* LEN 至少包含 TYPE 和 SEQ 两个字节。 */
    if (frame_length < 2U)
    {
        return;
    }

    frame_type = frame_content[0];
    payload_length = (uint16_t)frame_length - 2U;

    if (frame_type == USB_FRAME_CMD_CHASSIS_ACTION &&
        payload_length == 3U)
    {
        /* 数据格式：动作编号、速度低字节、速度高字节。 */
        latest_chassis_command.action = frame_content[2];
        latest_chassis_command.speed =
            (uint16_t)frame_content[3]
            | ((uint16_t)frame_content[4] << 8);

        chassis_command_valid = 1U;
    }
    else if (frame_type == USB_FRAME_CMD_MOTOR_ENABLE &&
             payload_length == 1U)
    {
        /* 数据格式：0 失能，1 使能。 */
        latest_motor_enable =
            frame_content[2] ? 1U : 0U;

        motor_enable_valid = 1U;
    }
}

/* 初始化协议解析器和最近一次命令。 */
void UsbProtocolInit(void)
{
    UsbProtocolResetParser();

    latest_chassis_command.action = USB_CHASSIS_STOP;
    latest_chassis_command.speed = 0U;
    chassis_command_valid = 0U;

    latest_motor_enable = 0U;
    motor_enable_valid = 0U;
}

/* 逐字节解析 USB CDC 接收到的数据。 */
void UsbProtocolInput(
    const uint8_t *data,
    uint16_t length)
{
    if (data == 0)
    {
        return;
    }

    for (uint16_t i = 0U; i < length; ++i)
    {
        uint8_t byte = data[i];

        switch (parse_state)
        {
        case USB_PARSE_HEAD_0:
            if (byte == USB_PROTOCOL_HEAD_0)
            {
                parse_state = USB_PARSE_HEAD_1;
            }
            break;

        case USB_PARSE_HEAD_1:
            if (byte == USB_PROTOCOL_HEAD_1)
            {
                parse_state = USB_PARSE_LENGTH;
            }
            else if (byte != USB_PROTOCOL_HEAD_0)
            {
                parse_state = USB_PARSE_HEAD_0;
            }
            break;

        case USB_PARSE_LENGTH:
            if (byte < 2U || byte > USB_PROTOCOL_MAX_LENGTH)
            {
                UsbProtocolResetParser();
            }
            else
            {
                frame_length = byte;
                frame_index = 0U;
                parse_state = USB_PARSE_CONTENT;
            }
            break;

        case USB_PARSE_CONTENT:
            frame_content[frame_index++] = byte;

            if (frame_index >= frame_length)
            {
                parse_state = USB_PARSE_CRC_LOW;
            }
            break;

        case USB_PARSE_CRC_LOW:
            received_crc = byte;
            parse_state = USB_PARSE_CRC_HIGH;
            break;

        case USB_PARSE_CRC_HIGH:
            received_crc |= (uint16_t)byte << 8;
            UsbProtocolHandleFrame();
            UsbProtocolResetParser();
            break;

        default:
            UsbProtocolResetParser();
            break;
        }
    }
}

/* 读取最近一次有效的底盘动作命令。 */
uint8_t UsbProtocolGetChassisCommand(
    UsbChassisCommand *command)
{
    if (command == 0 || chassis_command_valid == 0U)
    {
        return 0U;
    }

    command->action = latest_chassis_command.action;
    command->speed = latest_chassis_command.speed;

    return 1U;
}

/* 读取最近一次有效的电机使能命令。 */
uint8_t UsbProtocolGetMotorEnable(
    uint8_t *enable)
{
    if (enable == 0 || motor_enable_valid == 0U)
    {
        return 0U;
    }

    *enable = latest_motor_enable;

    return 1U;
}