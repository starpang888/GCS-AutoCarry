#ifndef USB_PROTOCOL_H
#define USB_PROTOCOL_H

#include <stdint.h>

/* 二进制帧帧头。 */
#define USB_PROTOCOL_HEAD_0 0xAAU
#define USB_PROTOCOL_HEAD_1 0x55U

/* LEN 字段允许的最大数据长度。 */
#define USB_PROTOCOL_MAX_LENGTH 18U

/* USB 帧类型。 */
#define USB_FRAME_CMD_CHASSIS_ACTION 0x01U
#define USB_FRAME_CMD_MOTOR_ENABLE   0x02U

/* 底盘动作编号。 */
typedef enum
{
    USB_CHASSIS_STOP = 0U,
    USB_CHASSIS_FORWARD,
    USB_CHASSIS_BACKWARD,
    USB_CHASSIS_LEFT,
    USB_CHASSIS_RIGHT,
    USB_CHASSIS_ROTATE_LEFT,
    USB_CHASSIS_ROTATE_RIGHT
} UsbChassisAction;

/* 最近一次有效的底盘动作命令。 */
typedef struct
{
    uint8_t action;      /* 动作编号。 */
    uint16_t speed;      /* 平移单位 mm/s，旋转单位 mrad/s。 */
} UsbChassisCommand;

/**
 * @brief 初始化 USB 二进制协议解析器。
 */
void UsbProtocolInit(void);

/**
 * @brief 将 USB CDC 收到的数据交给协议解析器。
 * @param data 接收数据。
 * @param length 接收长度。
 */
void UsbProtocolInput(
    const uint8_t *data,
    uint16_t length);

/**
 * @brief 读取最近一次有效的底盘动作命令。
 * @param command 输出命令。
 * @return 1 表示存在有效命令，0 表示尚未收到命令。
 */
uint8_t UsbProtocolGetChassisCommand(
    UsbChassisCommand *command);

/**
 * @brief 读取最近一次有效的电机使能命令。
 * @param enable 输出使能状态。
 * @return 1 表示存在有效命令，0 表示尚未收到命令。
 */
uint8_t UsbProtocolGetMotorEnable(
    uint8_t *enable);

/* 编码器反馈帧类型。 */
#define USB_FRAME_DATA_ENCODER 0x81U

/* 编码器电机数量及反馈帧长度。 */
#define USB_ENCODER_MOTOR_COUNT 4U
#define USB_ENCODER_FRAME_SIZE 27U

/**
 * @brief 将四个电机的位置反馈打包为 USB 二进制帧。
 * @param sequence 帧序号。
 * @param negative 四个电机的位置符号，0 为正，1 为负。
 * @param magnitude 四个电机的位置绝对值。
 * @param frame 输出的完整反馈帧。
 */
void UsbProtocolBuildEncoderFrame(
    uint8_t sequence,
    const uint8_t negative[USB_ENCODER_MOTOR_COUNT],
    const uint32_t magnitude[USB_ENCODER_MOTOR_COUNT],
    uint8_t frame[USB_ENCODER_FRAME_SIZE]);

#endif