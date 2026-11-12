/* USB 主机业务协议：固定小端帧、CRC 校验和流式命令解析，不依赖 USB 硬件。 */
#ifndef HOST_PROTOCOL_H
#define HOST_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HOST_FRAME_CAPACITY 240U
#define HOST_VELOCITY 1U
#define HOST_ARM 2U
#define HOST_STOP 3U
#define HOST_POSITION 4U  /* 位置目标：固定坐标系下的绝对X、Y、yaw。 */
#define HOST_CRANE_TARGET 5U /* axis:u8 reserved:3 target:i64LE，12字节。 */
#define HOST_CRANE_CONFIG 6U /* axis/reserved:4 speed/acc/dec/reserved:u16LE，12字节。 */
#define HOST_CRANE_STOP 7U   /* axis:u8，1字节；原STOP停底盘和两轴。 */
#define HOST_SERVO_PULSE 8U  /* rack servo pulse:u16LE，单位us。 */
#define HOST_GRIPPER_SERVO_PULSE 9U /* gripper servo pulse:u16LE，单位us。 */
#define HOST_CRANE_PICK 0x0AU /* 独立夹取事件，无载荷；重复sequence不重复执行。 */
#define HOST_CRANE_PLACE 0x0BU /* 独立放下事件，无载荷。 */
#define HOST_CRANE_DISABLE 0x0CU /* axis:u8，显式失能用于手动找点，继续反馈。 */
#define HOST_STATUS 0x81U
#define HOST_POSE 0x82U          /* 独立位姿/到达反馈；旧STATUS长度不变。 */
#define HOST_POSE_PAYLOAD_SIZE 64U
#define HOST_CRANE 0x83U
#define HOST_CRANE_PAYLOAD_SIZE 116U
#define HOST_CRANE_ACTION 0x84U
#define HOST_CRANE_ACTION_PAYLOAD_SIZE 40U

/**
 * @brief 解析后的上位机命令。
 *
 * type决定哪些字段有效：
 *   HOST_VELOCITY：读取vx_mm_s、vy_mm_s、wz_mrad_s；
 *   HOST_POSITION：读取target_x_mm、target_y_mm、target_yaw_mrad；
 *   HOST_ARM/STOP/CRANE_PICK/CRANE_PLACE：没有载荷。
 *   HOST_CRANE_TARGET/CONFIG/STOP/DISABLE：读取crane_*，原生驱动位置单位见X模块。
 *   HOST_SERVO_PULSE：读取servo_pulse_us，应用层按板级脉宽范围校验。
 *   HOST_GRIPPER_SERVO_PULSE：读取gripper_servo_pulse_us，夹爪舵机脉宽。
 *
 * 这是程序内部结构，不直接作为通信帧发送。
 * 增加成员不会改变已有USB帧格式。
 */
typedef struct
{
    uint8_t type;

    /* 普通目标可重复；联动事件按动作类型记录最近接受值去重，再执行需换值。 */
    uint32_t sequence;

    /* 本帧最后一字节的接收时间，单位ms。 */
    uint32_t received_ms;

    /* 速度命令：相对于当前车体坐标系。 */
    int32_t vx_mm_s;
    int32_t vy_mm_s;
    int32_t wz_mrad_s;

    /*
     * 位置命令：相对于启动时建立的固定坐标系。
     *
     * 这些是绝对目标，不是追加移动量。
     * 重复发送同一目标，不应重复增加行程。
     */
    int32_t target_x_mm;
    int32_t target_y_mm;
    int32_t target_yaw_mrad;
    int64_t crane_target_units;
    uint32_t crane_reserved; /* 合并所有保留字节，应用要求0。 */
    uint16_t crane_speed_deci_rpm;
    uint16_t crane_acceleration_rpm_s;
    uint16_t crane_deceleration_rpm_s;
    uint8_t crane_axis;
    uint16_t servo_pulse_us;
    uint16_t gripper_servo_pulse_us;
} HostCommand;

typedef struct
{
    void (*command)(void *context, const HostCommand *command);
    void *context;
    uint8_t bytes[24]; /* 最大输入帧：10字节头 + 12字节载荷 + 2字节CRC。 */
    size_t count;
    uint32_t last_byte_ms;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t format_errors;
} HostProtocolInstance;

/* 仅由拥有者任务调用。解析成功时同步调用 command(context, command)，
 * 命令指针仅在回调期间有效，需长期保存时由接收者复制。
 * 字节时间必须反映接收时刻，不能使用延迟后的任务处理时间。
 * 帧：A5 5A 01 type len:u16LE seq:u32LE payload crc:u16LE。
 * CRC-16/CCITT-FALSE 覆盖帧头和载荷，poly=0x1021、init=0xFFFF、
 * 不反射、末尾不异或。这里只处理帧合法性，运动许可由应用层判定。
 */
/** @brief 初始化解析缓存、回调和计数，instance 必须有效。 */
void HostProtocolInit(HostProtocolInstance *instance,
                       void (*command)(void *, const HostCommand *), void *context);
/** @brief 仅丢弃未完成帧，保留历史统计及回调。 */
void HostProtocolReset(HostProtocolInstance *instance);
/** @brief 逐字节解析，支持任意拆包/粘包，字节间隔达到 100 ms 则丢弃半帧。 */
void HostProtocolReceive(HostProtocolInstance *instance, uint8_t byte, uint32_t received_ms);
/** @brief 计算指定字节序列的 CRC；bytes 至少包含 length 字节。 */
uint16_t HostProtocolCRC(const uint8_t *bytes, size_t length);
/** @brief 将载荷复制到完整帧中；容量不足或参数非法返回 0。 */
size_t HostProtocolEncode(uint8_t type, uint32_t sequence, const uint8_t *payload,
                           uint16_t length, uint8_t *frame, size_t capacity);
/** @brief 写入小端 uint32，目标至少有 4 字节。 */
void HostPutU32(uint8_t *bytes, uint32_t value);
/** @brief 按字节读取小端 uint32，不依赖指针对齐。 */
uint32_t HostGetU32(const uint8_t *bytes);

#endif
