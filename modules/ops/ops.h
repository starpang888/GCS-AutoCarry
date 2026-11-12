/* OPS11 接收模块：只解析字节和保存反馈，不依赖 HAL，也不控制电机。 */
#ifndef OPS_H
#define OPS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 与 OPS11 当前发送端一致：最多111字节报文，另留一个字符串结束符。 */
#define OPS_FRAME_CAPACITY 112U

typedef struct
{
    uint32_t feedback_timeout_ms; /* 只判断定位数据新鲜度，不是上位机命令期限。 */
} OPS_Init_Config_s;

typedef struct
{
    /* 最近一次有效且采样时间推进的位姿；无效帧不会覆盖这些坐标。
     * 启动后尚无有效数据时为0，必须同时看valid，不能把0理解为定位成功。
     * 坐标直接使用OPS启动原点，X向前、Y向左、yaw逆时针为正。
     */
    float x_m;
    float y_m;
    float yaw_rad;
    uint32_t sequence;       /* 最近一帧通过格式/校验检查的发送序号。 */
    uint32_t sample_us;      /* 最近一帧的OPS采样时间，不能与本板ms直接相减。 */
    uint32_t received_ms;    /* 本板解析最近一帧的时间。 */
    uint32_t pose_ms;        /* 本板最近接受有效新采样的时间。 */
    uint32_t age_ms;         /* 有效位姿年龄；从未收到时为UINT32_MAX。 */
    uint32_t frames;         /* 格式及校验正确的帧总数，包括无效定位帧。 */
    uint32_t checksum_errors;
    uint32_t format_errors;
    uint32_t repeated_samples; /* 采样时间相同，不刷新位姿新鲜度。 */
    uint32_t time_restarts;    /* 采样时间倒退，可能是OPS重启或乱序。 */
    uint32_t transport_errors;
    uint32_t continuity_epoch; /* 曾有效后失效、时间/序号倒退时+1；跨同批多帧也保留。 */
    uint8_t state;           /* OPS原始状态：0等待、1运行、2丢跟踪、3错误。 */
    uint8_t error;           /* OPS原始错误码，正常为0。 */
    bool source_valid;      /* OPS报文中的valid。 */
    bool continuous;        /* OPS报文中的continuous。 */
    bool online;            /* 最近有校验及格式正确的帧，不保证定位有效。 */
    bool valid;             /* 有效标志、连续跟踪、状态、错误码及新鲜度综合结果。 */
} OPSFeedback;

typedef struct
{
    OPS_Init_Config_s config;
    OPSFeedback feedback;
    char frame[OPS_FRAME_CAPACITY];
    size_t length;
    uint32_t previous_sample_us;
    bool collecting;
    bool have_sample;
    bool have_pose;
} OPSInstance;

/* 以下API由同一个拥有者任务调用。跨任务读取须由拥有者提供一致快照。 */
bool OPSInit(OPSInstance *ops, const OPS_Init_Config_s *config);
/* 可一次输入半帧、多帧或任意切片。data只在调用期间有效，不会被保留。 */
void OPSReceive(OPSInstance *ops, const uint8_t *data, size_t length, uint32_t now_ms);
/* 每周期调用，即使没有新字节，也更新online/valid/age_ms。 */
void OPSUpdate(OPSInstance *ops, uint32_t now_ms);
/* UART错误/溢出后丢弃半帧，撤销有效性；不触发底盘停止或使能。 */
void OPSTransportFault(OPSInstance *ops);

#endif
