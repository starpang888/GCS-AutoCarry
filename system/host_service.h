#ifndef HOST_SERVICE_H
#define HOST_SERVICE_H

#include "robot.h"
#include "board_motor.h"
#include "bsp_usb.h"

typedef enum { HOST_FEEDBACK_BINARY = 0, HOST_FEEDBACK_FIREWATER = 1 } HostFeedbackMode;
#define HOST_VOFA_CHANNEL_COUNT 22U

typedef struct
{
    uint32_t received_frames;
    uint32_t crc_errors;
    uint32_t format_errors;
    uint32_t rx_overflows;
    uint32_t tx_skipped;
    uint32_t epoch;
    bool connected;
    HostFeedbackMode feedback_mode;
} HostServiceStatus;

typedef struct
{
    RobotInstance *robot;
    HostProtocolInstance protocol;
    HostServiceStatus status;
    BSPUSBByte rx[32];
    uint8_t payload[196];
    uint8_t tx[HOST_FRAME_CAPACITY];
    int64_t vofa_channels[HOST_VOFA_CHANNEL_COUNT];
    uint32_t tx_sequence;
    uint32_t last_tx_ms;
    uint32_t now_ms;
    bool stop_in_batch;
    bool next_pose; /* 二进制每25ms轮流发送STATUS/POSE；发送成功才轮换。 */
} HostServiceInstance;

/* Construct before USB starts; all service calls owned by motorTask. */
void HostServiceInit(HostServiceInstance *service, RobotInstance *robot);
/* Owner task only. Invalid modes or changes during a host motion session are
 * rejected; accepted mode remains observable in status.feedback_mode.
 */
bool HostServiceSelectFeedback(HostServiceInstance *service, uint32_t mode);
void HostServiceReceive(HostServiceInstance *service, uint32_t now_ms);
void HostServiceTransmit(HostServiceInstance *service, const BoardMotorStatus *motors, uint32_t now_ms);

#endif
