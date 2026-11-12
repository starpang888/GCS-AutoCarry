/* 张大头X固件单机串口：多圈绝对梯形位置与独立反馈。 */
#ifndef X_MOTOR_H
#define X_MOTOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct
{
    /* 在唯一拥有者任务执行。send须复制数据或在tx_busy=false前保持引用；
     * 本实例的request在发送结束前不会改写。context须长期有效。 */
    bool (*send)(void *context, const uint8_t *data, size_t length, uint32_t now_ms);
    bool (*tx_busy)(void *context);
    void *context;
    uint32_t query_period_ms; /* 相邻查询最短间隔；位置、状态交替。 */
    uint32_t response_timeout_ms;
    uint32_t quiet_ms;
    uint32_t quarantine_ms; /* 超时/传输异常后隔离迟到回复。 */
    uint32_t feedback_timeout_ms;
    uint8_t address;
    uint32_t position_tolerance_units; /* 目标关联到达窗口，单位0.1°。 */
} XMotor_Init_Config_s;

typedef struct
{
    uint16_t speed_deci_rpm; /* 1..30000，0.1 RPM。 */
    uint16_t acceleration_rpm_s; /* 1..65535；本接口要求梯形斜坡，不接受0。 */
    uint16_t deceleration_rpm_s;
} XMotorMotionProfile;

typedef enum
{
    X_MOTION_IDLE = 0, X_MOTION_PENDING = 1, X_MOTION_MOVING = 2,
    X_MOTION_REACHED = 3, X_MOTION_STOPPING = 4, X_MOTION_FAULT = 5,
    X_MOTION_DISABLING = 6,
} XMotorMotionState;

typedef enum
{
    X_FAULT_NONE = 0, X_FAULT_FEEDBACK_LOST = 1, X_FAULT_DRIVER = 2,
    X_FAULT_DISABLED = 3, X_FAULT_REJECTED = 4, X_FAULT_COMMAND_TIMEOUT = 5,
    X_FAULT_TRANSPORT = 6, X_FAULT_LINK = 7,
} XMotorFault;

typedef struct
{
    int64_t target_units; /* 最后目标；取消后保留用于观察。 */
    XMotorMotionProfile profile; /* 当前目标的参数快照。 */
    uint32_t generation;
    uint32_t sequence;
    uint32_t rejected_commands;
    XMotorMotionState state;
    XMotorFault fault;
    bool goal_active;
    bool command_accepted; /* 当前代次FD收到了02；不是USB帧合法标志。 */
    bool stop_pending; /* 停止需要确认，确认前拒绝新目标。 */
    bool disable_requested; /* 显式失能意图；完成须看新鲜status_valid && !enabled。 */
} XMotorMotionStatus;

typedef struct
{
    /* 0x36的符号+32位幅值，完整保留±4294967295；X单位0.1°/单位，
     * 一圈3600，支持多圈，不是0x31单圈65536编码器计数，也不使用EMM比例。 */
    int64_t position_units;
    uint32_t position_ms;
    uint32_t status_ms;
    bool position_valid;
    bool status_valid;
    bool enabled;
    bool position_reached; /* 驱动器bit1；尚不代表某条应用目标已完成。 */
    bool stalled;
    bool stall_protection;
    bool left_limit_high;
    bool right_limit_high;
    bool power_loss;
    uint8_t status_raw;
} XMotorFeedback;

typedef struct
{
    uint32_t queries_sent;
    uint32_t position_replies;
    uint32_t status_replies;
    uint32_t rx_bytes;
    uint32_t timeouts;
    uint32_t protocol_errors;
    uint32_t rejected_queries;
    uint32_t transport_errors;
    uint32_t controls_sent;
    uint32_t rejected_controls;
    uint32_t asynchronous_arrivals; /* FD 9F仅计数，不当作当前目标到达。 */
    uint8_t last_rejected_function;
    uint8_t last_rejected_status;
} XMotorDiagnostics;

typedef struct
{
    XMotor_Init_Config_s config;
    XMotorFeedback feedback;
    XMotorDiagnostics diagnostics;
    XMotorMotionStatus motion;
    uint8_t request[16]; /* DMA结束前不可改写。 */
    uint8_t response[8];
    uint8_t response_count;
    uint8_t next_function;
    uint32_t started_ms;
    uint32_t last_query_ms;
    uint32_t last_rx_ms;
    uint32_t guard_started_ms;
    uint32_t guard_duration_ms;
    bool initialized;
    bool active;
    bool response_received;
    bool response_failed;
    uint32_t request_generation;
    uint32_t accepted_position_replies;
    uint32_t accepted_status_replies;
    bool enable_sent;
    uint32_t enable_status_replies;
    bool stop_probe; /* 停止失败后专门查询新状态，不能用发送前的失能位确认。 */
    bool disable_probe; /* F3失能之后查询新状态，ACK不能单独确认失能。 */
} XMotorInstance;

/* 全部接口由一个任务调用；跨任务观察须使用拥有者发布的一致快照。
 * Init不发送任何帧，不清零、不回零、不使能/失能或停止。 */
bool XMotorInit(XMotorInstance *motor, const XMotor_Init_Config_s *config, uint32_t now_ms);
void XMotorUpdate(XMotorInstance *motor, uint32_t now_ms);
/* 任意切片字节流；数据仅在调用内有效，now_ms为任务处理时间，非硬件采样时刻。 */
void XMotorReceive(XMotorInstance *motor, const uint8_t *data, size_t length, uint32_t now_ms);
/* 丢弃在途事务/半帧，作废反馈；活动目标取消并排入停止，恢复后不续跑。 */
void XMotorTransportFault(XMotorInstance *motor, uint32_t now_ms);
/* true仅表示实例已初始化；数值和状态须与各自valid位一起使用。 */
bool XMotorGetFeedback(const XMotorInstance *motor, uint32_t now_ms, XMotorFeedback *feedback);
bool XMotorProfileValid(const XMotorMotionProfile *profile);
/* 唯一拥有者调用；参数错误/反馈未就绪/停止待确认时拒绝，保留旧目标。
 * 相同活动目标仅更新sequence，不重发FD、不增加generation。新目标覆盖待发目标。
 * 目标范围±UINT32_MAX，绝对目标符号不取决于从当前位置运动的方向。 */
bool XMotorSetTarget(XMotorInstance *motor, int64_t target_units,
                     const XMotorMotionProfile *profile, uint32_t sequence, uint32_t now_ms);
/* 取消旧目标，优先停止，保留使能；失败持续尝试，不重发位置。 */
void XMotorStop(XMotorInstance *motor, XMotorFault fault);
/* 显式释放保持力并取消旧目标，继续位置查询。失能确认前拒绝新目标；
 * 确认后新的外部目标仍自动使能。Z轴须由操作者先支撑负载。 */
void XMotorDisable(XMotorInstance *motor);
bool XMotorIsBusy(const XMotorInstance *motor);
bool XMotorGetMotion(const XMotorInstance *motor, uint32_t now_ms, XMotorMotionStatus *motion);

#endif
