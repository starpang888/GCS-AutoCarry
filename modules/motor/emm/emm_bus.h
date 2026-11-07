/* EMM 多机串行总线：唯一拥有者串行事务、按地址分发、统一运动许可与停止。 */
#ifndef EMM_BUS_H
#define EMM_BUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define EMM_MOTOR_CAPACITY 4U
#define EMM_QUEUE_CAPACITY 16U
#define EMM_CHECK_BYTE 0x6BU
/* EMM指令支持的原生速度范围；底盘应用不另设轮速上限。 */
#define EMM_MAX_SPEED_UNITS 5000

typedef struct EmmMotorInstance EmmMotorInstance;
typedef enum
{
    EMM_COMMAND_SPEED,
    EMM_COMMAND_ENABLE,
    EMM_QUERY_POSITION,
    EMM_QUERY_STATUS,
    EMM_STOP_ALL,
    EMM_DISABLE_ALL
} EmmTransactionKind;

typedef struct
{
    EmmMotorInstance *motor;
    EmmTransactionKind kind;
    uint8_t frame[8]; /* 事务自带帧副本，避免保存调用者栈指针。 */
    uint8_t length;
    uint8_t response_length; /* 固定预期长度，不能按数据中的 0x6B 提前截帧。 */
} EmmTransaction;

/* 回调在总线拥有者任务内执行，不在 ISR 中调用。send 返回前须复制数据，
 * 或保证异步使用结束前帧仍有效；本总线 current.frame 保持到 tx_busy=false。
 * context 由板级注入，生命周期覆盖总线。全部总线和电机 API 只能由
 * 同一个任务调用；BSP 中断只缓存原始字节，不能并发调用本模块。
 */
typedef struct
{
    bool (*send)(void *context, const uint8_t *data, size_t length, uint32_t now_ms);
    bool (*tx_busy)(void *context);
    void *context;
    uint32_t response_timeout_ms; /* 从开始发送起算的回复期限，包含发送过程。 */
    uint32_t quiet_ms; /* 最近接收及事务间最短静默间隔。 */
    uint32_t quarantine_ms; /* 失败后的隔离时长，防止迟到回复误配新事务。 */
    uint32_t command_timeout_ms; /* 每个已授权电机的速度目标刷新期限。 */
    uint32_t feedback_timeout_ms; /* 位置、使能和堵转反馈新鲜度期限。 */
} EmmBus_Init_Config_s;

typedef struct
{
    EmmBus_Init_Config_s config;
    EmmMotorInstance *motors[EMM_MOTOR_CAPACITY];
    EmmTransaction queue[EMM_QUEUE_CAPACITY]; /* 固定队列，同轮同类项采用最新值覆盖。 */
    EmmTransaction current; /* 当前唯一在途事务，物理发送结束前不能覆盖。 */
    uint8_t motor_count;
    uint8_t queue_count;
    uint8_t response[8];
    uint8_t response_count;
    bool initialized;
    bool active;
    bool response_received;
    bool response_failed;
    bool motion_allowed; /* 软件运动许可，不能代替驱动器真实使能反馈。 */
    uint8_t armed_motor_mask; /* 按注册顺序记录许可轮，不是按地址移位。 */
    bool stop_pending; /* 停止请求优先于普通队列，不会因队列满丢失。 */
    bool disable_pending; /* 停止之后还需执行广播失能。 */
    bool quarantine_active;
    uint32_t started_ms;
    uint32_t last_rx_ms;
    uint32_t guard_started_ms;
    uint32_t guard_duration_ms;
    uint32_t quarantine_started_ms;
    uint32_t last_command_ms; /* 最近成功入队速度的时间，单位 ms。 */
    uint32_t stop_repeat_ms;
    uint32_t timeout_count;
    uint32_t protocol_error_count;
    uint32_t rejected_command_count;
    uint8_t last_rejected_address;
    uint8_t last_rejected_function;
    uint8_t last_rejected_status;
    uint32_t transport_error_count;
    uint32_t queue_full_count;
} EmmBusInstance;

/** @brief 校验回调和时间参数，建立静态总线并请求上电停止失能。 */
bool EmmBusInit(EmmBusInstance *bus, const EmmBus_Init_Config_s *config, uint32_t now_ms);
/* 显式开放全部已注册电机的运动许可：每轮位置/使能/堵转反馈必须新鲜，
 * 实际已使能且无堵转和保护；故障后不自动重新授权。此接口本身不发送使能。
 */
bool EmmBusArm(EmmBusInstance *bus, uint32_t now_ms);
/* 底层单机许可能力：目标轮健康且已使能，其他轮须有新鲜失能反馈。
 * 仍监测所有电机，停止/故障影响整条总线；正式底盘使用 EmmBusArm 全轮许可，
 * 本接口不构成独立运动流程或调试启动入口。
 */
bool EmmBusArmMotor(EmmBusInstance *bus, EmmMotorInstance *motor, uint32_t now_ms);
/** @brief 检查指定实例是否在当前运动许可掩码内。 */
bool EmmBusMotorIsArmed(const EmmBusInstance *bus, const EmmMotorInstance *motor);
/** @brief 撤销许可并清空普通队列，安排广播停止及可选失能。 */
void EmmBusStop(EmmBusInstance *bus, bool disable, uint32_t now_ms);
/** @brief 记录传输故障、作废全部反馈并进入停止和回复隔离。 */
void EmmBusTransportFault(EmmBusInstance *bus, uint32_t now_ms);
/** @brief 周期推进事务与安全仲裁，非阻塞等待 DMA 和回复。 */
void EmmBusUpdate(EmmBusInstance *bus, uint32_t now_ms);
/* 任务提交任意分段字节流，模块不保存输入指针；data 至少有 length 字节。
 * now_ms 是任务处理时间而非硬件采样时刻，与 Update/电机 API 同毫秒域。
 */
void EmmBusReceive(EmmBusInstance *bus, const uint8_t *data, size_t length, uint32_t now_ms);

/* 模块内部注册/入队入口，同样仅允许拥有者任务使用。 */
/** @brief 登记电机，重复实例、重复地址或容量超限均拒绝。 */
bool EmmBusRegister(EmmBusInstance *bus, EmmMotorInstance *motor);
/** @brief 提交普通事务；同轮同类覆盖，队列满返回 false，速度队列满还触发停止。 */
bool EmmBusSubmit(EmmBusInstance *bus, const EmmTransaction *transaction);

#endif
