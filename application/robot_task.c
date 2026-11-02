/* 应用调度层：管理状态请求，周期调用灯效模块，不直接操作 GPIO。 */
#include "robot_task.h"
#include "status_led.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "cmsis_os.h"
#include "zdt_motor.h" /* 张大头电机的位置读取接口。 */
#include "main.h"      /* 提供 HAL_GetTick()。 */
#include "chassis.h"  /* 底盘目标速度与运动学接口。 */
#include "car_def.h"  /* 整车尺寸、电机地址和电机配置。 */
#include "usb_protocol.h" /* USB 二进制命令接口。 */
#include "usbd_cdc_if.h" /* USB CDC 发送接口。 */

#define STATUS_TASK_PERIOD_MS 20U /* 状态灯刷新间隔，单位 ms。 */
#define STARTUP_DISPLAY_MS 1000U  /* 就绪前保留的启动灯效展示时间，单位 ms。 */

/* 长度为 1 的队列只保存最新状态，同时保证任务间传递的一致性。 */
static QueueHandle_t status_queue;

/* 将系统节拍换算为毫秒；当前 FreeRTOS 配置为每秒 1000 个节拍。 */
static uint32_t GetTimeMs(void)
{
    return (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/* 将业务状态映射为显示模式，让业务调用者不必依赖灯效实现。 */
static StatusLedMode GetLedMode(RobotStatus status)
{
    switch (status)
    {
    case ROBOT_STATUS_STARTUP:
        return STATUS_LED_STARTUP;
    case ROBOT_STATUS_READY:
        return STATUS_LED_READY;
    case ROBOT_STATUS_MOVING:
        return STATUS_LED_MOVING;
    case ROBOT_STATUS_FAULT:
    default:
        return STATUS_LED_FAULT;
    }
}

/* 独立低优先级任务：接收最新状态，计算灯效，再主动让出处理器。 */
static void StartStatusTask(void const *argument)
{
    RobotStatus requested_status = ROBOT_STATUS_STARTUP; /* 最近一次请求的状态。 */
    const uint32_t start_ms = GetTimeMs(); /* 从任务首次执行起计算展示时间。 */
    (void)argument; /* 当前任务不使用创建时传入的参数。 */

    for (;;)
    {
        const uint32_t now_ms = GetTimeMs();
        RobotStatus displayed_status;

        /* 非阻塞取出最新请求；队列为空时保留 requested_status 原值。 */
        (void)xQueueReceive(status_queue, &requested_status, 0U);
        displayed_status = requested_status;

        /* USB 初始化可能很快完成，因此在就绪前保留约 1 秒启动快闪。
         * 运动或故障请求不受这个展示时间限制，会在本轮直接显示。
         */
        if (requested_status == ROBOT_STATUS_READY &&
            (uint32_t)(now_ms - start_ms) < STARTUP_DISPLAY_MS)
        {
            displayed_status = ROBOT_STATUS_STARTUP;
        }

        StatusLedUpdate(GetLedMode(displayed_status), now_ms);
        /* 让出处理器，避免为了闪灯持续占用 CPU。 */
        osDelay(STATUS_TASK_PERIOD_MS);
    }
}

/* 四轮位置反馈。
 * 当前由通信任务写入；后续 USB 任务读取时再加入一致性快照接口。
 */
typedef struct
{
    uint32_t read_ok_count;   /* 成功查询次数。 */
    uint32_t read_fail_count; /* 失败查询次数。 */
    uint8_t position_negative; /* 位置符号：0 为正，1 为负。 */
    uint32_t position_raw;    /* 位置绝对值，65536 对应一圈。 */
    float position_deg;      /* 带符号累计角度，单位为度。 */
    uint32_t last_update_ms; /* 最近一次有效反馈的时间。 */
    uint8_t last_read_ok;     /* 最近一次查询是否成功。 */
} MotorFeedback;

/* 下标依次对应左前、右前、右后、左后。
 * volatile 用于调试观察，不代表结构体能被原子读写。
 */
volatile MotorFeedback motor_feedback[4];

/* 电机地址顺序与运动学输出数组一致。 */
static const uint8_t motor_addresses[CAR_MOTOR_COUNT] =
{
    CAR_MOTOR_ADDRESS_LEFT_FRONT,
    CAR_MOTOR_ADDRESS_RIGHT_FRONT,
    CAR_MOTOR_ADDRESS_RIGHT_REAR,
    CAR_MOTOR_ADDRESS_LEFT_REAR
};

/* 总使能请求：0 为四轮失能，1 为四轮使能。
 * 默认失能，上电后不会由本任务主动启动电机。
 */
volatile uint8_t motor_enable_request = 0U;

/* 四轮目标转速，单位为电机轴 RPM。
 * 下标依次为：左前、右前、右后、左后。
 * 正负表示驱动器协议方向，暂时不代表底盘前进方向。
 */
volatile int16_t motor_target_rpm[4] = {0, 0, 0, 0};

/* 最近一次控制指令的结果：1 应答成功，0 失败。 */
volatile uint8_t motor_command_ok[4];

/* 通信任务记录已成功设置的使能状态。
 * 初始值 2 表示尚未设置，确保启动时向每台发送一次失能指令。
 */
static uint8_t motor_enabled[4] = {2U, 2U, 2U, 2U};

/**
 * @brief 将 USB 底盘动作命令转换为底盘动作函数调用。
 * @param command 最近一次有效的 USB 动作命令。
 */
static void ApplyUsbChassisCommand(
    const UsbChassisCommand *command)
{
    float speed;

    /* USB 平移和旋转速度都使用千分之一单位。 */
    speed = (float)command->speed / 1000.0f;

    switch (command->action)
    {
    case USB_CHASSIS_STOP:
        Chassis.stop();
        break;

    case USB_CHASSIS_FORWARD:
        Chassis.forward(speed);
        break;

    case USB_CHASSIS_BACKWARD:
        Chassis.backward(speed);
        break;

    case USB_CHASSIS_LEFT:
        Chassis.left(speed);
        break;

    case USB_CHASSIS_RIGHT:
        Chassis.right(speed);
        break;

    case USB_CHASSIS_ROTATE_LEFT:
        Chassis.rotate_left(speed);
        break;

    case USB_CHASSIS_ROTATE_RIGHT:
        Chassis.rotate_right(speed);
        break;

    default:
        Chassis.stop();
        break;
    }
}

/**
 * @brief 下发指定电机的使能状态和目标转速。
 * @note 仅由 UART5 通信任务调用，与位置查询顺序执行。
 */
static void UpdateMotorControl(uint8_t index)
{
    uint8_t requested_enable = motor_enable_request ? 1U : 0U;

    /* 使能状态发生变化时才发送，避免重复下发相同设置。 */
    if (motor_enabled[index] != requested_enable)
    {
        motor_command_ok[index] = ZdtMotorEnable(
            motor_addresses[index],
            requested_enable);

        if (motor_command_ok[index] == 0U)
        {
            return;
        }

        motor_enabled[index] = requested_enable;
    }

    /* 使能后周期下发最新转速；目标为 0 时停止转动但保持使能。
     * 加速度参数先统一为 10，之后根据机械响应调整。
     */
    if (requested_enable != 0U)
    {
        motor_command_ok[index] = ZdtMotorSetSpeed(
            motor_addresses[index],
            motor_target_rpm[index],
            CAR_MOTOR_ACCELERATION);
    }
}



/* 四电机通信任务：统一执行控制下发和位置查询。 */
static void StartMotorTestTask(void const *argument)
{
    /* 保存当前底盘目标和运动学计算结果。 */
    ChassisTarget chassis_target;
    int16_t chassis_rpm[CAR_MOTOR_COUNT];

    /* 保存最近一次 USB 底盘动作命令。 */
    UsbChassisCommand usb_chassis_command;

    /* 保存最近一次 USB 电机使能命令。 */
    uint8_t usb_motor_enable;

    /* USB CDC 异步发送期间必须保持有效的帧缓存。 */
    static uint8_t encoder_frame[USB_ENCODER_FRAME_SIZE];
    static uint8_t encoder_negative[USB_ENCODER_MOTOR_COUNT];
    static uint32_t encoder_magnitude[USB_ENCODER_MOTOR_COUNT];
    static uint8_t encoder_sequence = 0U;

    ZdtPosition position = {0};
    (void)argument;

    for (;;)
    {
    #if CAR_CHASSIS_DEBUG_ENABLE
        /* 调试开关打开时，先执行调试器设置的动作。 */
        ChassisDebugUpdate();
    #endif

        /* 读取最近一次有效的 USB 底盘动作命令。 */
        if (UsbProtocolGetChassisCommand(&usb_chassis_command))
        {
            ApplyUsbChassisCommand(&usb_chassis_command);
        }

        /* 读取最近一次有效的 USB 电机使能命令。 */
        if (UsbProtocolGetMotorEnable(&usb_motor_enable))
        {
            motor_enable_request = usb_motor_enable;
        }

        /* 读取目标速度并计算四个电机的 RPM。 */
        ChassisGetTarget(&chassis_target);
        ChassisCalculateMotorRpm(&chassis_target, chassis_rpm);

        /* 把运动学结果交给现有电机控制流程。 */
        for (uint8_t i = 0U; i < CAR_MOTOR_COUNT; ++i)
        {
            motor_target_rpm[i] = chassis_rpm[i];
    }

    /* 后面保留原来的四电机控制和位置读取代码。 */

        for (uint8_t i = 0U; i < 4U; ++i)
        {
            /* 先更新这台电机的使能状态和转速。 */
            UpdateMotorControl(i);

            /* 再查询位置，沿用已经验证过的 DMA 收发接口。 */
            if (ZdtMotorReadPosition(motor_addresses[i], &position))
            {
                motor_feedback[i].position_negative = position.negative;
                motor_feedback[i].position_raw = position.magnitude;
                motor_feedback[i].position_deg = position.position_deg;
                motor_feedback[i].last_update_ms = HAL_GetTick();
                motor_feedback[i].read_ok_count++;
                motor_feedback[i].last_read_ok = 1U;
            }
            else
            {
                /* 保留上一次有效位置，同时记录本次失败。 */
                motor_feedback[i].read_fail_count++;
                motor_feedback[i].last_read_ok = 0U;
            }

            osDelay(5);
        }

        /* 按左前、右前、右后、左后的顺序复制最新有效位置。 */
        for (uint8_t i = 0U; i < USB_ENCODER_MOTOR_COUNT; ++i)
        {
            encoder_negative[i] = motor_feedback[i].position_negative;
            encoder_magnitude[i] = motor_feedback[i].position_raw;
        }

        /* 每轮发送一帧；CDC 忙时跳过本次，不阻塞电机通信。 */
        UsbProtocolBuildEncoderFrame(
            encoder_sequence++,
            encoder_negative,
            encoder_magnitude,
            encoder_frame);
        (void)CDC_Transmit_FS(encoder_frame, sizeof(encoder_frame));

        /* 当前先沿用原反馈采集节奏，后续按底盘响应要求调整。 */
        osDelay(100);
    }
}


uint8_t RobotTasksInit(void)
{
    RobotStatus initial_status = ROBOT_STATUS_STARTUP;
    osThreadId status_task;

    /* 初始化底盘目标，确保系统启动时处于停止状态。 */
    ChassisInit();

    /* 队列复制枚举值，不保存调用者局部变量的地址。 */
    status_queue = xQueueCreate(1U, sizeof(RobotStatus));
    if (status_queue == NULL)
    {
        return 0U;
    }

    /* 先准备初始状态和灯效，再创建可能访问这些资源的任务。 */
    (void)xQueueOverwrite(status_queue, &initial_status);
    StatusLedInit(GetTimeMs());

    /* 当前 Cortex-M4 移植中栈参数以 32 位单元计，256 个单元即 1024 字节。
     * 灯效不要求高实时性，使用低于普通业务任务的优先级。
     */
    osThreadDef(statusTask, StartStatusTask, osPriorityBelowNormal, 0, 256);
    status_task = osThreadCreate(osThread(statusTask), NULL);
    if (status_task == NULL)
    {
        /* 任务创建失败时释放已申请的队列，并标记服务不可用。 */
        vQueueDelete(status_queue);
        status_queue = NULL;
        return 0U;
    }

    /* 创建单电机测试任务，栈大小为 256 个 32 位单元。
    * 使用较低优先级，避免影响后续普通业务任务。
    */
    osThreadDef(motorTest, StartMotorTestTask,
                osPriorityBelowNormal, 0, 256);

    if (osThreadCreate(osThread(motorTest), NULL) == NULL)
    {
        return 0U;
    }

    return 1U;
}

uint8_t RobotSetStatus(RobotStatus status)
{
    /* 先检查服务和参数，避免向空队列或显示模块传递非法状态。 */
    if (status_queue == NULL || (unsigned int)status > ROBOT_STATUS_FAULT)
    {
        return 0U;
    }

    /* 覆盖待处理的旧状态，不等待队列腾出空间；返回值统一为 0 或 1。 */
    return xQueueOverwrite(status_queue, &status) == pdPASS;
}
