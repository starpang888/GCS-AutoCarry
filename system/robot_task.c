#include "robot_task.h"
#include "board.h"
#include "robot.h"
#include "cmsis_os.h"
#include "task.h"
#include "host_service.h"
#include "chassis_odometry.h"
#include "board_ops.h"
#include "robot_debug_position.h"
#include "board_servo.h"
#include "board_crane_action.h"

/* Keeps the timestamp API in milliseconds without truncating fractional ticks. */
_Static_assert(configTICK_RATE_HZ == 1000, "Task monitor requires a 1 ms RTOS tick");

static TaskMonitorInstance task_monitor;
static uint8_t default_task_id = TASK_MONITOR_INVALID_ID;
static uint8_t motor_task_id = TASK_MONITOR_INVALID_ID;
static StaticTask_t status_task_tcb;
static StackType_t status_task_stack[256];
static osThreadId status_task_handle;
static StaticTask_t motor_task_tcb;
static StackType_t motor_task_stack[512];
static osThreadId motor_task_handle;
static BoardMotorStatus motor_status;
static bool motor_status_ready;
/* axes[0]=yaw，axes[1]=z；所有收发及目标由motorTask拥有。 */
static BoardCraneStatus crane_status;
static bool crane_status_ready;
static CraneActionStatus crane_action_status; /* motorTask周期发布的一致观察快照。 */
static RobotInstance robot;

#if ROBOT_DEBUG_POSITION_ENABLED
/* 唯一调试邮箱，开机全0。先写x/y/yaw，最后写command=1；停止写command=2。
 * command非0时等待任务消费，不能边处理边改坐标。
 * 宏关闭或Release构建后，这个变量和调用一起消失。
 */
_Alignas(4) static volatile RobotDebugPositionMailbox debug_position;
#endif
static OmniKinematicsInstance chassis_kinematics;
static ChassisControlInstance chassis_control;
static ChassisControlStatus chassis_control_status;
static HostServiceInstance host_service;
static HostServiceStatus host_status;

/* 里程计实例：只由motorTask更新。 */
static ChassisOdometryInstance chassis_odometry;

/* 位姿观察快照，可在调试器中展开查看。 */
static ChassisPose chassis_pose; /* 当前控制位姿：改为OPS。 */
static ChassisPose wheel_pose;   /* 电机编码器里程计，只作独立对照。 */
static ChassisLocalizationFeedback chassis_localization; /* Robot与USB读同一份输入。 */

/* OPS原始观察快照。位姿同时经统一定位接口提供给Robot和USB反馈。
 * motorTask唯一更新，镜像在短临界区内发布；调试器可展开ops_status查看。
 */
static BoardOPSStatus ops_status;
/* 调试观察快照：state=0无目标、1接近、2稳定计时、3稳定到达。 */
static RobotArrivalStatus chassis_arrival_status;

/* 静态存储，初始化后由Robot持有指针，不能定义成初始化函数的局部实例。 */
static ChassisPositionControlInstance chassis_position_control;

/* Retained selector for readback: 0=binary, 1=VOFA FireWater. Inspect
 * host_status.feedback_mode for acknowledgment. Change only while stopped.
 */
_Alignas(4) static volatile uint32_t host_feedback_mode = BOARD_HOST_FEEDBACK_DEFAULT;
static void StartMotorTask(void const *argument)
{
    (void)argument;
    (void)BoardMotorStart((uint32_t)xTaskGetTickCount());
    /* DMA启动放在任务内，中断已开启；失败由周期服务自动恢复。 */
    (void)BoardOPSStart((uint32_t)xTaskGetTickCount());
    (void)BoardCraneStart((uint32_t)xTaskGetTickCount());
    for (;;)
    {
        uint32_t now_ms = (uint32_t)xTaskGetTickCount();
        BoardMotorUpdate(now_ms);
        BoardCraneUpdate(now_ms);

        /* 与电机总线共用任务调度，但UART4、UART5各有独立DMA及实例。
         * ISR只搬运字节，OPS ASCII协议在此解析，不在中断里做浮点换算。
         */
        BoardOPSUpdate(now_ms);
        BoardOPSStatus ops_snapshot;
        BoardOPSGetStatus(&ops_snapshot);
        /* 唯一定位输入为OPS；没有自动切回轮式里程计，避免原点和朝向突变。
         * 200ms与板级OPS新鲜度一致，不是主机命令有效期。
         */
        ChassisLocalizationFromOPS(&chassis_localization, &ops_snapshot.feedback,
                                  BOARD_OPS_FEEDBACK_TIMEOUT_MS);

        /* 轮式里程计继续更新供对照；控制位置与USB POSE已由上面的OPS更新。 */
        BoardMotorStatus snapshot;
        BoardMotorGetStatus(&snapshot, now_ms);
        (void)ChassisOdometryUpdate(&chassis_odometry, snapshot.wheels, now_ms);

#if ROBOT_DEBUG_POSITION_ENABLED
        /* 先处理调试请求，再处理USB；同周期USB STOP/异常会覆盖调试启动。
         * 共享一个Robot和一条motorTask输出路径，不创建第二条总线所有者。
         */
        RobotDebugPositionUpdate(&debug_position, &robot, now_ms);
#endif
        HostServiceReceive(&host_service, now_ms);
        taskENTER_CRITICAL();
        uint32_t feedback_mode = host_feedback_mode;
        taskEXIT_CRITICAL();
        (void)HostServiceSelectFeedback(&host_service, feedback_mode);

        /* Robot负责选择速度或位置模式，底盘仍唯一拥有电机目标输出。 */
        RobotUpdate(&robot, now_ms);
        BoardCraneStatus crane_snapshot;
        BoardCraneGetStatus(&crane_snapshot, now_ms);

        /* Robot可能更新了运动许可，再取快照用于状态上传。 */
        BoardMotorGetStatus(&snapshot, now_ms);
        HostServiceTransmit(&host_service, &snapshot, now_ms);
        taskENTER_CRITICAL();
        motor_status = snapshot;
        crane_status = crane_snapshot;
        crane_action_status = robot.crane_action.status;
        crane_status_ready = true;
        chassis_pose = chassis_localization.pose;
        wheel_pose = chassis_odometry.pose;
        ops_status = ops_snapshot;
        chassis_arrival_status = robot.arrival;
        motor_status_ready = true;
        chassis_control_status = chassis_control.status;
        host_status = host_service.status;
        (void)TaskMonitorFeed(&task_monitor, motor_task_id, (uint32_t)xTaskGetTickCount());
        taskEXIT_CRITICAL();
        osDelay(1U);
    }
}

static void StartStatusTask(void const *argument)
{
    (void)argument;
    for (;;)
    {
        /* Feed and state/LED updates are serialized in a short critical section.
         * The LED callback only writes GPIO BSRR; it never blocks or uses RTOS APIs.
         */
        taskENTER_CRITICAL();
        TaskMonitorUpdate(&task_monitor, (uint32_t)xTaskGetTickCount());
        taskEXIT_CRITICAL();
        osDelay(10U);
    }
}

bool RobotTaskInit(void)
{
    if (status_task_handle != NULL)
    {
        return false;
    }
    if (!BoardTaskMonitorInit(&task_monitor, (uint32_t)xTaskGetTickCount()))
    {
        return false;
    }

    default_task_id = TaskMonitorRegister(&task_monitor, "defaultTask", 200U);
    if (default_task_id == TASK_MONITOR_INVALID_ID)
    {
        return false;
    }

    motor_task_id = TaskMonitorRegister(&task_monitor, "motorTask", 200U);
    /*
     * 初始化顺序：
     *
     * 1. 电机板级资源：
     *    创建UART、EMM总线和四个电机实例。
     *
     * 2. 底盘运动学：
     *    保存轮径、轴距、轮距等几何参数。
     *
     * 3. 里程计：
     *    使用运动学模型和已初始化电机的方向、减速比。
     *
     * 4. 底盘控制器：
     *    绑定电机总线、四个电机及运动学模型。
     *
     * 5. Robot应用：
     *    绑定底盘控制器。
     *
     * || 表达式从左向右执行，任一初始化失败，
     * 就立即返回false，不继续创建后面的任务。
     */
    if (motor_task_id == TASK_MONITOR_INVALID_ID ||
        !BoardMotorInit((uint32_t)xTaskGetTickCount()) ||
        !BoardChassisKinematicsInit(&chassis_kinematics) ||
        !BoardChassisOdometryInit(&chassis_odometry, &chassis_kinematics) ||
        !BoardChassisControlInit(&chassis_control, &chassis_kinematics) ||
        !BoardChassisPositionControlInit(&chassis_position_control) ||
        !BoardRackServoInit() ||
        !BoardGripperServoInit() ||
        !RobotInit(&robot, &chassis_control) ||
        !BoardRackServoRobotInit(&robot) ||
        !BoardGripperServoRobotInit(&robot) ||
        !BoardRobotPositionInit(&robot, &chassis_localization, &chassis_position_control))
    {
        return false;
    }

    /* OPS配置失败体现在ops_status.initialized；位置命令将因定位无效被拒绝。
     * 此处放在BoardMotorInit之后，随后注册两条塔吊串口，共四个BSP实例。
     */
    (void)BoardOPSInit();
    /* 单轴缺失不阻断底盘及另一轴；错误可在crane_status内分别观察。 */
    (void)BoardCraneInit((uint32_t)xTaskGetTickCount());
    (void)BoardCraneRobotInit(&robot);
    (void)BoardCraneActionRobotInit(&robot); /* 坐标未配置仅禁用联动，不影响单轴。 */

    HostServiceInit(&host_service, &robot);
    (void)HostServiceSelectFeedback(&host_service, BOARD_HOST_FEEDBACK_DEFAULT);

    osThreadStaticDef(motorTask, StartMotorTask, osPriorityNormal, 0,
                      512, motor_task_stack, &motor_task_tcb);
    motor_task_handle = osThreadCreate(osThread(motorTask), NULL);
    if (motor_task_handle == NULL)
    {
        return false;
    }

    /* This CMSIS v1 adapter passes stack depth as 32-bit words, not bytes.
     * Higher priority lets this task detect a non-yielding normal-priority worker.
     */
    osThreadStaticDef(statusTask, StartStatusTask, osPriorityAboveNormal, 0,
                      256, status_task_stack, &status_task_tcb);
    status_task_handle = osThreadCreate(osThread(statusTask), NULL);
    return status_task_handle != NULL;
}

void RobotTaskHeartbeat(void)
{
    taskENTER_CRITICAL();
    (void)TaskMonitorFeed(&task_monitor, default_task_id, (uint32_t)xTaskGetTickCount());
    taskEXIT_CRITICAL();
}

bool RobotMotorGetStatus(BoardMotorStatus *status)
{
    if (status == NULL)
    {
        return false;
    }
    taskENTER_CRITICAL();
    bool ready = motor_status_ready;
    if (ready)
    {
        *status = motor_status;
    }
    taskEXIT_CRITICAL();
    if (ready)
    {
        uint32_t now_ms = (uint32_t)xTaskGetTickCount();
        for (uint8_t i = 0; i < BOARD_WHEEL_COUNT; ++i)
        {
            EmmMotorFeedback *feedback = &status->wheels[i];
            feedback->position_valid &= (uint32_t)(now_ms - feedback->position_ms) < BOARD_MOTOR_FEEDBACK_TIMEOUT_MS;
            feedback->speed_valid &= feedback->position_valid;
            feedback->enabled_valid &= (uint32_t)(now_ms - feedback->enabled_ms) < BOARD_MOTOR_FEEDBACK_TIMEOUT_MS;
            feedback->stall_valid &= (uint32_t)(now_ms - feedback->stall_ms) < BOARD_MOTOR_FEEDBACK_TIMEOUT_MS;
        }
        if ((uint32_t)(now_ms - status->timestamp_ms) >= BOARD_MOTOR_FEEDBACK_TIMEOUT_MS)
        {
            status->motion_allowed = false;
        }
    }
    return ready;
}

bool RobotCraneGetStatus(BoardCraneStatus *status)
{
    if (status == NULL) return false;
    taskENTER_CRITICAL();
    bool ready = crane_status_ready;
    if (ready) *status = crane_status;
    taskEXIT_CRITICAL();
    if (ready)
    {
        uint32_t now_ms = (uint32_t)xTaskGetTickCount();
        for (size_t i = 0U; i < BOARD_CRANE_AXIS_COUNT; ++i)
        {
            XMotorFeedback *feedback = &status->axes[i].feedback;
            feedback->position_valid &= (uint32_t)(now_ms - feedback->position_ms) < BOARD_CRANE_FEEDBACK_TIMEOUT_MS;
            feedback->status_valid &= (uint32_t)(now_ms - feedback->status_ms) < BOARD_CRANE_FEEDBACK_TIMEOUT_MS;
            if ((!feedback->position_valid || !feedback->status_valid) &&
                status->axes[i].motion.state == X_MOTION_REACHED)
                status->axes[i].motion.state = X_MOTION_MOVING;
        }
    }
    return ready;
}
