#include "robot_task.h"
#include "board.h"
#include "robot.h"
#include "cmsis_os.h"
#include "task.h"
#include "host_service.h"
#include "chassis_odometry.h"

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
static RobotInstance robot;
static OmniKinematicsInstance chassis_kinematics;
static ChassisControlInstance chassis_control;
static ChassisControlStatus chassis_control_status;
static HostServiceInstance host_service;
static HostServiceStatus host_status;

/* 里程计实例：只由motorTask更新。 */
static ChassisOdometryInstance chassis_odometry;

/* 位姿观察快照，可在调试器中展开查看。 */
static ChassisPose chassis_pose;
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
    for (;;)
    {
        uint32_t now_ms = (uint32_t)xTaskGetTickCount();
        BoardMotorUpdate(now_ms);

        /* 先更新定位，再处理本周期USB目标，避免位置环使用上一周期位姿。 */
        BoardMotorStatus snapshot;
        BoardMotorGetStatus(&snapshot, now_ms);
        (void)ChassisOdometryUpdate(&chassis_odometry, snapshot.wheels, now_ms);

        HostServiceReceive(&host_service, now_ms);
        taskENTER_CRITICAL();
        uint32_t feedback_mode = host_feedback_mode;
        taskEXIT_CRITICAL();
        (void)HostServiceSelectFeedback(&host_service, feedback_mode);

        /* Robot负责选择速度或位置模式，底盘仍唯一拥有电机目标输出。 */
        RobotUpdate(&robot, now_ms);

        /* Robot可能更新了运动许可，再取快照用于状态上传。 */
        BoardMotorGetStatus(&snapshot, now_ms);
        HostServiceTransmit(&host_service, &snapshot, now_ms);
        taskENTER_CRITICAL();
        motor_status = snapshot;
        chassis_pose = chassis_odometry.pose;
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
        !RobotInit(&robot, &chassis_control) ||
        !BoardRobotPositionInit(&robot, &chassis_odometry, &chassis_position_control))
    {
        return false;
    }

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
