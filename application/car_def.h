#ifndef CAR_DEF_H
#define CAR_DEF_H

#include <stdint.h>

/* 底盘调试开关：设为 1 可通过调试器切换动作，设为 0 则移除调试接口。 */
#define CAR_CHASSIS_DEBUG_ENABLE 0U

/* 电机数量。 */
#define CAR_MOTOR_COUNT 4U

/* 电机数组下标顺序：左前、右前、右后、左后。 */
typedef enum
{
    CAR_MOTOR_LEFT_FRONT = 0,
    CAR_MOTOR_RIGHT_FRONT,
    CAR_MOTOR_RIGHT_REAR,
    CAR_MOTOR_LEFT_REAR
} CarMotorIndex;

/* 以下尺寸请按底盘实测值填写，单位为 mm。
 * 轮子半径是轮胎外径的一半。
 * 轴距是前后两排轮轴中心之间的距离。
 * 轮距是左右两侧轮轴中心之间的距离。
 */
#define CAR_WHEEL_RADIUS_MM  37.5f  /* 示例值，替换成实测轮子半径。 */
#define CAR_WHEEL_BASE_MM    240U /* 示例值，替换成实测轴距。 */
#define CAR_TRACK_WIDTH_MM   240U /* 示例值，替换成实测轮距。 */

/* 电机轴转速与车轮转速之比。
 * 电机直接连接车轮时为 1.0f；
 * 若电机转 19 圈、车轮转 1 圈，则填写 19.0f。
 */
#define CAR_MOTOR_GEAR_RATIO 1.0f

/* 电机速度指令使用的加速度参数，先沿用目前已验证的值。 */
#define CAR_MOTOR_ACCELERATION 10U

/* 四个电机的驱动器地址。 */
#define CAR_MOTOR_ADDRESS_LEFT_FRONT  0x01U
#define CAR_MOTOR_ADDRESS_RIGHT_FRONT 0x02U
#define CAR_MOTOR_ADDRESS_RIGHT_REAR  0x03U
#define CAR_MOTOR_ADDRESS_LEFT_REAR   0x04U

/* 电机协议方向相对于运动学方向的修正值。
 * 1 表示不反向，-1 表示反向；后续根据实际转向校准。
 */
#define CAR_MOTOR_DIRECTION_LEFT_FRONT   1
#define CAR_MOTOR_DIRECTION_RIGHT_FRONT  -1
#define CAR_MOTOR_DIRECTION_RIGHT_REAR   -1
#define CAR_MOTOR_DIRECTION_LEFT_REAR    1

#endif
