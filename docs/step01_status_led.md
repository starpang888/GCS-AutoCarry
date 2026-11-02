# 第一步：PB2 状态灯

## 硬件与分层

- 使用已有的 `STATUS_LED`，GPIOB Pin 2，低电平点亮。
- `Core/Src/gpio.c` 继续由 CubeMX 初始化引脚。
- `bsp/led/bsp_led.c` 负责有效电平和 GPIO 写入。
- `modules/status_led/status_led.c` 负责非阻塞灯效，不调用 RTOS 延时。
- `application/robot_task.c` 创建状态任务，接收状态并每 20 ms 更新灯效。
- 新文件通过根目录 CMakeLists.txt 加入构建。freertos.c 的接入点都在 USER CODE 区。

## 灯效

| 状态 | 一轮灯效 | 含义 |
| --- | --- | --- |
| STARTUP | 亮 100 ms、灭 100 ms，重复 | 软件正在启动 |
| READY | 亮 100 ms、灭 900 ms | 本地初始化完成 |
| MOVING | 亮 100、灭 100、亮 100、灭 700 ms | 应用报告正在运动 |
| FAULT | 三次亮 100 ms，中间灭 100 ms，最后灭 500 ms | 应用报告故障 |

启动灯效从软件服务初始化开始，默认至少展示到状态任务运行满 1 秒。
默认任务完成 USB_DEVICE 初始化后请求 READY，然后删除自身。
READY 不表示电脑已连接、USB 已枚举或电机在线。MOVING/FAULT 当前仅提供接口，
还没有电机、USB 断线或任务超时检测。运动和故障请求会立即打断启动展示。

状态任务优先级 BelowNormal，任务栈 256 个 32 位单元（1024 字节）。
显示每 20 ms 更新，时序受调度影响。这是状态指示，不是精密定时输出。

## 状态接口

在普通任务上下文中使用，初始化成功后返回 1：

```c
#include "robot_task.h"

RobotSetStatus(ROBOT_STATUS_READY);
RobotSetStatus(ROBOT_STATUS_MOVING);
RobotSetStatus(ROBOT_STATUS_FAULT);
```

这里只存最新状态，使用长度 1 的 FreeRTOS 队列保证跨任务传递；不是动作事件队列。
不要在中断中调用。当前遵循最后一次请求，后续由统一状态仲裁处理故障优先级，
不要让多个模块各自循环写 READY 覆盖故障。

LED 闪烁只说明状态任务得到了调度，不代表其他任务都存活。
中断关闭、调度器停转或 Error_Handler 死循环时，不保证继续显示故障灯效。

## 编译与上板观察

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

1. 烧录 `build/Debug/GongChuangsai.elf`，复位后观察快闪约 1 秒，然后每秒短闪一次。
2. 如需分别检查双闪、三连闪，可临时把 freertos.c 中的 READY 请求改为 MOVING 或 FAULT，
   重新编译烧录；验证完恢复 READY。这些修改仅改变指示灯，没有电机控制功能。
3. 若灯始终不闪，先核对固件是否运行、PB2 接线和 LED 极性，再检查任务创建和系统时钟。

下一步是单电机串口通信，需要先确认驱动器型号/固件协议、通信电平、地址和波特率。
