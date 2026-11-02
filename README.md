# GCS-AutoCarry

基于 STM32F407VET6 的四轮 X 型全向底盘控制固件。工程使用 STM32 HAL、FreeRTOS 和 CMake，支持 UART5 控制四台张大头 EMM_V5.0 闭环步进电机，并通过 USB CDC 与上位机交换控制命令和编码器位置数据。

## 已实现功能

- PB2 状态 LED 控制，低电平点亮
- UART5 TTL 通信，115200 baud，控制四台 EMM_V5.0 电机
- 四轮速度分配与 X 型全向轮底盘运动学
- USB CDC 二进制协议：接收底盘动作和电机使能命令
- USB CDC 周期发送四个电机的位置反馈

底盘动作包括前进、后退、左移、右移、左旋转、右旋转和停止。

## 硬件配置

- MCU：STM32F407VET6
- RTOS：FreeRTOS
- 电机驱动器：张大头 EMM_V5.0
- 电机通信：UART5 TTL，115200 baud
- 电机地址：
  - 左前：`0x01`
  - 右前：`0x02`
  - 右后：`0x03`
  - 左后：`0x04`
- 状态 LED：PB2，低电平点亮
- 上位机接口：USB CDC 虚拟串口

## 工程结构

- `Core/`：STM32 和 FreeRTOS 初始化代码
- `USB_DEVICE/`：USB CDC 设备接口
- `bsp/`：板级 LED、UART 服务
- `modules/`：状态灯、电机和 USB 协议模块
- `application/`：底盘运动学、整车配置和应用任务
- `docs/`：状态灯说明及 USB CDC 通信协议

## 编译

需要安装 STM32 Arm GNU Toolchain、CMake 3.22 或更高版本和 Ninja，并确保 `arm-none-eabi-gcc` 可从命令行调用。

```powershell
cmake --preset Debug
cmake --build --preset Debug