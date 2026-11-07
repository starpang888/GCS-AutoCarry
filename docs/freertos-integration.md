# FreeRTOS 与任务状态灯

## 当前配置

2026-10-28：用户已手动完成 CubeMX 配置及代码生成，已检查生成源码并构建通过。
早期候选配置已由实际 IOC 取代，不再使用。

| 配置 | 实际值 |
| --- | --- |
| MCU / 时钟 | STM32F407VET6；HSE 8 MHz，SYSCLK 168 MHz |
| RTOS | FreeRTOS V10.3.1 / CMSIS-RTOS v1，tick 1000 Hz |
| HAL 时基 | TIM6；SysTick、SVC、PendSV 由 FreeRTOS CM4F port 提供 |
| 内存管理 | Dynamic / Static，heap_4，FreeRTOS heap 16384 B |
| defaultTask | Normal；动态分配；512 个 32 位 word，即 2048 B |
| statusTask | AboveNormal；静态分配；256 个 word，即 1024 B；每轮 osDelay(10) |
| motorTask | Normal；静态分配；512 个 word，即 2048 B；每轮 osDelay(1)，管理 UART5 电机总线 |
| 状态灯 | PB2 / STATUS_LED；推挽输出、低速、无上下拉，初始低电平；高电平亮 |
| 周期接口 | 当前 INCLUDE_vTaskDelayUntil=0；任务采用已有 osDelay 接口，不宣称严格固定周期 |

当前生成配置未开启栈溢出和分配失败 hook；自有任务接入检查默认任务、电机任务及状态任务的创建结果。
`configENABLE_FPU=0` 未用于裁剪当前 GCC/ARM_CM4F port 的 VFP 上下文保存路径，
不能据此判断硬浮点任务上下文不受支持。

## 分层与启动链

- `bsp/led/bsp_led.c/.h`：显式 LED 实例、有效电平和 GPIO 输出；不重做 CubeMX 初始化。
- `modules/task_monitor/task_monitor.c/.h`：最多 8 个任务的心跳、超时与灯态；无 HAL/RTOS 依赖。
- `board/board.c/.h`：将 PB2、高电平有效及显示参数注入模块。
- `system/robot_task.c/.h`：创建独立监测 / 电机任务、注册默认和电机任务并串行化 Feed/Update。
- `Core/Src/freertos.c`：只在 USER CODE 区接入初始化和默认任务心跳。

启动顺序：

```text
main → HAL / 时钟 / MX_* 外设初始化
     → MX_FREERTOS_Init
         → 创建 defaultTask
         → RobotTaskInit：板级状态灯 / 电机实例、心跳注册、静态 motorTask / statusTask
     → osKernelStart
         → defaultTask：USB_DEVICE 初始化 → 每轮工作完成后心跳 → osDelay(1)
         → statusTask：检查所有已注册心跳并更新 LED → osDelay(10)
         → motorTask：UART5 收发与 EMM 总线更新 → 状态快照 / 心跳 → osDelay(1)
```

目前 defaultTask 和 motorTask 被注册监测；电机接入说明见 `docs/emm-motor-integration.md`，底盘控制已在motorTask内执行，无独立底盘任务；INS尚未接入。
以后加入新任务时，在调度器启动前注册唯一名称和超时，在任务实际完成工作后上报心跳；
跨任务的模块访问应沿用 system 层的短临界区。不要用另一个定时中断代替任务喂心跳。

## 灯态

| 条件 | PB2 表现 |
| --- | --- |
| 尚未收到全部初次心跳，且仍在 2 秒启动宽限期内 | 熄灭 |
| 全部已注册任务的心跳有效 | 亮 500 ms / 灭 500 ms，约 1 Hz 慢闪 |
| 任务未在启动宽限期内完成首次工作，或运行后 200 ms 未喂心跳 | 亮 100 ms / 灭 100 ms，约 5 Hz 快闪 |
| 心跳恢复 | 自动回到慢闪，重置闪烁相位 |
| 状态任务、调度器或整个内核停止执行 | 灯可能保持最后电平；不是硬件看门狗 |

200 ms 是 defaultTask 和 motorTask 的监测超时，检测延迟还包括监测扫描及调度延迟。
监测任务优先级高于默认任务，因此能发现默认任务不主动让出 CPU 的情况；
更高优先级任务、长中断或关闭中断仍可能阻止监测执行。
慢闪仅说明已注册任务持续上报进度，不证明所有外设或业务正确。

## 验证

- `cmake --preset Debug`、`cmake --build --preset Debug` 通过。
- LED 首版链接占用：FLASH 42268 B；主 RAM 28224 B，包含静态内核堆及链接保留区域，
  不等于运行时栈峰值或当前空闲堆。
- ELF 符号核对：SysTick/SVC/PendSV、TIM6 IRQ 和 HAL_InitTick 均有唯一已定义实现。
- 边界测试使用 ARM GCC 编译真实任务监测模块，再由 Unicorn 执行：

```powershell
python -m pip install --target build/test-deps unicorn==2.1.4
python tests/run_task_monitor_test.py
```

测试覆盖启动超时、多任务心跳、闪烁边界、失联与恢复、uint32 毫秒回绕、重复注册和容量。
该测试不模拟 RTOS 调度和 GPIO 电路；用户已自行烧录 LED 首版并确认正常闪烁。
新增 motorTask 后的固件仍待用户上板验证，不能沿用 LED 首版的实物验证结论。

## 重新生成注意

继续使用 CubeMX 的 Keep User Code 设置，保留 `freertos.c` 的 USER CODE 调用和
根 CMake 的自有源文件、包含路径。重新生成后再次构建，并核对任务栈、时基和中断归属。
