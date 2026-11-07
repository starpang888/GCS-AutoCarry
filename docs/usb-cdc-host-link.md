# USB CDC 下位机通信（协议 v1）

> VOFA 适配更新：下位机交付版当前默认上传**二进制反馈（模式0）**；模式1保留 **FireWater 文本反馈**，设置和通道表见 [VOFA 反馈](vofa-feedback.md)。本文下述 STATUS 帧、上位机二进制对接要求 `host_feedback_mode=0`（二进制），静止切换后用 `host_status.feedback_mode` 确认。FireWater 不是本二进制协议的解码插件，两种反馈不混发。

本工程交付STM32固件和USB CDC v1二进制接口，ROS 2节点和串口/桥接实现由上位机负责人负责。本协议用于正常连续控制，无需短时运动测试程序。职责与交接步骤见 [下位机交接说明](lower-controller-handoff.md)。

2026-11-01 控制语义已按用户要求简化：收到 VELOCITY 自动使能，保持最新目标，不要求 ARM、固定发帧频率或递增序号。旧版250 ms主机期限不再适用。

2026-11-07 已接入绝对位置目标POSITION（0x04）和位姿/稳定到达反馈POSE（0x82）。v1公共帧和原STATUS载荷布局不变；二进制模式每25ms交替发送STATUS/POSE，各名义20Hz。完整位置契约、64字节位姿布局及编码示例见[位置反馈扩展](position-feedback.md)。用户已通过USB位置命令验证单轴移动和原地旋转，并确认本步数据显示正常；ROS 2双方实机联调尚未完成。

## CubeMX 配置核对

| 项目 | 当前配置 |
| --- | --- |
| MCU | STM32F407VET6 |
| USB | USB_OTG_FS / Device Only / CDC |
| D− / D+ | PA11 / PA12 |
| 晶振和 PLL | HSE 8 MHz，M=4，N=168，P=2，Q=7 |
| 系统 / USB 时钟 | 168 MHz / 48 MHz |
| VBUS sensing | 关闭；通过USB挂起、反初始化或重新配置事件停止，无主机命令期限兜底 |
| IRQ | OTG_FS 优先级 5；USB 回调不调用 RTOS API |
| 初始化 | `defaultTask` 调用 `MX_USB_DEVICE_Init()`，注册 CDC 类和接口并启动 USB |

以上配置与当前生成代码一致，本轮不修改 `.ioc` 或外设初始化区域，无需重新生成 CubeMX。USB 用户回调的接入保留在 `USER CODE` 区。连接 **MCU 原生 USB FS 插座**，ST-Link 的 USB/虚拟串口不是这条链路。需要能传数据的 USB 线。

CDC 串口工具可选择 115200；这里传输的是 USB 数据，CDC 的 line-coding 波特率不会改变实际 USB 速率，也不会改变电机 UART5 的 115200 配置。普通文本串口助手会显示二进制数据，正式对接需由上位机按本协议解析。

## 代码与执行上下文

| 文件 | 职责 |
| --- | --- |
| `bsp/usb/bsp_usb.c/.h` | USB ISR 接收缓存、连接代次、静态发送缓冲所有权 |
| `modules/host_link/host_protocol.c/.h` | 帧编码、CRC、流式分帧；同步回调通知命令拥有者 |
| `application/chassis/chassis_control.c/.h` | 速度自动使能、最新目标保持、设备故障与停止确认 |
| `application/robot.c` | 编排USB命令和连续底盘更新，统一停止路径 |
| `system/host_service.c/.h` | 接收时间转换、协议接入、反馈打包 |
| `system/robot_task.c` | 唯一 motorTask 调用更新入口、发布调试状态快照 |

USB 接收中断只复制字节和 HAL 接收时间，再重启 OUT 接收。512 字节静态环形缓存满时丢弃当前包并清空旧数据、递增连接代次和溢出计数；不部分接收或覆盖。任务每次最多处理一环缓存；连接代次变化会停止当前主机控制并丢弃本轮输入。已完整接收的命令不因任务处理延迟达到100 ms而丢弃或停机；接收时间仅供解析字节间隔和诊断使用。

TX 使用独立静态存储，直到 IN 完成才可复用；挂起恢复不能提前释放发送缓冲。类重新初始化后重新允许发送。二进制反馈每25ms尝试一帧，STATUS/POSE各名义20Hz；成功提交后轮换类型，忙时保留待发送类型，下次编码最新快照，不排队重发旧状态。`tx_skipped` 在未连接或电脑不读取时增加是可能的。

## 帧格式

所有多字节数均为小端整数，显式按字节编码，不直接发送 C 结构体。

| 字节偏移 | 字段 | 长度 |
| --- | --- | --- |
| 0 | 帧头 `A5 5A` | 2 |
| 2 | 版本 `01` | 1 |
| 3 | 类型 | 1 |
| 4 | payload 长度 `uint16` | 2 |
| 6 | 序号 `uint32` | 4 |
| 10 | payload | N |
| 10+N | CRC16 小端 | 2 |

CRC 为 CRC-16/CCITT-FALSE，poly=0x1021，init=0xFFFF，不反射、无末尾异或；覆盖帧头到 payload 最后一个字节，标准向量 `123456789 → 0x29B1`。输入最大 24 字节，输出帧最大容量 240 字节。支持拆包/粘包，输入字节间隔 ≥100 ms 丢弃半帧，错 CRC/长度/版本后扫描下一帧头。

| 类型 | 含义 | payload |
| --- | --- | --- |
| `01` | VELOCITY | 三个 `int32`：vx mm/s，vy mm/s，wz mrad/s，共 12 字节 |
| `02` | ARM | 0 字节，可选零速预使能；运行或已有启动请求时忽略 |
| `03` | STOP | 0 字节，无条件停止并失能 |
| `04` | POSITION | 三个 `int32`：绝对X mm、Y mm、yaw mrad，共12字节 |
| `81` | STATUS | 196 字节，名义20 Hz；总帧长208字节 |
| `82` | POSE | 64字节，名义20 Hz；总帧长76字节，见[位置反馈扩展](position-feedback.md) |

示例 ARM(seq=1)：`A5 5A 01 02 00 00 01 00 00 00 4B 4C`。

示例速度(seq=0xFFFFFFFF，vx=−50，vy=25，wz=−300)：
`A5 5A 01 01 0C 00 FF FF FF FF CE FF FF FF 19 00 00 00 D4 FE FF FF 47 91`。

## 控制契约与恢复

上电不运动；USB枚举和状态反馈不会使能。收到一条有效 VELOCITY 即保存目标，待总线和四轮反馈就绪后自动使能，不需要先发ARM。

1. VELOCITY 三轴值使用整数 mm/s、mrad/s，转换为应用层 m/s、rad/s；`x`向前、`y`向左、正`wz`俯视逆时针。
2. 使能过程中允许直接发送非零目标，新速度覆盖旧目标。四轮均取得本次使能之后的新状态才输出；实际使能操作超过500 ms仍未确认则停止。
3. 速度目标保持到新的VELOCITY、POSITION或STOP（故障/USB异常也取消），不设置250 ms主机期限，不要求20 Hz续发。sequence仅用于回传，可重复、倒序或固定为0，按接收顺序覆盖。
4. 取消每轴±0.1 m/s和±0.5 rad/s越界停机规则及20 RPM轮速上限。载荷可编码完整int32范围；底盘直接解算四轮速度，不做比例缩放。驱动协议仍只支持原生±5000字段单位（名义1 RPM/单位），超出时进入MOTOR_ERROR并停止，不截断或回绕。
5. 零速度使四轮零速保持，仍使能；STOP清零目标并请求停止失能。可选ARM用于零速预使能，运行中或已有启动请求时重复ARM忽略。

STOP所在接收批次的后续输入丢弃。停止确认中重复STOP不延长期限；后续独立批次的新速度可以保存，待停止完成后再自动使能。1秒内未取得四轮新鲜失能反馈则STOP_TIMEOUT，并再请求停止失能。当前失能事务失败后持续重试仍有已知缺口，失能须以四轮新鲜反馈确认，不能仅看STOP已发送。

偶发EMM查询超时保留尚新鲜的历史反馈并继续轮询；单纯协议错误计数增加不再触发应用停机。实际控制写失败、驱动明确拒绝、500 ms反馈失效、失能、堵转及UART传输故障仍会停止。底层每轮200 ms输出刷新检查保留，motorTask每50 ms自动刷新已保存目标，它不是ROS/USB主机命令期限。

USB挂起、反初始化、重新配置及接收溢出会清除旧目标；恢复后需要新的VELOCITY或满足定位条件的POSITION，旧目标不会自动恢复。首次配置变化所在接收批次会丢弃，上位机应先读取STATUS再发首条速度。首条速度若早于电机反馈到达，会保留到就绪再启动，此时state可暂为DISABLED（或原FAULT），开始实际使能后变为ENABLING；STOP或USB异常会取消该请求。

主机停止发帧、ROS节点退出但USB仍保持配置时，下位机会继续保持最后速度。ROS侧需要停车时发送零速度或STOP；物理拔线的停止依赖USB挂起/断开相关事件被识别，现已没有命令静默超时作为兜底。

### 位置目标

POSITION自动申请使能，X/Y/yaw为相对启动原点的绝对目标，应用层使用m、rad，线协议使用mm、mrad。位置环每50ms计算一次速度；容差内输出零速并保留使能，稳定到达后仍保持位置。相同坐标重复发帧不增加行程、不重置到达计时。

POSITION需要定位有效且底盘不在STOPPING，否则仅增加拒绝计数；VELOCITY（包括零速）取消位置模式，ARM在位置模式中忽略。STOP/USB异常/设备故障/定位失效取消位置目标，恢复后需新命令。位置控制参数、接受条件、目标关联和到达语义见[位置反馈扩展](position-feedback.md)。

### 方向和反馈单位

板级四轮安装方向都为 −1，同时用于 SI 目标与测量反馈。原 `position_raw` 保留驱动器原始累计位置，不反转；`wheel_speed_rad_s` 为经过方向换算的轮轴实测速度。当前按 65536 位置计数/圈差分，速度命令按名义 1 RPM/字段单位转换。此前实测字段 20 约 19.08 RPM，未据此拟合补偿。几何尺寸仍暂用 240×240 mm，轮径 75 mm；未完成精确尺寸和完整圈数校准，不能把当前速度/里程视为高精度标定结果。

四轮通过 UART5 顺序发送速度帧，尚未实现 EMM 驱动器的同步触发指令；这里的一路四地址控制不等于四台驱动器同时启动。

## STATUS payload

下表偏移均相对 payload；有符号 milli 单位中的 `INT32_MIN` 表示无可用测量。命令和实测独立传输。无通用 ACK 帧，通过状态、最后接受的序号和拒绝计数判断执行结果。

| 偏移 | 类型 | 内容 |
| --- | --- | --- |
| 0 | u32 | MCU RTOS uptime_ms |
| 4…7 | 4×u8 | state、fault、connected、motion_allowed |
| 8 | u32 | last_command_sequence（最后接受的 ARM / VELOCITY / POSITION标记；允许重复或任意顺序） |
| 12 | u32 | control_rejected |
| 16…36 | 6×u32 | received_frames、crc_errors、format_errors、rx_overflows、tx_skipped、link_epoch |
| 40 | 3×i32 | requested：vx mm/s、vy mm/s、wz mrad/s |
| 52 | 3×i32 | applied：与requested相同的计算目标，未包含 RPM 整数取整误差；不代表已执行或实测速度 |
| 64 | 3×i32 | measured：实测底盘速度，同单位 |
| 76 | u32 | measured_valid |
| 80…96 | 5×u32 | 电机 bus timeouts、protocol_errors、rejected、transport_errors、rx_errors |
| 100 | 4×24 字节 | 依地址 1、2、3、4 的反馈块，见下表 |

| 每轮块偏移 | 类型 | 内容 |
| --- | --- | --- |
| 0 | i64 | position_raw，驱动器累计位置 |
| 8 | i32 | 实测轮速，mrad/s；无效为 INT32_MIN |
| 12 | u32 | position_ms |
| 16 | u32 | flags：bit0位置有效，1速度有效，2使能反馈有效，3已使能，4堵转反馈有效，5堵转，6堵转保护 |
| 20 | u32 | enabled_ms |

state：0 DISABLED，1 ENABLING，2 ACTIVE，3 STOPPING，4 FAULT。

fault：0 OK，1 COMMAND_TIMEOUT（保留，不再因主机静默产生），2 LINK_LOST，3 MOTOR_ERROR，4 ENABLE_TIMEOUT，5 STOP_TIMEOUT，6 INVALID_COMMAND（内部运动学输入异常），7 NOT_READY（位置模式定位不可用）。部分拒绝只增加计数而不改变 fault。

`measured_valid` 需要四轮位置和速度都有效、位置反馈年龄 <200 ms、四轮时间差 ≤80 ms，再做正解算。无效时速度字段为 INT32_MIN，不能用目标速度填补。反馈非硬件同步采样。MCU 时间未与 ROS 时钟同步，ROS 节点应记录接收时间并显式处理时间域。PB2 只表示任务心跳，不表示 USB 或驱动器一定正常。

## 对接与状态检查

固件默认模式0（二进制），不自动使能。上位机负责人依据 [交接说明](lower-controller-handoff.md) 实现收发、使能和停止确认。VOFA仅用于模式1的观测，正式模式不混入文本。

下位机调试器可查看 `host_status`、`chassis_control_status` 和 `motor_status`。`host_status.connected`只表示CDC配置/恢复状态，不证明ROS节点在线。旧手动运动模块及选择器已删除，停止使用USB STOP命令。

## 软件验证

```powershell
cmake --preset Debug
cmake --build --preset Debug
python tests/run_motor_tests.py
python -m unittest discover -s tests -p test_host_link.py -v
```

ARM 模拟器执行实际 C 代码，验证单条 VELOCITY 无 ARM 自动启动、10秒无新主机命令仍持续运行、重复/倒序标记更新目标、使能期间非零目标、高于旧20 RPM上限的原速输出、超驱动协议范围拒绝、偶发查询超时不中断运行、STOP 优先、USB 重连/溢出、完整延迟命令、堵转和设备离线。Python 校验同一黄金帧、分包解析、int32载荷范围和反馈布局。HAL/CDC 使用桩，不验证真实USB中断时序、电脑驱动或实物响应。

2026-11-07 位置/到达版的隔离完整Debug构建和ARM离线回归通过，验证目标覆盖、故障取消、连续低速到达、旧采样不计时、重复目标不饿死更新、毫秒回绕，以及STATUS/POSE长度、CRC和USB忙时公平轮换；源码接入及用户观察见[交接说明](lower-controller-handoff.md)。motorTask静态栈仍为512 words（2048 B），实际栈高水位需上板测量。
