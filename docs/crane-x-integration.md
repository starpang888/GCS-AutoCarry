# 塔吊 yaw / z：X 固件通信接入

2026-11-10，通信阶段用户已烧录并确认yaw/z反馈正常。后续阶段已实现普通0xFD绝对梯形位置、自动使能、停止/故障仲裁和USB塔吊命令/反馈；编译与离线回归通过，本次运动固件尚未烧录，梯形位置运动与USB塔吊联调尚待实机验证。完整USB接口见[USB CDC协议](usb-cdc-host-link.md)。

## 用户确认的控制约定

- 塔吊两台为张大头 X42 V1.0、X 固件，与底盘四台 EMM 固件区分。
- yaw 为 USART6，PC6/TX、PC7/RX；z 为 USART3，PD8/TX、PD9/RX。各自独立串口，地址均为 1，工程配置为 115200、8N1。
- 用户每次上电前人工将机构放到实体挡块参考位置；驱动器已保存单圈零位并启用单圈就近上电回零。用户保证回零结束后发外部目标，软件不增加自动等待、回零、清零流程。
- 使用驱动器累计位置数值作为绝对目标，不做齿轮角度或丝杆距离换算；最大速度、加速加速度、减速加速度按轴独立配置。正常流程为一次发送目标、持续读取实际位置与到达状态，到达后保持。
- 扩展当前 USB CDC 二进制协议，单独分配塔吊命令/反馈类型。相同目标不追加位移，新目标覆盖旧目标。

## 本阶段运行路径

```text
RobotTaskInit
  BoardMotorInit → BoardOPSInit → BoardCraneInit → BoardCraneRobotInit
    UART BSP容量由2扩为4，分别绑定UART5、UART4、USART6、USART3
osKernelStart
  motorTask
    BoardCraneStart：启动两轴循环接收DMA
    BoardCraneUpdate：UART恢复 → 读取字节 → X协议解析 → 反馈/故障仲裁 → 推进独立查询或控制事务
    BoardCraneGetStatus → 短临界区发布crane_status
```

`modules/motor/x/x_motor.c/.h` 不包含 HAL，也不引用底盘模块；回调和 context 由 `board/board_crane.c/.h` 注入。现有 motorTask 是两条塔吊串口的唯一拥有者；ISR 继续由 UART BSP 统一搬运字节。未新增任务或 HAL 回调，未手改 CubeMX 生成区域。

每轴交替发送以下查询，相邻查询最短 25 ms，正常位置和状态各约 20 Hz：

| 请求 | 固定回复 | 处理 |
| --- | --- | --- |
| `01 36 6B` | `01 36 符号 幅值[4字节大端] 6B`，8字节 | 符号0正、1负；用int64_t完整保存±4294967295 |
| `01 3A 6B` | `01 3A flags 6B`，4字节 | 保存完整状态位及独立时间戳 |

X 固件 `position_units` 每单位为电机轴 0.1°，每圈 3600；例如 72000 是正向20圈，-180000是负向50圈。它是驱动器维护的累计坐标，不按单圈取模，也不是每圈65536的编码器原始计数。本阶段协议依据本机张大头 X42S 第二代用户手册 V1.0.4（第72～74页）及配套 X 示例；用户已确认 X42 V1.0 实物yaw和z的实时位置/状态反馈均正常。

`status_raw` 保留所有位：bit0使能、bit1驱动器到达、bit2堵转、bit3堵转保护、bit4/5左右限位输入高电平、bit7掉电标志。实体挡块没有开关信号，不能据限位输入位推断是否贴到挡块。bit7默认可能为1，不单独当作故障。`feedback.position_reached`仍是原始位；当前目标已到达看`motion.state=3`，该值要求FD接受后两个独立查询都有新回复，且使能、驱动原始到达与实际位置窗口均满足。

初始化和没有外部目标时只查询。Robot经板级注入X实例及每轴参数；收到CRANE_TARGET后调用XMotorSetTarget，未使能先F3并等新状态，再FD。重复活动目标不重发，新坐标增加generation并覆盖。FD每目标仅发一次，失败不自动重发FD，转入本轴停止。完整范围±UINT32_MAX，固定0.1°单位；不添加应用距离/角度换算。没有回零/清零输出或上电等待状态；失能仅由显式CRANE_DISABLE请求触发。

普通FD固定16字节：`addr FD sign acc:u16BE dec:u16BE speed:u16BE magnitude:u32BE 01 00 6B`。速度为0.1RPM，最大30000（3000RPM）；加/减速度为RPM/s，本接口要求1..65535。符号为绝对坐标正负，不是运动方向。协议经本地厂家X示例核对，普通FD无需较新固件的F1/FC快速模式。X42 V1.0的运动中覆盖、应答Response设置仍须实机核对，驱动须开启控制命令应答02；无ACK会在20ms后取消目标并停止。

参数位于`board/board_crane.h`，yaw初值10RPM，z于2026-11-11按用户要求提高10倍至100RPM（0.1RPM字段为1000）；两轴加/减速度仍各100RPM/s。USB CONFIG只改下个不同目标的RAM参数，当前轨迹参数快照在`motion.profile`；联动启动快照沿用各轴配置。到达窗口±10单位（1°电机轴），尚未标定齿轮输出角或丝杆位移。z提速版已构建，尚未烧录或实机验证。

停止先取消goal_active/command_accepted，不续发旧FD，再发送FE停止且保留使能。USB异常只停止活动/停止待确认的轴，不在第一次USB枚举时打断驱动回零。设备/反馈/控制应答故障按本轴隔离。FE失败后先查询新状态；若新状态已失能则确认停止，否则再次FE，停止待确认时拒绝新目标。故障保留，反馈正常且停止确认后需新的外部目标恢复。底盘设备故障与塔吊故障相互隔离，全局HOST_STOP同时停止底盘和两轴（底盘失能，塔吊不失能）。

## 时序与错误边界

- 每轴一个在途事务，20 ms应答期限，正常帧间/接收后静默至少2 ms。
- 超时、查询拒绝或传输异常后隔离40 ms，避免迟到帧立即误配下一查询；没有事务序号，不能保证识别任意延迟的同功能旧回复。
- 固定长度解析，支持分片及噪声后重新同步，不能遇到载荷里的 `6B` 就提前截帧。
- `01 00 EE 6B`记录为当前事务拒绝；控制ACK必须02，FD异步9F只计数；`0x3A`载荷中的`E2`等数值是状态位，不能当作拒绝码。
- 位置和状态分别在500 ms时过期；UART传输异常立即作废本轴全部反馈，保留旧数值供排错。旧数值和布尔状态必须结合有效位使用。
- 时间为motorTask解析时刻，不是驱动器硬件采样时刻。读取其他任务的快照应使用 `RobotCraneGetStatus()`，该接口在读取时再次检查反馈过期。

## 烧录后的观察方法

在调试器中展开 `system/robot_task.c` 的 `crane_status`：

| 观察项 | 含义 / 预期 |
| --- | --- |
| `axes[0]` / `axes[1]` | yaw / z |
| `initialized` | 板级实例及串口注册成功 |
| `uart_running` | 接收DMA运行，不代表驱动已在线 |
| `feedback.position_valid` | 最近500 ms内收到合法位置回复 |
| `feedback.status_valid` | 最近500 ms内收到合法状态回复 |
| `feedback.position_units` | 实测多圈累计位置，不是目标值 |
| `feedback.position_reached` | 驱动bit1，须同时看status_valid |
| `feedback.position_ms / status_ms` | 对应查询回复的时间持续更新 |
| `diagnostics.position_replies / status_replies` | 正常持续增长 |
| `diagnostics.timeouts / protocol_errors / rejected_queries` | 排查未回复、格式不符和查询拒绝 |
| `diagnostics.rx_bytes / transport_errors`、`uart_errors` | 区分有无收字节、协议故障和UART故障 |

通信阶段已验证两轴数据正常。运动阶段不增加手动测试入口；用正式USB二进制CRANE_TARGET发送目标。烧录后先按既定方式完成驱动回零，确认feedback两类有效且无堵转/保护。按机械允许的方向和行程选一个小绝对目标；示例+360只表示电机轴+36°，不能据此断言机构方向安全。

展开`crane_status.axes[0/1].motion`：`target_units`为请求目标，`sequence/generation`为命令标记；state应从1变2，最终3；`command_accepted`应为true、`fault=0`、`stop_pending=false`，actual来自feedback.position_units。`diagnostics.controls_sent`每个新目标正常增加一次（若原失能则另加一次F3），重复发送同一活动目标不增长。停止后state=0或5，goal_active=false，原目标只保留观察。

USB具体布局、CRC和发帧示例见usb-cdc-host-link.md；默认模式0才输出0x83，模式1不混发塔吊反馈。

## 显式失能与手动找点（2026-11-11）

USB新增`0C axis:u8`（0=yaw，1=z），经motorTask调用`XMotorDisable`，取消旧目标并发送`01 F3 AB 00 00 6B`。已核对本机厂家HAL_X_TTL使能控制示例的`X_V2_En_Control(addr, state, snF)`：state=0失能、snF=0立即执行。本工程独立实现帧组装，没有复制示例源码。失能须由发送后的新3A回复确认，ACK丢失也先查状态；仍使能/查询失败则重试。`motion.state=6`、`stop_pending=true`表示待确认，确认后state=0、stop_pending=false、feedback.enabled=false；同时必须检查status_valid。无效/在途旧反馈不能提前确认。

确认后持续查询位置/状态，不重发旧FD。`motion.disable_requested`/83 flags bit8只表示失能意图，不能替代实际enabled位。新的外部位置目标或联动事件仍会按原逻辑自动使能。STOP、USB异常可覆盖尚未完成的失能流程，保持原FE停止语义；已释放的轴不会被STOP主动使能。联动运行中先取消整段、停止两轴，再只失能指定轴。Z轴先支撑负载，驱动保持供电；观察`crane_status.axes[0/1].feedback.position_units`且position_valid=true，记录带符号累计值，不改零点。失能时手转多圈的实际累计连续性尚需本机核对。

Debug构建通过（FLASH 98344 B、RAM 41528 B）。X模块测试覆盖失能帧、新状态确认、ACK丢失、拒绝/重试、通信恢复、DMA在途FD/使能/停止取消、手动位置回复持续更新和新目标自动使能；USB测试覆盖分轴路由、长度/轴号拒绝、批次优先及联动每一步失能取消。未烧录，尚未实机验证失能或手动找点。

## 验证记录

- `cmake --preset Debug`、`cmake --build --preset Debug` 通过。
- `python tests/run_x_motor_tests.py`：真实 X 模块、板级代码和 UART BSP 在 ARM/Unicorn 下通过。覆盖多圈正负及32位幅值边界、分片/噪声/载荷含6B、错误符号/帧尾、状态全位、过期及时间回绕、迟到回复隔离、查询拒绝、DMA在途数据所有权、四串口容量及重复注册、同地址两轴路由和单轴HAL故障恢复。
- 单独编译运行现有 `RunEmmTests()`、`RunUARTTests()` 通过，验证共享 BSP 扩容后的底盘驱动与UART回归。
- 实机：用户确认烧录后两轴反馈数据正常；该证据只覆盖只读通信和当前反馈字段，不代表梯形位置运动或USB塔吊控制已经验证。
- 现有`tests/run_motor_tests.py`补充当前模块依赖后仍在编译失败：旧测试使用RobotInstance.odometry和旧RobotPositionInit参数，尚未适配既有OPS定位输入。未修改运行时代码去迎合旧测试，不能宣称整套底盘位置应用回归通过。
- 本次最终Debug构建占用FLASH 91356 B、主RAM 41128 B；motorTask静态栈仍为2048 B，编译器给出的任务入口自身栈568 B，未测量实际调用链运行栈峰值。

- `python tests/run_x_motor_tests.py`新增FD字节向量、正负绝对目标及32位幅值边界、DMA在途换目标、重复目标不重发、到达去旧样本/出窗口撤销、自动使能/使能中换目标、拒绝/超时/失能/堵转/反馈超时/串口错误后停止、FE失败后新状态确认及FD9F隔离测试。
- `python tests/run_crane_host_tests.py`通过真实USB缓存→协议→Robot→X目标路径，验证分轴配置、64位大小端、范围/保留位/长度错误、单轴/全局停止批次优先、USB断连/溢出取消、0x83布局/CRC、原STATUS/POSE节奏和FireWater隔离；同时运行原EMM、UART、USB协议/FireWater回归通过。HAL仅模拟，未访问硬件。
