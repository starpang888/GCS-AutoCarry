# GCS-AutoCarry

基于STM32F407VET6、STM32 HAL、FreeRTOS和CMake的四轮X型全向底盘下位机固件。UART5统一管理四台EMM V5.0驱动器，USB CDC接收速度或绝对位置目标，并上传设备状态、轮式里程计与稳定到达状态。

## 当前功能

- VELOCITY自动申请使能并保持最新目标，无需ARM、周期续发或递增sequence。
- POSITION接收相对启动原点的绝对X/Y/yaw目标，使用轮式里程计闭环控制。
- 稳定到达要求进入几何容差、实测低速并持续至少300ms；到达后继续保持位置和使能。
- STOP取消运动目标并请求停止失能；USB异常、设备故障或位置模式定位失效也取消对应目标，恢复需新命令。
- 二进制模式轮流上传STATUS与POSE，各名义20Hz；可在静止时切换为22通道FireWater观测。
- PB2状态灯由任务心跳监测驱动；motorTask唯一拥有总线和控制更新。

当前位置容差5mm、yaw容差约1°，位置P=0.5、角度P=1.0。这些是控制参数，不能当作已完成标定的真实定位精度。当前yaw来自四轮编码器，尚未融合陀螺仪。

## 硬件与坐标

| 项目 | 当前配置 |
| --- | --- |
| MCU | STM32F407VET6 |
| 电机 | 四台EMM V5.0，UART5 TTL 115200/8N1 |
| 地址顺序 | 1左前、2右前、3右后、4左后 |
| 轮径 | 75mm，直驱；轮距/轴距暂用240/240mm，待精确标定 |
| 主机接口 | MCU原生USB FS CDC，PA11/PA12 |
| 坐标 | X前、Y左、yaw俯视逆时针为正 |
| 应用单位 | m、s、rad；USB使用对应的整数毫单位 |

## USB接口

小端公共帧：`A5 5A | 01 | type:u8 | length:u16 | sequence:u32 | payload | CRC16:u16`。CRC为CCITT-FALSE。USB接收须支持拆包和粘包，不能按一次读取或固定帧长处理所有消息。

| type | 方向 | 载荷 |
| --- | --- | --- |
| 0x01 VELOCITY | 主机→STM32 | 3×int32：vx mm/s、vy mm/s、wz mrad/s |
| 0x02 ARM | 主机→STM32 | 空；可选零速预使能 |
| 0x03 STOP | 主机→STM32 | 空；取消目标并请求停止失能 |
| 0x04 POSITION | 主机→STM32 | 3×int32：绝对X mm、Y mm、yaw mrad |
| 0x81 STATUS | STM32→主机 | 196字节载荷，总帧208字节 |
| 0x82 POSE | STM32→主机 | 64字节载荷，总帧76字节 |

POSITION需要有效定位且当前不在停止确认阶段；相同坐标重发不会追加位移或重置到达计时。VELOCITY（包括零速）取消位置模式。到达状态随当前条件变化，不是锁存事件；上位机结合目标、有效标记与arrival_state判断。

速度模式没有主机静默超时，主机停止发布而USB仍连接时会保持最后速度；需要停车时发送零速或STOP。底盘不设软件速度上限，保留驱动协议原生±5000字段单位范围。完整停止、故障和恢复条件见协议文档。

- [下位机交接说明](docs/lower-controller-handoff.md)
- [公共协议与STATUS布局](docs/usb-cdc-host-link.md)
- [POSITION、POSE与到达条件](docs/position-feedback.md)
- [FireWater观测说明](docs/vofa-feedback.md)
- [方向、里程计与位置控制观测记录](docs/chassis-calibration-record.md)
- [发给上位机的协议文档包](dist/stm32_usb_protocol_v1.zip)

ROS 2节点、USB桥接、Odometry消息、TF和部署由上位机负责人维护。POSE已是积分后的位姿，不应再次当作位移增量积分。

## 工程结构

| 目录 | 职责 |
| --- | --- |
| board/ | 硬件资源、机械参数与实例组装 |
| bsp/ | LED、UART、USB外设服务 |
| modules/ | EMM总线/电机、主机协议、任务心跳监测 |
| application/ | 底盘运动学、里程计、位置控制与整车命令仲裁 |
| system/ | 任务调度、USB业务收发及观察快照 |
| Core/、Drivers/、Middlewares/、USB_DEVICE/ | CubeMX生成边界与平台依赖 |
| tests/ | 离线固件回归，不编入STM32，不包含供用户操作电机的主机程序 |

架构借鉴湖南大学跃鹿HNUYueLuRM/basic_framework的分层、C实例组合与配置注入，阅读记录见[框架学习文档](docs/yuelu-framework-study.md)。

## 构建与离线验证

需安装Arm GNU工具链、CMake 3.22或以上版本和Ninja，并将工具链加入PATH。

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

产物：`build/Debug/GongChuangsai.elf`。构建目录不提交到仓库。

ARM离线测试使用Python与Unicorn；首次安装依赖后运行：

```powershell
python -m pip install --target build/test-deps unicorn
python tests/run_motor_tests.py
python -m unittest discover -s tests -p test_host_link.py -v
```

当前已通过位置/到达版完整Debug构建和离线回归；用户已报告单轴位置、原地旋转与反馈数据显示正常。平移旋转联合实机验证暂缓，完整到达状态转换、ROS 2双方联调及精确标定尚未逐项确认，软件模拟不等同于实机验证。
