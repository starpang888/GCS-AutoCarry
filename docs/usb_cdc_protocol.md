# USB CDC 底盘通信协议

本文档描述上位机通过 USB CDC 虚拟串口控制底盘，以及接收四个电机位置反馈的方式。接口传输的是二进制字节，不是十六进制文本；串口工具应选择十六进制发送或由上位机程序直接写入字节。

## 连接与调用顺序

1. 用 USB 数据线连接控制板，在上位机找到新出现的虚拟串口，例如 `COM5`。
2. 打开该串口。CDC 是 USB 虚拟串口，波特率参数通常不参与实际传输；若所用串口库要求填写，可填写 `115200`。
3. 控制板上电后电机默认为失能。先发送电机使能帧，再发送底盘动作帧。
4. 控制板会周期上报四个电机的位置反馈。收到停止命令后底盘目标速度清零；需要重新运动时再发送新的动作命令。

目前协议没有命令应答帧。上位机可以通过收到的编码器反馈和实际底盘行为确认运行状态。动作命令会保持为最近一次收到的有效命令，直到收到新的动作命令；当前固件尚未实现通信超时停车。

## 通用帧格式

```text
AA 55 LEN TYPE SEQ PAYLOAD... CRC_L CRC_H
```

| 字段 | 长度 | 含义 |
|---|---:|---|
| `AA 55` | 2 字节 | 固定帧头 |
| `LEN` | 1 字节 | `TYPE + SEQ + PAYLOAD` 的总字节数，不含帧头和 CRC |
| `TYPE` | 1 字节 | 帧类型 |
| `SEQ` | 1 字节 | 上位机命令序号；下位机当前不检查该值 |
| `PAYLOAD` | 可变 | 命令或反馈数据 |
| `CRC_L CRC_H` | 2 字节 | CRC16，低字节在前 |

CRC 使用 CRC16-CCITT：初值 `0xFFFF`，多项式 `0x1021`，不反射、结果不异或。计算范围从 `LEN` 开始，直到 `PAYLOAD` 结束；`AA 55` 和 CRC 字段本身不参与计算。

下位机命令接收解析器当前接受 `LEN=2..18`。编码器反馈是下位机发出的数据帧，`LEN=22`，整帧长度为 27 字节。上位机解析时应按帧内 `LEN` 取完整帧，不要把 USB 一次读到的数据块当成一帧：一次读取可能只有半帧，也可能包含多帧。

## 上位机控制命令

### 底盘动作 `TYPE=0x01`

```text
PAYLOAD = ACTION SPEED_L SPEED_H
```

`SPEED` 是无符号 16 位小端整数。平移速度单位为 `mm/s`，旋转速度单位为 `mrad/s`。动作编号如下：

| `ACTION` | 动作 | 速度含义 |
|---:|---|---|
| `0` | 停止 | 忽略，建议填 0 |
| `1` | 前进 | 平移速度 |
| `2` | 后退 | 平移速度 |
| `3` | 左移 | 平移速度 |
| `4` | 右移 | 平移速度 |
| `5` | 左旋转 | 角速度 |
| `6` | 右旋转 | 角速度 |

速度换算到固件内部时除以 1000。例如发送 `50 mm/s`，固件收到后使用 `0.05 m/s`；旋转速度 `500 mrad/s` 会换算为 `0.5 rad/s`。开始测试建议使用较小速度。

### 电机使能 `TYPE=0x02`

```text
PAYLOAD = ENABLE
```

`ENABLE=0x00` 表示失能，`ENABLE=0x01` 表示使能。控制板启动时默认失能。建议先发送停止动作，再根据需要失能电机。

### 命令示例

以下示例中的 `SEQ` 可由上位机逐帧递增，达到 `0xFF` 后回绕到 `0x00`。这些 CRC 已按本协议计算：

| 用途 | 完整帧 |
|---|---|
| 使能电机，`SEQ=0x01` | `AA 55 03 02 01 01 6C 52` |
| 前进 `50 mm/s`，`SEQ=0x01` | `AA 55 05 01 01 01 32 00 33 C5` |
| 停止，`SEQ=0x02` | `AA 55 05 01 02 00 00 00 28 0A` |
| 失能电机，`SEQ=0x02` | `AA 55 03 02 02 00 1E 17` |

动作命令只设定底盘目标；电机必须处于使能状态才能执行运动。停止命令会将四轮目标速度设为 0，但保持电机使能。

## 编码器反馈 `TYPE=0x81`

反馈数据帧格式：

```text
AA 55 16 81 SEQ
  LF_SIGN LF_COUNT0 LF_COUNT1 LF_COUNT2 LF_COUNT3
  RF_SIGN RF_COUNT0 RF_COUNT1 RF_COUNT2 RF_COUNT3
  RR_SIGN RR_COUNT0 RR_COUNT1 RR_COUNT2 RR_COUNT3
  LR_SIGN LR_COUNT0 LR_COUNT1 LR_COUNT2 LR_COUNT3
  CRC_L CRC_H
```

| 字段 | 含义 |
|---|---|
| `LEN=0x16` | 22 字节：类型、序号和 20 字节位置数据 |
| `TYPE=0x81` | 四轮位置反馈 |
| `SEQ` | 下位机发送序号，每帧递增并按 8 位回绕 |
| `*_SIGN` | 位置符号：`0` 正，`1` 负 |
| `*_COUNT0..3` | 32 位位置绝对值，小端序；`COUNT0` 是最低字节 |

电机顺序为左前、右前、右后、左后。位置计数每 `65536` 对应电机轴转一圈。将计数换算成带符号角度：

```text
angle_deg = (SIGN == 1 ? -1 : 1) * COUNT * 360 / 65536
```

反馈任务约每 100 ms 尝试发送一帧；实际间隔还受四路 UART5 反馈查询耗时影响。CDC 正在发送时，本轮可能跳过，因此上位机应允许序号偶尔不连续。位置读取失败时，下位机会保留该电机上一次成功读取的位置。

## Python 上位机发送示例

安装依赖：`python -m pip install pyserial`。将 `COM5` 改成设备管理器中实际出现的端口。示例先使能电机，以低速前进一秒，然后停止并关闭串口；如果要继续观察位置反馈，可以在串口打开期间循环读取并按上述格式解析 `0x81` 帧。

```python
import struct
import time

import serial


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def make_frame(frame_type: int, sequence: int, payload: bytes) -> bytes:
    body = bytes((len(payload) + 2, frame_type, sequence)) + payload
    crc = crc16_ccitt(body)
    return b"\xAA\x55" + body + struct.pack("<H", crc)


def chassis_command(sequence: int, action: int, speed: int) -> bytes:
    payload = bytes((action,)) + struct.pack("<H", speed)
    return make_frame(0x01, sequence, payload)


with serial.Serial("COM5", 115200, timeout=0.1) as port:
    port.write(make_frame(0x02, 0, b"\x01"))  # 使能电机
    time.sleep(0.1)
    port.write(chassis_command(1, 1, 50))       # 前进 50 mm/s
    time.sleep(1.0)
    port.write(chassis_command(2, 0, 0))        # 停止
```

## 上位机接收解析要点

1. 持续读取并追加到接收缓存，查找 `AA 55`；帧头前的无关字节可以丢弃。
2. 读取 `LEN`，整帧总长度为 `LEN + 5` 字节：2 字节帧头、`LEN` 字节内容和 2 字节 CRC。
3. 数据不足一帧时保留缓存并等待后续数据；数据包含多帧时循环解析。
4. 对从 `LEN` 到 `PAYLOAD` 末尾的数据计算 CRC16-CCITT，并与低字节在前的帧尾比较。
5. CRC 正确且 `TYPE=0x81` 时，按左前、右前、右后、左后的顺序解析四组符号和位置计数。
