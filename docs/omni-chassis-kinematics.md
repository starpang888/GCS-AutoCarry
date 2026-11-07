# 四全向轮 X 型 45° 底盘解算

正式版通过USB命令持续更新底盘目标，并根据四轮实测速度反解反馈。已删除旧预览和手动标定入口。通信见 [USB协议](usb-cdc-host-link.md)，既往核对结果见 [标定记录](chassis-calibration-record.md)。

## 几何与符号

采用普通全向轮模型：俯视四轮轮轴沿底盘对角线呈 X 型，主动滚动方向与对应轮轴垂直。不是麦克纳姆轮模型。底盘中心为原点，x 向前、y 向左、z 向上，正 yaw 从上往下看为逆时针。轮序固定为 1 左前、2 右前、3 右后、4 左后。

板级参数位于 `board/board_motor.c`，由 `BoardChassisKinematicsInit()` 注入：

- 轮半径 r = 0.0375 m，直驱。
- 前后轮中心间距 L = 0.240 m，左右轮中心间距 W = 0.240 m；两项仍是用户指定的暂定尺寸。
- 旋转几何量 k = (L + W) / 2 = 0.240 m。
- 底盘不设轮速上限，不对解算结果缩放；驱动模块仍校验EMM协议支持的原生±5000字段单位范围（名义1 RPM/单位）。

数学轮速正方向沿各轮驱动方向定义，以纯正 yaw 时四轮均为正为约定。各轮单位驱动向量依次为：左前 (-s,+s)、右前 (+s,+s)、右后 (+s,-s)、左后 (-s,-s)，其中 s = 1/√2。这是解算坐标的约定，不能直接当成驱动器 CW 方向；驱动器与数学轮速之间的安装符号保留在板级边界。

## 解算关系

输入 `vx/vy` 为 m/s，`wz` 为 rad/s；输出为轮轴 rad/s：

```text
omega_1 = (-vx + vy + k*wz) / (sqrt(2)*r)
omega_2 = (+vx + vy + k*wz) / (sqrt(2)*r)
omega_3 = (+vx - vy + k*wz) / (sqrt(2)*r)
omega_4 = (-vx - vy + k*wz) / (sqrt(2)*r)
```

四轮目标直接按上述关系计算，不限幅、不缩放。`target.requested` 与 `target.applied` 相同，均为RPM整数取整前的计算目标，不是实测反馈。NaN、Inf、溢出及非法参数返回失败并清空解算输出。超过驱动协议原生范围的目标在驱动提交时返回失败，底盘进入MOTOR_ERROR并停止，不截断或回绕字段。

由正确符号的实际轮轴速度反算底盘速度：

```text
vx = r/(2*sqrt(2))   * (-omega_1 + omega_2 + omega_3 - omega_4)
vy = r/(2*sqrt(2))   * (+omega_1 + omega_2 - omega_3 - omega_4)
wz = r/(2*sqrt(2)*k) * (+omega_1 + omega_2 + omega_3 + omega_4)
```

`OmniKinematicsForward()` 只提供数学接口，不擅自认定反馈有效，不执行目标限幅，不积分位置。调用方需要先核验四轮真实反馈、采样时间及安装符号；当前四轮SI安装符号均为−1，已同时用于目标和反馈。

## 运行接入与验证

`BoardChassisKinematicsInit()`在任务启动前构造模型，`ChassisControlCommand()`解算USB目标，`ChassisControlUpdate()`刷新电机输出；`HostServiceTransmit()`核验反馈有效性、新鲜度和四轮时间差后反解实测底盘速度。无效反馈保留无效标志，不能用目标代替测量。里程计积分和TF由上位机负责。

离线回归验证不等前后/左右间距、三种基本运动、混合正反解算、超过旧20 RPM上限仍保持原始目标、非法浮点输入及方向单位；编译和回归结果见 [交接说明](lower-controller-handoff.md)。
