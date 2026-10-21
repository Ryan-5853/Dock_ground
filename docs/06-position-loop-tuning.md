# 位置环调参与四通道映射

## 控制结构

```text
VRPN local XYZ (+X nose, +Y left, +Z up)
                 │
          X/Y/Z incremental PID
                 │
        X/Y acceleration -> roll/pitch
        Z throttle       -> upper collective
        target yaw       -> yaw rate
                 │
                 v
GS-Link DATA_RC [roll, pitch, yaw, throttle]
                 │
                 v
PX4 LqiManualReference + identified 19-state composite LQI
                 │
                 v
[lower_motor, servo_1, servo_2, servo_3] + upper throttle
```

地面站不直接产生四个执行器命令。上桨由 `DATA_RC.throttle` 外部持有；飞控的
辨识 LQR 才生成下桨和三路舵机命令。

## RC → 地面站无扰切换

进入地面站控制时不允许直接跳到位置环的悬停或零姿态输出。飞控确认
`GS_MODE_ACTIVE` 后，地面站重新读取最新 RC 通道并锁存为四通道基线：

```text
roll_base     = rc[mapping.roll]
pitch_base    = rc[mapping.pitch]
throttle_base = rc[mapping.throttle]
yaw_base      = rc[mapping.yaw]
```

基线锁存条件：

- FC 遥测和心跳新鲜；
- `ARMED`；
- `RC_VALID` 且 `RC_SWITCH_VALID`；
- RC 通道 8 位于地面站选定端点；
- `RC_CHANNELS_VALID`，且所有映射通道不是 `-128`；
- 当前没有 `FAILSAFE`；
- FC 遥测已报告 `GS_MODE_ACTIVE`。

飞控确认前，地面站只把遥测中的最新 RC 四通道实时透传为 `DATA_RC`，以建立
有效的 `ground_station_valid`；此时 `handoff_active=0`，不运行位置环。飞控通道
8 的 100 ms 端点确认和 External Mode 状态仍是独立硬门控。

遥测确认 `GS_MODE_ACTIVE` 后，地面站才重新采样最新 RC 四通道作为交接基线，
并检查：

- 滤波后 `|vz| <= handoff_max_vertical_speed_mps`，默认 `0.20 m/s`；
- 当前上桨油门与 `hover_throttle` 的偏差不超过
  `handoff_max_throttle_delta`，默认 `0.10`。

门控不满足时继续实时 RC 透传，位置环不启动。满足后发送至少两个 50 Hz 基线包
（默认 `bumpless_hold_ms=40`）。deadman 释放、姿态过期、RC 门控离开、链路
超时或重新进入模式都会清除基线；再次进入时重新采样当前 RC。

确认后的保持结束后，R/P/Y 的 PID 增量叠加到固定 RC 基线上；高度通道不属于人工油门
基线，而是按实际差值限速到绝对上桨油门：

```text
throttle_pid = clamp(2*z_pid - 1, -1, 1)
max_step     = 2 * dt / (throttle_transition_ms / 1000)
throttle[k]  = throttle[k-1]
             + clamp(throttle_pid - throttle[k-1], -max_step, max_step)
```

Z PID 第一次运行使用捕获的 RC 上桨油门，而不是固定悬停默认值。之后命令中
再叠加垂速阻尼：

```text
z_command = clamp(z_pid - z_velocity_damping * vz,
                  min_throttle, max_throttle)
```

下降时 `vz<0`，阻尼项增加油门帮助制动；上升时降低油门。默认
`z_velocity_damping=0.08`，单位是“上桨油门/m/s”。

`throttle_transition_ms` 是走完 `[-1,1]` 满量程所需的最长时间尺度，默认 300 ms。
实际变化量与基线到目标的差值成正比：差值很小时一到两个周期即可完成，差值
接近满量程时才需要约 300 ms。过渡完成后 `throttle=throttle_pid`，高度环始终
拥有从最小油门到最大油门的完整权限。不能把高度 PID 永久加到较高的切换油门
基线上；否则即使飞机高于目标，最低指令仍可能维持爬升。

RC 通道使用接收机原始顺序，1 基配置如下：

```json
{
  "rc_roll_channel": 1,
  "rc_pitch_channel": 2,
  "rc_throttle_channel": 3,
  "rc_yaw_channel": 4,
  "bumpless_hold_ms": 40,
  "throttle_transition_ms": 300,
  "handoff_max_vertical_speed_mps": 0.20,
  "handoff_max_throttle_delta": 0.10
}
```

这是 AETR 默认映射，必须与 PX4 `RC_MAP_ROLL/PITCH/THROTTLE/YAW` 一致。当前
实际发送的四通道和锁存基线会同时显示在 WebUI 顶部红色/琥珀色横幅中。

## 增量 PID

每个轴独立保存参数和输出：

```text
du = Kp*(e[k]-e[k-1]) + Ki*e[k]*dt
     + Kd*(e[k]-2e[k-1]+e[k-2])/dt
u[k] = clamp(u[k-1] + du)
```

- X/Y 输出：世界系水平加速度，单位 `m/s²`。
- Z 输出：上桨归一化绝对油门，单位不是加速度；映射为
  `DATA_RC.throttle=2*Z-1`，不再与切换时的 RC 油门基线相加。
- 首次控制周期只应用 P/I 和初始油门，不产生 D 脉冲。
- 首次控制周期把当前误差同时写入两拍 D 历史，因此第二个控制周期也只计算
  `(e[1]-e[0])/dt`，不会产生虚假的反向 D 冲击。
- 输出饱和时增量状态停在边界，避免继续 windup。
- 位置误差在进入 PID 前限幅。
- 高度通道额外使用轻量垂速阻尼；X/Y 仍不增加额外速度环。
- 切换油门只作为限速起点，完成后不能限制高度环的向下权限。

## 手动飞行辨识后的保守初值

X/Y 初值来自 `flight_20261021T061051_495Z_aircraft-01_2.csv` 的 RC
手动环绕段；水平速度最大约 X 0.92 m/s、Y 1.31 m/s，roll/pitch 约 ±13°。
对位置二次拟合得到水平加速度后，拟合
`a = K * tilt - c * velocity`，得到 `K≈1.0`、速度阻尼约
`1.5–3.0 s⁻¹`。

垂向参数改由最新日志
`flight_20261021T073641_768Z_aircraft-01_1.csv` 的 8.2–13.55 s RC 段估计。
对垂速差分与 CH3 映射做线性拟合，有效悬停上桨油门约 `0.564`，因此配置改用
保守值 `0.56`。该估计的拟合 RMSE 约 `0.39 m/s²`，但仍需台架和限高飞行复核，
不是闭环飞行验收结论。

| 项目 | 值 |
|---|---:|
| 最大 roll/pitch 指令 | 20° |
| 最大 yaw-rate | 1.2 rad/s |
| 上桨最小/悬停/最大油门 | 0.20 / 0.56 / 0.96 |
| 保守 Z PID | Kp=0.06, Ki=0.01, Kd=0.08 |
| Z 垂速阻尼 | 0.08 油门/(m/s) |
| 接管最大垂速 | 0.20 m/s |
| 保守 X/Y PID | Kp=0.60, Ki=0.01, Kd=0.30 |

控制极性：

| 期望运动 | 日志中的实测关系 | 控制器输出 |
|---|---|---|
| 世界 `+X`（前） | 正 pitch → `+X` 加速度 | `DATA_RC.pitch > 0` |
| 世界 `+Y`（左） | 正 roll → `-Y` 加速度 | `DATA_RC.roll < 0` |
| 世界 `+Z`（上） | 正 throttle → `+Z` 加速度 | 上桨油门增加 |
| 世界 `+Yaw`（左转） | 正 `DATA_RC.yaw` → PX4 右偏航 | `DATA_RC.yaw < 0` |

实测带符号相关系数分别为：roll→Y 加速度 `-0.47`、pitch→X 加速度
`+0.33`、CH3→Z 加速度 `+0.77`、CH4→PX4 偏航率 `+0.91`。这些关系与
代码中的坐标变换及 PX4 通道正方向一致。`DATA_RC.throttle=2*upper_throttle-1`；
30% 上桨油门对应 `-0.4`。

## WebUI 调参

“位置环调参”面板为 X/Y/Z 分别提供 Kp/Ki/Kd，并可调整：

- 最大水平加速度。
- 最大倾角。
- yaw-rate 增益和最大 yaw-rate。
- 上桨最小、悬停、最大油门。

点击“应用增量 PID”后通过 `/ws/v1/realtime` 热更新，不中断控制线程，也不重置
已有 PID 输出状态。页面同时显示三轴输出和最终
`[roll,pitch,yaw,throttle]`。只有飞控遥测报告 `GS_MODE_ACTIVE` 时，顶部横幅才显示
`GROUND STATION CONTROL ACTIVE`；命令已发送但飞控尚未确认时显示等待接手，并明确
确认前显示 `RC PASSTHROUGH · WAITING FOR FC`；确认后的最短保持期间显示
`GROUND HANDOFF · HOLDING RC`，并列出锁存
基线和当前四通道。

## 安全边界

- 只有 deadman 持续有效且 VRPN 位姿新鲜时才发送 `DATA_RC`。
- FC 遥测和心跳不新鲜时，GS-Link 层拒绝所有 `DATA_RC`。
- 再次点击或按空格释放 deadman、窗口失焦或链路断开时停止新 `DATA_RC`，
  由飞控超时立即回 RC。
- 地面站不能 arm，不能修改 RC 通道 8，不能绕过 `GroundStationGate`。
- 参数可以在线调整不代表已完成飞行验证；真实悬停油门、VRPN Z 基准和闭环增益
  必须在拆桨台架、HIL 和系留测试中重新确认。
