# 位置环调参与四通道映射

## 控制结构

```text
VRPN local XYZ (+X nose, +Y left, +Z up)
                 │
          X/Y/Z position PID + v damping
                 │
        X/Y acceleration + VRPN yaw -> roll/pitch
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

- FC 遥测新鲜；
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

门控不满足时继续实时 RC 透传，位置环不启动。满足后发送至少四个 100 Hz 基线包
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

Z PID 第一次运行后使用标定悬停上桨油门前馈，而不是永久保留捕获的 RC 油门；
之后再叠加位置误差、积分和垂速阻尼：

```text
z_command = clamp(hover_throttle + Kp_z*e + integral
                  - (Kd_z + z_velocity_damping)*vz,
                  min_throttle, max_throttle)
```

下降时 `vz<0`，阻尼项增加油门帮助制动；上升时降低油门。默认
`z_velocity_damping=0.35`，单位是“上桨油门/m/s”。

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

## 位置 PID 与速度阻尼

每个轴独立保存参数和积分状态：

```text
integral[k] = integral[k-1] + Ki*e[k]*dt
u[k] = Kp*e[k] + integral[k]
       + Kd*(v_target[k]-v_measured[k])
```

- X/Y 输出：世界系水平加速度，单位 `m/s²`。
- Z 输出：上桨归一化绝对油门，单位不是加速度；映射为
  `DATA_RC.throttle=2*Z-1`，不再与切换时的 RC 油门基线相加。
- Kp/Ki/Kd 沿用原增量式参数尺度；对静止目标，两者在未饱和时代数等价。
- D 项直接使用 VRPN 实测速度和目标斜坡速度，不发生目标位置二阶差分冲击。
- 输出饱和且误差继续推向边界时暂停积分，离开饱和后可立即反向修正。
- 位置误差在进入 PID 前限幅。
- WebUI 请求的三维位置目标先按 `control.target_slew_rate_mps` 做欧氏距离
  限速，再进入 PID；默认 `0.30 m/s`，避免目标阶跃直接形成 D 项冲击。
- 高度通道额外使用轻量垂速阻尼 `-z_velocity_damping*vz`。
- 世界加速度使用 VRPN 绝对当前航向分解为 body-relative roll/pitch；目标偏航
  只生成 yaw-rate。飞控端从上电开始积分的无磁航向不用于位置解耦。
- 切换油门只作为限速起点，完成后不能限制高度环的向下权限。

## 三态位置调度

位置控制器内部使用三个互斥工作模式，不改变用于链路和故障处理的顶层
`ControlState`：

| 模式 | 进入条件 | 使用参数 |
|---|---|---|
| `FAR_TRACK` | 距目标较远，或目标正在移动 | `position_loop.x/y/z` |
| `APPROACH` | 目标静止且水平、垂直误差同时进入制动区 | `position_loop.approach` |
| `PRECISION_HOLD` | 水平、垂直误差和速度同时进入近场门限并持续驻留 | `position_loop.precision_hold` |

模式转换使用滞回和驻留时间：

- 目标速度超过 `target_motion_speed_mps` 时立即退出近场模式；
- 进入精保要求 `hold_enter_distance_m`、
  `hold_enter_vertical_error_m`、`hold_enter_speed_mps` 和
  `hold_enter_vertical_speed_mps` 同时满足；
- 任一退出门限持续 `hold_exit_dwell_s` 后才退出，避免门限附近频繁切换；
- 水平误差超过 `brake_exit_distance_m` 或垂直误差超过
  `brake_exit_vertical_error_m` 时直接回到远场快速模式；
- 模式切换不清零速度估计或积分状态，三轴增益在
  `gain_transition_time_s` 内平滑过渡；
- 目标正在按台架限速滑动时冻结位置积分，防止远距离目标阶跃积累过大积分。

`PRECISION_HOLD` 不是把飞机锁在进入模式时的当前位置，目标点始终是操作者设定的
绝对位置。该模式使用更低带宽的 PD 和更小的积分增益，静差仍需由慢积分或 trim
修正；不能用冻结当前位置的假保持换取表面上的低振动。

当前生效配置使用：

```json
{
  "position_loop": {
    "x": {"kp": 1.35, "ki": 0.05, "kd": 0.60},
    "y": {"kp": 1.50, "ki": 0.06, "kd": 0.72},
    "z": {"kp": 0.22, "ki": 0.015, "kd": 0.10},
    "approach": {
      "x": {"kp": 0.80, "ki": 0.01, "kd": 1.10},
      "y": {"kp": 0.85, "ki": 0.01, "kd": 1.20},
      "z": {"kp": 0.14, "ki": 0.005, "kd": 0.16}
    },
    "precision_hold": {
      "x": {"kp": 0.30, "ki": 0.005, "kd": 0.90},
      "y": {"kp": 0.34, "ki": 0.005, "kd": 1.00},
      "z": {"kp": 0.08, "ki": 0.003, "kd": 0.14}
    },
    "mode_scheduling": {
      "enabled": true,
      "brake_enter_distance_m": 0.55,
      "brake_exit_distance_m": 0.75,
      "brake_enter_vertical_error_m": 0.30,
      "brake_exit_vertical_error_m": 0.40,
      "hold_enter_distance_m": 0.12,
      "hold_exit_distance_m": 0.22,
      "hold_enter_vertical_error_m": 0.06,
      "hold_exit_vertical_error_m": 0.12,
      "hold_enter_speed_mps": 0.08,
      "hold_exit_speed_mps": 0.15,
      "hold_enter_vertical_speed_mps": 0.05,
      "hold_exit_vertical_speed_mps": 0.10,
      "target_motion_speed_mps": 0.02,
      "hold_enter_dwell_s": 0.50,
      "hold_exit_dwell_s": 0.20,
      "gain_transition_time_s": 0.60
    }
  }
}
```

当前 WebUI 的 XYZ 编辑框对应 `FAR_TRACK` 参数；`approach`、
`precision_hold` 和调度门限从配置文件加载，热更新时会原样保留。实时状态页显示
当前模式、水平/垂直误差、速度和增益过渡状态。

## 手动飞行辨识后的保守初值

X/Y 初值来自 `flight_20261021T061051_495Z_aircraft-01_2.csv` 的 RC
手动环绕段；水平速度最大约 X 0.92 m/s、Y 1.31 m/s，roll/pitch 约 ±13°。
对位置二次拟合得到水平加速度后，拟合
`a = K * tilt - c * velocity`，得到 `K≈1.0`、速度阻尼约
`1.5–3.0 s⁻¹`。

垂向参数由 `flight_20261021T092543_068Z_aircraft-01_2.csv` 在控制器闭环段
重新拟合，估计有效悬停上桨油门约 `0.606`，响应延迟约 `160 ms`。该结果作为
当前控制前馈，仍需台架和限高飞行复核，不是闭环飞行验收结论。

| 项目 | 值 |
|---|---:|
| 最大 roll/pitch 指令 | 18° |
| 地面站 yaw-rate 限幅 | 0.50 rad/s |
| 飞控满杆 yaw-rate 量程 | 1.5 rad/s |
| 上桨最小/悬停/最大油门 | 0.20 / 0.606 / 0.96 |
| 当前 Z PID | Kp=0.22, Ki=0.015, Kd=0.10 |
| Z 垂速阻尼 | 0.35 油门/(m/s) |
| 接管最大垂速 | 0.20 m/s |
| 当前 X PID | Kp=1.35, Ki=0.05, Kd=0.60 |
| 当前 Y PID | Kp=1.50, Ki=0.06, Kd=0.72 |

水平参数来自 `2026-10-22` 两次闭环日志的折中：较高一组把 Y 轴误差 RMSE
从约 `0.47 m` 降到 `0.14 m`、Z 轴静差降到约 `0.05 m`，但 X/Y 速度 RMS
分别增至约 `0.19/0.28 m/s`，后段存在明显往复修正。当前组回收部分 Kp、Ki
和 Kd，保留新版响应速度，同时降低水平控制活动量；必须通过下一轮固定目标
悬停再次确认。

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

航向控制使用绝对动捕 yaw，并对实测 yaw-rate 制动：

```text
yaw_rate_command = clamp(
    Kp_yaw*(target_yaw-current_yaw) - Kd_yaw*measured_yaw_rate,
    -max_yaw_rate, max_yaw_rate)
```

随后对 yaw-rate 指令按 `yaw_rate_slew_rate_rad_s2` 限速，再除以飞控模型的
满杆量程 `fc_yaw_rate_scale_rad_s` 得到 `DATA_RC.yaw`。当前使用
`Kp=1.0, Kd=0.50`、物理指令限幅 `0.50 rad/s`、斜率 `1.5 rad/s²` 和飞控
量程 `1.5 rad/s`。

## WebUI 调参

“位置环调参”面板为 X/Y/Z 分别提供 Kp/Ki/Kd，并可调整：

- 最大水平加速度。
- 最大倾角。
- yaw-rate 角度增益、角速度阻尼、物理最大 yaw-rate、指令斜率和飞控满杆量程。
- 上桨最小、悬停、最大油门。

点击“应用位置 PID”后通过 `/ws/v1/realtime` 热更新，不中断控制线程，也不重置
已有 PID 输出状态。页面同时显示三轴输出和最终
`[roll,pitch,yaw,throttle]`。只有飞控遥测报告 `GS_MODE_ACTIVE` 时，顶部横幅才显示
`GROUND STATION CONTROL ACTIVE`；命令已发送但飞控尚未确认时显示等待接手，并明确
确认前显示 `RC PASSTHROUGH · WAITING FOR FC`；确认后的最短保持期间显示
`GROUND HANDOFF · HOLDING RC`，并列出锁存
基线和当前四通道。

## 安全边界

- 只有 deadman 持续有效且 VRPN 位姿新鲜时才发送 `DATA_RC`。
- FC 遥测不新鲜时，GS-Link 层拒绝 `DATA_RC`；心跳只限制需要 session 的
  起飞/降落/abort 命令。
- 再次点击或按空格释放 deadman、窗口失焦或链路断开时停止新 `DATA_RC`，
  由飞控超时立即回 RC。
- 地面站不能 arm，不能修改 RC 通道 8，不能绕过 `GroundStationGate`。
- 参数可以在线调整不代表已完成飞行验证；真实悬停油门、VRPN Z 基准和闭环增益
  必须在拆桨台架、HIL 和系留测试中重新确认。
