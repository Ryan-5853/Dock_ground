# ARM 自动飞行日志

## 触发规则

日志由飞控遥测中的 `ARMED` 状态触发，不依赖地面站 deadman：

- FC 遥测新鲜、`telemetry.valid=true`；
- `status_flags.ARMED` 从 0 变为 1；
- 若服务启动时已经 armed，第一帧 armed 遥测也会开始记录。

一次连续可观测的 arm 对应一个独立会话。以下情况写最后一行并封存：

- 收到明确的 disarm；
- armed 状态连续未知超过 `telemetry_stale_timeout_ms`，默认 2 s；
- 服务正常停止；
- CSV 写入失败。

RC 模式、地面站控制、等待切换、deadman 释放和 failsafe 都处于同一会话内，
不会因控制源切换拆文件。
如果 FC 遥测中断超过阈值而无法确认 disarm，当前文件会封存；遥测恢复且飞控仍
处于 armed 时，会新建续接文件，避免无限等待旧会话。

## 配置

```json
{
  "flight_log": {
    "enabled": true,
    "directory": "logs/flights",
    "telemetry_stale_timeout_ms": 2000
  }
}
```

目录必须由地面站进程可写。日志写盘在独立线程中完成，使用有界队列；不会在
100 Hz 控制线程中创建目录、打开文件或同步 flush。

## 文件

每个 arm 会话生成三个文件：

```text
flight_<UTC>_<aircraft>_<serial>.csv
flight_<UTC>_<aircraft>_<serial>.json
flight_<UTC>_<aircraft>_<serial>.summary.json
```

- CSV：逐控制周期数据，默认 100 Hz。
- JSON：会话开始时间、机体、RC 映射和初始 PID 参数。
- summary JSON：结束原因、持续时间、帧数。

## CSV 内容

每行包含：

- 系统 UTC 时间和地面站单调纳秒时间；
- FC 遥测/心跳新鲜度、各自龄期、GS-Link 连接与 `uplink_allowed`；
- 命令发送器连接状态、成功/失败计数、最近错误类型和 UDP `errno`；
- `ARMED`、GS mode、RC valid、通道 8 selected、failsafe；
- 当前控制源：`RC`、`RC_PASSTHROUGH`、`GROUND_STATION` 或 `LOOPBACK`；
- RC 透传、垂速门控值、deadman、交接保持状态和高度油门过渡混合比例；
- VRPN 局部位置、姿态四元数、位姿序号和龄期；
- 目标位置和偏航；
- RC 锁存基线与当前四通道；
- 当前 PID 三轴输出和期望姿态；
- FC roll/pitch/yaw、双电机 RPM、电压、电流、电量；
- 18 路原始 RC 通道；
- X/Y/Z 当前 Kp/Ki/Kd 及限幅/油门参数；
- GS-Link RX/TX、CRC、序号缺口、重试和 ACK 统计。

FC 基础遥测没有位置/速度，所以位置和姿态来自 VRPN。速度列为地面站内部估计；
离线分析也可以直接对位置列差分。

## 调参用途

该日志先记录真实 RC 飞行作为位置环调参基线。建议分析时至少绘制：

- 位置与目标阶跃；
- 三轴速度、加速度和饱和；
- roll/pitch/yaw-rate/throttle 指令；
- 原始 RC 通道与 bumpless 基线；
- PID 参数变更时间点；
- 位姿龄期、丢帧和 FC failsafe。

日志仅用于观测和离线分析，不进入飞控控制闭环。
