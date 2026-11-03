# ARM 自动飞行日志

## 触发规则

日志由飞控遥测中的 `ARMED` 状态触发，不依赖地面站 deadman：

- FC 遥测新鲜、`telemetry.valid=true`；
- `status_flags.ARMED` 从 0 变为 1；
- 若服务启动时已经 armed，第一帧 armed 遥测也会开始记录。

`ARMED` 上升沿前默认保留 `pre_roll_s=5` 秒的内存环形样本。创建会话后先把这段
预录写入 CSV，因此能看到解锁前的 VRPN 状态、控制线程抖动和 handoff 准备过程。
预录只占用有界内存，不影响 100 Hz 控制线程。

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
    "telemetry_stale_timeout_ms": 2000,
    "pre_roll_s": 5.0
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
- JSON：会话开始时间、机体、VRPN endpoint/sensor/坐标变换、控制时序门限、
  RC 映射和初始 PID 参数。
- summary JSON：结束原因、持续时间、帧数、VRPN 丢包/间隔、handoff 重置、
  failsafe、地面站控制线程异常和日志队列丢帧统计。

当前文件 schema 为 `2`。追加的新字段位于 CSV 末尾，已有列的名称和顺序保持不变。

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
- 当前位置调度模式、水平/垂直误差、速度、增益过渡状态和实际生效的三轴
  Kp/Ki/Kd；
- FC roll/pitch/yaw、双电机 RPM、电压、电流、电量；
- 18 路原始 RC 通道；
- X/Y/Z 当前 Kp/Ki/Kd 及限幅/油门参数；
- GS-Link RX/TX、CRC、序号缺口、重试和 ACK 统计。
- VRPN 原始坐标与原始四元数、接收单调时间、源时间戳、源时钟差、样本到达间隔；
- VRPN callback/accepted/rejected 累计计数，以及未选中 sensor、位置/姿态无效、
  重复或倒退时间戳、长间隔、连接和断开次数；
- 控制环是否认为位姿新鲜、RC 交接门限是否满足、handoff 被清除的原因；
- 控制周期、实际周期、执行耗时、抖动、p99 和 overrun 计数。

FC 基础遥测没有位置/速度，所以位置和姿态来自 VRPN。速度列为地面站内部估计；
离线分析也可以直接对位置列差分。

## VRPN 诊断字段

`position_*_m` 和 `pose_q_*` 是经过轴映射、比例缩放和局部原点处理后的控制
输入。`pose_raw_*` 是 VRPN callback 交付的原始位置，单位由 metadata 中的
`position_scale` 决定。两者必须同时保留，才能区分源端跳变、坐标变换错误和控制
侧异常。

关键时间字段口径：

- `pose_received_mono_ns`：VRPN callback 在本机收到的 `steady_clock` 时间；
- `pose_source_time_ns`：VRPN 消息携带的源时间戳，即使当前不可信也保存；
- `pose_source_clock_delta_ms`：本机 wall clock 与源时间戳之差，不是单向延迟；
- `pose_local_age_ms`：控制周期距本机收到样本的时间；
- `pose_source_age_ms`：仅 `trust_source_timestamp=true` 时进入总龄期的源时间龄期；
- `pose_arrival_interval_ms`：同一 sensor 相邻有效 callback 的接收间隔；
- `vrpn_last_arrival_gap_ms` / `vrpn_max_arrival_gap_ms`：VRPN 接收层当前/累计最大
  到达间隔。

排查控制异常时，应先比较 `pose_sequence`、`vrpn_callbacks_received` 和
`vrpn_samples_accepted` 的帧间增量：

- callback 增量接近零：源端或网络确实没有数据；
- callback 增量明显大于 accepted：回调被 sensor 选择或数据有效性过滤；
- 短时间内 callback/accepted 大幅增加且 `pose_arrival_interval_ms < 1 ms`：
  是接收线程追帧或突发到达，不是稳定高频输出；
- `handoff_reset=1` 时，结合 `handoff_reset_reason`、`pose_fresh`、
  `rc_gate_ready` 和 `handoff_error` 判断控制回退的触发条件。

## 动捕稳定性测试

测试地面站到动捕服务器的实际 callback 流，不启动飞控通信：

```bash
python3 scripts/test_vrpn_stability.py --duration 300
```

脚本从 `config/ground.gs-link.json` 读取 endpoint、sensor 和期望帧率。静置测试
期间保持目标刚体不动、不要遮挡，并固定网络和供电状态。结果写入
`logs/vrpn-stability/run_<UTC>_<label>/`：

- `vrpn_samples.csv`：每个 callback 的原始时间、位置、四元数和间隔；
- `probe.log`：探针实时输出；
- `ping.log`：尽力而为的 ICMP 丢包/时延记录；ICMP 与 VRPN 优先级可能不同，
  仅作为辅助证据；
- `summary.json` / `summary.txt`：帧率、长间隔、追帧突发、时间戳异常、位姿跳变
  和明确 PASS/WARN/FAIL 结论。

验证主动运动或遮挡时加 `--motion`，避免把预期运动当作静置指标失败：

```bash
python3 scripts/test_vrpn_stability.py \
  --duration 120 --motion --label occlusion
```

已有 CSV 可以离线重跑，不会访问网络：

```bash
python3 scripts/test_vrpn_stability.py \
  --analyze-only logs/vrpn-stability/run_xxx/vrpn_samples.csv
```

## VRPN 网络层联合测试

长 callback 间隔需要区分“UDP 包根本没到主机”和“包已到主机但 VRPN callback
被批量处理”。使用联合测试同时采集网卡 UDP、VRPN callback 和连续 ping：

```bash
sudo apt install tcpdump
python3 scripts/test_vrpn_wire.py \
  --duration 180 \
  --label wire-baseline
```

脚本自动从配置取得 endpoint 和 sensor，通过 `ip route get` 找到出口网卡，并在
sudo 鉴权后启动 `tcpdump`。测试过程中不要移动、遮挡刚体或改变网络拓扑。
输出位于 `logs/vrpn-wire/run_<UTC>_<label>/`：

- `capture.pcap`：原始 UDP 抓包；
- `packets.txt`：tcpdump 解码文本；
- `stability/vrpn_samples.csv`：逐 callback 数据；
- `ping-concurrent.log`：与探针同时运行的分组 ping；
- `wire_summary.json` / `wire_summary.txt`：callback 与 UDP 到达间隔对照。

判定含义：

- `LOCAL_CALLBACK_BATCHING`：callback 空档期间 UDP 包仍正常到达，重点检查地面站
  VRPN 接收线程、调度和 `mainloop()` 停顿；
- `SOURCE_OR_WIRE_DELAY`：callback 空档期间也没有 UDP 包，重点检查 MCAvatar
  发送端、交换机、网卡和链路；
- `MIXED_OR_INCONCLUSIVE`：两种现象都存在，或抓包流无法稳定对应目标 sensor。

如果 Linux 已给 `tcpdump` 配置了 `cap_net_raw,cap_net_admin` capability，可加
`--no-sudo` 避免 sudo。联合测试默认使用 50 ms 作为 callback 长间隔失败阈值，
与地面站 `max_pose_age_ms` 一致。

## 调参用途

该日志先记录真实 RC 飞行作为位置环调参基线。建议分析时至少绘制：

- 位置与目标阶跃；
- 三轴速度、加速度和饱和；
- roll/pitch/yaw-rate/throttle 指令；
- 原始 RC 通道与 bumpless 基线；
- PID 参数变更时间点；
- 位姿龄期、丢帧和 FC failsafe。

日志仅用于观测和离线分析，不进入飞控控制闭环。
