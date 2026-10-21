# 外部接口与协议契约

飞控线协议已冻结为 GS-Link v1，具体字节布局以
`/home/ryan/RL_Flight/px4_trans/GS_LINK_PROTOCOL_V1_zh.md` 和
[GS-Link v1 接入说明](05-gs-link-v1.md) 为准。下文保留地面站语义边界和历史
契约说明；若 GS-Link 无法满足这里的时序、坐标、失效和可观测性要求，应作为
架构变更评审，而不是在适配器里静默绕过。

## 1. 系统边界

```text
VRPN tracker source ──> IPoseSource ──> latest pose ──> PositionLoop
Operator WebUI ───────> target intent ────────────────┘
PositionLoop ──> ManualControlSetpoint ──> GS-Link DATA_RC
                                           │
                                           v
                                      flight transport
                                           │ UDP
                                           v
                              ESP32-C5 / Dronebridge / FC
```

核心接口不依赖具体飞控协议：

```cpp
struct PoseSample {
    std::uint64_t received_mono_ns;
    std::uint64_t source_time_ns;   // 仅在时间基准明确时可比较
    std::uint32_t sequence;
    Vec3 position_xyz_m;
    Quaternion q_body_to_world;
    bool source_time_valid;
    bool orientation_valid;
};

struct ManualControlSetpoint {
    double roll;       // [-1,1], right roll positive
    double pitch;      // [-1,1], nose down positive
    double yaw;        // [-1,1], right yaw rate positive
    double throttle;   // [-1,1], upper rotor [0,1] after mapping
    bool valid;
};

class IPoseSource {
public:
    virtual ~IPoseSource() = default;
    virtual std::optional<PoseSample> latest() const = 0;
};

class ICommandSink {
public:
    virtual ~ICommandSink() = default;
    virtual bool send(const ManualControlSetpoint&) = 0;
};
```

实现约束：

- `latest()` 必须是无阻塞读取最新样本。
- `send()` 不得内部排队、重传或同步写日志。
- codec 不得在控制周期内分配堆内存。
- transport、codec 与控制器分别可替换、分别测试。

## 2. Dronebridge / ESP32-C5 链路

已知条件：

- ESP32-C5 地址：`192.168.31.100`。
- 固件：Dronebridge。
- 工作方式：透明传输。
- C5 监听：`APP_PORT_PROXY_UDP=14550`。
- GCS socket：`39050 → C5:14550`，同一 socket 接收 39050 回传。
- 飞控接收协议：GS-Link v1。

联调前必须确认并记录：

| 项目 | 要求/待确认值 |
|---|---|
| 链路模式 | UDP，datagram 边界保留 |
| ESP32 UDP 端口 | `14550` |
| 地面站本地端口 | `39050`，同一 socket 双向 |
| UART 波特率/校验/流控 | `4,000,000`、`8N1`、无硬件流控 |
| 透明转发方向 | 双向，遥测/心跳/GC 四通道 |
| 最大报文/MTU | GS-Link 最大 524 B |

硬性要求：

- 真实控制模式未配置目标端口时启动失败并给出明确错误，不能静默使用可能错误的端口。
- 控制报文包含版本、类型、序号和时间信息。
- 接收端必须检查长度、CRC、版本和序号；失败不传递给飞控。
- `DATA_RC` 不做重传；只允许最新值。
- 地面站和飞控都必须有独立的超时保护。
- 透明链路是否低延迟由实际回显测量证明，不能只看 Wi-Fi 信号强度。

## 3. 飞控线协议契约

字节布局以 GS-Link v1 为准。

### 3.1 上行四通道手动指令

建议语义字段：

| 字段 | 语义 |
|---|---|
| `roll` | `[-1,1]`，右滚为正，映射 `±20°` |
| `pitch` | `[-1,1]`，低头为正，映射 `±20°` |
| `yaw` | `[-1,1]`，PX4/FRD 右偏航角速度为正，映射 `±1.2 rad/s` |
| `throttle` | `[-1,1]`，映射上桨 `[0,1]` |

飞控端 `LqiManualReference` 将 roll/pitch 解释为 heading-relative Euler 态度，
yaw 解释为 rate。辨识 LQR 只生成 `[lower_motor, servo_1, servo_2,
servo_3]`；上桨由 throttle 外部持有。

### 3.2 下行状态/回显

最小状态报文：

| 字段 | 用途 |
|---|---|
| `sequence_echo` | 证明具体哪个姿态指令被飞控接收 |
| `fc_time` | 计算接收/执行延迟，需说明时间基准 |
| `armed` / `mode` | 防止 UI 状态与飞控不一致 |
| `q_actual` | 姿态跟踪状态 |
| `thrust_actual` / 饱和位 | 判断输出是否受限 |
| `battery` / `fault_bits` | 必要安全状态 |
| `rx_counter` / `crc_errors` | 评估透明链路质量 |

姿态指令周期可以高，但状态回传可降频；回显至少应覆盖每个控制帧或按固定窗口
汇总。若只回传 1 Hz 遥测，无法验证 100 Hz 指令的真实延迟和丢失。

### 3.3 管理与控制模式

解锁、模式切换、起飞、降落、hold 和紧急处理不能塞进高频姿态帧中靠重复发送
实现。它们使用独立的管理消息，必须：

- 有请求 ID 和明确 ACK/NACK。
- 有超时和幂等语义。
- 有操作者确认和服务器状态门禁。
- 受飞控当前模式、链路和 failsafe 状态约束。

## 4. VRPN 输入契约

配置文件需要提供：

```json
{
  "vrpn": {
    "endpoint": "MCAvatar@192.168.31.221:3883",
    "sensor_id": 1,
    "position_scale": 0.001,
    "auto_origin": true,
    "axes_from_vrpn": [
      [1.0, 0.0, 0.0],
      [0.0, 1.0, 0.0],
      [0.0, 0.0, 1.0]
    ]
  }
}
```

当前现场已完成以下确认：

- 服务器：`192.168.31.221:3883`。
- tracker sender：`MCAvatar`。
- 当前目标刚体为 `body1`，对应 sensor `1`。WebUI 绑定菜单展示后端发现的所有
  sensor；`bodyN` 与 `sensor N` 一一对应。配置仍支持 `sensor_id: null`
  自动选择第一路，但当前默认配置固定为 `1`。
- Pos_Quat 消息约 110–125 Hz；位置数值按毫米解释，当前配置
  `position_scale: 0.001`。
- 首样本只对齐 X/Y 原点，Z 保留映射后的高度。现场分别沿 X/Y/Z 单轴移动，
  已确认 VRPN 原始 X/Y/Z 与地面站 X/Y/Z 一一对应，映射使用单位矩阵；机头零
  偏航对齐 `+X`。
- 联调观察到 VRPN 消息时间戳比本机约早 368 ms，而网络 ping 约 2–3 ms，不能
  把该差值当作真实网络时延。默认 `trust_source_timestamp: false`，UI 龄期从
  本机收到消息开始计算；只有 NTP/PTP 同步验证后才能开启。

`expected_rate_hz`、`max_pose_age_ms` 和完整标定矩阵仍是后续字段。当前实际配置
使用 `endpoint`、`sensor_id`、`position_scale`、`auto_origin`、
`trust_source_timestamp` 和 `axes_from_vrpn`。设计还必须处理：

- VRPN 时间戳是否可用、是否与地面站同一时钟域。
- 服务器帧率、丢帧和 tracking valid 标志。
- 动捕坐标系、单位、手性、原点及刚体安装偏置。
- 位姿重复发送和乱序；只接受最新有效样本。
- 服务器断线重连，不允许重放断线期间的历史轨迹。

## 5. WebUI / 后端接口

接口版本前缀为 `/api/v1` 和 `/ws/v1`。生产环境同源，无 CORS。

### 5.1 HTTP

| 方法 | 路径 | 用途 |
|---|---|---|
| `GET` | `/api/v1/health` | 进程存活、版本和启动时间 |
| `GET` | `/api/v1/status` | 安全状态、链路、配置摘要，不返回密钥 |
| `GET` | `/api/v1/events` | 最近事件和故障，分页/限长 |
| `GET` | `/` | 静态 WebUI |

### 5.2 实时 WebSocket

`/ws/v1/realtime` 为双向通道：

- 服务端以 30 Hz 推送最新状态快照；慢客户端丢旧快照，不反压控制。
- 客户端以持久连接发送操作意图，关键消息带客户端序号。
- 服务器为每条意图打上自己的单调接收时间，不信任浏览器时间。

操作意图示例（语义草案，不是飞控线协议）：

```json
{
  "v": 1,
  "type": "operator_intent",
  "client_seq": 42,
  "mode": "position_track",
  "target_xyz_m": [1.2, -0.4, 1.5],
  "target_yaw_rad": 0.2,
  "velocity_xyz_mps": [0.0, 0.0, 0.0],
  "deadman": true
}
```

刚体与飞机实例绑定消息：

```json
{
  "v": 1,
  "type": "bind_body",
  "aircraft_id": "aircraft-01",
  "body_id": "body1",
  "sensor_id": 1
}
```

成功后返回 `binding_ack`。后端切换 sensor 时释放 deadman、清空旧刚体位姿，
以新刚体首帧重建局部原点和目标。切换期间不允许保持控制流。

服务端必须：

- 限幅位置、速度和偏航，拒绝非有限数值。
- 检查状态机、链路和操作者权限。
- 对速度/摇杆意图实行 deadman 和失联超时。
- 永远不因 UI 报文而绕过飞控安全状态。

状态快照包含：位姿及序号、位姿龄期、目标、控制输出、控制周期、链路 RTT/丢包、
当前模式、失效原因、飞机实例列表、活动绑定和后端发现到的刚体列表。JSON 仅用于
UI 侧，不进入控制热路径。

### 5.3 3D 显示数据量

- 实时位姿和目标是小型定长数据，30 Hz 足够。
- 轨迹由后端限长/抽样后下发，禁止每帧发送完整历史。
- 前端使用插值平滑显示，不把插值后的位姿反馈给后端。
- 连接断开时画面明确标红并冻结，不继续显示“实时”假象。

## 6. 配置草案

当前阶段使用无注释 JSON 配置，避免为了配置解析额外引入第三方依赖；后续配置格式
变化时应保持下列字段语义不变。

```json
{
  "aircraft": {
    "id": "aircraft-01",
    "name": "飞行器 01"
  },
  "server": {
    "http_bind": "0.0.0.0",
    "http_port": 8080,
    "web_root": "frontend/dist"
  },
  "control": {
    "rate_hz": 100,
    "warning_pose_age_ms": 30,
    "max_pose_age_ms": 50,
    "deadman_timeout_ms": 250
  },
  "position_loop": {
    "x": {"kp": 0.60, "ki": 0.01, "kd": 0.30},
    "y": {"kp": 0.60, "ki": 0.01, "kd": 0.30},
    "z": {"kp": 0.06, "ki": 0.01, "kd": 0.08},
    "max_accel_mps2": 4.0,
    "max_tilt_rad": 0.3490658504,
    "min_throttle": 0.20,
    "hover_throttle": 0.56,
    "max_throttle": 0.96,
    "z_velocity_damping": 0.08,
    "max_yaw_rate_rad_s": 1.2,
    "yaw_rate_kp": 2.0,
    "max_position_error_m": 10.0,
    "gravity_mps2": 9.80665
  },
  "pose": {
    "source": "vrpn"
  },
  "initial_target": {
    "position_xyz_m": [0.0, 0.0, 0.0],
    "yaw_rad": 0.0,
    "use_current_pose": true
  },
  "flight": {
    "transport": "loopback",
    "protocol": null,
    "remote_ip": "192.168.31.100",
    "remote_port": null,
    "local_ip": "0.0.0.0",
    "local_port": 39050,
    "rc_roll_channel": 1,
    "rc_pitch_channel": 2,
    "rc_throttle_channel": 3,
    "rc_yaw_channel": 4,
    "bumpless_hold_ms": 40,
    "throttle_transition_ms": 300,
    "handoff_max_vertical_speed_mps": 0.20,
    "handoff_max_throttle_delta": 0.10
  },
  "flight_log": {
    "enabled": true,
    "directory": "logs/flights",
    "telemetry_stale_timeout_ms": 2000
  },
  "vrpn": {
    "endpoint": "MCAvatar@192.168.31.221:3883",
    "sensor_id": 1,
    "position_scale": 0.001,
    "auto_origin": true,
    "trust_source_timestamp": false,
    "axes_from_vrpn": [
      [1.0, 0.0, 0.0],
      [0.0, 1.0, 0.0],
      [0.0, 0.0, 1.0]
    ]
  }
}
```

启动门禁：

- `transport: udp` 但端口为空 → 拒绝启动真实控制模式。
- `protocol: null` → 只能运行 VRPN/仿真与回环，UI 显示实际位姿源和
  `LOOPBACK ONLY`。
- 只读诊断模式可以在没有飞控协议时运行，用于验证 VRPN、3D 和网络。

## 7. 协议冻结清单

真实飞控接入前，双方必须共同确认：

1. 字节序、magic、版本、长度、CRC、分帧和最大帧长。
2. 四元数、轴系、单位、正方向和推力范围。
3. 频率、序号回绕、重复帧和过期帧处理。
4. 飞控 ACK/状态回传与时间基准。
5. 飞控超时、地面站超时、人工接管和 failsafe 的优先级。
6. Dronebridge 模式、端口、UDP 边界、串口参数和双向通路。
7. 台架回显测试通过，才允许去掉螺旋桨后的第一次闭环测试。
