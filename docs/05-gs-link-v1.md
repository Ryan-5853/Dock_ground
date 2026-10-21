# GS-Link v1 接入说明

本实现以 `/home/ryan/RL_Flight/px4_trans/GS_LINK_PROTOCOL_V1_zh.md` 为准。
`dock_ground` 是 GCS 端点，通过 UDP 与 ESP32 透明桥通信；地面站不直接打开
飞控 `/dev/ttyS0`。

## 链路与安全门

- 一个 UDP datagram 只承载一个完整 GS-Link 包，不合并控制帧。
- 包头、长度、版本、包类别和 CRC-16/CCITT-FALSE 均在接收时检查。
- 发送方向维护独立 16 位序号；数据帧不重传，ACK 指令复用序号并设置 `RETRY`。
- GCS 以本地接收时间判断新鲜度，不比较 FC 的 `timestamp_ms` 与 PC 时钟。
- 只有 FC 遥测和心跳同时新鲜时，`DATA_RC` 与指令才允许上行。
- 起飞/降落/abort 使用 FC 心跳中的当前 `session_id`；执行结果以遥测和 ACK 为准。
- 地面站不修改 RC 通道 8，也不提供 arm；飞控的 `GroundStationGate` 和物理 RC
  始终是最终安全门。
- GS-Link CRC 和 WebSocket 都不是认证机制。`http_bind` 默认只监听
  `127.0.0.1`；开放到 LAN 前必须由防火墙限制可信来源，协议级 HMAC/重放保护
  尚未实现。

## 配置

```json
{
  "flight": {
    "transport": "udp",
    "protocol": "gs_link_v1",
    "local_ip": "0.0.0.0",
    "local_port": 39050,
    "remote_ip": "192.168.31.100",
    "remote_port": 14550,
    "telemetry_timeout_ms": 250,
    "heartbeat_timeout_ms": 300,
    "heartbeat_rate_hz": 10.0,
    "rc_roll_channel": 1,
    "rc_pitch_channel": 2,
    "rc_throttle_channel": 3,
    "rc_yaw_channel": 4,
    "bumpless_hold_ms": 40,
    "throttle_transition_ms": 300,
    "handoff_max_vertical_speed_mps": 0.20,
    "handoff_max_throttle_delta": 0.10
  }
}
```

飞控确认前地面站只实时透传遥测中的最新 RC 四通道，不锁存或启动位置环。
确认 `GS_MODE_ACTIVE` 后重新采样最新 RC，并等待垂速和油门进入安全接管窗口。
稳定后至少保持 `bumpless_hold_ms`（默认 40 ms，约两个 50 Hz 包）；之后 R/P/Y
从该基线叠加位置环增量，高度 Z PID 从捕获油门起步并使用垂速阻尼，再按
`throttle_transition_ms` 对应的满量程速率自适应过渡到绝对高度环输出，避免高
RC 油门基线吞掉向下制动权限，同时不在差值很小时制造固定延迟。

`remote_port: 14550` 是 C5 `APP_PORT_PROXY_UDP` 固定监听端口；
`local_port: 39050` 是电脑接收 UART 回传的端口。`transport=udp` 时缺少任一
端口、协议名不匹配或地址非法会拒绝启动。

### C5 自动学习

`dock_ground` 创建一个长期复用的 UDP socket，并 bind 到
`0.0.0.0:39050`。连接 C5 后，后台线程立即从该 socket 发送一个有效
`CMD_HEARTBEAT` 到 `C5_IP:14550`。C5 根据收到的实际源 IP/源端口记录回传目标，
后续把 UART 业务帧回传到同一个电脑 socket。

没有单独截获的 HELLO 包，第一包心跳本身就是有效业务数据。不要用另一个临时
socket 发送“唤醒包”，否则 C5 会学习到错误的源端口。

### C5 主动登记

也可直接登记电脑回传地址；IP 必须替换成电脑在 C5 网段的实际地址：

```bash
curl -sS -X POST http://192.168.31.100/api/settings/clients/udp \
  -H 'Content-Type: application/json' \
  -d '{"udp_client_ip":"192.168.31.120","udp_client_port":39050,"save":true}'
```

`udp_client_port` 只改变回传目标，不改变 C5 的 `14550` 监听端口。
`save:true` 表示重启后保留，最多持久保存一组目标。主动登记和自动学习可二选一；
无论使用哪种方式，`dock_ground` 都必须继续从同一个 `39050` socket 接收。

## 下行遥测

`DATA_TELEMETRY` 和 `CMD_HEARTBEAT` 写入 Runtime 快照的 `gs_link` 对象，
WebUI 每帧刷新：

- Roll/Pitch/Yaw，原始 `0.01°` 字段。
- 双电机机械 RPM，`0xFFFF` 显示为未知。
- 电池电压、电流、剩余百分比，协议哨兵值显示为未知。
- 当前有效 RC 通道数和 `int8` 通道值。
- armed、GS mode、failsafe、电池警告等状态位。
- session、FC 状态、链路龄期、RX/TX、CRC、序号缺口和 ACK 结果。

v1 没有位置、速度、角速度或四元数。界面不会用 0 伪造这些字段。

## WebSocket 上行接口

连接 `/ws/v1/realtime` 后可发送 `gs_link_uplink`。该接口用于之后的摇杆、
位置环或指令 UI；位置环 deadman 已使用 C++ 内部接口自动发送 `DATA_RC`，页面
仍未开放起飞/降落/abort 按钮。

### `DATA_RC`

```json
{
  "v": 1,
  "type": "gs_link_uplink",
  "kind": "rc",
  "roll": 0.0,
  "pitch": 0.0,
  "yaw": 0.0,
  "throttle": -1.0,
  "valid": true,
  "gs_ready": false,
  "link_quality": 100
}
```

四个轴使用 `[-1,1]`，后端按 `value * 10000` 转成协议 `int16`。`throttle=-1`
对应 LQI 空闲/零集体。`valid` 必须由调用方显式给出；链路不新鲜时后端拒绝发送。

### Ping 与指令

```json
{"v":1,"type":"gs_link_uplink","kind":"ping","token":77}
```

```json
{
  "v": 1,
  "type": "gs_link_uplink",
  "kind": "takeoff",
  "target_altitude_mm": 1500,
  "max_climb_cm_s": 100,
  "timeout_ms": 10000,
  "token": 88001
}
```

```json
{
  "v": 1,
  "type": "gs_link_uplink",
  "kind": "land",
  "max_descent_cm_s": 50,
  "timeout_ms": 20000,
  "token": 88002
}
```

```json
{
  "v": 1,
  "type": "gs_link_uplink",
  "kind": "abort",
  "reason_code": 1,
  "token": 88003
}
```

成功进入发送/ACK 管理后返回 `gs_link_accept`，失败返回 `gs_link_error`。
`gs_link_accept` 只表示已发送且进入 ACK 重试流程，不表示飞控已接受或动作已完成。
飞控真实结果看快照中的 `gs_link.last_ack`、`state` 和遥测状态位。

C++ 集成也可直接调用：

```cpp
dock_ground::gs_link::UplinkRequest request;
request.kind = dock_ground::gs_link::UplinkKind::Rc;
request.rc.valid = true;
runtime.apply_gs_uplink(request, error);
```

## 当前未完成

本接入完成的是协议、UDP 链路、遥测可视化和上行边界。实际 ESP32 固件端口、
AP 拓扑、串口分帧、端到端时延、飞控 failsafe 和位置环现场调参仍需台架联调。
通信-only 固件对起飞/降落/abort 返回 `UNSUPPORTED` 时，界面和 `last_ack`
会如实显示。
