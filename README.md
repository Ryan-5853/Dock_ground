# Dock Ground Station

面向局域网飞行的低延迟地面站。地面站从 VRPN 动捕服务器取得位姿，在 C++
后端运行三轴增量 PID 位置闭环，经 ESP32-C5 + Dronebridge 透明链路向飞控发送
`roll / pitch / yaw-rate / throttle` 四通道指令；WebUI 负责三维示意、状态监控、
位置目标输入和在线调参。

> 当前状态：C++ 后端已接入 `192.168.31.221:3883` 的真实 VRPN 位姿，100 Hz
> 位置环、loopback、GS-Link 四通道输出、同源 HTTP/WebSocket 和 Three.js WebUI
> 已联通。WebUI 可按 X/Y/Z 分别调整 Kp/Ki/Kd，并热更新运行中的增量 PID。
> 飞控侧已按 `GS_LINK_PROTOCOL_V1_zh.md` 接入 GS-Link v1：UDP 双向收发、
> 心跳/遥测新鲜度、会话、ACK 重试和显式上行接口均已实现，WebUI 显示飞控姿态、
> 电池、双电机 RPM、RC 通道和链路统计。飞控每次 ARM 都会自动开始独立飞行
> 日志，不要求地面站控制或 deadman 激活。
>
> 当前飞控通信-only 固件仍不会执行起飞/降落；地面站上行接口也不会绕过 RC
> 通道 8、Commander、failsafe 或执行器授权。

## 快速启动

依赖：GCC 11+、CMake 3.22+、Node.js 18+。HTTP/WebSocket 使用 C++20 标准库和
POSIX socket；VRPN 使用官方客户端库，由 CMake 首次配置时拉取固定提交，或者在
离线环境通过 `-DDOCK_GROUND_VRPN_SOURCE_DIR=/path/to/vrpn` 指向已有源码。
官方 VRPN 使用 Boost Software License 1.0。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
cd frontend
npm ci
npm run build
cd ..
./build/dock_ground_station --config config/ground.json
```

浏览器打开 `http://127.0.0.1:8080`。点击“使能控制”或按空格键切换下行控制后，
前端以 20 Hz 发送操作意图，C++ 后端以 100 Hz 运行三轴增量 PID，并以最高
50 Hz 发送 `DATA_RC` 四通道；再次点击、再次按空格或窗口失焦会释放 deadman。
若要从另一台局域网设备访问 WebUI，启动时显式改为
`--http-bind 0.0.0.0`，并先用主机防火墙限制可访问来源。

开发模式可以分别运行后端和 Vite：

```bash
./build/dock_ground_station --config config/ground.json
```

```bash
cd frontend
npm run dev
```

浏览器打开 `http://127.0.0.1:5173`，Vite 将 `/api` 和 `/ws` 代理到 8080。

测试：

```bash
bash scripts/check.sh
```

### 启用 GS-Link v1

复制模板并确认 C5 地址。模板按 C5 固定监听 `14550`、电脑本地回传端口
`39050` 配置：

```bash
cp config/ground.gs-link.example.json config/ground.gs-link.json
./build/dock_ground_station --config config/ground.gs-link.json
```

`remote_port: 14550` 对应 C5 的 `APP_PORT_PROXY_UDP`，不能改成回传端口。
`local_port: 39050` 是电脑接收回传的端口。地面站从启动开始复用同一个 UDP
socket：启动后第一包发送有效 `CMD_HEARTBEAT`，C5 自动学习该 socket 的实际源
IP/源端口，后续 UART 数据回传到这个端口。链路中没有单独的 HELLO 握手，因此
第一包已经是正常 GS-Link 业务帧。

如果不用自动学习，也可以主动登记。把 IP 换成电脑在 C5 网段的实际地址：

```bash
curl -sS -X POST http://192.168.31.100/api/settings/clients/udp \
  -H 'Content-Type: application/json' \
  -d '{"udp_client_ip":"192.168.31.120","udp_client_port":39050,"save":true}'
```

`save:true` 会持久保存，最多保留一组回传目标。该 HTTP 请求只设置 C5 回传目标，
不会改变 C5 的 `14550` 监听端口。

RC 切换基线使用飞控 `DATA_TELEMETRY.rc_channels` 的原始接收机序号。配置必须与
PX4 `RC_MAP_ROLL/PITCH/THROTTLE/YAW` 一致；默认 AETR 为
`roll=CH1, pitch=CH2, throttle=CH3, yaw=CH4`。进入地面站模式时，地面站先发送
飞控遥测中最新 RC 四通道的实时透传值。飞控确认 `GS_MODE_ACTIVE` 后，地面站
重新采样最新 RC，并检查垂速和油门是否位于安全接管窗口。只有 `|vz|` 和当前
上桨油门相对悬停值的偏差都满足限制后，才至少保持 `bumpless_hold_ms`
（默认 40 ms，约两个 50 Hz 包）并启动位置环。R/P/Y 从该基线叠加增量 PID；
Z PID 从捕获的上桨油门起步，并叠加 `z_velocity_damping` 垂速制动。高度通道按
`throttle_transition_ms`（默认 300 ms 走完满量程）限制变化速率，从切换油门
过渡到绝对上桨油门。基线越接近目标完成越快，不会再固定等待完整 300 ms。

GS-Link v1 的 UDP 数据通路是：

```text
DATA_TELEMETRY / CMD_HEARTBEAT
FC UART ── ESP32 定帧 ── UDP remote_port ──> dock_ground local_port

DATA_RC / CMD_PING / CMD_TAKEOFF / CMD_LAND / CMD_ABORT
dock_ground local_port ── UDP ──> ESP32 remote_port ── FC UART
```

即电脑 `39050 → C5:14550`；C5 自动学习或主动登记后，回传
`C5 → 电脑:39050`，双向始终使用同一个本地 UDP socket。

后端在 FC 遥测和心跳均新鲜时才允许 `DATA_RC` 和指令上行。遥测超过 250 ms 或
心跳超过 300 ms 后，上行立即停止；恢复后需要重新收到新心跳/遥测。这里的门控
只是地面站侧保护，飞控侧 `GroundStationGate` 和物理 RC 仍是最终授权来源。
GS-Link CRC 与 WebSocket 均不是认证；开放 LAN 访问前必须限制可信来源，协议级
HMAC/重放保护尚未实现。

查看 VRPN 当前实际发布的 sensor 和原始位姿：

```bash
./build/dock_ground_vrpn_probe MCAvatar@192.168.31.221:3883 5
```

## 已确定的技术路线

- **实时控制只在 C++ 后端。** 100 Hz 外环读取最新 VRPN 位姿，计算结果直接
  编码并发送。浏览器刷新、React 调度、WebSocket 抖动和人工操作都不进入实时环。
- **位置闭环采用三轴独立增量 PID。** X/Y 输出世界系加速度，再转换为
  heading-relative roll/pitch 态度；Z 输出上桨集体油门。飞控里的最新辨识
  19 状态复合 LQI 负责 `[lower_motor, servo_1, servo_2, servo_3]` 和全部高频
  安全层，上桨油门仍由地面站/飞手侧高度环持有。
- **RC → 地面站采用 bumpless handoff。** 仅在物理 RC 已解锁、有效、通道 8
  已选定且全部映射通道有效时发送实时 RC 透传；飞控确认 GS 模式后才重新采样
  最新 RC 并检查垂速/油门窗口；稳定后保持 40 ms，再让 Z 从中性捕获值起步、
  叠加垂速阻尼，并按实际差值自适应限速到绝对高度环输出。
  UI 会用红色高对比横幅显示当前实际发送的 R/P/Y/T 数值和高度油门混合比例。
- **ARM 自动日志。** FC telemetry 出现 `ARMED` 上升沿后自动创建 CSV；RC 模式、
  地面站模式和 deadman 释放阶段连续记录，disarm 后写 summary 并封存。写盘在
  独立线程中完成。
- **控制链路以 UDP 为主。** ESP32-C5 地址固定为 `192.168.31.100`，目标 UDP
  监听口固定为 `14550`，电脑本地端口使用 `39050`。GS-Link v1 一包占一个 UDP
  datagram；`DATA_RC` 不重传，指令按 20 ms/最多 4 次发送重试并校验 ACK。
- **飞控线协议固定为 GS-Link v1。** 帧头、CRC-16/CCITT-FALSE、小端定宽字段、
  遥测状态位、会话和上行指令均按协议实现。位置/速度不在 v1 基础遥测中，前端
  不会把缺失值当作零。
- **VRPN 直连后端。** 已连接 `MCAvatar@192.168.31.221:3883`，使用官方
  `libvrpn` 客户端；独立接收线程只保留最新有效样本，浏览器不直接连接 VRPN。
- **刚体与飞机实例绑定。** 默认 `body1 → sensor 1 → aircraft-01`。WebUI
  “管理绑定”菜单会发现 `body0/body1` 等实际 sensor，并支持不重连 VRPN 的热切换。
  切换会释放 deadman、重建局部原点和首帧目标。绑定当前保存在进程内，重启后
  回到配置文件中的默认 `body1`。
- **首帧安全对齐。** VRPN 第一帧到达后，位置目标和偏航自动对准当前实测位姿；
  在操作者明确点击使能 deadman 前不会输出控制流。
- **时间戳默认不参与龄期。** 本次联调发现服务器与本机时钟约差 368 ms、而网络
  ping 仅约 2–3 ms；在 NTP/PTP 同步确认前，龄期只计算本机接收后的时间。
- **WebUI 使用 Vite + TypeScript + React + Three.js。** 实时数据通过同源
  WebSocket 下发，3D 场景使用插值显示；控制意图通过持久 WebSocket 上行，禁止
  每帧一次 HTTP 请求。
- **统一坐标和单位。** 地面站、飞机实例和 VRPN 适配层统一使用右手局部 XYZ：
  `+X=前、+Y=左、+Z=上`，飞机机头固定为 `+X`，偏航绕 `+Z`。输出到
  `DATA_RC` 时转换为 PX4 约定：roll 右滚为正、pitch 低头为正、yaw rate
  右偏航为正、throttle 上桨 `[-1,1]`。
- **正式部署采用 Linux 原生进程。** 单个 C++ 服务同时提供静态 WebUI、HTTP 和
  WebSocket。Docker、反向代理和浏览器不放在飞行控制数据路径上。

## 数据通路

```text
VRPN 动捕服务器 ──TCP 建链 + UDP 数据──> C++ VRPN 接收线程
                              │ 最新位姿（覆盖式，不排队）
                              v
WebUI 操作意图 ──WS──> ── 100 Hz X/Y/Z 增量 PID ──> roll / pitch / yaw-rate / throttle
                                                     │ DATA_RC @ ≤50 Hz
                                                     v
                                      GS-Link UDP / Dronebridge
                                                     │
                                     192.168.31.100:<配置端口>
                                                     │ Wi-Fi
                                                     v
                                           ESP32-C5 透明传输
                                                     │ UART
                                                     v
                         辨识 LQR：lower motor + servo 1/2/3 + 安全层
```

## 时延设计门

以下是首轮硬件实测要验证的工程目标，不是仅凭架构即可承诺的指标：

| 指标 | 目标 p95 | 目标 p99 | 说明 |
|---|---:|---:|---|
| VRPN 到达后端 → 姿态 UDP 发出 | 3 ms | 5 ms | 不包含网络传输 |
| 地面站 → ESP32 UDP 单程 | 10 ms | 25 ms | 专用局域网、固定信道 |
| ESP32 透明转发 + UART | 5 ms | 10 ms | 需实测固件与串口配置 |
| 飞控解析并应用姿态指令 | 5 ms | 10 ms | 飞控需回显序号/时间戳 |
| VRPN 产生到飞控执行的端到端 | 40 ms | 80 ms | 最终以飞控回显实测为准 |
| 100 Hz 控制周期抖动 | — | < 2 ms | 控制线程统计 |

超过 30 ms 的位姿在 UI 中标黄，超过 50 ms 的位姿不参与新的控制输出。链路超时、
VRPN 丢失和后端崩溃都必须由飞控自身 failsafe 兜底，不能只依赖地面站。

## 文档

- [系统架构与决策](docs/01-system-architecture.md)
- [时延预算与控制方案](docs/02-latency-and-control.md)
- [外部接口与协议契约](docs/03-interface-contracts.md)
- [实施路线与验收门](docs/04-roadmap-and-acceptance.md)
- [GS-Link v1 接入说明](docs/05-gs-link-v1.md)
- [位置环调参与四通道映射](docs/06-position-loop-tuning.md)
- [ARM 自动飞行日志](docs/07-flight-logging.md)

## 项目目录

```text
include/dock_ground/      公共接口、数学、控制、网络与运行时
src/                      控制循环、VRPN/仿真位姿、JSON、HTTP/WS
tests/                    数学、控制、配置、WebSocket 协议测试
frontend/                 Vite + TypeScript + React + Three.js
config/ground.json        阶段 1 运行配置
docs/                     技术文档
CMakeLists.txt
```

当前已实现 GS-Link v1 的 ESP32 UDP 端点、飞控遥测显示、三轴增量 PID 在线调参
和四通道 `DATA_RC` 输出；尚未完成实际 ESP32/AP/飞控台架联调、端到端时延报告
和真实闭环参数整定。默认配置仍是 `LOOPBACK ONLY`，不能把开发机结果当作可飞
状态。

## 开始实现前仍需确认

1. 真机 `DATA_RC` 四通道的最终量程、上桨实际悬停油门和闭环启用授权。
2. Dronebridge 工作模式、ESP32 的 UDP 端口、地面站本地端口、串口波特率和分帧方式。
3. 飞控在指令超时、动捕丢失和地面站退出时各自的 failsafe 动作。
4. 毫米到米和地面 Z 基准的最终确认；当前已明确目标刚体为 `body1`，VRPN 原始
   X/Y/Z 与地面站 X/Y/Z 一一对应。
5. VRPN 局部 Z 地面基准、现场起飞高度和位置环安全包线。

上述信息阻塞真实闭环调参和飞行授权；VRPN、语义 loopback、WebUI 和 GS-Link
台架通信已可独立验证。

## 安全边界

- 地面站不是飞控安全层的替代品。独立遥控器、飞控失控保护、地理围栏和动力
  切断必须始终有效。
- 启动后不自动使能控制流；必须有明确操作、持续 deadman 和链路健康检查。
- 禁止在未完成 UDP 链路、飞控回显、断链保护和地面台架测试前进行飞行。
- 延迟结论必须来自实际地面站主机、实际 AP 和实际 ESP32 环境，不能用开发机、
  虚拟机或 WSL 的结果代替。
