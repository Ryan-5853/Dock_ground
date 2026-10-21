import {
  useCallback,
  useEffect,
  useRef,
  useState,
} from "react";
import { GroundScene } from "./scene";
import type {
  AxisPidGains,
  BindingAck,
  ControlState,
  OperatorIntent,
  PositionLoopTuning,
  PositionTuningAck,
  PositionTuningError,
  StateMessage,
  TargetDraft,
} from "./types";

type ConnectionState = "CONNECTING" | "ONLINE" | "OFFLINE";
type TuningScalarKey = Exclude<
  keyof PositionLoopTuning,
  "x" | "y" | "z"
>;

const controlStateLabel: Record<ControlState, string> = {
  STANDBY: "待命",
  TRACK: "位置跟踪",
  DEGRADED: "数据降级",
  FAILSAFE: "失效保护",
};

function ageClass(ageMs: number | null): string {
  if (ageMs === null || !Number.isFinite(ageMs)) {
    return "metric critical";
  }
  if (ageMs <= 30) {
    return "metric healthy";
  }
  if (ageMs <= 50) {
    return "metric warning";
  }
  return "metric critical";
}

function format(value: number | null | undefined, digits = 2): string {
  return value === null || value === undefined || !Number.isFinite(value)
    ? "—"
    : value.toFixed(digits);
}

function clamp(value: number, minimum: number, maximum: number): number {
  return Math.min(maximum, Math.max(minimum, value));
}

function copyPositionTuning(
  tuning: PositionLoopTuning,
): PositionLoopTuning {
  return {
    x: { ...tuning.x },
    y: { ...tuning.y },
    z: { ...tuning.z },
    max_accel_mps2: tuning.max_accel_mps2,
    max_tilt_rad: tuning.max_tilt_rad,
    min_throttle: tuning.min_throttle,
    hover_throttle: tuning.hover_throttle,
    max_throttle: tuning.max_throttle,
    z_velocity_damping: tuning.z_velocity_damping,
    max_yaw_rate_rad_s: tuning.max_yaw_rate_rad_s,
    yaw_rate_kp: tuning.yaw_rate_kp,
    max_position_error_m: tuning.max_position_error_m,
    gravity_mps2: tuning.gravity_mps2,
  };
}

function hasStatusFlag(flags: number, bit: number): boolean {
  return (flags & (1 << bit)) !== 0;
}

function SceneView({ state }: { state: StateMessage | null }) {
  const containerRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<GroundScene | null>(null);

  useEffect(() => {
    if (!containerRef.current) {
      return;
    }
    const scene = new GroundScene(containerRef.current);
    sceneRef.current = scene;
    return () => {
      scene.dispose();
      sceneRef.current = null;
    };
  }, []);

  useEffect(() => {
    if (state) {
      sceneRef.current?.update(state);
    }
  }, [state]);

  return (
    <div className="scene-host" ref={containerRef}>
      <div className="axis-legend">
        <span className="axis x">X</span>
        <span className="axis y">Y</span>
        <span className="axis z">Z</span>
        <span>X 前 · Y 左 · Z 上</span>
        <span>拖拽旋转 · 滚轮缩放</span>
      </div>
    </div>
  );
}

export default function App() {
  const [connection, setConnection] =
    useState<ConnectionState>("CONNECTING");
  const [state, setState] = useState<StateMessage | null>(null);
  const [lastStateAt, setLastStateAt] = useState(0);
  const [deadman, setDeadman] = useState(false);
  const [clientSequence, setClientSequence] = useState(0);
  const [bindingOpen, setBindingOpen] = useState(false);
  const [bindingAircraft, setBindingAircraft] = useState("");
  const [bindingSensor, setBindingSensor] = useState<number | null>(null);
  const [bindingSubmitting, setBindingSubmitting] = useState(false);
  const [bindingError, setBindingError] = useState<string | null>(null);
  const [bindingNotice, setBindingNotice] = useState<string | null>(null);
  const [tuningDraft, setTuningDraft] =
    useState<PositionLoopTuning | null>(null);
  const [tuningSubmitting, setTuningSubmitting] = useState(false);
  const [tuningError, setTuningError] = useState<string | null>(null);
  const [tuningNotice, setTuningNotice] = useState<string | null>(null);
  const [draft, setDraft] = useState<TargetDraft>({
    x: 0,
    y: 0,
    z: 0,
    yawDegrees: 0,
  });

  const socketRef = useRef<WebSocket | null>(null);
  const deadmanRef = useRef(false);
  const draftRef = useRef(draft);
  const sequenceRef = useRef(0);
  const initializedTargetRef = useRef(false);
  const initializedTuningRef = useRef(false);

  useEffect(() => {
    draftRef.current = draft;
  }, [draft]);

  useEffect(() => {
    let disposed = false;
    let reconnectTimer = 0;

    const connect = () => {
      if (disposed) {
        return;
      }
      setConnection("CONNECTING");
      const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
      const socket = new WebSocket(
        `${protocol}//${window.location.host}/ws/v1/realtime`,
      );
      socketRef.current = socket;

      socket.onopen = () => {
        if (!disposed) {
          setConnection("ONLINE");
        }
      };
      socket.onmessage = (event) => {
        try {
          const message = JSON.parse(String(event.data)) as
            | StateMessage
            | BindingAck
            | PositionTuningAck
            | PositionTuningError
            | { type: "error"; message: string };
          if (message.type === "state") {
            setState(message);
            setLastStateAt(Date.now());
            if (!initializedTargetRef.current) {
              initializedTargetRef.current = true;
              const [x, y, z] =
                message.target.position_xyz_m;
              setDraft({
                x,
                y,
                z,
                yawDegrees: (message.target.yaw_rad * 180) / Math.PI,
              });
            }
            if (!initializedTuningRef.current) {
              initializedTuningRef.current = true;
              setTuningDraft(
                copyPositionTuning(message.position_tuning),
              );
            }
          } else if (message.type === "binding_ack") {
            setBindingSubmitting(false);
            setBindingError(null);
            setBindingNotice(
              `已绑定 ${message.body_id} → ${message.aircraft_id}`,
            );
          } else if (message.type === "position_tuning_ack") {
            setTuningSubmitting(false);
            setTuningError(null);
            setTuningDraft(copyPositionTuning(message.tuning));
            setTuningNotice("位置环参数已热更新");
          } else if (message.type === "position_tuning_error") {
            setTuningSubmitting(false);
            setTuningNotice(null);
            setTuningError(message.message);
          } else if (message.type === "error") {
            setBindingSubmitting(false);
            setBindingError(message.message);
          }
        } catch (error) {
          console.warn("invalid backend message", error);
        }
      };
      socket.onerror = () => {
        if (!disposed) {
          setConnection("OFFLINE");
        }
      };
      socket.onclose = () => {
        socketRef.current = null;
        deadmanRef.current = false;
        initializedTuningRef.current = false;
        setDeadman(false);
        setBindingSubmitting(false);
        setTuningSubmitting(false);
        if (!disposed) {
          setConnection("OFFLINE");
          reconnectTimer = window.setTimeout(connect, 1000);
        }
      };
    };

    connect();
    return () => {
      disposed = true;
      window.clearTimeout(reconnectTimer);
      socketRef.current?.close();
      socketRef.current = null;
      deadmanRef.current = false;
    };
  }, []);

  const sendIntent = useCallback((active: boolean) => {
    const socket = socketRef.current;
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      return;
    }
    const target = draftRef.current;
    const sequence = ++sequenceRef.current;
    setClientSequence(sequence);
    const intent: OperatorIntent = {
      v: 1,
      type: "operator_intent",
      client_seq: sequence,
      target_xyz_m: [
        clamp(target.x, -25, 25),
        clamp(target.y, -25, 25),
        clamp(target.z, -25, 25),
      ],
      target_yaw_rad: (clamp(target.yawDegrees, -180, 180) * Math.PI) / 180,
      deadman: active,
    };
    socket.send(JSON.stringify(intent));
  }, []);

  useEffect(() => {
    const timer = window.setInterval(() => {
      if (deadmanRef.current) {
        sendIntent(true);
      }
    }, 50);
    return () => window.clearInterval(timer);
  }, [sendIntent]);

  const setControlActive = useCallback(
    (active: boolean) => {
      if (active && connection !== "ONLINE") {
        return;
      }
      deadmanRef.current = active;
      setDeadman(active);
      sendIntent(active);
    },
    [connection, sendIntent],
  );

  useEffect(() => {
    const keyDown = (event: KeyboardEvent) => {
      if (
        event.code === "Space" &&
        !(
          event.target instanceof HTMLInputElement ||
          event.target instanceof HTMLTextAreaElement ||
          event.target instanceof HTMLSelectElement ||
          event.target instanceof HTMLButtonElement ||
          (event.target instanceof HTMLElement &&
            event.target.isContentEditable)
        )
      ) {
        event.preventDefault();
        if (!event.repeat) {
          setControlActive(!deadmanRef.current);
        }
      }
    };
    const blur = () => setControlActive(false);
    window.addEventListener("keydown", keyDown);
    window.addEventListener("blur", blur);
    return () => {
      window.removeEventListener("keydown", keyDown);
      window.removeEventListener("blur", blur);
    };
  }, [setControlActive]);

  const nudge = useCallback(
    (x: number, y: number, z: number) => {
      setDraft((current) => ({
        ...current,
        x: clamp(current.x + x, -25, 25),
        y: clamp(current.y + y, -25, 25),
        z: clamp(current.z + z, -25, 25),
      }));
    },
    [],
  );

  const updateTuningAxis = useCallback(
    (axis: "x" | "y" | "z", field: keyof AxisPidGains, value: number) => {
      setTuningDraft((current) =>
        current
          ? {
              ...current,
              [axis]: {
                ...current[axis],
                [field]: Number.isFinite(value) ? value : 0,
              },
            }
          : current,
      );
      setTuningError(null);
      setTuningNotice(null);
    },
    [],
  );

  const updateTuningScalar = useCallback(
    (field: TuningScalarKey, value: number) => {
      setTuningDraft((current) =>
        current
          ? {
              ...current,
              [field]: Number.isFinite(value) ? value : 0,
            }
          : current,
      );
      setTuningError(null);
      setTuningNotice(null);
    },
    [],
  );

  const applyPositionTuning = useCallback(() => {
    const socket = socketRef.current;
    if (
      !socket ||
      socket.readyState !== WebSocket.OPEN ||
      !tuningDraft
    ) {
      setTuningError("链路未就绪或尚未读取位置环参数");
      return;
    }
    setTuningSubmitting(true);
    setTuningError(null);
    setTuningNotice(null);
    socket.send(
      JSON.stringify({
        v: 1,
        type: "position_tuning",
        tuning: tuningDraft,
      }),
    );
  }, [tuningDraft]);

  const restorePositionTuning = useCallback(() => {
    if (!state) {
      return;
    }
    setTuningDraft(copyPositionTuning(state.position_tuning));
    setTuningError(null);
    setTuningNotice("已恢复为控制器当前参数");
  }, [state]);

  const resetToMeasured = () => {
    if (!state) {
      return;
    }
    const [x, y, z] = state.pose.position_xyz_m;
    setDraft((current) => ({
      ...current,
      x,
      y,
      z,
    }));
  };

  const openBindingMenu = () => {
    setBindingError(null);
    setBindingNotice(null);
    setBindingSubmitting(false);
    setBindingAircraft(
      state?.binding.aircraft_id ?? state?.aircrafts[0]?.id ?? "",
    );
    const boundSensor = state?.binding.sensor_id ?? -1;
    setBindingSensor(
      boundSensor >= 0
        ? boundSensor
        : (state?.rigid_bodies[0]?.sensor_id ?? null),
    );
    setBindingOpen(true);
  };

  const submitBinding = () => {
    const socket = socketRef.current;
    if (
      !socket ||
      socket.readyState !== WebSocket.OPEN ||
      bindingSensor === null ||
      !bindingAircraft
    ) {
      setBindingError("链路未就绪或尚未选择刚体/飞机实例");
      return;
    }

    setControlActive(false);
    setBindingSubmitting(true);
    setBindingError(null);
    setBindingNotice(null);
    socket.send(
      JSON.stringify({
        v: 1,
        type: "bind_body",
        aircraft_id: bindingAircraft,
        body_id: `body${bindingSensor}`,
        sensor_id: bindingSensor,
      }),
    );
  };

  const stale =
    Date.now() - lastStateAt > 1000 && connection === "ONLINE";
  const isVrpn = state?.mode === "vrpn";
  const vrpnConnected = state?.pose_source.connected ?? false;
  const vrpnLinked = state?.pose_source.transport_connected ?? false;
  const gs = state?.gs_link;
  const gsTelemetry = gs?.telemetry;
  const gsHasTelemetry = gsTelemetry?.valid ?? false;
  const gsFresh =
    (gs?.telemetry_fresh ?? false) && (gs?.heartbeat_fresh ?? false);
  const gsLabel = !gs?.enabled
    ? "GS-LINK OFF"
    : gsFresh
      ? "GS-LINK LIVE"
      : gs.heartbeat_fresh
        ? "GS-LINK RX"
        : "GS-LINK WAIT";
  const groundStationActive =
    state?.control_output.ground_station_active ?? false;
  const groundStationCommanding =
    state?.control_output.ground_station_commanding ?? false;
  const handoffHold = state?.control_output.handoff_hold ?? false;
  const rcPassthrough =
    state?.control_output.rc_passthrough ?? false;
  const throttleBlend =
    state?.control_output.handoff_throttle_blend ?? 0;
  const throttleTransitioning = !handoffHold && throttleBlend < 0.999;
  const controlState = state?.control_state ?? "STANDBY";
  const poseAge = state?.pose.age_ms ?? null;
  const stateTone =
    controlState === "TRACK"
      ? "track"
      : controlState === "STANDBY"
        ? "standby"
        : "fault";

  return (
    <main className="app-shell">
      <header className="topbar">
        <div>
          <div className="eyebrow">DOCK · LAN FLIGHT CONTROL</div>
          <h1>地面站</h1>
        </div>
        <div className="topbar-status">
          <span className={`connection ${connection.toLowerCase()}`}>
            <span className="status-dot" />
            {connection === "ONLINE"
              ? stale
                ? "数据超时"
                : "实时链路"
              : connection === "CONNECTING"
                ? "正在连接"
                : "连接断开"}
          </span>
          <span className={`control-state ${stateTone}`}>
            {controlStateLabel[controlState]}
          </span>
          <span
            className={`simulation-badge ${
              isVrpn && vrpnConnected ? "live" : ""
            }`}
          >
            {isVrpn
              ? vrpnConnected
                ? "VRPN MOCAP"
                : vrpnLinked
                  ? "VRPN NO DATA"
                  : "VRPN OFFLINE"
              : "SIMULATION"}
          </span>
          <span
            className={`simulation-badge gs-link-badge ${
              gsFresh ? "live" : gs?.enabled ? "waiting" : ""
            }`}
          >
            {gsLabel}
          </span>
          <span
            className={`simulation-badge flight-log-badge ${
              state?.flight_log.recording
                ? "recording"
                : state?.flight_log.enabled
                  ? "ready"
                  : ""
            }`}
          >
            {state?.flight_log.recording
              ? "REC · ARM"
              : state?.flight_log.enabled
                ? "LOG READY"
                : "LOG OFF"}
          </span>
        </div>
      </header>

      <section className="workspace">
        {(groundStationActive || groundStationCommanding) && (
          <aside
            className={`control-live-banner ${
              groundStationActive &&
              !handoffHold &&
              !throttleTransitioning
                ? "active"
                : "handoff"
            }`}
            aria-live="assertive"
          >
            <div>
              <span className="control-live-kicker">
                {groundStationActive && !groundStationCommanding
                  ? "FC STILL IN GS MODE · COMMAND STREAM STOPPED"
                  : groundStationActive && rcPassthrough
                  ? "GS SAFE HOLD · RC PASSTHROUGH"
                  : !groundStationActive
                  ? rcPassthrough
                    ? "RC PASSTHROUGH · WAITING FOR FC"
                    : "GS COMMANDS SENT · WAITING FOR FC"
                  : handoffHold
                    ? "GROUND HANDOFF · HOLDING RC"
                    : throttleTransitioning
                      ? "GROUND HANDOFF · THROTTLE TRANSITION"
                    : "GROUND STATION CONTROL ACTIVE"}
              </span>
              <strong>
                {groundStationActive && !groundStationCommanding
                  ? "飞控仍处于 GS 模式并保持最后指令，直到超时回 RC；立即准备物理遥控器接管"
                  : groundStationActive && rcPassthrough
                  ? "飞控已进入 GS，但垂速或油门尚未进入安全窗口；地面站继续透传 RC，不启动位置环"
                  : !groundStationActive
                  ? rcPassthrough
                    ? "飞控仍在 RC 控制；地面站仅实时透传当前 RC，确认后重新采样并建立交接基线"
                    : "飞控尚未确认 GS MODE，不能认为已接管；立即准备物理遥控器接手中止"
                  : handoffHold
                    ? "地面站接管中：四通道保持切换瞬间 RC 信号"
                    : throttleTransitioning
                      ? "高度油门正在从 RC 基线过渡到绝对位置控制"
                    : "飞控已确认 GS MODE，地面站正在发送四通道控制"}
              </strong>
            </div>
            <div className="control-live-channels">
              <span>
                R{" "}
                <b>{format(state?.command.manual.roll, 3)}</b>
              </span>
              <span>
                P{" "}
                <b>{format(state?.command.manual.pitch, 3)}</b>
              </span>
              <span>
                Y{" "}
                <b>{format(state?.command.manual.yaw, 3)}</b>
              </span>
              <span>
                T{" "}
                <b>{format(state?.command.manual.throttle, 3)}</b>
              </span>
            </div>
            <div className="control-live-base">
              RC base R{" "}
              {format(state?.control_output.rc_base.roll, 3)} · P{" "}
              {format(state?.control_output.rc_base.pitch, 3)} · Y{" "}
              {format(state?.control_output.rc_base.yaw, 3)} · T{" "}
              {format(state?.control_output.rc_base.throttle, 3)}
              {" · vz "}
              {format(
                state?.control_output.handoff_vertical_speed_mps,
                2,
              )}{" "}
              m/s
              {throttleTransitioning
                ? ` · mix ${format(throttleBlend * 100, 0)}%`
                : ""}
            </div>
          </aside>
        )}
        <div className="viewport-panel">
          <SceneView state={state} />
          <div className="viewport-overlay top-left">
            <div className="coordinate-title">LOCAL XYZ → THREE</div>
            <div>X {format(state?.pose.position_xyz_m[0])} m</div>
            <div>Y {format(state?.pose.position_xyz_m[1])} m</div>
            <div>Z {format(state?.pose.position_xyz_m[2])} m</div>
          </div>
          <div className="viewport-overlay bottom-left">
            <div className="warning-line">{state?.warning ?? "WAITING FOR BACKEND"}</div>
            <div>
              output: {state?.link.mode ?? "—"} · protocol:{" "}
              {state?.link.protocol ?? "—"}
            </div>
            <div>
              source: {state?.pose_source.endpoint ?? "—"}
            </div>
            {isVrpn && (
              <div>
                origin: XY auto · Z absolute · axes: source XYZ
              </div>
            )}
            {isVrpn && (
              <div>
                age basis:{" "}
                {state?.pose.source_time_measured
                  ? "source clock + local"
                  : "local receive"}
              </div>
            )}
          </div>
        </div>

        <aside className="side-panel">
          <section className="panel-section binding-summary">
            <div className="section-heading">
              <span>刚体与飞机绑定</span>
              <span
                className={`binding-state ${
                  state?.binding.active ? "active" : ""
                }`}
              >
                {state?.binding.active ? "ACTIVE" : "WAITING"}
              </span>
            </div>
            <div className="binding-route">
              <div>
                <span>刚体</span>
                <strong>{state?.binding.body_id ?? "—"}</strong>
              </div>
              <span className="binding-arrow">→</span>
              <div>
                <span>飞机实例</span>
                <strong>{state?.binding.aircraft_id ?? "—"}</strong>
              </div>
            </div>
            <button className="secondary-button" onClick={openBindingMenu}>
              管理绑定
            </button>
          </section>

          <section className="panel-section">
            <div className="section-heading">
              <span>目标指令</span>
              <span className="muted">XYZ / m · 机头 +X</span>
            </div>
            <div className="field-grid">
              <label>
                <span>X · 前</span>
                <input
                  type="number"
                  step="0.1"
                  min="-25"
                  max="25"
                  value={draft.x.toFixed(2)}
                  onChange={(event) =>
                    setDraft((current) => ({
                      ...current,
                      x: Number(event.target.value),
                    }))
                  }
                />
              </label>
              <label>
                <span>Y · 左</span>
                <input
                  type="number"
                  step="0.1"
                  min="-25"
                  max="25"
                  value={draft.y.toFixed(2)}
                  onChange={(event) =>
                    setDraft((current) => ({
                      ...current,
                      y: Number(event.target.value),
                    }))
                  }
                />
              </label>
              <label>
                <span>Z · 上</span>
                <input
                  type="number"
                  step="0.1"
                  min="-25"
                  max="25"
                  value={draft.z.toFixed(2)}
                  onChange={(event) =>
                    setDraft((current) => ({
                      ...current,
                      z: Number(event.target.value),
                    }))
                  }
                />
              </label>
              <label>
                <span>偏航 °（+ 左转）</span>
                <input
                  type="number"
                  step="1"
                  min="-180"
                  max="180"
                  value={draft.yawDegrees.toFixed(0)}
                  onChange={(event) =>
                    setDraft((current) => ({
                      ...current,
                      yawDegrees: Number(event.target.value),
                    }))
                  }
                />
              </label>
            </div>

            <div className="nudge-pad">
              <button onClick={() => nudge(0.25, 0, 0)}>X +0.25</button>
              <button onClick={() => nudge(-0.25, 0, 0)}>X −0.25</button>
              <button onClick={() => nudge(0, 0.25, 0)}>Y +0.25</button>
              <button onClick={() => nudge(0, -0.25, 0)}>Y −0.25</button>
              <button onClick={() => nudge(0, 0, 0.1)}>Z +0.1</button>
              <button onClick={() => nudge(0, 0, -0.1)}>Z −0.1</button>
            </div>
            <button className="secondary-button" onClick={resetToMeasured}>
              目标对准当前位姿
            </button>
          </section>

          <section className="panel-section position-tuning-panel">
            <div className="section-heading">
              <span>位置环调参</span>
              <span className="muted">增量 PID · 100 Hz</span>
            </div>
            <p className="tuning-axis-note">
              世界系 X 前 / Y 左 / Z 上；机头为 +X。FC 输入为
              roll 右滚、pitch 低头、yaw rate 右偏航、上桨 throttle。
            </p>
            {state && (
              <div className="rc-map-strip">
                <span>RC 映射</span>
                <strong>
                  R CH{state.rc_mapping.roll} · P CH
                  {state.rc_mapping.pitch} · T CH
                  {state.rc_mapping.throttle} · Y CH
                  {state.rc_mapping.yaw}
                </strong>
                <span>
                  最短保持 {state.rc_mapping.handoff_hold_ms} ms · 油门满量程限速{" "}
                  {state.rc_mapping.throttle_transition_ms} ms · 接管窗口{" "}
                  {format(
                    state.rc_mapping.handoff_max_vertical_speed_mps,
                    2,
                  )}{" "}
                  m/s
                </span>
              </div>
            )}
            {state?.control_output.handoff_error && (
              <div className="binding-message error">
                {state.control_output.handoff_error}
              </div>
            )}
            {!tuningDraft ? (
              <p className="telemetry-empty">等待后端控制参数…</p>
            ) : (
              <>
                <div className="pid-axis-grid">
                  {(["x", "y", "z"] as const).map((axis) => (
                    <div className="pid-axis-card" key={axis}>
                      <div className="pid-axis-title">
                        <strong>{axis.toUpperCase()} 轴</strong>
                        <span>
                          {axis === "x"
                            ? "前向 / m/s²"
                            : axis === "y"
                              ? "左向 / m/s²"
                              : "升向 / 油门"}
                        </span>
                      </div>
                      {(["kp", "ki", "kd"] as const).map((field) => (
                        <label key={field}>
                          <span>{field.toUpperCase()}</span>
                          <input
                            type="number"
                            step={field === "ki" ? "0.01" : "0.05"}
                            value={tuningDraft[axis][field]}
                            onChange={(event) =>
                              updateTuningAxis(
                                axis,
                                field,
                                Number(event.target.value),
                              )
                            }
                          />
                        </label>
                      ))}
                    </div>
                  ))}
                </div>

                <div className="tuning-limits-grid">
                  <label>
                    <span>最大水平加速度 m/s²</span>
                    <input
                      type="number"
                      step="0.1"
                      min="0.1"
                      value={tuningDraft.max_accel_mps2}
                      onChange={(event) =>
                        updateTuningScalar(
                          "max_accel_mps2",
                          Number(event.target.value),
                        )
                      }
                    />
                  </label>
                  <label>
                    <span>最大倾角 °</span>
                    <input
                      type="number"
                      step="1"
                      min="1"
                      max="80"
                      value={(tuningDraft.max_tilt_rad * 180) / Math.PI}
                      onChange={(event) =>
                        updateTuningScalar(
                          "max_tilt_rad",
                          (Number(event.target.value) * Math.PI) / 180,
                        )
                      }
                    />
                  </label>
                  <label>
                    <span>偏航率增益 1/s</span>
                    <input
                      type="number"
                      step="0.1"
                      min="0"
                      value={tuningDraft.yaw_rate_kp}
                      onChange={(event) =>
                        updateTuningScalar(
                          "yaw_rate_kp",
                          Number(event.target.value),
                        )
                      }
                    />
                  </label>
                  <label>
                    <span>最大偏航率 °/s</span>
                    <input
                      type="number"
                      step="5"
                      min="1"
                      value={
                        (tuningDraft.max_yaw_rate_rad_s * 180) / Math.PI
                      }
                      onChange={(event) =>
                        updateTuningScalar(
                          "max_yaw_rate_rad_s",
                          (Number(event.target.value) * Math.PI) / 180,
                        )
                      }
                    />
                  </label>
                  <label>
                    <span>最小上桨油门</span>
                    <input
                      type="number"
                      step="0.01"
                      min="0"
                      max="1"
                      value={tuningDraft.min_throttle}
                      onChange={(event) =>
                        updateTuningScalar(
                          "min_throttle",
                          Number(event.target.value),
                        )
                      }
                    />
                  </label>
                  <label>
                    <span>悬停上桨油门</span>
                    <input
                      type="number"
                      step="0.01"
                      min="0"
                      max="1"
                      value={tuningDraft.hover_throttle}
                      onChange={(event) =>
                        updateTuningScalar(
                          "hover_throttle",
                          Number(event.target.value),
                        )
                      }
                    />
                  </label>
                  <label>
                    <span>最大上桨油门</span>
                    <input
                      type="number"
                      step="0.01"
                      min="0"
                      max="1"
                      value={tuningDraft.max_throttle}
                      onChange={(event) =>
                        updateTuningScalar(
                          "max_throttle",
                          Number(event.target.value),
                        )
                      }
                    />
                  </label>
                  <label>
                    <span>垂速阻尼 / 油门每 m/s</span>
                    <input
                      type="number"
                      step="0.01"
                      min="0"
                      max="0.5"
                      value={tuningDraft.z_velocity_damping}
                      onChange={(event) =>
                        updateTuningScalar(
                          "z_velocity_damping",
                          Number(event.target.value),
                        )
                      }
                    />
                  </label>
                </div>

                <div className="tuning-actions">
                  <button
                    className="secondary-button"
                    onClick={restorePositionTuning}
                    disabled={tuningSubmitting}
                  >
                    恢复当前参数
                  </button>
                  <button
                    className="primary-button"
                    onClick={applyPositionTuning}
                    disabled={
                      tuningSubmitting || connection !== "ONLINE"
                    }
                  >
                    {tuningSubmitting ? "正在应用…" : "应用增量 PID"}
                  </button>
                </div>
                {tuningError && (
                  <div className="binding-message error">{tuningError}</div>
                )}
                {tuningNotice && (
                  <div className="binding-message success">
                    {tuningNotice}
                  </div>
                )}

                <div className="pid-output-grid">
                  <div>
                    <span>X 输出</span>
                    <strong>
                      {format(
                        state?.command.position_loop_output_xyz[0],
                        3,
                      )}{" "}
                      m/s²
                    </strong>
                  </div>
                  <div>
                    <span>Y 输出</span>
                    <strong>
                      {format(
                        state?.command.position_loop_output_xyz[1],
                        3,
                      )}{" "}
                      m/s²
                    </strong>
                  </div>
                  <div>
                    <span>Z 增量油门</span>
                    <strong>
                      {format(
                        handoffHold
                          ? 0
                          : (state?.command
                                .position_loop_output_xyz[2] ?? 0) -
                              (state?.position_tuning
                                .hover_throttle ?? 0),
                        3,
                      )}{" "}
                      Δthr
                    </strong>
                  </div>
                  <div>
                    <span>RC 四通道 R / P / Y / T</span>
                    <strong>
                      {format(state?.command.manual.roll, 2)} /{" "}
                      {format(state?.command.manual.pitch, 2)} /{" "}
                      {format(state?.command.manual.yaw, 2)} /{" "}
                      {format(state?.command.manual.throttle, 2)}
                    </strong>
                  </div>
                </div>
              </>
            )}
          </section>

          <section className="panel-section deadman-section">
            <div className="section-heading">
              <span>GS 下行控制</span>
              <span className="muted">click toggle</span>
            </div>
            <button
              className={`deadman-button ${deadman ? "active" : ""}`}
              type="button"
              role="switch"
              aria-checked={deadman}
              onClick={() => setControlActive(!deadmanRef.current)}
              disabled={connection !== "ONLINE"}
            >
              <strong>{deadman ? "下行控制已使能" : "点击使能下行控制"}</strong>
              <span>再次点击或按空格释放；窗口失焦时自动释放</span>
            </button>
            <div className="deadman-meta">
              <span>client seq {clientSequence}</span>
              <span>{deadman ? "HEARTBEAT 20 Hz" : "NO COMMAND"}</span>
            </div>
          </section>

          <section className="panel-section flight-log-panel">
            <div className="section-heading">
              <span>飞行日志</span>
              <span
                className={`flight-log-state ${
                  state?.flight_log.recording ? "recording" : ""
                }`}
              >
                {state?.flight_log.recording
                  ? "RECORDING"
                  : state?.flight_log.enabled
                    ? "ARM AUTO-START"
                    : "DISABLED"}
              </span>
            </div>
            <p className="log-note">
              FC 从 disarm 进入 armed 时自动创建 CSV；RC 模式、地面站模式和
              deadman 释放期间均持续记录，disarm 后封存。
            </p>
            <div className="log-file-path">
              {state?.flight_log.current_file || "等待首次 ARM…"}
            </div>
            {state?.flight_log.last_error && (
              <div className="binding-message error">
                {state.flight_log.last_error}
              </div>
            )}
            <div className="log-metrics">
              <span>帧 {state?.flight_log.frames_written ?? 0}</span>
              <span>会话 {state?.flight_log.sessions_written ?? 0}</span>
              <span>丢弃 {state?.flight_log.frames_dropped ?? 0}</span>
            </div>
          </section>

          <section className="panel-section gs-telemetry-panel">
            <div className="section-heading">
              <span>飞控遥测 · GS-Link v1</span>
              <span
                className={`gs-link-state ${
                  gsFresh ? "live" : gs?.enabled ? "waiting" : "off"
                }`}
              >
                {!gs?.enabled
                  ? "OFF"
                  : gs.telemetry_fresh
                    ? gsFresh
                      ? "FRESH"
                      : "TX BLOCKED"
                    : "STALE"}
              </span>
            </div>
            {!gs?.enabled ? (
              <p className="telemetry-empty">
                当前配置为 loopback。启用 UDP + GS-Link v1 后，这里显示飞控
                姿态、电池、电机和 RC 通道。
              </p>
            ) : (
              <>
                <div className="telemetry-attitude">
                  <div>
                    <span>Roll</span>
                    <strong>
                      {gsHasTelemetry
                        ? format(
                            (gsTelemetry?.roll_cdeg ?? 0) / 100,
                            2,
                          )
                        : "—"}
                      °
                    </strong>
                  </div>
                  <div>
                    <span>Pitch</span>
                    <strong>
                      {gsHasTelemetry
                        ? format(
                            (gsTelemetry?.pitch_cdeg ?? 0) / 100,
                            2,
                          )
                        : "—"}
                      °
                    </strong>
                  </div>
                  <div>
                    <span>Yaw</span>
                    <strong>
                      {gsHasTelemetry
                        ? format(
                            (gsTelemetry?.yaw_cdeg ?? 0) / 100,
                            2,
                          )
                        : "—"}
                      °
                    </strong>
                  </div>
                </div>
                <div className="telemetry-state-row">
                  <span
                    className={
                      hasStatusFlag(gsTelemetry?.status_flags ?? 0, 0)
                        ? "telemetry-tag active"
                        : "telemetry-tag"
                    }
                  >
                    ARMED
                  </span>
                  <span
                    className={
                      hasStatusFlag(gsTelemetry?.status_flags ?? 0, 1)
                        ? "telemetry-tag active"
                        : "telemetry-tag"
                    }
                  >
                    GS MODE
                  </span>
                  <span
                    className={
                      hasStatusFlag(gsTelemetry?.status_flags ?? 0, 4)
                        ? "telemetry-tag fault"
                        : "telemetry-tag"
                    }
                  >
                    FAILSAFE
                  </span>
                  <span
                    className={
                      hasStatusFlag(gsTelemetry?.status_flags ?? 0, 8)
                        ? "telemetry-tag warning"
                        : "telemetry-tag"
                    }
                  >
                    BATT WARN
                  </span>
                </div>
                <div className="telemetry-data-grid">
                  <div>
                    <span>电压</span>
                    <strong>
                      {gsHasTelemetry &&
                      gsTelemetry?.battery_voltage_mv
                        ? format(
                            gsTelemetry.battery_voltage_mv / 1000,
                            2,
                          )
                        : "—"}{" "}
                      V
                    </strong>
                  </div>
                  <div>
                    <span>电流</span>
                    <strong>
                      {!gsHasTelemetry ||
                      gsTelemetry?.battery_current_ca === -32768
                        ? "—"
                        : format(
                            (gsTelemetry?.battery_current_ca ?? 0) / 100,
                            2,
                          )}{" "}
                      A
                    </strong>
                  </div>
                  <div>
                    <span>电量</span>
                    <strong>
                      {!gsHasTelemetry ||
                      gsTelemetry?.battery_remaining === 255
                        ? "—"
                        : `${gsTelemetry?.battery_remaining ?? 0}%`}
                    </strong>
                  </div>
                  <div>
                    <span>电机 RPM</span>
                    <strong>
                      {!gsHasTelemetry ||
                      gsTelemetry?.motor1_rpm === 65535
                        ? "—"
                        : gsTelemetry?.motor1_rpm}
                      {" / "}
                      {!gsHasTelemetry ||
                      gsTelemetry?.motor2_rpm === 65535
                        ? "—"
                        : gsTelemetry?.motor2_rpm}
                    </strong>
                  </div>
                </div>
                <div className="telemetry-rc-heading">
                  <span>RC channels</span>
                  <span>
                    {gsTelemetry?.rc_channel_count ?? 0} ch · age{" "}
                    {format(gs.telemetry_age_ms, 1)} ms
                  </span>
                </div>
                <div className="telemetry-rc-channels">
                  {(gsTelemetry?.rc_channels ?? []).map((value, index) => (
                    <div key={index}>
                      <span>CH{index + 1}</span>
                      <strong>{value === -128 ? "—" : value}</strong>
                    </div>
                  ))}
                  {(gsTelemetry?.rc_channel_count ?? 0) === 0 && (
                    <span className="muted">FC 未发送 RC 通道</span>
                  )}
                </div>
                <div className="telemetry-link-row">
                  <span>
                    {gs.transport} ·{" "}
                    {gs.heartbeat.valid ? gs.state : "NO FC"}
                  </span>
                  <span>
                    session{" "}
                    {gs.session_id
                      ? gs.session_id.toString(16).toUpperCase()
                      : "—"}
                  </span>
                  <span>
                    RX {gs.counters.rx_packets} / TX{" "}
                    {gs.counters.tx_packets}
                  </span>
                  <span>
                    CRC {gs.counters.crc_errors} · ack{" "}
                    {gs.last_ack.valid ? gs.last_ack.result : "—"}
                  </span>
                </div>
              </>
            )}
          </section>

          <section className="panel-section metrics-grid">
            <div className={ageClass(poseAge)}>
              <span>位姿龄期</span>
              <strong>{format(poseAge, 2)} ms</strong>
            </div>
            <div className="metric">
              <span>控制周期</span>
              <strong>{format(state?.metrics.last_period_ms, 2)} ms</strong>
            </div>
            <div className="metric">
              <span>计算耗时 p99</span>
              <strong>{format(state?.metrics.p99_work_ms, 3)} ms</strong>
            </div>
            <div className="metric">
              <span>周期抖动 p99</span>
              <strong>{format(state?.metrics.p99_jitter_ms, 2)} ms</strong>
            </div>
            <div className="metric">
              <span>DATA_RC 发送</span>
              <strong>{state?.link.commands_sent ?? 0}</strong>
            </div>
            <div className="metric">
              <span>上桨油门</span>
              <strong>{format(state?.command.upper_throttle, 3)}</strong>
            </div>
          </section>

          <section className="panel-section compact-state">
            <div>
              <span>位姿序号</span>
              <strong>{state?.pose.sequence ?? "—"}</strong>
            </div>
            <div>
              <span>指令序号</span>
              <strong>{state?.command.sequence ?? "—"}</strong>
            </div>
            <div>
              <span>VRPN sensor</span>
              <strong>{state?.pose_source.sensor_id ?? "—"}</strong>
            </div>
            <div>
              <span>动捕消息</span>
              <strong>{state?.pose_source.messages_received ?? 0}</strong>
            </div>
            <div>
              <span>发送错误</span>
              <strong>{state?.link.send_errors ?? 0}</strong>
            </div>
            <div>
              <span>控制超时</span>
              <strong>{state?.metrics.overruns ?? 0}</strong>
            </div>
          </section>
        </aside>
      </section>

      {bindingOpen && (
        <div
          className="binding-overlay"
          role="presentation"
          onMouseDown={() => setBindingOpen(false)}
        >
          <section
            className="binding-dialog"
            role="dialog"
            aria-modal="true"
            aria-labelledby="binding-title"
            onMouseDown={(event) => event.stopPropagation()}
          >
            <header className="binding-dialog-header">
              <div>
                <div className="eyebrow">RIGID BODY ROUTING</div>
                <h2 id="binding-title">绑定刚体与飞机实例</h2>
              </div>
              <button
                className="icon-button"
                aria-label="关闭"
                onClick={() => setBindingOpen(false)}
              >
                ×
              </button>
            </header>

            <div className="binding-fields">
              <label>
                <span>飞机实例</span>
                <select
                  value={bindingAircraft}
                  onChange={(event) =>
                    setBindingAircraft(event.target.value)
                  }
                >
                  {(state?.aircrafts ?? []).map((aircraft) => (
                    <option key={aircraft.id} value={aircraft.id}>
                      {aircraft.name} · {aircraft.id}
                    </option>
                  ))}
                </select>
              </label>

              <label>
                <span>目标刚体</span>
                <select
                  value={bindingSensor ?? ""}
                  onChange={(event) =>
                    setBindingSensor(Number(event.target.value))
                  }
                >
                  {bindingSensor !== null &&
                    !(state?.rigid_bodies ?? []).some(
                      (body) => body.sensor_id === bindingSensor,
                    ) && (
                      <option value={bindingSensor}>
                        body{bindingSensor} · sensor {bindingSensor} · waiting
                        for VRPN
                      </option>
                    )}
                  {(state?.rigid_bodies ?? []).map((body) => (
                    <option key={body.body_id} value={body.sensor_id}>
                      {body.body_id} · sensor {body.sensor_id} ·{" "}
                      {body.messages_received} msgs
                    </option>
                  ))}
                </select>
              </label>
            </div>

            <div className="binding-current">
              <div>
                <span>当前绑定</span>
                <strong>
                  {state?.binding.body_id ?? "—"} →{" "}
                  {state?.binding.aircraft_id ?? "—"}
                </strong>
              </div>
              <div>
                <span>当前实时位姿</span>
                <strong>
                  {state?.pose_source.connected ? "VRPN LIVE" : "NO POSE"}
                </strong>
              </div>
            </div>

            <p className="binding-help">
              <code>body1</code> 对应 VRPN sensor 1。切换后将以新刚体首帧重新
              建立局部原点，位置目标和偏航随之对齐；控制流必须先释放。
            </p>

            {bindingError && (
              <div className="binding-message error">{bindingError}</div>
            )}
            {bindingNotice && (
              <div className="binding-message success">{bindingNotice}</div>
            )}

            <footer className="binding-actions">
              <button
                className="secondary-button"
                onClick={() => setBindingOpen(false)}
              >
                取消
              </button>
              <button
                className="primary-button"
                disabled={
                  bindingSubmitting ||
                  bindingSensor === null ||
                  !bindingAircraft
                }
                onClick={submitBinding}
              >
                {bindingSubmitting ? "正在切换…" : "绑定并设为当前"}
              </button>
            </footer>
          </section>
        </div>
      )}
    </main>
  );
}
