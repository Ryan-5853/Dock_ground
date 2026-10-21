export type ControlState =
  | "STANDBY"
  | "TRACK"
  | "DEGRADED"
  | "FAILSAFE";

export interface AxisPidGains {
  kp: number;
  ki: number;
  kd: number;
}

export interface PositionLoopTuning {
  x: AxisPidGains;
  y: AxisPidGains;
  z: AxisPidGains;
  max_accel_mps2: number;
  max_tilt_rad: number;
  min_throttle: number;
  hover_throttle: number;
  max_throttle: number;
  z_velocity_damping: number;
  max_yaw_rate_rad_s: number;
  yaw_rate_kp: number;
  max_position_error_m: number;
  gravity_mps2: number;
}

export interface StateMessage {
  v: 1;
  type: "state";
  mode: string;
  control_state: ControlState;
  generated_mono_ms: number;
  warning: string;
  pose_source: {
    name: string;
    connected: boolean;
    transport_connected: boolean;
    sensor_id: number;
    messages_received: number;
    endpoint: string;
    position_scale: number;
    auto_origin: boolean;
    trust_source_timestamp: boolean;
  };
  position_tuning: PositionLoopTuning & {
    mode: "incremental_pid";
    frame: string;
  };
  rc_mapping: {
    roll: number;
    pitch: number;
    throttle: number;
    yaw: number;
    handoff_hold_ms: number;
    throttle_transition_ms: number;
    handoff_max_vertical_speed_mps: number;
    handoff_max_throttle_delta: number;
  };
  control_output: {
    source: "GROUND_STATION" | "RC" | "LOOPBACK";
    ground_station_active: boolean;
    ground_station_commanding: boolean;
    fc_gs_mode_active: boolean;
    rc_passthrough: boolean;
    handoff_active: boolean;
    handoff_hold: boolean;
    handoff_vertical_speed_mps: number;
    handoff_throttle_blend: number;
    handoff_error: string;
    rc_base: {
      roll: number;
      pitch: number;
      yaw: number;
      throttle: number;
    };
    current: {
      roll: number;
      pitch: number;
      yaw: number;
      throttle: number;
    };
  };
  flight_log: {
    enabled: boolean;
    recording: boolean;
    current_file: string;
    last_error: string;
    sessions_written: number;
    frames_written: number;
    frames_dropped: number;
  };
  gs_link: {
    enabled: boolean;
    transport: string;
    protocol: string;
    transport_connected: boolean;
    telemetry_fresh: boolean;
    heartbeat_fresh: boolean;
    uplink_allowed: boolean;
    command_pending: boolean;
    telemetry_age_ms: number;
    heartbeat_age_ms: number;
    session_id: number;
    state: string;
    error_code: number;
    heartbeat_status_flags: number;
    telemetry: {
      valid: boolean;
      fc_timestamp_ms: number;
      status_flags: number;
      roll_cdeg: number;
      pitch_cdeg: number;
      yaw_cdeg: number;
      motor1_rpm: number;
      motor2_rpm: number;
      battery_voltage_mv: number;
      battery_current_ca: number;
      battery_remaining: number;
      rc_channel_count: number;
      rc_channels: number[];
    };
    heartbeat: {
      valid: boolean;
      uptime_ms: number;
      session_id: number;
      status_flags: number;
      protocol_version: number;
      state: string;
      error_code: number;
    };
    last_ack: {
      valid: boolean;
      acked_class: number;
      acked_id: number;
      acked_sequence: number;
      result: string;
      detail: number;
      echo_token: number;
    };
    counters: {
      rx_packets: number;
      tx_packets: number;
      crc_errors: number;
      length_errors: number;
      version_errors: number;
      unknown_data: number;
      unknown_commands: number;
      duplicates: number;
      sequence_gaps: number;
      tx_retries: number;
      ack_timeouts: number;
      send_errors: number;
    };
  };
  aircrafts: Array<{
    id: string;
    name: string;
  }>;
  binding: {
    aircraft_id: string;
    body_id: string;
    sensor_id: number;
    active: boolean;
  };
  rigid_bodies: Array<{
    body_id: string;
    sensor_id: number;
    messages_received: number;
    last_seen_ms: number | null;
    bound: boolean;
  }>;
  pose: {
    valid: boolean;
    sequence: number;
    source_time_measured: boolean;
    source_age_us_at_receive: number;
    age_ms: number | null;
    position_xyz_m: [number, number, number];
    orientation_wxyz: [number, number, number, number];
  };
  target: {
    position_xyz_m: [number, number, number];
    yaw_rad: number;
    deadman: boolean;
    client_sequence: number;
  };
  command: {
    valid: boolean;
    sequence: number;
    pose_sequence: number;
    pose_age_us: number;
    orientation_wxyz: [number, number, number, number];
    thrust: number;
    upper_throttle: number;
    manual: {
      valid: boolean;
      roll: number;
      pitch: number;
      yaw: number;
      throttle: number;
    };
    position_loop_output_xyz: [number, number, number];
    roll_right_deg: number;
    pitch_down_deg: number;
    yaw_rate_right_deg_s: number;
  };
  link: {
    mode: string;
    protocol: string;
    connected: boolean;
    commands_sent: number;
    send_errors: number;
  };
  metrics: {
    target_period_ms: number;
    last_period_ms: number;
    work_ms: number;
    jitter_ms: number;
    p99_work_ms: number;
    p99_jitter_ms: number;
    overruns: number;
  };
}

export interface OperatorIntent {
  v: 1;
  type: "operator_intent";
  client_seq: number;
  target_xyz_m: [number, number, number];
  target_yaw_rad: number;
  deadman: boolean;
}

export interface BindingAck {
  v: 1;
  type: "binding_ack";
  aircraft_id: string;
  body_id: string;
  sensor_id: number;
}

export interface GsLinkAccept {
  v: 1;
  type: "gs_link_accept";
  kind: string;
}

export interface GsLinkError {
  v: 1;
  type: "gs_link_error";
  kind: string;
  message: string;
}

export interface PositionTuningMessage {
  v: 1;
  type: "position_tuning";
  tuning: PositionLoopTuning;
}

export interface PositionTuningAck {
  v: 1;
  type: "position_tuning_ack";
  tuning: PositionLoopTuning;
}

export interface PositionTuningError {
  v: 1;
  type: "position_tuning_error";
  message: string;
}

export interface TargetDraft {
  x: number;
  y: number;
  z: number;
  yawDegrees: number;
}
