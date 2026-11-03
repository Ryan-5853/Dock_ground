#pragma once

#include "dock_ground/gs_link.hpp"
#include "dock_ground/math.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

namespace dock_ground {

using SteadyClock = std::chrono::steady_clock;

[[nodiscard]] inline std::uint64_t monotonic_ns(
    SteadyClock::time_point time = SteadyClock::now()) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            time.time_since_epoch())
            .count());
}

enum class ControlState : std::uint8_t {
    Standby,
    Track,
    Degraded,
    Failsafe,
};

[[nodiscard]] inline const char* to_string(ControlState state) {
    switch (state) {
        case ControlState::Standby:
            return "STANDBY";
        case ControlState::Track:
            return "TRACK";
        case ControlState::Degraded:
            return "DEGRADED";
        case ControlState::Failsafe:
            return "FAILSAFE";
    }
    return "UNKNOWN";
}

enum class PositionRegulationMode : std::uint8_t {
    FarTrack,
    Approach,
    PrecisionHold,
};

[[nodiscard]] inline const char* to_string(
    PositionRegulationMode mode) {
    switch (mode) {
        case PositionRegulationMode::FarTrack:
            return "FAR_TRACK";
        case PositionRegulationMode::Approach:
            return "APPROACH";
        case PositionRegulationMode::PrecisionHold:
            return "PRECISION_HOLD";
    }
    return "UNKNOWN";
}

struct PoseSample {
    std::uint64_t received_mono_ns{0};
    std::uint64_t source_mono_ns{0};
    std::uint64_t source_age_at_receive_ns{0};
    std::uint64_t arrival_interval_ns{0};
    std::uint64_t source_time_interval_ns{0};
    std::int64_t source_clock_delta_ns{0};
    std::uint32_t sequence{0};
    std::int32_t source_sensor_id{-1};
    Vec3 position_xyz_m{};
    Quaternion q_body_to_world{};
    Vec3 raw_position{};
    Quaternion raw_orientation{};
    bool source_time_present{false};
    bool source_time_valid{false};
    bool source_time_regression{false};
    bool valid{false};
};

struct OperatorTarget {
    Vec3 position_xyz_m{0.0, 0.0, 1.5};
    Vec3 target_velocity_xyz_mps{};
    double yaw_rad{0.0};
    bool deadman{false};
    std::uint64_t updated_mono_ns{0};
    std::uint64_t client_sequence{0};
};

struct AxisPidGains {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
};

struct PositionProfile {
    AxisPidGains x{};
    AxisPidGains y{};
    AxisPidGains z{};
};

struct PositionModeScheduling {
    bool enabled{true};
    double brake_enter_distance_m{0.55};
    double brake_exit_distance_m{0.75};
    double brake_enter_vertical_error_m{0.30};
    double brake_exit_vertical_error_m{0.40};
    double hold_enter_distance_m{0.12};
    double hold_exit_distance_m{0.22};
    double hold_enter_vertical_error_m{0.06};
    double hold_exit_vertical_error_m{0.12};
    double hold_enter_speed_mps{0.08};
    double hold_exit_speed_mps{0.15};
    double hold_enter_vertical_speed_mps{0.05};
    double hold_exit_vertical_speed_mps{0.10};
    double target_motion_speed_mps{0.02};
    double hold_enter_dwell_s{0.50};
    double hold_exit_dwell_s{0.20};
    double gain_transition_time_s{0.60};
};

struct PositionLoopTuning {
    // x/y/z are the far-track profile. approach and precision_hold
    // are selected by the position regulation state machine.
    AxisPidGains x{0.60, 0.01, 0.30};
    AxisPidGains y{0.60, 0.01, 0.30};
    AxisPidGains z{0.06, 0.01, 0.08};
    PositionProfile approach{
        {0.80, 0.01, 1.10},
        {0.85, 0.01, 1.20},
        {0.14, 0.005, 0.16},
    };
    PositionProfile precision_hold{
        {0.30, 0.005, 0.90},
        {0.34, 0.005, 1.00},
        {0.08, 0.003, 0.14},
    };
    PositionModeScheduling mode_scheduling{};
    double max_accel_mps2{4.0};
    double max_tilt_rad{0.3141592653589793};
    double min_throttle{0.20};
    double hover_throttle{0.56};
    double max_throttle{0.96};
    double z_velocity_damping{0.08};
    double max_yaw_rate_rad_s{0.50};
    double yaw_rate_kp{1.0};
    double yaw_rate_kd{0.50};
    double yaw_rate_slew_rate_rad_s2{1.5};
    double fc_yaw_rate_scale_rad_s{1.5};
    double max_position_error_m{10.0};
    double gravity_mps2{9.80665};
};

struct PositionRegulationStatus {
    PositionRegulationMode mode{PositionRegulationMode::FarTrack};
    bool active{false};
    bool scheduling_enabled{false};
    bool gain_transition_active{false};
    double horizontal_error_m{0.0};
    double vertical_error_m{0.0};
    double horizontal_speed_mps{0.0};
    double vertical_speed_mps{0.0};
    AxisPidGains active_x{};
    AxisPidGains active_y{};
    AxisPidGains active_z{};
};

struct ManualControlSetpoint {
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
    double throttle{-1.0};
    bool valid{false};
};

struct RcChannelMapping {
    int roll{1};
    int pitch{2};
    int throttle{3};
    int yaw{4};
    int handoff_hold_ms{40};
    int throttle_transition_ms{300};
    double handoff_max_vertical_speed_mps{0.20};
    double handoff_max_throttle_delta{0.10};
};

struct PoseSourceStatus {
    bool connected{false};
    bool transport_connected{false};
    const char* name{"simulated"};
    std::int32_t sensor_id{0};
    std::uint64_t messages_received{0};
    std::uint64_t callbacks_received{0};
    std::uint64_t samples_rejected{0};
    std::uint64_t unselected_sensor_samples{0};
    std::uint64_t invalid_position_samples{0};
    std::uint64_t invalid_orientation_samples{0};
    std::uint64_t duplicate_timestamp_samples{0};
    std::uint64_t timestamp_regression_samples{0};
    std::uint64_t long_gap_count{0};
    std::uint64_t connect_count{0};
    std::uint64_t disconnect_count{0};
    std::uint64_t last_received_mono_ns{0};
    double expected_rate_hz{0.0};
    double last_arrival_gap_ms{-1.0};
    double max_arrival_gap_ms{0.0};
};

struct TrackerBodyInfo {
    std::int32_t sensor_id{0};
    std::uint64_t messages_received{0};
    std::uint64_t last_seen_mono_ns{0};
};

struct AttitudeThrustSetpoint {
    std::uint32_t sequence{0};
    std::uint32_t pose_sequence{0};
    std::uint32_t pose_age_us{0};
    Quaternion q_body_to_world_desired{};
    float collective_thrust{0.0F};
    ManualControlSetpoint manual{};
    Vec3 position_loop_output_xyz{};
    double roll_right_command_rad{0.0};
    double pitch_down_command_rad{0.0};
    double yaw_rate_right_command_rad_s{0.0};
    std::uint32_t flags{0};
};

struct LinkStatus {
    const char* mode{"loopback"};
    const char* protocol{"none"};
    bool connected{true};
    std::uint64_t commands_sent{0};
    std::uint64_t send_errors{0};
    std::uint64_t suppressed_sends{0};
    double target_rate_hz{0.0};
    double actual_rate_hz{0.0};
    double last_send_interval_ms{0.0};
    const char* last_error{"none"};
    int last_errno{0};
};

struct LoopMetrics {
    double target_period_ms{10.0};
    double last_period_ms{0.0};
    double work_ms{0.0};
    double jitter_ms{0.0};
    double p99_work_ms{0.0};
    double p99_jitter_ms{0.0};
    std::uint64_t overruns{0};
};

struct FlightLogStatus {
    bool enabled{false};
    bool recording{false};
    std::string current_file;
    std::string last_error;
    std::uint64_t sessions_written{0};
    std::uint64_t frames_written{0};
    std::uint64_t frames_dropped{0};
};

struct RuntimeSnapshot {
    std::uint64_t generated_mono_ns{0};
    ControlState state{ControlState::Standby};
    const char* mode{"simulation"};
    PoseSample pose{};
    OperatorTarget target{};
    AttitudeThrustSetpoint command{};
    LinkStatus link{};
    gs_link::Status gs_link{};
    PoseSourceStatus source_status{};
    LoopMetrics metrics{};
    PositionLoopTuning position_tuning{};
    RcChannelMapping rc_mapping{};
    ManualControlSetpoint rc_handoff_base{};
    bool rc_passthrough_active{false};
    bool rc_handoff_active{false};
    bool rc_handoff_hold{false};
    double handoff_vertical_speed_mps{0.0};
    double rc_handoff_throttle_blend{0.0};
    PositionRegulationStatus position_mode{};
    std::string rc_handoff_error{};
    FlightLogStatus flight_log{};
    double pose_age_ms{0.0};
    bool command_valid{false};
    bool deadman_active{false};
};

}  // namespace dock_ground
