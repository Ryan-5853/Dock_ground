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

struct PoseSample {
    std::uint64_t received_mono_ns{0};
    std::uint64_t source_mono_ns{0};
    std::uint64_t source_age_at_receive_ns{0};
    std::uint32_t sequence{0};
    Vec3 position_xyz_m{};
    Quaternion q_body_to_world{};
    bool source_time_valid{false};
    bool valid{false};
};

struct OperatorTarget {
    Vec3 position_xyz_m{0.0, 0.0, 1.5};
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

struct PositionLoopTuning {
    AxisPidGains x{0.60, 0.01, 0.30};
    AxisPidGains y{0.60, 0.01, 0.30};
    AxisPidGains z{0.06, 0.01, 0.08};
    double max_accel_mps2{4.0};
    double max_tilt_rad{0.3490658503988659};
    double min_throttle{0.20};
    double hover_throttle{0.56};
    double max_throttle{0.96};
    double z_velocity_damping{0.08};
    double max_yaw_rate_rad_s{1.2};
    double yaw_rate_kp{2.0};
    double max_position_error_m{10.0};
    double gravity_mps2{9.80665};
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
    std::string rc_handoff_error{};
    FlightLogStatus flight_log{};
    double pose_age_ms{0.0};
    bool command_valid{false};
    bool deadman_active{false};
};

}  // namespace dock_ground
