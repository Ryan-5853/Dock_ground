#pragma once

#include "dock_ground/types.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace dock_ground {

struct FlightLoggerOptions {
    bool enabled{true};
    std::string directory{"logs/flights"};
    std::string aircraft_id{"aircraft-01"};
    std::string aircraft_name{"飞行器 01"};
    RcChannelMapping rc_mapping{};
    std::uint16_t telemetry_stale_timeout_ms{2000};
    bool udp_flight_transport{true};
    double pre_roll_s{5.0};
    double control_rate_hz{100.0};
    double warning_pose_age_ms{30.0};
    double max_pose_age_ms{50.0};
    std::string pose_source{"unknown"};
    std::string pose_endpoint{};
    int pose_sensor_id{-1};
    double pose_position_scale{1.0};
    double pose_expected_rate_hz{0.0};
    bool pose_auto_origin{false};
    bool pose_trust_source_timestamp{false};
    Mat3 pose_axes_from_vrpn{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    };
};

struct FlightLogSample {
    std::uint64_t mono_ns{0};
    std::int64_t wall_time_ns{0};
    ControlState control_state{ControlState::Standby};
    PoseSample pose{};
    OperatorTarget target{};
    PositionLoopTuning tuning{};
    PositionRegulationStatus regulation{};
    AttitudeThrustSetpoint command{};
    ManualControlSetpoint rc_base{};
    gs_link::FlightTelemetry telemetry{};
    gs_link::Heartbeat heartbeat{};
    gs_link::Counters counters{};
    PoseSourceStatus source_status{};
    LoopMetrics loop_metrics{};
    double pose_age_ms{0.0};
    double pose_local_age_ms{0.0};
    double pose_source_age_ms{0.0};
    bool telemetry_fresh{false};
    bool heartbeat_fresh{false};
    bool gs_transport_connected{false};
    bool uplink_allowed{false};
    double telemetry_age_ms{-1.0};
    double heartbeat_age_ms{-1.0};
    LinkStatus command_link{};
    bool command_valid{false};
    bool deadman_active{false};
    bool rc_passthrough_active{false};
    bool handoff_active{false};
    bool handoff_hold{false};
    double handoff_vertical_speed_mps{0.0};
    double handoff_throttle_blend{0.0};
    bool pose_fresh{false};
    bool rc_gate_ready{false};
    bool handoff_reset{false};
    std::string handoff_reset_reason{};
    std::string handoff_error{};
};

class FlightLogger {
public:
    explicit FlightLogger(FlightLoggerOptions options);
    ~FlightLogger();

    FlightLogger(const FlightLogger&) = delete;
    FlightLogger& operator=(const FlightLogger&) = delete;

    [[nodiscard]] bool start(std::string& error);
    void stop();

    void observe(const FlightLogSample& sample) noexcept;
    [[nodiscard]] FlightLogStatus status() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dock_ground
