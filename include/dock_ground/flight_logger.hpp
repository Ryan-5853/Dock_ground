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
};

struct FlightLogSample {
    std::uint64_t mono_ns{0};
    std::int64_t wall_time_ns{0};
    ControlState control_state{ControlState::Standby};
    PoseSample pose{};
    OperatorTarget target{};
    PositionLoopTuning tuning{};
    AttitudeThrustSetpoint command{};
    ManualControlSetpoint rc_base{};
    gs_link::FlightTelemetry telemetry{};
    gs_link::Heartbeat heartbeat{};
    gs_link::Counters counters{};
    double pose_age_ms{0.0};
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
