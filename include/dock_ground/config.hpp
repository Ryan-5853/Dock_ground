#pragma once

#include "dock_ground/types.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace dock_ground {

struct AppConfig {
    std::string aircraft_id{"aircraft-01"};
    std::string aircraft_name{"飞行器 01"};

    std::string http_bind{"127.0.0.1"};
    std::uint16_t http_port{8080};
    std::string web_root{"frontend/dist"};

    double control_rate_hz{100.0};
    double warning_pose_age_ms{30.0};
    double max_pose_age_ms{50.0};
    double deadman_timeout_ms{250.0};

    PositionLoopTuning position_loop{};

    Vec3 initial_target_xyz_m{0.0, 0.0, 0.0};
    double initial_yaw_rad{0.0};
    bool initial_target_use_current_pose{false};

    double simulated_pose_rate_hz{100.0};
    std::string pose_source{"vrpn"};

    std::string flight_transport{"loopback"};
    std::string flight_protocol{"null"};
    std::string esp32_ip{"192.168.31.100"};
    std::optional<std::uint16_t> esp32_port{gs_link::kC5ProxyUdpPort};
    std::string gs_link_bind_ip{"0.0.0.0"};
    std::optional<std::uint16_t> gs_link_bind_port{
        gs_link::kDefaultGcsProxyUdpPort};
    std::uint16_t gs_link_telemetry_timeout_ms{250};
    std::uint16_t gs_link_heartbeat_timeout_ms{300};
    double gs_link_heartbeat_rate_hz{10.0};
    RcChannelMapping rc_mapping{};
    bool flight_log_enabled{true};
    std::string flight_log_directory{"logs/flights"};
    std::uint16_t flight_log_stale_timeout_ms{2000};

    std::string vrpn_endpoint{"MCAvatar@192.168.31.221:3883"};
    int vrpn_sensor_id{1};
    double vrpn_position_scale{0.001};
    bool vrpn_auto_origin{true};
    bool vrpn_trust_source_timestamp{false};
    Mat3 vrpn_axes_from_vrpn{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    };
};

[[nodiscard]] std::optional<AppConfig> load_config_file(
    const std::string& path, std::string& error);

void validate_config(const AppConfig& config);

}  // namespace dock_ground
