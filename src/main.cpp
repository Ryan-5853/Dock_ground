#include "dock_ground/config.hpp"
#include "dock_ground/runtime.hpp"
#include "dock_ground/web_server.hpp"

#include <csignal>
#include <cstdlib>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

void handle_signal(int) {
    g_stop_requested = 1;
}

void print_usage(const char* executable) {
    std::cout
        << "Usage: " << executable << " [options]\n"
        << "  --config PATH       JSON configuration (default: config/ground.json)\n"
        << "  --http-bind ADDR    Override HTTP bind address\n"
        << "  --http-port PORT    Override HTTP/WebSocket port\n"
        << "  --web-root PATH     Override built WebUI directory\n"
        << "  --run-seconds N     Stop automatically after N seconds (smoke test)\n"
        << "  --help              Show this help\n";
}

bool parse_port(const std::string& value, std::uint16_t& port) {
    try {
        std::size_t consumed = 0;
        const unsigned long parsed = std::stoul(value, &consumed);
        if (consumed != value.size() || parsed < 1UL || parsed > 65535UL) {
            return false;
        }
        port = static_cast<std::uint16_t>(parsed);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool parse_seconds(const std::string& value, double& seconds) {
    try {
        std::size_t consumed = 0;
        seconds = std::stod(value, &consumed);
        return consumed == value.size() && seconds > 0.0 &&
               seconds <= 3600.0;
    } catch (const std::exception&) {
        return false;
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string config_path = "config/ground.json";
    bool config_was_explicit = false;
    std::optional<std::string> bind_override;
    std::optional<std::uint16_t> port_override;
    std::optional<std::string> web_root_override;
    double run_seconds = 0.0;

    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--help" || argument == "-h") {
            print_usage(argv[0]);
            return 0;
        }
        if (index + 1 >= argc) {
            std::cerr << "missing value for " << argument << '\n';
            return 2;
        }
        const std::string value = argv[++index];
        if (argument == "--config") {
            config_path = value;
            config_was_explicit = true;
        } else if (argument == "--http-bind") {
            bind_override = value;
        } else if (argument == "--http-port") {
            std::uint16_t port = 0;
            if (!parse_port(value, port)) {
                std::cerr << "invalid --http-port: " << value << '\n';
                return 2;
            }
            port_override = port;
        } else if (argument == "--web-root") {
            web_root_override = value;
        } else if (argument == "--run-seconds") {
            if (!parse_seconds(value, run_seconds)) {
                std::cerr << "invalid --run-seconds: " << value << '\n';
                return 2;
            }
        } else {
            std::cerr << "unknown option: " << argument << '\n';
            print_usage(argv[0]);
            return 2;
        }
    }

    dock_ground::AppConfig config;
    if (std::filesystem::exists(config_path) || config_was_explicit) {
        std::string error;
        const auto loaded = dock_ground::load_config_file(config_path, error);
        if (!loaded) {
            std::cerr << "configuration error: " << error << '\n';
            return 1;
        }
        config = *loaded;
    } else {
        std::cerr << "configuration file not found: " << config_path
                  << "; using built-in simulation defaults\n";
    }

    if (bind_override.has_value()) {
        config.http_bind = *bind_override;
    }
    if (port_override.has_value()) {
        config.http_port = *port_override;
    }
    if (web_root_override.has_value()) {
        config.web_root = *web_root_override;
    }

    try {
        dock_ground::validate_config(config);
    } catch (const std::exception& exception) {
        std::cerr << "configuration error: " << exception.what() << '\n';
        return 1;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    try {
        dock_ground::Runtime runtime(config);
        runtime.start();

        dock_ground::WebServer server(
            dock_ground::WebServerOptions{
                .bind_address = config.http_bind,
                .port = config.http_port,
                .web_root = config.web_root,
            },
            [&runtime](const dock_ground::OperatorTarget& target) {
                return runtime.apply_operator_target(target);
            },
            [&runtime](
                const std::string& aircraft_id, const std::string& body_id,
                int sensor_id, std::string& error) {
                return runtime.bind_body(
                    aircraft_id, body_id, sensor_id, error);
            },
            [&runtime](
                const dock_ground::gs_link::UplinkRequest& request,
                std::string& error) {
                return runtime.apply_gs_uplink(request, error);
            },
            [&runtime](
                const dock_ground::PositionLoopTuning& tuning,
                std::string& error) {
                return runtime.apply_position_tuning(tuning, error);
            },
            [&runtime] { return runtime.snapshot_json(); });

        std::string server_error;
        if (!server.start(server_error)) {
            std::cerr << "web server error: " << server_error << '\n';
            runtime.stop();
            return 1;
        }

        std::cout
            << "Dock Ground Station 0.1.0\n"
            << "  pose:     " << config.pose_source << '\n'
            << "  control:  " << config.control_rate_hz << " Hz\n"
            << "  flight log: "
            << (config.flight_log_enabled
                    ? config.flight_log_directory + " (ARM -> CSV)"
                    : std::string("disabled"))
            << '\n'
            << "  web:      http://" << config.http_bind << ':'
            << server.bound_port() << "\n";
        if (config.flight_transport == "udp") {
            std::cout
                << "  flight:   GS-Link v1 UDP "
                << config.gs_link_bind_ip << ':'
                << *config.gs_link_bind_port << " -> "
                << config.esp32_ip << ':' << *config.esp32_port
                << "\n"
                << "  position: incremental PID -> DATA_RC "
                   "roll/pitch/yaw/throttle / hover "
                << config.position_loop.hover_throttle
                << " / z damping " << config.position_loop.z_velocity_damping
                << "\n"
                << "  RC handoff: R CH" << config.rc_mapping.roll
                << ", P CH" << config.rc_mapping.pitch
                << ", T CH" << config.rc_mapping.throttle
                << ", Y CH" << config.rc_mapping.yaw
                << " / hold " << config.rc_mapping.handoff_hold_ms
                << " ms / throttle transition "
                << config.rc_mapping.throttle_transition_ms
                << " ms / max |vz| "
                << config.rc_mapping.handoff_max_vertical_speed_mps
                << " m/s\n"
                << "  warning:  FC safety gates remain authoritative; "
                   "bench validation required\n";
        } else {
            std::cout
                << "  protocol: none (real flight output disabled)\n"
                << "  warning:  LOOPBACK ONLY; real flight output "
                   "disabled\n";
        }

        const auto started = dock_ground::SteadyClock::now();
        while (g_stop_requested == 0) {
            if (run_seconds > 0.0 &&
                std::chrono::duration<double>(
                    dock_ground::SteadyClock::now() - started)
                        .count() >= run_seconds) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        server.stop();
        runtime.stop();
    } catch (const std::exception& exception) {
        std::cerr << "fatal error: " << exception.what() << '\n';
        return 1;
    }
    return 0;
}
