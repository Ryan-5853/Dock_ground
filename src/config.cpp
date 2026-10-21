#include "dock_ground/config.hpp"

#include "dock_ground/json.hpp"

#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace dock_ground {
namespace {

using json::Value;

bool set_error(
    std::string& error, std::string_view path, std::string_view message) {
    error = std::string(path) + ": " + std::string(message);
    return false;
}

const Value* get_child(
    const Value* object, std::string_view key, std::string_view path,
    std::string& error, bool required = false) {
    if (object == nullptr) {
        if (required) {
            set_error(error, path, "missing object");
        }
        return nullptr;
    }
    if (object->type != Value::Type::Object) {
        set_error(error, path, "expected object");
        return nullptr;
    }
    const Value* value = object->find(key);
    if (value == nullptr && required) {
        set_error(error, path, "missing required value");
    }
    return value;
}

bool reject_unknown_keys(
    const Value* object, std::string_view path,
    const std::vector<std::string_view>& allowed, std::string& error) {
    if (object == nullptr) {
        return true;
    }
    if (object->type != Value::Type::Object) {
        return set_error(error, path, "expected object");
    }
    for (const auto& [key, unused] : object->object) {
        (void)unused;
        bool found = false;
        for (const auto allowed_key : allowed) {
            if (key == allowed_key) {
                found = true;
                break;
            }
        }
        if (!found) {
            return set_error(
                error, path, "unknown configuration key '" + key + "'");
        }
    }
    return true;
}

bool read_string(
    const Value* object, std::string_view key, std::string_view path,
    std::string& destination, std::string& error, bool required = false) {
    const Value* value =
        get_child(object, key, path, error, required);
    if (value == nullptr) {
        return !required;
    }
    if (value->type != Value::Type::String) {
        return set_error(error, path, "expected string");
    }
    destination = value->string;
    return true;
}

bool read_optional_string(
    const Value* object, std::string_view key, std::string_view path,
    std::string& destination, std::string& error) {
    const Value* value = get_child(object, key, path, error);
    if (value == nullptr || value->type == Value::Type::Null) {
        return true;
    }
    if (value->type != Value::Type::String) {
        return set_error(error, path, "expected string or null");
    }
    destination = value->string;
    return true;
}

bool read_number(
    const Value* object, std::string_view key, std::string_view path,
    double& destination, std::string& error, bool required = false) {
    const Value* value =
        get_child(object, key, path, error, required);
    if (value == nullptr) {
        return !required;
    }
    if (value->type != Value::Type::Number) {
        return set_error(error, path, "expected number");
    }
    if (!std::isfinite(value->number)) {
        return set_error(error, path, "number must be finite");
    }
    destination = value->number;
    return true;
}

bool read_integer(
    const Value* object, std::string_view key, std::string_view path,
    int& destination, std::string& error, bool required = false) {
    double number = static_cast<double>(destination);
    if (!read_number(
            object, key, path, number, error, required)) {
        return false;
    }
    if (number < static_cast<double>(std::numeric_limits<int>::min()) ||
        number > static_cast<double>(std::numeric_limits<int>::max()) ||
        std::floor(number) != number) {
        return set_error(error, path, "expected integer");
    }
    destination = static_cast<int>(number);
    return true;
}

bool read_optional_integer(
    const Value* object, std::string_view key, std::string_view path,
    int& destination, std::string& error) {
    const Value* value = get_child(object, key, path, error);
    if (value == nullptr || value->type == Value::Type::Null) {
        destination = -1;
        return true;
    }
    if (value->type != Value::Type::Number ||
        !std::isfinite(value->number) ||
        std::floor(value->number) != value->number ||
        value->number < static_cast<double>(std::numeric_limits<int>::min()) ||
        value->number > static_cast<double>(std::numeric_limits<int>::max())) {
        return set_error(error, path, "expected integer or null");
    }
    destination = static_cast<int>(value->number);
    return true;
}

bool read_boolean(
    const Value* object, std::string_view key, std::string_view path,
    bool& destination, std::string& error, bool required = false) {
    const Value* value =
        get_child(object, key, path, error, required);
    if (value == nullptr) {
        return !required;
    }
    if (value->type != Value::Type::Boolean) {
        return set_error(error, path, "expected boolean");
    }
    destination = value->boolean;
    return true;
}

bool read_optional_port(
    const Value* object, std::string_view key, std::string_view path,
    std::optional<std::uint16_t>& destination, std::string& error) {
    const Value* value = get_child(object, key, path, error);
    if (value == nullptr || value->type == Value::Type::Null) {
        destination.reset();
        return true;
    }
    if (value->type != Value::Type::Number) {
        return set_error(error, path, "expected port or null");
    }
    if (value->number < 1.0 || value->number > 65535.0 ||
        std::floor(value->number) != value->number) {
        return set_error(error, path, "expected port in [1, 65535]");
    }
    destination = static_cast<std::uint16_t>(value->number);
    return true;
}

bool read_optional_u16(
    const Value* object, std::string_view key, std::string_view path,
    std::uint16_t& destination, std::string& error) {
    const Value* value = get_child(object, key, path, error);
    if (value == nullptr || value->type == Value::Type::Null) {
        return true;
    }
    if (value->type != Value::Type::Number ||
        !std::isfinite(value->number) ||
        value->number < 0.0 || value->number > 65535.0 ||
        std::floor(value->number) != value->number) {
        return set_error(error, path, "expected integer in [0, 65535]");
    }
    destination = static_cast<std::uint16_t>(value->number);
    return true;
}

bool read_mat3(
    const Value* object, std::string_view key, std::string_view path,
    Mat3& destination, std::string& error, bool required = false) {
    const Value* value =
        get_child(object, key, path, error, required);
    if (value == nullptr) {
        return !required;
    }
    if (value->type != Value::Type::Array || value->array.size() != 3U) {
        return set_error(error, path, "expected 3x3 numeric array");
    }
    for (std::size_t row = 0; row < 3U; ++row) {
        const Value* row_value = value->array[row].get();
        if (row_value->type != Value::Type::Array ||
            row_value->array.size() != 3U) {
            return set_error(error, path, "expected 3x3 numeric array");
        }
        for (std::size_t column = 0; column < 3U; ++column) {
            const Value* component = row_value->array[column].get();
            if (component->type != Value::Type::Number ||
                !std::isfinite(component->number)) {
                return set_error(
                    error, path, "matrix entries must be finite numbers");
            }
            destination[row * 3U + column] = component->number;
        }
    }
    return true;
}

bool read_vec3(
    const Value* object, std::string_view key, std::string_view path,
    Vec3& destination, std::string& error, bool required = false) {
    const Value* value =
        get_child(object, key, path, error, required);
    if (value == nullptr) {
        return !required;
    }
    if (value->type != Value::Type::Array || value->array.size() != 3U) {
        return set_error(error, path, "expected array of three numbers");
    }
    double components[3]{};
    for (std::size_t index = 0; index < 3U; ++index) {
        if (value->array[index]->type != Value::Type::Number ||
            !std::isfinite(value->array[index]->number)) {
            return set_error(
                error, path, "XYZ position components must be finite numbers");
        }
        components[index] = value->array[index]->number;
    }
    destination = {components[0], components[1], components[2]};
    return true;
}

bool read_axis_gains(
    const Value* object, std::string_view key, std::string_view path,
    AxisPidGains& destination, std::string& error) {
    const Value* value = get_child(object, key, path, error);
    if (value == nullptr) {
        return true;
    }
    if (!reject_unknown_keys(
            value, path, {"kp", "ki", "kd"}, error)) {
        return false;
    }
    return read_number(
               value, "kp", path, destination.kp, error, true) &&
           read_number(
               value, "ki", path, destination.ki, error, true) &&
           read_number(
               value, "kd", path, destination.kd, error, true);
}

bool read_file(const std::string& path, std::string& contents) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        return false;
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    contents = buffer.str();
    return stream.good() || stream.eof();
}

}  // namespace

std::optional<AppConfig> load_config_file(
    const std::string& path, std::string& error) {
    error.clear();
    std::string contents;
    if (!read_file(path, contents)) {
        error = "cannot read configuration file: " + path;
        return std::nullopt;
    }

    auto root = json::parse(contents, error);
    if (!root) {
        error = path + ": " + error;
        return std::nullopt;
    }
    if (root->type != Value::Type::Object) {
        error = path + ": root value must be an object";
        return std::nullopt;
    }
    if (!reject_unknown_keys(
            root.get(), "$",
            {"aircraft", "server", "control", "initial_target", "simulation",
             "pose", "flight", "vrpn", "position_loop", "flight_log"},
            error)) {
        error = path + ": " + error;
        return std::nullopt;
    }

    AppConfig config;
    const Value* aircraft =
        get_child(root.get(), "aircraft", "aircraft", error, true);
    const Value* server =
        get_child(root.get(), "server", "server", error, true);
    const Value* control =
        get_child(root.get(), "control", "control", error, true);
    const Value* target = get_child(
        root.get(), "initial_target", "initial_target", error, true);
    const Value* simulation = get_child(
        root.get(), "simulation", "simulation", error, true);
    const Value* pose =
        get_child(root.get(), "pose", "pose", error, true);
    const Value* flight =
        get_child(root.get(), "flight", "flight", error, true);
    const Value* vrpn = get_child(root.get(), "vrpn", "vrpn", error, true);
    const Value* position_loop =
        get_child(root.get(), "position_loop", "position_loop", error);
    const Value* flight_log =
        get_child(root.get(), "flight_log", "flight_log", error);

    const bool sections_valid =
        reject_unknown_keys(
            aircraft, "aircraft", {"id", "name"}, error) &&
        reject_unknown_keys(
            server, "server", {"http_bind", "http_port", "web_root"}, error) &&
        reject_unknown_keys(
            control, "control",
            {"rate_hz", "warning_pose_age_ms", "max_pose_age_ms",
             "deadman_timeout_ms"},
            error) &&
        reject_unknown_keys(
            position_loop, "position_loop",
            {"x", "y", "z", "max_accel_mps2", "max_tilt_rad",
             "min_throttle", "hover_throttle", "max_throttle",
             "z_velocity_damping",
             "max_yaw_rate_rad_s", "yaw_rate_kp",
             "max_position_error_m", "gravity_mps2"},
            error) &&
        reject_unknown_keys(
            flight_log, "flight_log",
            {"enabled", "directory", "telemetry_stale_timeout_ms"},
            error) &&
        reject_unknown_keys(
            target, "initial_target",
            {"position_xyz_m", "yaw_rad", "use_current_pose"}, error) &&
        reject_unknown_keys(
            simulation, "simulation", {"pose_rate_hz"}, error) &&
        reject_unknown_keys(pose, "pose", {"source"}, error) &&
        reject_unknown_keys(
            flight, "flight",
            {"transport", "protocol", "remote_ip", "remote_port",
             "local_ip", "local_port", "telemetry_timeout_ms",
             "heartbeat_timeout_ms", "heartbeat_rate_hz",
             "rc_roll_channel", "rc_pitch_channel",
             "rc_throttle_channel", "rc_yaw_channel",
             "bumpless_hold_ms", "throttle_transition_ms",
             "handoff_max_vertical_speed_mps",
             "handoff_max_throttle_delta"},
            error) &&
        reject_unknown_keys(
            vrpn, "vrpn",
            {"endpoint", "sensor_id", "position_scale", "auto_origin",
             "axes_from_vrpn", "trust_source_timestamp"},
            error);
    if (!sections_valid) {
        error = path + ": " + error;
        return std::nullopt;
    }

    int http_port = config.http_port;
    bool valid =
        read_string(
            aircraft, "id", "aircraft.id", config.aircraft_id, error) &&
        read_string(
            aircraft, "name", "aircraft.name", config.aircraft_name, error) &&
        read_string(
            server, "http_bind", "server.http_bind", config.http_bind, error) &&
        read_integer(
            server, "http_port", "server.http_port", http_port, error) &&
        read_string(
            server, "web_root", "server.web_root", config.web_root, error) &&
        read_number(
            control, "rate_hz", "control.rate_hz",
            config.control_rate_hz, error) &&
        read_number(
            control, "warning_pose_age_ms",
            "control.warning_pose_age_ms", config.warning_pose_age_ms, error) &&
        read_number(
            control, "max_pose_age_ms", "control.max_pose_age_ms",
            config.max_pose_age_ms, error) &&
        read_number(
            control, "deadman_timeout_ms", "control.deadman_timeout_ms",
            config.deadman_timeout_ms, error) &&
        read_axis_gains(
            position_loop, "x", "position_loop.x",
            config.position_loop.x, error) &&
        read_axis_gains(
            position_loop, "y", "position_loop.y",
            config.position_loop.y, error) &&
        read_axis_gains(
            position_loop, "z", "position_loop.z",
            config.position_loop.z, error) &&
        read_number(
            position_loop, "max_accel_mps2",
            "position_loop.max_accel_mps2",
            config.position_loop.max_accel_mps2, error) &&
        read_number(
            position_loop, "max_tilt_rad",
            "position_loop.max_tilt_rad",
            config.position_loop.max_tilt_rad, error) &&
        read_number(
            position_loop, "min_throttle",
            "position_loop.min_throttle",
            config.position_loop.min_throttle, error) &&
        read_number(
            position_loop, "hover_throttle",
            "position_loop.hover_throttle",
            config.position_loop.hover_throttle, error) &&
        read_number(
            position_loop, "max_throttle",
            "position_loop.max_throttle",
            config.position_loop.max_throttle, error) &&
        read_number(
            position_loop, "z_velocity_damping",
            "position_loop.z_velocity_damping",
            config.position_loop.z_velocity_damping, error) &&
        read_number(
            position_loop, "max_yaw_rate_rad_s",
            "position_loop.max_yaw_rate_rad_s",
            config.position_loop.max_yaw_rate_rad_s, error) &&
        read_number(
            position_loop, "yaw_rate_kp",
            "position_loop.yaw_rate_kp",
            config.position_loop.yaw_rate_kp, error) &&
        read_number(
            position_loop, "max_position_error_m",
            "position_loop.max_position_error_m",
            config.position_loop.max_position_error_m, error) &&
        read_number(
            position_loop, "gravity_mps2",
            "position_loop.gravity_mps2",
            config.position_loop.gravity_mps2, error) &&
        read_vec3(
            target, "position_xyz_m", "initial_target.position_xyz_m",
            config.initial_target_xyz_m, error) &&
        read_number(
            target, "yaw_rad", "initial_target.yaw_rad",
            config.initial_yaw_rad, error) &&
        read_boolean(
            target, "use_current_pose", "initial_target.use_current_pose",
            config.initial_target_use_current_pose, error) &&
        read_number(
            simulation, "pose_rate_hz", "simulation.pose_rate_hz",
            config.simulated_pose_rate_hz, error) &&
        read_string(
            pose, "source", "pose.source", config.pose_source, error) &&
        read_string(
            flight, "transport", "flight.transport",
            config.flight_transport, error) &&
        read_optional_string(
            flight, "protocol", "flight.protocol",
            config.flight_protocol, error) &&
        read_string(
            flight, "remote_ip", "flight.remote_ip", config.esp32_ip, error) &&
        read_optional_port(
            flight, "remote_port", "flight.remote_port", config.esp32_port,
            error) &&
        read_string(
            flight, "local_ip", "flight.local_ip",
            config.gs_link_bind_ip, error) &&
        read_optional_port(
            flight, "local_port", "flight.local_port",
            config.gs_link_bind_port, error) &&
        read_optional_u16(
            flight, "telemetry_timeout_ms",
            "flight.telemetry_timeout_ms",
            config.gs_link_telemetry_timeout_ms, error) &&
        read_optional_u16(
            flight, "heartbeat_timeout_ms",
            "flight.heartbeat_timeout_ms",
            config.gs_link_heartbeat_timeout_ms, error) &&
        read_number(
            flight, "heartbeat_rate_hz", "flight.heartbeat_rate_hz",
            config.gs_link_heartbeat_rate_hz, error) &&
        read_integer(
            flight, "rc_roll_channel", "flight.rc_roll_channel",
            config.rc_mapping.roll, error) &&
        read_integer(
            flight, "rc_pitch_channel", "flight.rc_pitch_channel",
            config.rc_mapping.pitch, error) &&
        read_integer(
            flight, "rc_throttle_channel",
            "flight.rc_throttle_channel",
            config.rc_mapping.throttle, error) &&
        read_integer(
            flight, "rc_yaw_channel", "flight.rc_yaw_channel",
            config.rc_mapping.yaw, error) &&
        read_integer(
            flight, "bumpless_hold_ms", "flight.bumpless_hold_ms",
            config.rc_mapping.handoff_hold_ms, error) &&
        read_integer(
            flight, "throttle_transition_ms",
            "flight.throttle_transition_ms",
            config.rc_mapping.throttle_transition_ms, error) &&
        read_number(
            flight, "handoff_max_vertical_speed_mps",
            "flight.handoff_max_vertical_speed_mps",
            config.rc_mapping.handoff_max_vertical_speed_mps, error) &&
        read_number(
            flight, "handoff_max_throttle_delta",
            "flight.handoff_max_throttle_delta",
            config.rc_mapping.handoff_max_throttle_delta, error) &&
        read_boolean(
            flight_log, "enabled", "flight_log.enabled",
            config.flight_log_enabled, error) &&
        read_string(
            flight_log, "directory", "flight_log.directory",
            config.flight_log_directory, error) &&
        read_optional_u16(
            flight_log, "telemetry_stale_timeout_ms",
            "flight_log.telemetry_stale_timeout_ms",
            config.flight_log_stale_timeout_ms, error) &&
        read_string(
            vrpn, "endpoint", "vrpn.endpoint", config.vrpn_endpoint, error) &&
        read_optional_integer(
            vrpn, "sensor_id", "vrpn.sensor_id", config.vrpn_sensor_id, error) &&
        read_number(
            vrpn, "position_scale", "vrpn.position_scale",
            config.vrpn_position_scale, error) &&
        read_boolean(
            vrpn, "auto_origin", "vrpn.auto_origin", config.vrpn_auto_origin,
            error) &&
        read_boolean(
            vrpn, "trust_source_timestamp", "vrpn.trust_source_timestamp",
            config.vrpn_trust_source_timestamp, error) &&
        read_mat3(
            vrpn, "axes_from_vrpn", "vrpn.axes_from_vrpn",
            config.vrpn_axes_from_vrpn, error);

    if (!valid) {
        error = path + ": " + error;
        return std::nullopt;
    }
    if (http_port < 1 || http_port > 65535) {
        error = path + ": server.http_port must be in [1, 65535]";
        return std::nullopt;
    }
    config.http_port = static_cast<std::uint16_t>(http_port);

    try {
        validate_config(config);
    } catch (const std::exception& exception) {
        error = path + ": " + exception.what();
        return std::nullopt;
    }
    return config;
}

void validate_config(const AppConfig& config) {
    const auto require = [](bool condition, const char* message) {
        if (!condition) {
            throw std::invalid_argument(message);
        }
    };

    require(!config.http_bind.empty(), "server.http_bind cannot be empty");
    require(!config.aircraft_id.empty(), "aircraft.id cannot be empty");
    require(!config.aircraft_name.empty(), "aircraft.name cannot be empty");
    require(!config.web_root.empty(), "server.web_root cannot be empty");
    require(
        config.control_rate_hz >= 10.0 && config.control_rate_hz <= 500.0,
        "control.rate_hz must be in [10, 500]");
    require(
        config.warning_pose_age_ms > 0.0,
        "control.warning_pose_age_ms must be positive");
    require(
        config.max_pose_age_ms > config.warning_pose_age_ms,
        "control.max_pose_age_ms must exceed warning_pose_age_ms");
    require(
        config.deadman_timeout_ms >= 100.0,
        "control.deadman_timeout_ms must be at least 100");
    const auto valid_axis = [](const AxisPidGains& axis) {
        return std::isfinite(axis.kp) && std::isfinite(axis.ki) &&
               std::isfinite(axis.kd) && axis.kp >= 0.0 &&
               axis.ki >= 0.0 && axis.kd >= 0.0 &&
               axis.kp + axis.ki + axis.kd > 0.0;
    };
    require(
        valid_axis(config.position_loop.x) &&
            valid_axis(config.position_loop.y) &&
            valid_axis(config.position_loop.z),
        "position_loop X/Y/Z PID gains must be finite, non-negative, "
        "and not all zero");
    require(
        std::isfinite(config.position_loop.max_accel_mps2) &&
            config.position_loop.max_accel_mps2 > 0.0,
        "position_loop.max_accel_mps2 must be positive");
    require(
        std::isfinite(config.position_loop.max_tilt_rad) &&
            config.position_loop.max_tilt_rad > 0.0 &&
            config.position_loop.max_tilt_rad < 1.4,
        "position_loop.max_tilt_rad must be in (0, 1.4)");
    require(
        std::isfinite(config.position_loop.min_throttle) &&
            std::isfinite(config.position_loop.hover_throttle) &&
            std::isfinite(config.position_loop.max_throttle) &&
            config.position_loop.min_throttle >= 0.0 &&
            config.position_loop.hover_throttle >=
                config.position_loop.min_throttle &&
            config.position_loop.hover_throttle <=
                config.position_loop.max_throttle &&
            config.position_loop.max_throttle <= 1.0,
        "position_loop throttle range/hover is invalid");
    require(
        std::isfinite(config.position_loop.z_velocity_damping) &&
            config.position_loop.z_velocity_damping >= 0.0 &&
            config.position_loop.z_velocity_damping <= 0.5,
        "position_loop.z_velocity_damping must be in [0, 0.5]");
    require(
        std::isfinite(config.position_loop.max_yaw_rate_rad_s) &&
            config.position_loop.max_yaw_rate_rad_s > 0.0,
        "position_loop.max_yaw_rate_rad_s must be positive");
    require(
        std::isfinite(config.position_loop.yaw_rate_kp) &&
            config.position_loop.yaw_rate_kp >= 0.0,
        "position_loop.yaw_rate_kp cannot be negative");
    require(
        std::isfinite(config.position_loop.max_position_error_m) &&
            config.position_loop.max_position_error_m > 0.0,
        "position_loop.max_position_error_m must be positive");
    require(
        std::isfinite(config.position_loop.gravity_mps2) &&
            config.position_loop.gravity_mps2 > 0.0,
        "position_loop.gravity_mps2 must be positive");
    const auto valid_rc_channel = [](int channel) {
        return channel >= 1 && channel <= 18;
    };
    require(
        valid_rc_channel(config.rc_mapping.roll) &&
            valid_rc_channel(config.rc_mapping.pitch) &&
            valid_rc_channel(config.rc_mapping.throttle) &&
            valid_rc_channel(config.rc_mapping.yaw),
        "flight RC channel mapping must use 1-based channels 1..18");
    const std::array<int, 4> mapped_channels{
        config.rc_mapping.roll,
        config.rc_mapping.pitch,
        config.rc_mapping.throttle,
        config.rc_mapping.yaw,
    };
    for (std::size_t first = 0; first < mapped_channels.size(); ++first) {
        for (std::size_t second = first + 1;
             second < mapped_channels.size(); ++second) {
            require(
                mapped_channels[first] != mapped_channels[second],
                "flight RC channel mapping entries must be unique");
        }
    }
    require(
        config.rc_mapping.handoff_hold_ms >= 40 &&
            config.rc_mapping.handoff_hold_ms <= 2000,
        "flight.bumpless_hold_ms must be in [40, 2000]");
    require(
        config.rc_mapping.throttle_transition_ms >= 100 &&
            config.rc_mapping.throttle_transition_ms <= 2000,
        "flight.throttle_transition_ms must be in [100, 2000]");
    require(
        std::isfinite(
            config.rc_mapping.handoff_max_vertical_speed_mps) &&
            config.rc_mapping.handoff_max_vertical_speed_mps >= 0.01 &&
            config.rc_mapping.handoff_max_vertical_speed_mps <= 2.0,
        "flight.handoff_max_vertical_speed_mps must be in [0.01, 2.0]");
    require(
        std::isfinite(config.rc_mapping.handoff_max_throttle_delta) &&
            config.rc_mapping.handoff_max_throttle_delta >= 0.01 &&
            config.rc_mapping.handoff_max_throttle_delta <= 0.5,
        "flight.handoff_max_throttle_delta must be in [0.01, 0.5]");
    if (config.flight_log_enabled) {
        require(
            !config.flight_log_directory.empty(),
            "flight_log.directory cannot be empty");
    }
    require(
        config.flight_log_stale_timeout_ms >= 100 &&
            config.flight_log_stale_timeout_ms <= 30000,
        "flight_log.telemetry_stale_timeout_ms must be in [100, 30000]");
    require(
        config.simulated_pose_rate_hz >= 10.0 &&
            config.simulated_pose_rate_hz <= 500.0,
        "simulation.pose_rate_hz must be in [10, 500]");
    require(
        config.pose_source == "simulated" || config.pose_source == "vrpn",
        "pose.source must be 'simulated' or 'vrpn'");
    require(!config.vrpn_endpoint.empty(), "vrpn.endpoint cannot be empty");
    require(
        config.vrpn_endpoint.find('@') != std::string::npos,
        "vrpn.endpoint must use device@host:port form");
    require(
        config.vrpn_sensor_id >= -1,
        "vrpn.sensor_id must be -1/null or a non-negative sensor id");
    require(
        config.vrpn_position_scale > 0.0 &&
            std::isfinite(config.vrpn_position_scale),
        "vrpn.position_scale must be positive");
    require(
        is_finite(config.vrpn_axes_from_vrpn),
        "vrpn.axes_from_vrpn must contain finite numbers");
    const Mat3 gram =
        transpose(config.vrpn_axes_from_vrpn) * config.vrpn_axes_from_vrpn;
    for (std::size_t row = 0; row < 3U; ++row) {
        for (std::size_t column = 0; column < 3U; ++column) {
            const double expected = row == column ? 1.0 : 0.0;
            require(
                std::abs(gram[row * 3U + column] - expected) < 1e-6,
                "vrpn.axes_from_vrpn must be orthonormal");
        }
    }
    require(
        config.flight_transport == "loopback" ||
            config.flight_transport == "udp",
        "flight.transport must be 'loopback' or 'udp'");
    if (config.flight_transport == "loopback") {
        require(
            config.flight_protocol == "null" ||
                config.flight_protocol.empty(),
            "flight.protocol must be null for loopback transport");
    } else {
        require(
            config.flight_protocol == "gs_link_v1",
            "flight.protocol must be 'gs_link_v1' for UDP transport");
        require(
            !config.esp32_ip.empty(),
            "flight.remote_ip cannot be empty");
        require(
            config.esp32_port.has_value(),
            "flight.remote_port is required for UDP transport");
        require(
            !config.gs_link_bind_ip.empty(),
            "flight.local_ip cannot be empty");
        require(
            config.gs_link_bind_port.has_value(),
            "flight.local_port is required for UDP transport");
        require(
            config.gs_link_telemetry_timeout_ms >= 50U &&
                config.gs_link_telemetry_timeout_ms <= 5000U,
            "flight.telemetry_timeout_ms must be in [50, 5000]");
        require(
            config.gs_link_heartbeat_timeout_ms >= 50U &&
                config.gs_link_heartbeat_timeout_ms <= 5000U,
            "flight.heartbeat_timeout_ms must be in [50, 5000]");
        require(
            config.gs_link_heartbeat_rate_hz >= 1.0 &&
                config.gs_link_heartbeat_rate_hz <= 50.0,
            "flight.heartbeat_rate_hz must be in [1, 50]");
    }
}

}  // namespace dock_ground
