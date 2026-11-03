#include "dock_ground/flight_logger.hpp"

#include "dock_ground/json.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace dock_ground {
namespace {

constexpr std::size_t kMaximumQueuedSamples = 16384U;
constexpr std::uint64_t kFlushIntervalNs = 1'000'000'000ULL;
constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] std::tm utc_time(std::int64_t wall_time_ns) {
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::nanoseconds(wall_time_ns));
    const std::time_t raw = static_cast<std::time_t>(seconds.count());
    std::tm result{};
    (void)::gmtime_r(&raw, &result);
    return result;
}

[[nodiscard]] std::string utc_string(std::int64_t wall_time_ns) {
    const std::tm time = utc_time(wall_time_ns);
    char buffer[32]{};
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &time);
    return buffer;
}

[[nodiscard]] std::string file_timestamp(std::int64_t wall_time_ns) {
    const std::tm time = utc_time(wall_time_ns);
    const auto nanoseconds =
        std::chrono::nanoseconds(wall_time_ns).count() %
        1'000'000'000LL;
    char buffer[40]{};
    std::strftime(buffer, sizeof(buffer), "%Y%m%dT%H%M%S", &time);
    std::ostringstream output;
    output << buffer << '_' << std::setw(3) << std::setfill('0')
           << nanoseconds / 1'000'000LL << 'Z';
    return output.str();
}

[[nodiscard]] std::string safe_component(std::string_view input) {
    std::string output;
    output.reserve(input.size());
    for (const char character : input) {
        const bool safe =
            (character >= 'a' && character <= 'z') ||
            (character >= 'A' && character <= 'Z') ||
            (character >= '0' && character <= '9') ||
            character == '-' || character == '_';
        output.push_back(safe ? character : '_');
    }
    return output.empty() ? "aircraft" : output;
}

void emit_csv_string(std::ostream& output, std::string_view value) {
    output << '"';
    for (const char character : value) {
        if (character == '"') {
            output << "\"\"";
        } else if (character == '\r' || character == '\n') {
            output << ' ';
        } else {
            output << character;
        }
    }
    output << '"';
}

[[nodiscard]] bool flag_set(
    std::uint16_t flags, gs_link::StatusFlag flag) {
    return (flags & static_cast<std::uint16_t>(flag)) != 0U;
}

[[nodiscard]] double degrees(double radians) {
    return radians * 180.0 / kPi;
}

[[nodiscard]] const char* control_source(
    const FlightLoggerOptions& options, const FlightLogSample& sample) {
    if (!options.udp_flight_transport) {
        return "LOOPBACK";
    }
    if (sample.command_valid && sample.deadman_active &&
        flag_set(
            sample.telemetry.status_flags,
            gs_link::StatusFlag::GsModeActive)) {
        if (sample.rc_passthrough_active) {
            return "GS_RC_PASSTHROUGH";
        }
        return "GROUND_STATION";
    }
    if (sample.command_valid && sample.deadman_active) {
        return "RC_PASSTHROUGH";
    }
    return "RC";
}

const std::vector<std::string>& csv_header() {
    static const std::vector<std::string> header{
        "schema", "wall_time_utc", "mono_ns", "aircraft_id",
        "control_state", "control_source", "telemetry_fresh",
        "heartbeat_fresh", "gs_transport_connected", "uplink_allowed",
        "telemetry_age_ms", "heartbeat_age_ms",
        "command_link_connected", "command_commands_sent",
        "command_send_errors", "command_last_error",
        "command_last_errno",
        "armed_known", "armed", "gs_mode_active", "rc_valid",
        "rc_switch_selected", "failsafe", "command_valid", "deadman",
        "rc_passthrough", "handoff_active", "handoff_hold",
        "handoff_vertical_speed_mps", "handoff_throttle_blend",
        "pose_valid", "pose_sequence", "pose_age_ms",
        "position_x_m", "position_y_m", "position_z_m",
        "pose_q_w", "pose_q_x", "pose_q_y", "pose_q_z",
        "velocity_x_mps", "velocity_y_mps", "velocity_z_mps",
        "target_x_m", "target_y_m", "target_z_m", "target_yaw_rad",
        "rc_base_roll", "rc_base_pitch", "rc_base_yaw",
        "rc_base_throttle", "cmd_valid", "cmd_roll", "cmd_pitch",
        "cmd_yaw", "cmd_throttle", "cmd_q_w", "cmd_q_x", "cmd_q_y",
        "cmd_q_z", "pid_x", "pid_y", "pid_z",
        "fc_roll_deg", "fc_pitch_deg", "fc_yaw_deg",
        "motor1_rpm", "motor2_rpm", "battery_voltage_v",
        "battery_current_a", "battery_remaining_pct",
        "rc_channel_count",
        "rc_ch1", "rc_ch2", "rc_ch3", "rc_ch4", "rc_ch5", "rc_ch6",
        "rc_ch7", "rc_ch8", "rc_ch9", "rc_ch10", "rc_ch11",
        "rc_ch12", "rc_ch13", "rc_ch14", "rc_ch15", "rc_ch16",
        "rc_ch17", "rc_ch18",
        "pid_x_kp", "pid_x_ki", "pid_x_kd",
        "pid_y_kp", "pid_y_ki", "pid_y_kd",
        "pid_z_kp", "pid_z_ki", "pid_z_kd",
        "max_accel_mps2", "max_tilt_deg", "min_throttle",
        "hover_throttle", "max_throttle", "z_velocity_damping",
        "max_yaw_rate_deg_s",
        "yaw_rate_kp", "yaw_rate_kd",
        "yaw_rate_slew_rate_rad_s2", "fc_yaw_rate_scale_rad_s",
        "max_position_error_m", "gravity_mps2",
        "rx_packets", "tx_packets", "crc_errors", "length_errors",
        "version_errors", "sequence_gaps", "tx_retries",
        "ack_timeouts", "send_errors", "fc_state", "fc_error_code",
        "heartbeat_status_flags",
        "position_mode", "position_mode_active",
        "position_mode_scheduling_enabled",
        "position_mode_transition_active",
        "position_mode_error_xy_m", "position_mode_error_z_m",
        "position_mode_speed_xy_mps", "position_mode_speed_z_mps",
        "active_pid_x_kp", "active_pid_x_ki", "active_pid_x_kd",
        "active_pid_y_kp", "active_pid_y_ki", "active_pid_y_kd",
        "active_pid_z_kp", "active_pid_z_ki", "active_pid_z_kd",
        "command_target_rate_hz", "command_actual_rate_hz",
        "command_last_interval_ms", "command_suppressed_sends",
        "pose_received_mono_ns", "pose_source_time_ns",
        "pose_source_time_present", "pose_source_time_valid",
        "pose_source_clock_delta_ms", "pose_source_age_ms_at_receive",
        "pose_local_age_ms", "pose_source_age_ms", "pose_fresh",
        "pose_arrival_interval_ms", "pose_source_interval_ms",
        "pose_source_time_regression",
        "pose_raw_x", "pose_raw_y", "pose_raw_z",
        "pose_raw_q_w", "pose_raw_q_x", "pose_raw_q_y", "pose_raw_q_z",
        "pose_source_sensor_id",
        "vrpn_connected", "vrpn_transport_connected",
        "vrpn_callbacks_received", "vrpn_samples_accepted",
        "vrpn_samples_rejected", "vrpn_unselected_sensor_samples",
        "vrpn_invalid_position_samples",
        "vrpn_invalid_orientation_samples",
        "vrpn_duplicate_timestamp_samples",
        "vrpn_timestamp_regression_samples",
        "vrpn_long_gap_count", "vrpn_last_arrival_gap_ms",
        "vrpn_max_arrival_gap_ms", "vrpn_connect_count",
        "vrpn_disconnect_count", "vrpn_last_received_mono_ns",
        "vrpn_expected_rate_hz",
        "rc_gate_ready", "handoff_reset", "handoff_reset_reason",
        "handoff_error",
        "loop_target_period_ms", "loop_period_ms", "loop_work_ms",
        "loop_jitter_ms", "loop_p99_work_ms", "loop_p99_jitter_ms",
        "loop_overruns",
    };
    return header;
}

}  // namespace

struct FlightLogger::Impl {
    explicit Impl(FlightLoggerOptions input_options)
        : options(std::move(input_options)) {
        const std::lock_guard lock(status_mutex);
        status.enabled = options.enabled;
    }

    ~Impl() {
        stop();
    }

    bool start(std::string& error) {
        if (!options.enabled) {
            error.clear();
            return true;
        }
        if (running.load(std::memory_order_relaxed)) {
            return true;
        }
        std::error_code filesystem_error;
        std::filesystem::create_directories(
            options.directory, filesystem_error);
        if (filesystem_error) {
            error = "cannot create flight log directory '" +
                    options.directory + "': " +
                    filesystem_error.message();
            set_error(error);
            return false;
        }
        running.store(true, std::memory_order_relaxed);
        try {
            thread = std::thread([this] { run(); });
        } catch (const std::exception& exception) {
            running.store(false, std::memory_order_relaxed);
            error = "cannot start flight logger: " +
                    std::string(exception.what());
            set_error(error);
            return false;
        }
        error.clear();
        return true;
    }

    void stop() {
        if (running.exchange(false, std::memory_order_relaxed) &&
            thread.joinable()) {
            queue_cv.notify_all();
            thread.join();
        } else if (thread.joinable()) {
            thread.join();
        }
        const std::lock_guard lock(status_mutex);
        status.recording = false;
    }

    void observe(const FlightLogSample& sample) noexcept {
        if (!options.enabled) {
            return;
        }
        {
            const std::lock_guard lock(queue_mutex);
            if (queue.size() >= kMaximumQueuedSamples) {
                const std::lock_guard status_lock(status_mutex);
                ++status.frames_dropped;
                return;
            }
            queue.push_back(sample);
        }
        queue_cv.notify_one();
    }

    [[nodiscard]] FlightLogStatus status_snapshot() const {
        const std::lock_guard lock(status_mutex);
        return status;
    }

    void set_error(std::string message) {
        const std::lock_guard lock(status_mutex);
        status.last_error = std::move(message);
    }

    void update_status_recording(bool recording_value) {
        const std::lock_guard lock(status_mutex);
        status.recording = recording_value;
    }

    void set_current_file(const std::filesystem::path& path) {
        const std::lock_guard lock(status_mutex);
        status.current_file = path.string();
    }

    void increment_frames(std::uint64_t count = 1U) {
        const std::lock_guard lock(status_mutex);
        status.frames_written += count;
    }

    void increment_sessions() {
        const std::lock_guard lock(status_mutex);
        ++status.sessions_written;
        status.recording = false;
    }

    bool open_session(const FlightLogSample& sample) {
        const std::string stamp = file_timestamp(sample.wall_time_ns);
        ++session_serial;
        const std::string stem =
            "flight_" + stamp + '_' +
            safe_component(options.aircraft_id) + '_' +
            std::to_string(session_serial);
        csv_path = std::filesystem::path(options.directory) /
                   (stem + ".csv");
        metadata_path = std::filesystem::path(options.directory) /
                        (stem + ".json");
        summary_path = std::filesystem::path(options.directory) /
                       (stem + ".summary.json");
        stream.open(csv_path, std::ios::out | std::ios::trunc);
        if (!stream) {
            set_error("cannot open flight CSV: " + csv_path.string());
            return false;
        }

        const auto& header = csv_header();
        for (std::size_t index = 0; index < header.size(); ++index) {
            if (index != 0U) {
                stream << ',';
            }
            stream << header[index];
        }
        stream << '\n';

        std::ofstream metadata(
            metadata_path, std::ios::out | std::ios::trunc);
        if (!metadata) {
            set_error(
                "cannot open flight metadata: " +
                metadata_path.string());
            stream.close();
            return false;
        }
        metadata << std::setprecision(12)
                 << "{\n"
                 << "  \"schema\": 2,\n"
                 << "  \"session_start_utc\": \""
                 << utc_string(sample.wall_time_ns) << "\",\n"
                 << "  \"aircraft_id\": \""
                 << json::escape(options.aircraft_id) << "\",\n"
                 << "  \"aircraft_name\": \""
                 << json::escape(options.aircraft_name) << "\",\n"
                 << "  \"control_transport\": \""
                 << (options.udp_flight_transport ? "GS-Link UDP"
                                                  : "loopback")
                 << "\",\n"
                 << "  \"control\": {\"rate_hz\": "
                 << options.control_rate_hz
                 << ", \"warning_pose_age_ms\": "
                 << options.warning_pose_age_ms
                 << ", \"max_pose_age_ms\": "
                 << options.max_pose_age_ms << "},\n"
                 << "  \"flight_log\": {\"pre_roll_s\": "
                 << options.pre_roll_s
                 << ", \"telemetry_stale_timeout_ms\": "
                 << options.telemetry_stale_timeout_ms << "},\n"
                 << "  \"pose_source\": {\"type\": \""
                 << json::escape(options.pose_source)
                 << "\", \"endpoint\": \""
                 << json::escape(options.pose_endpoint)
                 << "\", \"sensor_id\": " << options.pose_sensor_id
                 << ", \"position_scale\": "
                 << options.pose_position_scale
                 << ", \"expected_rate_hz\": "
                 << options.pose_expected_rate_hz
                 << ", \"auto_origin\": "
                 << (options.pose_auto_origin ? "true" : "false")
                 << ", \"trust_source_timestamp\": "
                 << (options.pose_trust_source_timestamp ? "true" : "false")
                 << ", \"axes_from_vrpn\": [["
                 << options.pose_axes_from_vrpn[0] << ", "
                 << options.pose_axes_from_vrpn[1] << ", "
                 << options.pose_axes_from_vrpn[2] << "], ["
                 << options.pose_axes_from_vrpn[3] << ", "
                 << options.pose_axes_from_vrpn[4] << ", "
                 << options.pose_axes_from_vrpn[5] << "], ["
                 << options.pose_axes_from_vrpn[6] << ", "
                 << options.pose_axes_from_vrpn[7] << ", "
                 << options.pose_axes_from_vrpn[8] << "]]},\n"
                 << "  \"rc_mapping\": {\"roll\": "
                 << options.rc_mapping.roll << ", \"pitch\": "
                 << options.rc_mapping.pitch << ", \"throttle\": "
                 << options.rc_mapping.throttle << ", \"yaw\": "
                 << options.rc_mapping.yaw
                 << ", \"handoff_hold_ms\": "
                 << options.rc_mapping.handoff_hold_ms
                 << ", \"throttle_transition_ms\": "
                 << options.rc_mapping.throttle_transition_ms
                 << ", \"handoff_max_vertical_speed_mps\": "
                 << options.rc_mapping.handoff_max_vertical_speed_mps
                 << ", \"handoff_max_throttle_delta\": "
                 << options.rc_mapping.handoff_max_throttle_delta
                 << "},\n"
                 << "  \"initial_tuning\": {\n"
                 << "    \"x\": [" << sample.tuning.x.kp << ", "
                 << sample.tuning.x.ki << ", " << sample.tuning.x.kd
                 << "], \"y\": [" << sample.tuning.y.kp << ", "
                 << sample.tuning.y.ki << ", " << sample.tuning.y.kd
                 << "], \"z\": [" << sample.tuning.z.kp << ", "
                 << sample.tuning.z.ki << ", " << sample.tuning.z.kd
                 << "],\n"
                 << "    \"approach\": {"
                 << "\"x\": [" << sample.tuning.approach.x.kp << ", "
                 << sample.tuning.approach.x.ki << ", "
                 << sample.tuning.approach.x.kd << "], "
                 << "\"y\": [" << sample.tuning.approach.y.kp << ", "
                 << sample.tuning.approach.y.ki << ", "
                 << sample.tuning.approach.y.kd << "], "
                 << "\"z\": [" << sample.tuning.approach.z.kp << ", "
                 << sample.tuning.approach.z.ki << ", "
                 << sample.tuning.approach.z.kd << "]},\n"
                 << "    \"precision_hold\": {"
                 << "\"x\": ["
                 << sample.tuning.precision_hold.x.kp << ", "
                 << sample.tuning.precision_hold.x.ki << ", "
                 << sample.tuning.precision_hold.x.kd << "], "
                 << "\"y\": ["
                 << sample.tuning.precision_hold.y.kp << ", "
                 << sample.tuning.precision_hold.y.ki << ", "
                 << sample.tuning.precision_hold.y.kd << "], "
                 << "\"z\": ["
                 << sample.tuning.precision_hold.z.kp << ", "
                 << sample.tuning.precision_hold.z.ki << ", "
                 << sample.tuning.precision_hold.z.kd << "]},\n"
                 << "    \"mode_scheduling\": {\"enabled\": "
                 << (sample.tuning.mode_scheduling.enabled ? "true" : "false")
                 << ", \"brake_enter_distance_m\": "
                 << sample.tuning.mode_scheduling.brake_enter_distance_m
                 << ", \"brake_exit_distance_m\": "
                 << sample.tuning.mode_scheduling.brake_exit_distance_m
                 << ", \"brake_enter_vertical_error_m\": "
                 << sample.tuning.mode_scheduling
                        .brake_enter_vertical_error_m
                 << ", \"brake_exit_vertical_error_m\": "
                 << sample.tuning.mode_scheduling
                        .brake_exit_vertical_error_m
                 << ", \"hold_enter_distance_m\": "
                 << sample.tuning.mode_scheduling.hold_enter_distance_m
                 << ", \"hold_exit_distance_m\": "
                 << sample.tuning.mode_scheduling.hold_exit_distance_m
                 << ", \"hold_enter_vertical_error_m\": "
                 << sample.tuning.mode_scheduling
                        .hold_enter_vertical_error_m
                 << ", \"hold_exit_vertical_error_m\": "
                 << sample.tuning.mode_scheduling
                        .hold_exit_vertical_error_m
                 << ", \"hold_enter_speed_mps\": "
                 << sample.tuning.mode_scheduling.hold_enter_speed_mps
                 << ", \"hold_exit_speed_mps\": "
                 << sample.tuning.mode_scheduling.hold_exit_speed_mps
                 << ", \"hold_enter_vertical_speed_mps\": "
                 << sample.tuning.mode_scheduling
                        .hold_enter_vertical_speed_mps
                 << ", \"hold_exit_vertical_speed_mps\": "
                 << sample.tuning.mode_scheduling
                        .hold_exit_vertical_speed_mps
                 << ", \"target_motion_speed_mps\": "
                 << sample.tuning.mode_scheduling.target_motion_speed_mps
                 << ", \"hold_enter_dwell_s\": "
                 << sample.tuning.mode_scheduling.hold_enter_dwell_s
                 << ", \"hold_exit_dwell_s\": "
                 << sample.tuning.mode_scheduling.hold_exit_dwell_s
                 << ", \"gain_transition_time_s\": "
                 << sample.tuning.mode_scheduling.gain_transition_time_s
                 << "},\n"
                 << "    \"max_accel_mps2\": "
                 << sample.tuning.max_accel_mps2
                 << ", \"max_tilt_rad\": "
                 << sample.tuning.max_tilt_rad
                 << ", \"hover_throttle\": "
                 << sample.tuning.hover_throttle
                 << ", \"z_velocity_damping\": "
                 << sample.tuning.z_velocity_damping << "\n"
                 << "  }\n"
                 << "}\n";
        if (!metadata) {
            set_error(
                "cannot write flight metadata: " +
                metadata_path.string());
            stream.close();
            return false;
        }

        session_frames = 0;
        session_start_ns = sample.mono_ns;
        last_flush_ns = sample.mono_ns;
        reset_session_stats(sample);
        set_current_file(csv_path);
        update_status_recording(true);
        return true;
    }

    void close_session(
        std::string end_reason, std::uint64_t end_mono_ns,
        std::int64_t end_wall_ns) {
        if (stream.is_open()) {
            stream.flush();
            stream.close();
        }
        std::ofstream summary(
            summary_path, std::ios::out | std::ios::trunc);
        if (summary) {
            const double duration_s =
                end_mono_ns >= session_start_ns
                    ? static_cast<double>(end_mono_ns - session_start_ns) /
                          1'000'000'000.0
                    : 0.0;
            summary << std::setprecision(12)
                    << "{\n"
                    << "  \"schema\": 2,\n"
                    << "  \"end_reason\": \""
                    << json::escape(end_reason) << "\",\n"
                    << "  \"session_end_utc\": \""
                    << utc_string(end_wall_ns) << "\",\n"
                    << "  \"duration_s\": " << duration_s << ",\n"
                    << "  \"frames\": " << session_frames << ",\n"
                    << "  \"pose\": {\"frames\": "
                    << session_pose_frames
                    << ", \"unique_samples\": "
                    << session_unique_pose_samples
                    << ", \"hold_frames\": "
                    << session_pose_hold_frames
                    << ", \"stale_frames\": "
                    << session_pose_stale_frames
                    << ", \"max_age_ms\": " << session_max_pose_age_ms
                    << ", \"max_arrival_interval_ms\": "
                    << session_max_arrival_interval_ms
                    << ", \"min_altitude_m\": "
                    << session_min_altitude_m
                    << ", \"max_altitude_m\": "
                    << session_max_altitude_m << "},\n"
                    << "  \"vrpn\": {\"callbacks\": "
                    << source_delta(
                           session_first_source_status.callbacks_received,
                           session_last_source_status.callbacks_received)
                    << ", \"accepted\": "
                    << source_delta(
                           session_first_source_status.messages_received,
                           session_last_source_status.messages_received)
                    << ", \"rejected\": "
                    << source_delta(
                           session_first_source_status.samples_rejected,
                           session_last_source_status.samples_rejected)
                    << ", \"unselected_sensor\": "
                    << source_delta(
                           session_first_source_status
                               .unselected_sensor_samples,
                           session_last_source_status
                               .unselected_sensor_samples)
                    << ", \"invalid_position\": "
                    << source_delta(
                           session_first_source_status
                               .invalid_position_samples,
                           session_last_source_status
                               .invalid_position_samples)
                    << ", \"invalid_orientation\": "
                    << source_delta(
                           session_first_source_status
                               .invalid_orientation_samples,
                           session_last_source_status
                               .invalid_orientation_samples)
                    << ", \"duplicate_timestamp\": "
                    << source_delta(
                           session_first_source_status
                               .duplicate_timestamp_samples,
                           session_last_source_status
                               .duplicate_timestamp_samples)
                    << ", \"timestamp_regression\": "
                    << source_delta(
                           session_first_source_status
                               .timestamp_regression_samples,
                           session_last_source_status
                               .timestamp_regression_samples)
                    << ", \"long_gaps\": "
                    << source_delta(
                           session_first_source_status.long_gap_count,
                           session_last_source_status.long_gap_count)
                    << ", \"last_max_gap_ms\": "
                    << session_last_source_status.max_arrival_gap_ms
                    << "},\n"
                    << "  \"safety\": {\"handoff_resets\": "
                    << session_handoff_reset_count
                    << ", \"first_handoff_reset_utc\": \""
                    << json::escape(session_first_handoff_reset_utc)
                    << "\", \"first_handoff_reset_reason\": \""
                    << json::escape(session_first_handoff_reset_reason)
                    << "\", \"failsafe_frames\": "
                    << session_failsafe_frames
                    << ", \"first_failsafe_utc\": \""
                    << json::escape(session_first_failsafe_utc)
                    << "\"},\n"
                    << "  \"control_loop\": {\"overruns\": "
                    << session_loop_overruns
                    << ", \"max_period_ms\": "
                    << session_max_loop_period_ms
                    << ", \"max_work_ms\": "
                    << session_max_loop_work_ms
                    << ", \"max_jitter_ms\": "
                    << session_max_loop_jitter_ms
                    << ", \"logger_frames_dropped\": "
                    << source_delta(
                           session_first_logger_drops,
                           status_snapshot().frames_dropped)
                    << "}\n"
                    << "}\n";
        }
        increment_sessions();
        arm_latched = false;
    }

    static std::uint64_t source_delta(
        std::uint64_t initial, std::uint64_t final) {
        return final >= initial ? final - initial : final;
    }

    void reset_session_stats(const FlightLogSample& sample) {
        session_pose_frames = 0;
        session_unique_pose_samples = 0;
        session_pose_hold_frames = 0;
        session_pose_stale_frames = 0;
        session_max_pose_age_ms = -1.0;
        session_max_arrival_interval_ms = 0.0;
        session_min_altitude_m = -1.0;
        session_max_altitude_m = -1.0;
        session_handoff_reset_count = 0;
        session_failsafe_frames = 0;
        session_loop_overruns = 0;
        session_max_loop_period_ms = 0.0;
        session_max_loop_work_ms = 0.0;
        session_max_loop_jitter_ms = 0.0;
        session_first_source_status = sample.source_status;
        session_last_source_status = sample.source_status;
        session_first_loop_overruns = sample.loop_metrics.overruns;
        session_first_logger_drops =
            status_snapshot().frames_dropped;
        session_first_handoff_reset_utc.clear();
        session_first_handoff_reset_reason.clear();
        session_first_failsafe_utc.clear();
        have_session_pose_sequence = false;
        last_session_pose_sequence = 0;
    }

    void update_session_stats(const FlightLogSample& sample) {
        ++session_pose_frames;
        session_last_source_status = sample.source_status;
        const bool derived_pose_fresh =
            sample.pose.valid &&
            std::isfinite(sample.pose_age_ms) &&
            sample.pose_age_ms <= options.max_pose_age_ms;
        if (!sample.pose_fresh && !derived_pose_fresh) {
            ++session_pose_stale_frames;
        }
        if (sample.pose.valid) {
            if (!have_session_pose_sequence ||
                sample.pose.sequence != last_session_pose_sequence) {
                ++session_unique_pose_samples;
                have_session_pose_sequence = true;
                last_session_pose_sequence = sample.pose.sequence;
            } else {
                ++session_pose_hold_frames;
            }
            session_max_pose_age_ms =
                std::max(session_max_pose_age_ms, sample.pose_age_ms);
            const double arrival_interval_ms =
                static_cast<double>(sample.pose.arrival_interval_ns) /
                1'000'000.0;
            session_max_arrival_interval_ms =
                std::max(
                    session_max_arrival_interval_ms,
                    arrival_interval_ms);
            const double altitude = sample.pose.position_xyz_m.z;
            session_min_altitude_m =
                session_min_altitude_m < 0.0
                    ? altitude
                    : std::min(session_min_altitude_m, altitude);
            session_max_altitude_m =
                session_max_altitude_m < 0.0
                    ? altitude
                    : std::max(session_max_altitude_m, altitude);
        }
        if (sample.handoff_reset) {
            ++session_handoff_reset_count;
            if (session_first_handoff_reset_utc.empty()) {
                session_first_handoff_reset_utc =
                    utc_string(sample.wall_time_ns);
                session_first_handoff_reset_reason =
                    sample.handoff_reset_reason;
            }
        }
        if (flag_set(
                sample.telemetry.status_flags,
                gs_link::StatusFlag::Failsafe)) {
            ++session_failsafe_frames;
            if (session_first_failsafe_utc.empty()) {
                session_first_failsafe_utc =
                    utc_string(sample.wall_time_ns);
            }
        }
        session_loop_overruns = source_delta(
            session_first_loop_overruns,
            sample.loop_metrics.overruns);
        session_max_loop_period_ms =
            std::max(
                session_max_loop_period_ms,
                sample.loop_metrics.last_period_ms);
        session_max_loop_work_ms =
            std::max(
                session_max_loop_work_ms,
                sample.loop_metrics.work_ms);
        session_max_loop_jitter_ms =
            std::max(
                session_max_loop_jitter_ms,
                sample.loop_metrics.jitter_ms);
    }

    void buffer_pre_arm(const FlightLogSample& sample) {
        pre_arm_buffer.push_back(sample);
        const std::uint64_t pre_roll_ns =
            static_cast<std::uint64_t>(
                std::max(0.0, options.pre_roll_s) * 1'000'000'000.0);
        while (pre_arm_buffer.size() > 1U &&
               sample.mono_ns >= pre_arm_buffer.front().mono_ns &&
               sample.mono_ns - pre_arm_buffer.front().mono_ns >
                   pre_roll_ns) {
            pre_arm_buffer.pop_front();
        }
    }

    void write_row(const FlightLogSample& sample) {
        if (!stream.is_open()) {
            return;
        }
        update_session_stats(sample);
        const bool armed_known =
            sample.telemetry.valid && sample.telemetry_fresh;
        const bool armed =
            armed_known &&
            flag_set(
                sample.telemetry.status_flags,
                gs_link::StatusFlag::Armed);
        const auto& flags = sample.telemetry.status_flags;
        const auto emit_double = [this](double value) {
            if (std::isfinite(value)) {
                stream << value;
            } else {
                stream << "nan";
            }
        };
        const auto emit_q = [this, &emit_double](const Quaternion& q) {
            emit_double(q.w);
            stream << ',';
            emit_double(q.x);
            stream << ',';
            emit_double(q.y);
            stream << ',';
            emit_double(q.z);
        };

        stream << "2," << utc_string(sample.wall_time_ns) << ','
               << sample.mono_ns << ',';
        emit_csv_string(stream, options.aircraft_id);
        stream << ',' << to_string(sample.control_state) << ','
               << control_source(options, sample) << ','
               << sample.telemetry_fresh << ','
               << sample.heartbeat_fresh << ','
               << sample.gs_transport_connected << ','
               << sample.uplink_allowed << ',';
        emit_double(sample.telemetry_age_ms);
        stream << ',';
        emit_double(sample.heartbeat_age_ms);
        stream << ',' << sample.command_link.connected << ','
               << sample.command_link.commands_sent << ','
               << sample.command_link.send_errors << ',';
        emit_csv_string(stream, sample.command_link.last_error);
        stream << ',' << sample.command_link.last_errno << ','
               << armed_known << ',' << armed << ','
               << flag_set(flags, gs_link::StatusFlag::GsModeActive)
               << ','
               << flag_set(flags, gs_link::StatusFlag::RcValid) << ','
               << flag_set(
                      flags, gs_link::StatusFlag::RcSwitchSelected)
               << ','
               << flag_set(flags, gs_link::StatusFlag::Failsafe) << ','
               << sample.command_valid << ',' << sample.deadman_active
               << ',' << sample.rc_passthrough_active << ','
               << sample.handoff_active << ','
               << sample.handoff_hold << ',';
        emit_double(sample.handoff_vertical_speed_mps);
        stream << ',';
        emit_double(sample.handoff_throttle_blend);
        stream << ',' << sample.pose.valid << ','
               << sample.pose.sequence << ',';
        emit_double(
            std::isfinite(sample.pose_age_ms)
                ? sample.pose_age_ms
                : -1.0);
        stream << ',';
        emit_double(sample.pose.position_xyz_m.x);
        stream << ',';
        emit_double(sample.pose.position_xyz_m.y);
        stream << ',';
        emit_double(sample.pose.position_xyz_m.z);
        stream << ',';
        emit_q(sample.pose.q_body_to_world);
        stream << ',';
        emit_double(estimated_velocity.x);
        stream << ',';
        emit_double(estimated_velocity.y);
        stream << ',';
        emit_double(estimated_velocity.z);
        stream << ',';
        emit_double(sample.target.position_xyz_m.x);
        stream << ',';
        emit_double(sample.target.position_xyz_m.y);
        stream << ',';
        emit_double(sample.target.position_xyz_m.z);
        stream << ',';
        emit_double(sample.target.yaw_rad);
        stream << ',';
        emit_double(sample.rc_base.roll);
        stream << ',';
        emit_double(sample.rc_base.pitch);
        stream << ',';
        emit_double(sample.rc_base.yaw);
        stream << ',';
        emit_double(sample.rc_base.throttle);
        stream << ',' << sample.command.manual.valid << ',';
        emit_double(sample.command.manual.roll);
        stream << ',';
        emit_double(sample.command.manual.pitch);
        stream << ',';
        emit_double(sample.command.manual.yaw);
        stream << ',';
        emit_double(sample.command.manual.throttle);
        stream << ',';
        emit_q(sample.command.q_body_to_world_desired);
        stream << ',';
        emit_double(sample.command.position_loop_output_xyz.x);
        stream << ',';
        emit_double(sample.command.position_loop_output_xyz.y);
        stream << ',';
        emit_double(sample.command.position_loop_output_xyz.z);
        stream << ',';
        emit_double(sample.telemetry.roll_cdeg / 100.0);
        stream << ',';
        emit_double(sample.telemetry.pitch_cdeg / 100.0);
        stream << ',';
        emit_double(sample.telemetry.yaw_cdeg / 100.0);
        stream << ',' << sample.telemetry.motor1_rpm << ','
               << sample.telemetry.motor2_rpm << ',';
        emit_double(sample.telemetry.battery_voltage_mv / 1000.0);
        stream << ',';
        emit_double(sample.telemetry.battery_current_ca / 100.0);
        stream << ',' << static_cast<unsigned int>(
                          sample.telemetry.battery_remaining)
               << ',' << static_cast<unsigned int>(
                          sample.telemetry.rc_channel_count);
        for (std::size_t channel = 0;
             channel < gs_link::kMaximumRcChannels; ++channel) {
            const int value =
                channel < sample.telemetry.rc_channel_count
                    ? static_cast<int>(
                          sample.telemetry.rc_channels[channel])
                    : -128;
            stream << ',' << value;
        }
        stream << ',';
        emit_double(sample.tuning.x.kp);
        stream << ',';
        emit_double(sample.tuning.x.ki);
        stream << ',';
        emit_double(sample.tuning.x.kd);
        stream << ',';
        emit_double(sample.tuning.y.kp);
        stream << ',';
        emit_double(sample.tuning.y.ki);
        stream << ',';
        emit_double(sample.tuning.y.kd);
        stream << ',';
        emit_double(sample.tuning.z.kp);
        stream << ',';
        emit_double(sample.tuning.z.ki);
        stream << ',';
        emit_double(sample.tuning.z.kd);
        stream << ',';
        emit_double(sample.tuning.max_accel_mps2);
        stream << ',';
        emit_double(degrees(sample.tuning.max_tilt_rad));
        stream << ',';
        emit_double(sample.tuning.min_throttle);
        stream << ',';
        emit_double(sample.tuning.hover_throttle);
        stream << ',';
        emit_double(sample.tuning.max_throttle);
        stream << ',';
        emit_double(sample.tuning.z_velocity_damping);
        stream << ',';
        emit_double(degrees(sample.tuning.max_yaw_rate_rad_s));
        stream << ',';
        emit_double(sample.tuning.yaw_rate_kp);
        stream << ',';
        emit_double(sample.tuning.yaw_rate_kd);
        stream << ',';
        emit_double(sample.tuning.yaw_rate_slew_rate_rad_s2);
        stream << ',';
        emit_double(sample.tuning.fc_yaw_rate_scale_rad_s);
        stream << ',';
        emit_double(sample.tuning.max_position_error_m);
        stream << ',';
        emit_double(sample.tuning.gravity_mps2);
        stream << ',' << sample.counters.rx_packets
               << ',' << sample.counters.tx_packets
               << ',' << sample.counters.crc_errors
               << ',' << sample.counters.length_errors
               << ',' << sample.counters.version_errors
               << ',' << sample.counters.sequence_gaps
               << ',' << sample.counters.tx_retries
               << ',' << sample.counters.ack_timeouts
               << ',' << sample.counters.send_errors
               << ',' << static_cast<unsigned int>(
                          sample.heartbeat.state)
               << ',' << static_cast<unsigned int>(
                          sample.heartbeat.error_code)
               << ',' << sample.heartbeat.status_flags
               << ',' << to_string(sample.regulation.mode)
               << ','
               << sample.regulation.active
               << ','
               << sample.regulation.scheduling_enabled
               << ','
               << sample.regulation.gain_transition_active << ',';
        emit_double(sample.regulation.horizontal_error_m);
        stream << ',';
        emit_double(sample.regulation.vertical_error_m);
        stream << ',';
        emit_double(sample.regulation.horizontal_speed_mps);
        stream << ',';
        emit_double(sample.regulation.vertical_speed_mps);
        stream << ',';
        emit_double(sample.regulation.active_x.kp);
        stream << ',';
        emit_double(sample.regulation.active_x.ki);
        stream << ',';
        emit_double(sample.regulation.active_x.kd);
        stream << ',';
        emit_double(sample.regulation.active_y.kp);
        stream << ',';
        emit_double(sample.regulation.active_y.ki);
        stream << ',';
        emit_double(sample.regulation.active_y.kd);
        stream << ',';
        emit_double(sample.regulation.active_z.kp);
        stream << ',';
        emit_double(sample.regulation.active_z.ki);
        stream << ',';
        emit_double(sample.regulation.active_z.kd);
        stream << ',';
        emit_double(sample.command_link.target_rate_hz);
        stream << ',';
        emit_double(sample.command_link.actual_rate_hz);
        stream << ',';
        emit_double(sample.command_link.last_send_interval_ms);
        stream << ',' << sample.command_link.suppressed_sends << ','
               << sample.pose.received_mono_ns << ','
               << sample.pose.source_mono_ns << ','
               << sample.pose.source_time_present << ','
               << sample.pose.source_time_valid << ',';
        emit_double(
            static_cast<double>(sample.pose.source_clock_delta_ns) /
            1'000'000.0);
        stream << ',';
        emit_double(
            static_cast<double>(sample.pose.source_age_at_receive_ns) /
            1'000'000.0);
        stream << ',';
        emit_double(sample.pose_local_age_ms);
        stream << ',';
        emit_double(sample.pose_source_age_ms);
        stream << ',' << sample.pose_fresh << ',';
        emit_double(
            static_cast<double>(sample.pose.arrival_interval_ns) /
            1'000'000.0);
        stream << ',';
        emit_double(
            static_cast<double>(sample.pose.source_time_interval_ns) /
            1'000'000.0);
        stream << ',' << sample.pose.source_time_regression << ',';
        emit_double(sample.pose.raw_position.x);
        stream << ',';
        emit_double(sample.pose.raw_position.y);
        stream << ',';
        emit_double(sample.pose.raw_position.z);
        stream << ',';
        emit_q(sample.pose.raw_orientation);
        stream << ',' << sample.pose.source_sensor_id << ','
               << sample.source_status.connected << ','
               << sample.source_status.transport_connected << ','
               << sample.source_status.callbacks_received << ','
               << sample.source_status.messages_received << ','
               << sample.source_status.samples_rejected << ','
               << sample.source_status.unselected_sensor_samples << ','
               << sample.source_status.invalid_position_samples << ','
               << sample.source_status.invalid_orientation_samples << ','
               << sample.source_status.duplicate_timestamp_samples << ','
               << sample.source_status.timestamp_regression_samples << ','
               << sample.source_status.long_gap_count << ',';
        emit_double(sample.source_status.last_arrival_gap_ms);
        stream << ',';
        emit_double(sample.source_status.max_arrival_gap_ms);
        stream << ',' << sample.source_status.connect_count << ','
               << sample.source_status.disconnect_count << ','
               << sample.source_status.last_received_mono_ns << ',';
        emit_double(sample.source_status.expected_rate_hz);
        stream << ',' << sample.rc_gate_ready << ','
               << sample.handoff_reset << ',';
        emit_csv_string(stream, sample.handoff_reset_reason);
        stream << ',';
        emit_csv_string(stream, sample.handoff_error);
        stream << ',';
        emit_double(sample.loop_metrics.target_period_ms);
        stream << ',';
        emit_double(sample.loop_metrics.last_period_ms);
        stream << ',';
        emit_double(sample.loop_metrics.work_ms);
        stream << ',';
        emit_double(sample.loop_metrics.jitter_ms);
        stream << ',';
        emit_double(sample.loop_metrics.p99_work_ms);
        stream << ',';
        emit_double(sample.loop_metrics.p99_jitter_ms);
        stream << ',' << sample.loop_metrics.overruns << '\n';
        if (!stream) {
            set_error("flight log write failed: " + csv_path.string());
            close_session("write_error", sample.mono_ns, sample.wall_time_ns);
            session_open = false;
            return;
        }
        ++session_frames;
        increment_frames();
        if (sample.mono_ns >= last_flush_ns + kFlushIntervalNs) {
            stream.flush();
            last_flush_ns = sample.mono_ns;
        }
    }

    void process(const FlightLogSample& sample) {
        const bool armed_known =
            sample.telemetry.valid && sample.telemetry_fresh;
        const bool armed =
            armed_known &&
            flag_set(
                sample.telemetry.status_flags,
                gs_link::StatusFlag::Armed);
        if (armed_known) {
            last_armed_state_known_ns = sample.mono_ns;
        }
        if (sample.pose.valid) {
            if (previous_pose.has_value() &&
                sample.pose.received_mono_ns >
                    previous_pose->received_mono_ns) {
                const double dt_s =
                    static_cast<double>(
                        sample.pose.received_mono_ns -
                        previous_pose->received_mono_ns) /
                    1'000'000'000.0;
                if (dt_s >= 0.001 && dt_s <= 0.25) {
                    const Vec3 raw_velocity =
                        (sample.pose.position_xyz_m -
                         previous_pose->position_xyz_m) /
                        dt_s;
                    constexpr double kAlpha = 0.25;
                    estimated_velocity =
                        estimated_velocity * (1.0 - kAlpha) +
                        raw_velocity * kAlpha;
                }
            }
            previous_pose = sample.pose;
        }

        bool row_written = false;
        if (!session_open) {
            if (armed_known && !armed) {
                arm_latched = false;
            }
            buffer_pre_arm(sample);
            if (!arm_latched && armed_known && armed) {
                arm_latched = true;
                const FlightLogSample& first =
                    pre_arm_buffer.empty() ? sample : pre_arm_buffer.front();
                session_open = open_session(first);
                while (session_open && !pre_arm_buffer.empty()) {
                    FlightLogSample buffered =
                        std::move(pre_arm_buffer.front());
                    pre_arm_buffer.pop_front();
                    write_row(buffered);
                }
                row_written = session_open;
            }
            if (!session_open) {
                return;
            }
        }

        if (!row_written) {
            write_row(sample);
        }
        if (!session_open) {
            return;
        }
        if (armed_known && !armed) {
            close_session("disarm", sample.mono_ns, sample.wall_time_ns);
            session_open = false;
        } else if (!armed_known &&
                   sample.mono_ns >=
                       last_armed_state_known_ns +
                           static_cast<std::uint64_t>(
                               options.telemetry_stale_timeout_ms) *
                               1'000'000ULL) {
            close_session(
                "telemetry_stale", sample.mono_ns,
                sample.wall_time_ns);
            session_open = false;
        }
    }

    void run() {
        while (true) {
            FlightLogSample sample;
            bool has_sample = false;
            {
                std::unique_lock lock(queue_mutex);
                queue_cv.wait(lock, [this] {
                    return !queue.empty() ||
                           !running.load(std::memory_order_relaxed);
                });
                if (queue.empty() &&
                    !running.load(std::memory_order_relaxed)) {
                    break;
                }
                if (!queue.empty()) {
                    sample = std::move(queue.front());
                    queue.pop_front();
                    has_sample = true;
                }
            }
            if (has_sample) {
                process(sample);
            }
        }
        if (session_open) {
            const auto wall = std::chrono::system_clock::now();
            const auto mono = SteadyClock::now();
            close_session(
                "service_stop",
                static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        mono.time_since_epoch())
                        .count()),
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    wall.time_since_epoch())
                    .count());
            session_open = false;
        }
    }

    FlightLoggerOptions options;
    std::atomic_bool running{false};
    std::thread thread;

    mutable std::mutex queue_mutex;
    std::condition_variable queue_cv;
    std::deque<FlightLogSample> queue;
    std::deque<FlightLogSample> pre_arm_buffer;

    mutable std::mutex status_mutex;
    FlightLogStatus status;

    std::ofstream stream;
    std::filesystem::path csv_path;
    std::filesystem::path metadata_path;
    std::filesystem::path summary_path;
    bool session_open{false};
    bool arm_latched{false};
    std::uint64_t session_start_ns{0};
    std::uint64_t session_serial{0};
    std::uint64_t session_frames{0};
    std::uint64_t last_flush_ns{0};
    std::uint64_t last_armed_state_known_ns{0};
    std::optional<PoseSample> previous_pose;
    Vec3 estimated_velocity{};
    std::uint64_t session_pose_frames{0};
    std::uint64_t session_unique_pose_samples{0};
    std::uint64_t session_pose_hold_frames{0};
    std::uint64_t session_pose_stale_frames{0};
    double session_max_pose_age_ms{-1.0};
    double session_max_arrival_interval_ms{0.0};
    double session_min_altitude_m{-1.0};
    double session_max_altitude_m{-1.0};
    std::uint64_t session_handoff_reset_count{0};
    std::uint64_t session_failsafe_frames{0};
    std::uint64_t session_loop_overruns{0};
    std::uint64_t session_first_loop_overruns{0};
    std::uint64_t session_first_logger_drops{0};
    double session_max_loop_period_ms{0.0};
    double session_max_loop_work_ms{0.0};
    double session_max_loop_jitter_ms{0.0};
    PoseSourceStatus session_first_source_status{};
    PoseSourceStatus session_last_source_status{};
    std::string session_first_handoff_reset_utc{};
    std::string session_first_handoff_reset_reason{};
    std::string session_first_failsafe_utc{};
    bool have_session_pose_sequence{false};
    std::uint32_t last_session_pose_sequence{0};
};

FlightLogger::FlightLogger(FlightLoggerOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}

FlightLogger::~FlightLogger() = default;

bool FlightLogger::start(std::string& error) {
    return impl_->start(error);
}

void FlightLogger::stop() {
    impl_->stop();
}

void FlightLogger::observe(const FlightLogSample& sample) noexcept {
    impl_->observe(sample);
}

FlightLogStatus FlightLogger::status() const {
    return impl_->status_snapshot();
}

}  // namespace dock_ground
