#include "dock_ground/runtime.hpp"

#include "dock_ground/command_sink.hpp"
#include "dock_ground/flight_logger.hpp"
#include "dock_ground/json.hpp"
#include "dock_ground/pose_source.hpp"
#include "dock_ground/position_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>

namespace dock_ground {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

class Histogram {
public:
    void add(double milliseconds) {
        if (!std::isfinite(milliseconds) || milliseconds < 0.0) {
            return;
        }
        const std::size_t index = std::min(
            bins_.size() - 1U,
            static_cast<std::size_t>(milliseconds * 100.0));
        ++bins_[index];
        ++count_;
    }

    [[nodiscard]] double p99() const {
        if (count_ == 0) {
            return 0.0;
        }
        const std::uint64_t target =
            static_cast<std::uint64_t>(
                std::ceil(static_cast<double>(count_) * 0.99));
        std::uint64_t cumulative = 0;
        for (std::size_t index = 0; index < bins_.size(); ++index) {
            cumulative += bins_[index];
            if (cumulative >= target) {
                return static_cast<double>(index + 1U) / 100.0;
            }
        }
        return static_cast<double>(bins_.size()) / 100.0;
    }

private:
    std::array<std::uint64_t, 1001> bins_{};
    std::uint64_t count_{0};
};

void append_number(std::ostringstream& output, double value) {
    if (!std::isfinite(value)) {
        output << "null";
        return;
    }
    output << std::setprecision(7) << value;
}

void append_vec3(std::ostringstream& output, const Vec3& value) {
    output << '[';
    append_number(output, value.x);
    output << ',';
    append_number(output, value.y);
    output << ',';
    append_number(output, value.z);
    output << ']';
}

void append_quaternion(
    std::ostringstream& output, const Quaternion& value) {
    output << '[';
    append_number(output, value.w);
    output << ',';
    append_number(output, value.x);
    output << ',';
    append_number(output, value.y);
    output << ',';
    append_number(output, value.z);
    output << ']';
}

[[nodiscard]] std::string body_label(int sensor_id) {
    return sensor_id >= 0 ? "body" + std::to_string(sensor_id) : "auto";
}

[[nodiscard]] double yaw_from_quaternion(const Quaternion& quaternion) {
    const Quaternion q = normalized(quaternion);
    return normalized_angle(std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
}

[[nodiscard]] std::unique_ptr<IPoseSource> make_pose_source(
    const AppConfig& config) {
    if (config.pose_source == "simulated") {
        return std::make_unique<SimulatedPoseSource>(
            config.simulated_pose_rate_hz);
    }
    if (config.pose_source == "vrpn") {
#if defined(DOCK_GROUND_HAS_VRPN)
        return std::make_unique<VrpnPoseSource>(
            VrpnPoseSourceOptions{
                .endpoint = config.vrpn_endpoint,
                .sensor_id = config.vrpn_sensor_id,
                .position_scale = config.vrpn_position_scale,
                .expected_rate_hz = config.vrpn_expected_rate_hz,
                .auto_origin = config.vrpn_auto_origin,
                .trust_source_timestamp = config.vrpn_trust_source_timestamp,
                .axes_from_vrpn = config.vrpn_axes_from_vrpn,
            });
#else
        throw std::runtime_error(
            "pose.source='vrpn' requires a build with DOCK_GROUND_WITH_VRPN=ON");
#endif
    }
    throw std::runtime_error("unsupported pose source: " + config.pose_source);
}

[[nodiscard]] std::unique_ptr<gs_link::UdpLink> make_gs_link_transport(
    const AppConfig& config) {
    if (config.flight_transport != "udp") {
        return nullptr;
    }
    return std::make_unique<gs_link::UdpLink>(
        gs_link::UdpConfig{
            .bind_ip = config.gs_link_bind_ip,
            .bind_port = *config.gs_link_bind_port,
            .remote_ip = config.esp32_ip,
            .remote_port = *config.esp32_port,
            .telemetry_timeout_ms =
                config.gs_link_telemetry_timeout_ms,
            .heartbeat_timeout_ms =
                config.gs_link_heartbeat_timeout_ms,
            .heartbeat_rate_hz =
                config.gs_link_heartbeat_rate_hz,
        });
}

[[nodiscard]] std::unique_ptr<ICommandSink> make_command_sink(
    const AppConfig& config, gs_link::UdpLink* link) {
    if (config.flight_transport == "udp" && link != nullptr) {
        return std::make_unique<GsLinkCommandSink>(
            *link, config.data_rc_rate_hz);
    }
    return std::make_unique<LoopbackCommandSink>();
}

[[nodiscard]] std::unique_ptr<FlightLogger> make_flight_logger(
    const AppConfig& config) {
    return std::make_unique<FlightLogger>(
        FlightLoggerOptions{
            .enabled = config.flight_log_enabled,
            .directory = config.flight_log_directory,
            .aircraft_id = config.aircraft_id,
            .aircraft_name = config.aircraft_name,
            .rc_mapping = config.rc_mapping,
            .telemetry_stale_timeout_ms =
                config.flight_log_stale_timeout_ms,
            .udp_flight_transport =
                config.flight_transport == "udp",
            .pre_roll_s = config.flight_log_pre_roll_s,
            .control_rate_hz = config.control_rate_hz,
            .warning_pose_age_ms = config.warning_pose_age_ms,
            .max_pose_age_ms = config.max_pose_age_ms,
            .pose_source = config.pose_source,
            .pose_endpoint =
                config.pose_source == "vrpn" ? config.vrpn_endpoint : "",
            .pose_sensor_id =
                config.pose_source == "vrpn" ? config.vrpn_sensor_id : 0,
            .pose_position_scale =
                config.pose_source == "vrpn"
                    ? config.vrpn_position_scale
                    : 1.0,
            .pose_expected_rate_hz =
                config.pose_source == "vrpn"
                    ? config.vrpn_expected_rate_hz
                    : config.simulated_pose_rate_hz,
            .pose_auto_origin =
                config.pose_source == "vrpn" && config.vrpn_auto_origin,
            .pose_trust_source_timestamp =
                config.pose_source == "vrpn" &&
                config.vrpn_trust_source_timestamp,
            .pose_axes_from_vrpn =
                config.pose_source == "vrpn"
                    ? config.vrpn_axes_from_vrpn
                    : Mat3{
                          1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0,
                      },
        });
}

[[nodiscard]] bool status_flag_set(
    std::uint16_t flags, gs_link::StatusFlag flag) {
    return (flags & static_cast<std::uint16_t>(flag)) != 0U;
}

[[nodiscard]] bool rc_handoff_gate_ready(
    const gs_link::Status& link) {
    const std::uint16_t flags = link.telemetry.status_flags;
    return link.enabled && link.telemetry_fresh &&
           link.telemetry.valid &&
           status_flag_set(
               flags, gs_link::StatusFlag::Armed) &&
           status_flag_set(
               flags, gs_link::StatusFlag::RcValid) &&
           status_flag_set(
               flags, gs_link::StatusFlag::RcSwitchValid) &&
           status_flag_set(
               flags, gs_link::StatusFlag::RcSwitchSelected) &&
           status_flag_set(
               flags, gs_link::StatusFlag::RcChannelsValid) &&
           !status_flag_set(
               flags, gs_link::StatusFlag::Failsafe);
}

[[nodiscard]] std::optional<ManualControlSetpoint> capture_rc_base(
    const gs_link::Status& link,
    const RcChannelMapping& mapping,
    std::string& error) {
    const auto& telemetry = link.telemetry;
    if (!rc_handoff_gate_ready(link)) {
        error =
            "RC handoff requires armed, valid RC, selected switch, "
            "valid channels, and no failsafe";
        return std::nullopt;
    }

    const std::array<int, 4> channels{
        mapping.roll,
        mapping.pitch,
        mapping.throttle,
        mapping.yaw,
    };
    const int maximum_channel =
        *std::max_element(channels.begin(), channels.end());
    if (telemetry.rc_channel_count < maximum_channel) {
        error = "FC telemetry does not contain all mapped RC channels";
        return std::nullopt;
    }
    for (const int channel : channels) {
        if (channel < 1 || channel > telemetry.rc_channel_count ||
            telemetry.rc_channels[channel - 1] == -128) {
            error = "mapped RC channel is invalid in FC telemetry";
            return std::nullopt;
        }
    }

    const auto normalized = [&telemetry](int channel) {
        return static_cast<double>(
                   telemetry.rc_channels[channel - 1]) /
               127.0;
    };
    ManualControlSetpoint base;
    base.roll = normalized(mapping.roll);
    base.pitch = normalized(mapping.pitch);
    base.throttle = normalized(mapping.throttle);
    base.yaw = normalized(mapping.yaw);
    base.valid = true;
    error.clear();
    return base;
}

[[nodiscard]] Vec3 slew_position_toward(
    const Vec3& current, const Vec3& requested, double maximum_step) {
    const Vec3 delta = requested - current;
    const double distance = norm(delta);
    if (distance <= maximum_step || distance < 1e-12) {
        return requested;
    }
    return current + delta * (maximum_step / distance);
}

}  // namespace

struct Runtime::Impl {
    explicit Impl(AppConfig input_config)
        : config(std::move(input_config)),
          controller(config.position_loop),
          pose_source(make_pose_source(config)),
          gs_link_transport(make_gs_link_transport(config)),
          command_sink(make_command_sink(config, gs_link_transport.get())),
          flight_logger(make_flight_logger(config)) {
        validate_config(config);
        target.position_xyz_m = config.initial_target_xyz_m;
        target.yaw_rad = normalized_angle(config.initial_yaw_rad);
        target.updated_mono_ns = monotonic_ns();

        published.mode =
            config.pose_source == "vrpn" ? "vrpn" : "simulation";
        published.target = target;
        published.position_tuning = controller.config();
        published.rc_mapping = config.rc_mapping;
        published.flight_log = flight_logger->status();
        published.link = command_sink->status();
        published.gs_link = gs_link_transport
                                ? gs_link_transport->status()
                                : gs_link::Status{};
        published.source_status = pose_source->status();
        published.pose_age_ms = kInfinity;
        published.metrics.target_period_ms = 1000.0 / config.control_rate_hz;

        const int initial_sensor =
            config.pose_source == "simulated" ? 0 : config.vrpn_sensor_id;
        binding.aircraft_id = config.aircraft_id;
        binding.body_id = body_label(initial_sensor);
        binding.sensor_id = initial_sensor;
    }

    void start() {
        if (running_.exchange(true)) {
            return;
        }
        pose_source->start();
        if (gs_link_transport) {
            std::string error;
            if (!gs_link_transport->start(error)) {
                pose_source->stop();
                running_.store(false, std::memory_order_relaxed);
                throw std::runtime_error(error);
            }
        }
        {
            std::string error;
            if (!flight_logger->start(error)) {
                if (gs_link_transport) {
                    gs_link_transport->stop();
                }
                pose_source->stop();
                running_.store(false, std::memory_order_relaxed);
                throw std::runtime_error(error);
            }
        }
        control_thread = std::thread([this] { run_control(); });
    }

    void stop() {
        if (!running_.exchange(false)) {
            return;
        }
        if (control_thread.joinable()) {
            control_thread.join();
        }
        flight_logger->stop();
        if (gs_link_transport) {
            gs_link_transport->stop();
        }
        pose_source->stop();
    }

    [[nodiscard]] std::optional<OperatorTarget> try_read_target() {
        std::unique_lock lock(target_mutex, std::try_to_lock);
        if (!lock.owns_lock()) {
            return std::nullopt;
        }
        return target;
    }

    bool update_target(const OperatorTarget& requested) {
        if (!is_finite(requested.position_xyz_m) ||
            !std::isfinite(requested.yaw_rad)) {
            return false;
        }
        OperatorTarget sanitized = requested;
        sanitized.position_xyz_m.x =
            clamp(sanitized.position_xyz_m.x, -25.0, 25.0);
        sanitized.position_xyz_m.y =
            clamp(sanitized.position_xyz_m.y, -25.0, 25.0);
        sanitized.position_xyz_m.z =
            clamp(sanitized.position_xyz_m.z, -25.0, 25.0);
        sanitized.yaw_rad = normalized_angle(sanitized.yaw_rad);
        sanitized.updated_mono_ns = monotonic_ns();

        const std::lock_guard lock(target_mutex);
        target = sanitized;
        operator_target_received.store(true, std::memory_order_relaxed);
        return true;
    }

    bool update_position_tuning(
        const PositionLoopTuning& tuning, std::string& error) {
        try {
            controller.set_config(tuning);
        } catch (const std::exception& exception) {
            error = exception.what();
            return false;
        }
        error.clear();
        return true;
    }

    bool bind_body(
        const std::string& aircraft_id, const std::string& body_id,
        int sensor_id, std::string& error) {
        if (aircraft_id != config.aircraft_id) {
            error = "unknown aircraft instance: " + aircraft_id;
            return false;
        }
        if (sensor_id < 0 ||
            body_id != "body" + std::to_string(sensor_id)) {
            error = "body id and sensor id do not match";
            return false;
        }
        {
            const std::lock_guard lock(target_mutex);
            if (target.deadman) {
                error = "release deadman before changing body binding";
                return false;
            }
            target.deadman = false;
            target.updated_mono_ns = monotonic_ns();
        }
        if (!pose_source->select_sensor(sensor_id)) {
            error = "pose source does not support sensor selection";
            return false;
        }
        {
            const std::lock_guard lock(binding_mutex);
            binding.aircraft_id = aircraft_id;
            binding.body_id = body_id;
            binding.sensor_id = sensor_id;
        }
        rebind_requested.store(true, std::memory_order_relaxed);
        return true;
    }

    void publish(const RuntimeSnapshot& next) {
        std::unique_lock lock(snapshot_mutex, std::try_to_lock);
        if (lock.owns_lock()) {
            published = next;
        }
    }

    void run_control() {
        const auto period = std::chrono::duration_cast<SteadyClock::duration>(
            std::chrono::duration<double>(1.0 / config.control_rate_hz));
        const double target_period_ms =
            static_cast<double>(period.count()) / 1'000'000.0;

        auto deadline = SteadyClock::now();
        auto previous_start = deadline;
        PoseSample last_pose{};
        AttitudeThrustSetpoint last_command{};
        std::uint32_t command_sequence = 0;
        Histogram work_histogram;
        Histogram jitter_histogram;
        bool initial_target_applied = !config.initial_target_use_current_pose;
        bool rc_passthrough_active = false;
        bool rc_handoff_active = false;
        bool rc_handoff_hold = false;
        double handoff_vertical_speed_mps = 0.0;
        std::uint32_t handoff_velocity_samples = 0;
        double rc_handoff_throttle_blend = 0.0;
        double rc_handoff_throttle_command = 0.0;
        std::uint64_t rc_handoff_until_ns = 0;
        ManualControlSetpoint rc_handoff_base{};
        std::string rc_handoff_error;
        bool handoff_reset = false;
        std::string handoff_reset_reason;
        OperatorTarget local_target;
        OperatorTarget slew_target;
        {
            const std::lock_guard lock(target_mutex);
            local_target = target;
            slew_target = target;
        }

        while (running_.load(std::memory_order_relaxed)) {
            deadline += period;
            std::this_thread::sleep_until(deadline);
            handoff_reset = false;
            handoff_reset_reason.clear();

            const auto cycle_start = SteadyClock::now();
            const double period_ms =
                std::chrono::duration<double, std::milli>(
                    cycle_start - previous_start)
                    .count();
            previous_start = cycle_start;

            if (rebind_requested.exchange(false, std::memory_order_relaxed)) {
                last_pose = {};
                initial_target_applied = false;
                controller.reset();
                rc_passthrough_active = false;
                rc_handoff_active = false;
                rc_handoff_hold = false;
                handoff_vertical_speed_mps = 0.0;
                handoff_velocity_samples = 0;
                rc_handoff_throttle_blend = 0.0;
                rc_handoff_throttle_command = 0.0;
                handoff_reset = true;
                handoff_reset_reason = "pose_binding_changed";
            }

            if (operator_target_received.load(std::memory_order_relaxed)) {
                if (const auto next_target = try_read_target()) {
                    local_target = *next_target;
                }
            }

            const std::uint64_t now_ns = monotonic_ns(cycle_start);

            if (const auto next_pose = pose_source->latest()) {
                if (last_pose.valid &&
                    next_pose->received_mono_ns >
                        last_pose.received_mono_ns) {
                    const double pose_dt_s =
                        static_cast<double>(
                            next_pose->received_mono_ns -
                            last_pose.received_mono_ns) /
                        1'000'000'000.0;
                    if (pose_dt_s >= 0.001 && pose_dt_s <= 0.25) {
                        const double raw_vertical_speed =
                            (next_pose->position_xyz_m.z -
                             last_pose.position_xyz_m.z) /
                            pose_dt_s;
                        handoff_vertical_speed_mps =
                            0.75 * handoff_vertical_speed_mps +
                            0.25 * raw_vertical_speed;
                        if (handoff_velocity_samples < 1000U) {
                            ++handoff_velocity_samples;
                        }
                    }
                }
                last_pose = *next_pose;
            }

            if (!initial_target_applied && last_pose.valid) {
                local_target.position_xyz_m = last_pose.position_xyz_m;
                local_target.yaw_rad =
                    yaw_from_quaternion(last_pose.q_body_to_world);
                local_target.updated_mono_ns = now_ns;
                slew_target = local_target;
                initial_target_applied = true;
            }

            OperatorTarget cycle_target = slew_target;
            cycle_target.deadman = local_target.deadman;
            cycle_target.yaw_rad = local_target.yaw_rad;
            cycle_target.updated_mono_ns = local_target.updated_mono_ns;
            cycle_target.client_sequence = local_target.client_sequence;
            const double target_dt_s =
                clamp(period_ms / 1'000.0, 0.001, 0.05);
            const Vec3 previous_slew_position =
                slew_target.position_xyz_m;
            slew_target.position_xyz_m = slew_position_toward(
                slew_target.position_xyz_m, local_target.position_xyz_m,
                config.target_slew_rate_mps * target_dt_s);
            cycle_target.position_xyz_m = slew_target.position_xyz_m;
            cycle_target.target_velocity_xyz_mps =
                (slew_target.position_xyz_m - previous_slew_position) /
                target_dt_s;
            if (cycle_target.deadman &&
                now_ns > cycle_target.updated_mono_ns &&
                static_cast<double>(now_ns - cycle_target.updated_mono_ns) /
                        1'000'000.0 >
                    config.deadman_timeout_ms) {
                cycle_target.deadman = false;
            }

            const double local_pose_age_ms =
                last_pose.valid && now_ns >= last_pose.received_mono_ns
                    ? static_cast<double>(
                          now_ns - last_pose.received_mono_ns) /
                          1'000'000.0
                    : 0.0;
            const double source_pose_age_ms =
                last_pose.valid && last_pose.source_time_valid
                    ? static_cast<double>(
                          last_pose.source_age_at_receive_ns) /
                          1'000'000.0
                    : 0.0;
            const double pose_age_ms =
                last_pose.valid
                    ? local_pose_age_ms + source_pose_age_ms
                    : kInfinity;
            const bool pose_fresh =
                last_pose.valid && pose_age_ms <= config.max_pose_age_ms;

            const gs_link::Status current_gs_status =
                gs_link_transport
                    ? gs_link_transport->status()
                    : gs_link::Status{};
            const bool gs_transport_enabled =
                config.flight_transport == "udp";
            const bool rc_gate_ready =
                rc_handoff_gate_ready(current_gs_status);
            if (!cycle_target.deadman || !pose_fresh ||
                (gs_transport_enabled && !rc_gate_ready)) {
                if (!handoff_reset) {
                    handoff_reset = true;
                    if (!cycle_target.deadman) {
                        handoff_reset_reason = "deadman_inactive";
                    } else if (!pose_fresh) {
                        handoff_reset_reason = "pose_stale";
                    } else {
                        handoff_reset_reason = "rc_gate_not_ready";
                    }
                }
                controller.reset();
                rc_passthrough_active = false;
                rc_handoff_active = false;
                rc_handoff_hold = false;
                rc_handoff_throttle_blend = 0.0;
                rc_handoff_throttle_command = 0.0;
                rc_handoff_until_ns = 0;
                if (!cycle_target.deadman) {
                    rc_handoff_error.clear();
                }
            }

            PositionControllerOutput controller_output{};
            ControlState state = ControlState::Standby;
            bool command_valid = false;
            bool have_outgoing_command = false;
            ManualControlSetpoint outgoing_command{};
            if (cycle_target.deadman && pose_fresh) {
                if (gs_transport_enabled) {
                    const bool fc_gs_confirmed =
                        status_flag_set(
                            current_gs_status.telemetry.status_flags,
                            gs_link::StatusFlag::GsModeActive);
                    if (!fc_gs_confirmed) {
                        if (rc_handoff_active) {
                            handoff_reset = true;
                            handoff_reset_reason = "fc_gs_mode_lost";
                            controller.reset();
                        }
                        rc_passthrough_active = false;
                        rc_handoff_active = false;
                        rc_handoff_hold = false;
                        rc_handoff_throttle_blend = 0.0;
                        rc_handoff_until_ns = 0;
                        const auto passthrough = capture_rc_base(
                            current_gs_status, config.rc_mapping,
                            rc_handoff_error);
                        if (passthrough.has_value()) {
                            rc_handoff_base = *passthrough;
                            rc_handoff_throttle_command =
                                rc_handoff_base.throttle;
                            outgoing_command = *passthrough;
                            have_outgoing_command = true;
                            rc_passthrough_active = true;
                            rc_handoff_error.clear();
                        }
                    } else if (!rc_handoff_active) {
                        const auto captured = capture_rc_base(
                            current_gs_status, config.rc_mapping,
                            rc_handoff_error);
                        if (captured.has_value()) {
                            rc_handoff_base = *captured;
                            const double captured_upper_throttle =
                                0.5 *
                                (rc_handoff_base.throttle + 1.0);
                            const double throttle_delta =
                                std::abs(
                                    captured_upper_throttle -
                                    config.position_loop.hover_throttle);
                            const bool vertical_speed_ready =
                                handoff_velocity_samples >= 5U &&
                                std::abs(handoff_vertical_speed_mps) <=
                                    config.rc_mapping
                                        .handoff_max_vertical_speed_mps;
                            const bool throttle_ready =
                                throttle_delta <=
                                config.rc_mapping
                                    .handoff_max_throttle_delta;
                            if (vertical_speed_ready && throttle_ready) {
                                rc_passthrough_active = false;
                                rc_handoff_active = true;
                                rc_handoff_hold = true;
                                rc_handoff_until_ns =
                                    now_ns +
                                    static_cast<std::uint64_t>(
                                        config.rc_mapping
                                            .handoff_hold_ms) *
                                        1'000'000ULL;
                                controller.reset();
                                rc_handoff_throttle_blend = 0.0;
                                rc_handoff_throttle_command =
                                    rc_handoff_base.throttle;
                                rc_handoff_error.clear();
                            } else {
                                rc_passthrough_active = true;
                                outgoing_command = *captured;
                                have_outgoing_command = true;
                                std::ostringstream reason;
                                reason << std::setprecision(3)
                                       << "waiting for stable handoff: |vz|="
                                       << std::abs(
                                              handoff_vertical_speed_mps)
                                       << " m/s (limit "
                                       << config.rc_mapping
                                              .handoff_max_vertical_speed_mps
                                       << "), throttle delta="
                                       << throttle_delta << " (limit "
                                       << config.rc_mapping
                                              .handoff_max_throttle_delta
                                       << ")";
                                rc_handoff_error = reason.str();
                            }
                        }
                    }

                    if (rc_handoff_active &&
                        now_ns < rc_handoff_until_ns) {
                        outgoing_command = rc_handoff_base;
                        have_outgoing_command = true;
                        rc_handoff_hold = true;
                        rc_passthrough_active = false;
                        rc_handoff_throttle_blend = 0.0;
                        rc_handoff_throttle_command =
                            rc_handoff_base.throttle;
                    } else if (rc_handoff_active) {
                        rc_handoff_hold = false;
                        controller_output = controller.step(
                            last_pose, cycle_target, now_ns,
                            rc_handoff_base);
                        if (controller_output.valid) {
                            const double desired_throttle =
                                controller_output.manual.throttle;
                            const double control_dt_s = clamp(
                                period_ms / 1'000.0, 0.001, 0.05);
                            const double maximum_step =
                                2.0 * control_dt_s /
                                (static_cast<double>(
                                     config.rc_mapping
                                         .throttle_transition_ms) /
                                 1'000.0);
                            rc_handoff_throttle_command = clamp(
                                rc_handoff_throttle_command +
                                    clamp(
                                        desired_throttle -
                                            rc_handoff_throttle_command,
                                        -maximum_step, maximum_step),
                                -1.0, 1.0);
                            const double transition_distance =
                                std::abs(
                                    rc_handoff_base.throttle -
                                    desired_throttle);
                            rc_handoff_throttle_blend =
                                transition_distance < 1e-9
                                    ? 1.0
                                    : clamp(
                                          1.0 -
                                              std::abs(
                                                  rc_handoff_throttle_command -
                                                  desired_throttle) /
                                                  transition_distance,
                                          0.0, 1.0);
                            controller_output.manual.throttle =
                                rc_handoff_throttle_command;
                            controller_output.upper_throttle = clamp(
                                0.5 *
                                    (controller_output.manual.throttle +
                                     1.0),
                                0.0, 1.0);
                            outgoing_command =
                                controller_output.manual;
                            have_outgoing_command = true;
                        } else {
                            handoff_reset = true;
                            handoff_reset_reason =
                                "controller_output_invalid";
                        }
                    }
                } else {
                    const ManualControlSetpoint loopback_base{
                        0.0, 0.0, 0.0, -0.4, true};
                    controller_output = controller.step(
                        last_pose, cycle_target, now_ns,
                        loopback_base);
                    if (controller_output.valid) {
                        outgoing_command = controller_output.manual;
                        have_outgoing_command = true;
                    }
                }
            }

            if (have_outgoing_command) {
                state = ControlState::Track;
                AttitudeThrustSetpoint command;
                command.sequence = ++command_sequence;
                command.pose_sequence = last_pose.sequence;
                command.pose_age_us = static_cast<std::uint32_t>(
                    std::min(
                        pose_age_ms * 1000.0,
                        static_cast<double>(
                            std::numeric_limits<std::uint32_t>::max())));
                command.q_body_to_world_desired =
                    (rc_handoff_hold || rc_passthrough_active)
                        ? last_pose.q_body_to_world
                        : controller_output.q_body_to_world_desired;
                command.collective_thrust = static_cast<float>(
                    0.5 * (outgoing_command.throttle + 1.0));
                command.manual = outgoing_command;
                command.position_loop_output_xyz =
                    (rc_handoff_hold || rc_passthrough_active)
                        ? Vec3{}
                        : controller_output.pid_output_xyz;
                command.roll_right_command_rad =
                    (rc_handoff_hold || rc_passthrough_active)
                        ? outgoing_command.roll * controller.config().max_tilt_rad
                        : controller_output.roll_right_command_rad;
                command.pitch_down_command_rad =
                    (rc_handoff_hold || rc_passthrough_active)
                        ? outgoing_command.pitch * controller.config().max_tilt_rad
                        : controller_output.pitch_down_command_rad;
                command.yaw_rate_right_command_rad_s =
                    (rc_handoff_hold || rc_passthrough_active)
                        ? outgoing_command.yaw *
                              controller.config().max_yaw_rate_rad_s
                        : controller_output.yaw_rate_right_command_rad_s;
                command.flags = 0x1U;
                last_command = command;
                command_valid =
                    command_sink->send(command.manual);
                if (!command_valid &&
                    config.flight_transport == "udp") {
                    state = ControlState::Degraded;
                }
            } else if (cycle_target.deadman) {
                state =
                    pose_age_ms > 500.0 ? ControlState::Failsafe
                                        : ControlState::Degraded;
            }

            const auto cycle_end = SteadyClock::now();
            const double work_ms =
                std::chrono::duration<double, std::milli>(
                    cycle_end - cycle_start)
                    .count();
            const double jitter_ms = std::abs(period_ms - target_period_ms);
            work_histogram.add(work_ms);
            jitter_histogram.add(jitter_ms);
            if (period_ms > target_period_ms * 1.5) {
                ++overruns;
            }

            RuntimeSnapshot next;
            next.generated_mono_ns = monotonic_ns(cycle_end);
            next.state = state;
            next.mode =
                config.pose_source == "vrpn" ? "vrpn" : "simulation";
            next.pose = last_pose;
            next.target = cycle_target;
            next.command = last_command;
            next.link = command_sink->status();
            next.gs_link = current_gs_status;
            next.source_status = pose_source->status();
            next.pose_age_ms = pose_age_ms;
            next.command_valid = command_valid;
            next.position_tuning = controller.config();
            next.rc_mapping = config.rc_mapping;
            next.rc_handoff_base = rc_handoff_base;
            next.rc_passthrough_active = rc_passthrough_active;
            next.rc_handoff_active = rc_handoff_active;
            next.rc_handoff_hold = rc_handoff_hold;
            next.handoff_vertical_speed_mps =
                handoff_vertical_speed_mps;
            next.rc_handoff_throttle_blend =
                rc_handoff_throttle_blend;
            next.rc_handoff_error = rc_handoff_error;
            next.position_mode = controller_output.regulation;
            next.deadman_active = cycle_target.deadman;
            next.metrics.target_period_ms = target_period_ms;
            next.metrics.last_period_ms = period_ms;
            next.metrics.work_ms = work_ms;
            next.metrics.jitter_ms = jitter_ms;
            next.metrics.p99_work_ms = work_histogram.p99();
            next.metrics.p99_jitter_ms = jitter_histogram.p99();
            next.metrics.overruns = overruns;
            next.flight_log = flight_logger->status();
            publish(next);

            FlightLogSample log_sample;
            log_sample.mono_ns = next.generated_mono_ns;
            log_sample.wall_time_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();
            log_sample.control_state = next.state;
            log_sample.pose = next.pose;
            log_sample.target = next.target;
            log_sample.tuning = next.position_tuning;
            log_sample.regulation = next.position_mode;
            log_sample.command = next.command;
            log_sample.rc_base = next.rc_handoff_base;
            log_sample.telemetry = next.gs_link.telemetry;
            log_sample.heartbeat = next.gs_link.heartbeat;
            log_sample.counters = next.gs_link.counters;
            log_sample.source_status = next.source_status;
            log_sample.loop_metrics = next.metrics;
            log_sample.pose_age_ms = next.pose_age_ms;
            log_sample.pose_local_age_ms = local_pose_age_ms;
            log_sample.pose_source_age_ms = source_pose_age_ms;
            log_sample.telemetry_fresh =
                next.gs_link.telemetry_fresh;
            log_sample.heartbeat_fresh =
                next.gs_link.heartbeat_fresh;
            log_sample.gs_transport_connected =
                next.gs_link.transport_connected;
            log_sample.uplink_allowed =
                next.gs_link.uplink_allowed;
            log_sample.telemetry_age_ms =
                next.gs_link.telemetry_age_ms;
            log_sample.heartbeat_age_ms =
                next.gs_link.heartbeat_age_ms;
            log_sample.command_link = next.link;
            log_sample.command_valid = next.command_valid;
            log_sample.deadman_active = next.deadman_active;
            log_sample.rc_passthrough_active =
                next.rc_passthrough_active;
            log_sample.handoff_active = next.rc_handoff_active;
            log_sample.handoff_hold = next.rc_handoff_hold;
            log_sample.handoff_vertical_speed_mps =
                next.handoff_vertical_speed_mps;
            log_sample.handoff_throttle_blend =
                next.rc_handoff_throttle_blend;
            log_sample.pose_fresh = pose_fresh;
            log_sample.rc_gate_ready = rc_gate_ready;
            log_sample.handoff_reset = handoff_reset;
            log_sample.handoff_reset_reason = handoff_reset_reason;
            log_sample.handoff_error = rc_handoff_error;
            flight_logger->observe(log_sample);
        }
        controller.reset();
    }

    AppConfig config;
    PositionController controller;
    std::unique_ptr<IPoseSource> pose_source;
    std::unique_ptr<gs_link::UdpLink> gs_link_transport;
    std::unique_ptr<ICommandSink> command_sink;
    std::unique_ptr<FlightLogger> flight_logger;

    std::atomic_bool running_{false};
    std::thread control_thread;

    mutable std::mutex target_mutex;
    OperatorTarget target{};
    std::atomic_bool operator_target_received{false};

    mutable std::mutex snapshot_mutex;
    RuntimeSnapshot published{};

    struct BindingState {
        std::string aircraft_id;
        std::string body_id;
        int sensor_id{-1};
    };
    mutable std::mutex binding_mutex;
    BindingState binding;
    std::atomic_bool rebind_requested{false};

    std::uint64_t overruns{0};
};

Runtime::Runtime(AppConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

Runtime::~Runtime() {
    stop();
}

void Runtime::start() {
    impl_->start();
}

void Runtime::stop() {
    impl_->stop();
}

bool Runtime::apply_operator_target(const OperatorTarget& target) {
    return impl_->update_target(target);
}

bool Runtime::apply_position_tuning(
    const PositionLoopTuning& tuning, std::string& error) {
    return impl_->update_position_tuning(tuning, error);
}

PositionLoopTuning Runtime::position_tuning() const {
    return impl_->controller.config();
}

bool Runtime::bind_body(
    const std::string& aircraft_id, const std::string& body_id,
    int sensor_id, std::string& error) {
    return impl_->bind_body(aircraft_id, body_id, sensor_id, error);
}

bool Runtime::apply_gs_uplink(
    const gs_link::UplinkRequest& request, std::string& error) {
    if (!impl_->gs_link_transport) {
        error =
            "GS-Link uplink is unavailable while flight.transport=loopback";
        return false;
    }
    return impl_->gs_link_transport->send(request, error);
}

RuntimeSnapshot Runtime::snapshot() const {
    const std::lock_guard lock(impl_->snapshot_mutex);
    return impl_->published;
}

std::string Runtime::snapshot_json() const {
    const RuntimeSnapshot state = snapshot();
    const bool fc_gs_mode_active =
        state.gs_link.telemetry_fresh &&
        status_flag_set(
            state.gs_link.telemetry.status_flags,
            gs_link::StatusFlag::GsModeActive) &&
        !status_flag_set(
            state.gs_link.telemetry.status_flags,
            gs_link::StatusFlag::Failsafe);
    const bool ground_station_commanding =
        impl_->config.flight_transport == "udp" &&
        state.command_valid && state.target.deadman;
    std::ostringstream output;
    output << "{\"v\":1,\"type\":\"state\"";
    output << ",\"mode\":\"" << json::escape(state.mode) << '"';
    output << ",\"control_state\":\"" << to_string(state.state) << '"';
    output << ",\"generated_mono_ms\":" << state.generated_mono_ns / 1'000'000U;
    std::string warning;
    if (state.mode != nullptr && std::string_view(state.mode) == "vrpn") {
        if (state.source_status.connected) {
            warning = "VRPN LIVE";
        } else if (state.source_status.transport_connected) {
            warning = "VRPN LINKED / NO POSE";
        } else {
            warning = "VRPN OFFLINE";
        }
    } else {
        warning = "SIMULATION";
    }
    if (state.gs_link.enabled) {
        warning += " / ";
        if (state.gs_link.telemetry_fresh &&
            state.gs_link.heartbeat_fresh) {
            if (fc_gs_mode_active) {
                if (ground_station_commanding) {
                    warning +=
                        state.rc_passthrough_active
                            ? "GS-LINK LIVE / GS SAFE HOLD / RC PASSTHROUGH"
                            : state.rc_handoff_hold
                            ? "GS-LINK LIVE / HOLDING RC BASE"
                            : "GS-LINK LIVE / GROUND CONTROL ACTIVE";
                } else {
                    warning +=
                        "FC IN GS MODE / COMMAND STREAM STOPPED / "
                        "RC FALLBACK PENDING";
                }
            } else if (ground_station_commanding) {
                warning +=
                    state.rc_handoff_hold
                        ? "GS-LINK LIVE / HOLDING RC BASE / WAITING FC GS MODE"
                        : state.rc_passthrough_active
                          ? "GS-LINK LIVE / RC PASSTHROUGH / WAITING FC GS MODE"
                        : "GS-LINK LIVE / COMMANDING / FC NOT IN GS MODE";
            } else {
                warning += "GS-LINK LIVE / WAITING FOR HANDOFF";
            }
        } else if (state.gs_link.heartbeat_fresh) {
            warning +=
                "GS-LINK HEARTBEAT / NO TELEMETRY / UPLINK BLOCKED";
        } else {
            warning += "GS-LINK WAITING FC / UPLINK BLOCKED";
        }
    } else {
        warning += " / LOOPBACK ONLY";
    }
    output << ",\"warning\":\"" << json::escape(warning) << '"';

    output << ",\"pose_source\":{\"name\":\""
           << json::escape(state.source_status.name)
           << "\",\"connected\":"
           << (state.source_status.connected ? "true" : "false")
           << ",\"transport_connected\":"
           << (state.source_status.transport_connected ? "true" : "false")
           << ",\"sensor_id\":" << state.source_status.sensor_id
           << ",\"messages_received\":"
           << state.source_status.messages_received
           << ",\"callbacks_received\":"
           << state.source_status.callbacks_received
           << ",\"samples_rejected\":"
           << state.source_status.samples_rejected
           << ",\"unselected_sensor_samples\":"
           << state.source_status.unselected_sensor_samples
           << ",\"invalid_position_samples\":"
           << state.source_status.invalid_position_samples
           << ",\"invalid_orientation_samples\":"
           << state.source_status.invalid_orientation_samples
           << ",\"duplicate_timestamp_samples\":"
           << state.source_status.duplicate_timestamp_samples
           << ",\"timestamp_regression_samples\":"
           << state.source_status.timestamp_regression_samples
           << ",\"long_gap_count\":"
           << state.source_status.long_gap_count
           << ",\"connect_count\":"
           << state.source_status.connect_count
           << ",\"disconnect_count\":"
           << state.source_status.disconnect_count
           << ",\"last_received_mono_ns\":"
           << state.source_status.last_received_mono_ns
           << ",\"expected_rate_hz\":";
    append_number(output, state.source_status.expected_rate_hz);
    output << ",\"last_arrival_gap_ms\":";
    append_number(output, state.source_status.last_arrival_gap_ms);
    output << ",\"max_arrival_gap_ms\":";
    append_number(output, state.source_status.max_arrival_gap_ms);
    output
           << ",\"endpoint\":\"" << json::escape(impl_->config.vrpn_endpoint)
           << "\",\"position_scale\":" << impl_->config.vrpn_position_scale
           << ",\"auto_origin\":"
           << (impl_->config.vrpn_auto_origin ? "true" : "false")
           << ",\"trust_source_timestamp\":"
           << (impl_->config.vrpn_trust_source_timestamp ? "true" : "false")
           << '}';

    const auto& tuning = state.position_tuning;
    const auto append_axis = [&output](
                                 const AxisPidGains& axis) {
        output << "{\"kp\":";
        append_number(output, axis.kp);
        output << ",\"ki\":";
        append_number(output, axis.ki);
        output << ",\"kd\":";
        append_number(output, axis.kd);
        output << '}';
    };
    const auto append_profile = [&output, &append_axis](
                                    const PositionProfile& profile) {
        output << "{\"x\":";
        append_axis(profile.x);
        output << ",\"y\":";
        append_axis(profile.y);
        output << ",\"z\":";
        append_axis(profile.z);
        output << '}';
    };
    output << ",\"position_tuning\":{\"mode\":\"position_pid\""
           << ",\"frame\":\"world_xyz_x_forward_y_left_z_up\""
           << ",\"x\":";
    append_axis(tuning.x);
    output << ",\"y\":";
    append_axis(tuning.y);
    output << ",\"z\":";
    append_axis(tuning.z);
    output << ",\"approach\":";
    append_profile(tuning.approach);
    output << ",\"precision_hold\":";
    append_profile(tuning.precision_hold);
    output << ",\"mode_scheduling\":{\"enabled\":"
           << (tuning.mode_scheduling.enabled ? "true" : "false")
           << ",\"brake_enter_distance_m\":";
    append_number(
        output, tuning.mode_scheduling.brake_enter_distance_m);
    output << ",\"brake_exit_distance_m\":";
    append_number(
        output, tuning.mode_scheduling.brake_exit_distance_m);
    output << ",\"brake_enter_vertical_error_m\":";
    append_number(
        output,
        tuning.mode_scheduling.brake_enter_vertical_error_m);
    output << ",\"brake_exit_vertical_error_m\":";
    append_number(
        output,
        tuning.mode_scheduling.brake_exit_vertical_error_m);
    output << ",\"hold_enter_distance_m\":";
    append_number(
        output, tuning.mode_scheduling.hold_enter_distance_m);
    output << ",\"hold_exit_distance_m\":";
    append_number(
        output, tuning.mode_scheduling.hold_exit_distance_m);
    output << ",\"hold_enter_vertical_error_m\":";
    append_number(
        output,
        tuning.mode_scheduling.hold_enter_vertical_error_m);
    output << ",\"hold_exit_vertical_error_m\":";
    append_number(
        output, tuning.mode_scheduling.hold_exit_vertical_error_m);
    output << ",\"hold_enter_speed_mps\":";
    append_number(
        output, tuning.mode_scheduling.hold_enter_speed_mps);
    output << ",\"hold_exit_speed_mps\":";
    append_number(
        output, tuning.mode_scheduling.hold_exit_speed_mps);
    output << ",\"hold_enter_vertical_speed_mps\":";
    append_number(
        output,
        tuning.mode_scheduling.hold_enter_vertical_speed_mps);
    output << ",\"hold_exit_vertical_speed_mps\":";
    append_number(
        output,
        tuning.mode_scheduling.hold_exit_vertical_speed_mps);
    output << ",\"target_motion_speed_mps\":";
    append_number(
        output, tuning.mode_scheduling.target_motion_speed_mps);
    output << ",\"hold_enter_dwell_s\":";
    append_number(
        output, tuning.mode_scheduling.hold_enter_dwell_s);
    output << ",\"hold_exit_dwell_s\":";
    append_number(
        output, tuning.mode_scheduling.hold_exit_dwell_s);
    output << ",\"gain_transition_time_s\":";
    append_number(
        output, tuning.mode_scheduling.gain_transition_time_s);
    output << '}';
    output << ",\"max_accel_mps2\":";
    append_number(output, tuning.max_accel_mps2);
    output << ",\"max_tilt_rad\":";
    append_number(output, tuning.max_tilt_rad);
    output << ",\"min_throttle\":";
    append_number(output, tuning.min_throttle);
    output << ",\"hover_throttle\":";
    append_number(output, tuning.hover_throttle);
    output << ",\"max_throttle\":";
    append_number(output, tuning.max_throttle);
    output << ",\"z_velocity_damping\":";
    append_number(output, tuning.z_velocity_damping);
    output << ",\"max_yaw_rate_rad_s\":";
    append_number(output, tuning.max_yaw_rate_rad_s);
    output << ",\"yaw_rate_kp\":";
    append_number(output, tuning.yaw_rate_kp);
    output << ",\"yaw_rate_kd\":";
    append_number(output, tuning.yaw_rate_kd);
    output << ",\"yaw_rate_slew_rate_rad_s2\":";
    append_number(output, tuning.yaw_rate_slew_rate_rad_s2);
    output << ",\"fc_yaw_rate_scale_rad_s\":";
    append_number(output, tuning.fc_yaw_rate_scale_rad_s);
    output << ",\"max_position_error_m\":";
    append_number(output, tuning.max_position_error_m);
    output << ",\"gravity_mps2\":";
    append_number(output, tuning.gravity_mps2);
    output << '}';
    const auto& position_mode = state.position_mode;
    output << ",\"position_mode\":{\"mode\":\""
           << to_string(position_mode.mode)
           << "\",\"active\":"
           << (position_mode.active ? "true" : "false")
           << ",\"scheduling_enabled\":"
           << (position_mode.scheduling_enabled ? "true" : "false")
           << ",\"gain_transition_active\":"
           << (position_mode.gain_transition_active ? "true" : "false")
           << ",\"horizontal_error_m\":";
    append_number(output, position_mode.horizontal_error_m);
    output << ",\"vertical_error_m\":";
    append_number(output, position_mode.vertical_error_m);
    output << ",\"horizontal_speed_mps\":";
    append_number(output, position_mode.horizontal_speed_mps);
    output << ",\"vertical_speed_mps\":";
    append_number(output, position_mode.vertical_speed_mps);
    output << ",\"active_gains\":{\"x\":";
    append_axis(position_mode.active_x);
    output << ",\"y\":";
    append_axis(position_mode.active_y);
    output << ",\"z\":";
    append_axis(position_mode.active_z);
    output << "}}";
    output << ",\"rc_mapping\":{\"roll\":"
           << state.rc_mapping.roll << ",\"pitch\":"
           << state.rc_mapping.pitch << ",\"throttle\":"
           << state.rc_mapping.throttle << ",\"yaw\":"
           << state.rc_mapping.yaw << ",\"handoff_hold_ms\":"
           << state.rc_mapping.handoff_hold_ms
           << ",\"throttle_transition_ms\":"
           << state.rc_mapping.throttle_transition_ms
           << ",\"handoff_max_vertical_speed_mps\":";
    append_number(
        output, state.rc_mapping.handoff_max_vertical_speed_mps);
    output << ",\"handoff_max_throttle_delta\":";
    append_number(
        output, state.rc_mapping.handoff_max_throttle_delta);
    output << '}';
    const bool ground_station_active = fc_gs_mode_active;
    output << ",\"control_output\":{\"source\":\""
           << (ground_station_active
                   ? "GROUND_STATION"
                   : impl_->config.flight_transport == "udp"
                         ? "RC"
                         : "LOOPBACK")
           << "\",\"ground_station_active\":"
           << (ground_station_active ? "true" : "false")
           << ",\"ground_station_commanding\":"
           << (ground_station_commanding ? "true" : "false")
           << ",\"fc_gs_mode_active\":"
           << (fc_gs_mode_active ? "true" : "false")
           << ",\"rc_passthrough\":"
           << (state.rc_passthrough_active ? "true" : "false")
           << ",\"handoff_active\":"
           << (state.rc_handoff_active ? "true" : "false")
           << ",\"handoff_hold\":"
           << (state.rc_handoff_hold ? "true" : "false")
           << ",\"handoff_vertical_speed_mps\":";
    append_number(output, state.handoff_vertical_speed_mps);
    output
           << ",\"handoff_throttle_blend\":";
    append_number(output, state.rc_handoff_throttle_blend);
    output
           << ",\"handoff_error\":\""
           << json::escape(state.rc_handoff_error)
           << "\",\"rc_base\":{\"roll\":";
    append_number(output, state.rc_handoff_base.roll);
    output << ",\"pitch\":";
    append_number(output, state.rc_handoff_base.pitch);
    output << ",\"yaw\":";
    append_number(output, state.rc_handoff_base.yaw);
    output << ",\"throttle\":";
    append_number(output, state.rc_handoff_base.throttle);
    output << "},\"current\":{\"roll\":";
    append_number(output, state.command.manual.roll);
    output << ",\"pitch\":";
    append_number(output, state.command.manual.pitch);
    output << ",\"yaw\":";
    append_number(output, state.command.manual.yaw);
    output << ",\"throttle\":";
    append_number(output, state.command.manual.throttle);
    output << "}}";
    output << ",\"flight_log\":{\"enabled\":"
           << (state.flight_log.enabled ? "true" : "false")
           << ",\"recording\":"
           << (state.flight_log.recording ? "true" : "false")
           << ",\"current_file\":\""
           << json::escape(state.flight_log.current_file)
           << "\",\"last_error\":\""
           << json::escape(state.flight_log.last_error)
           << "\",\"sessions_written\":"
           << state.flight_log.sessions_written
           << ",\"frames_written\":"
           << state.flight_log.frames_written
           << ",\"frames_dropped\":"
           << state.flight_log.frames_dropped << '}';

    const auto& gs = state.gs_link;
    output << ",\"gs_link\":{\"enabled\":"
           << (gs.enabled ? "true" : "false")
           << ",\"transport\":\"" << json::escape(gs.transport)
           << "\",\"protocol\":\"" << json::escape(gs.protocol)
           << "\",\"transport_connected\":"
           << (gs.transport_connected ? "true" : "false")
           << ",\"telemetry_fresh\":"
           << (gs.telemetry_fresh ? "true" : "false")
           << ",\"heartbeat_fresh\":"
           << (gs.heartbeat_fresh ? "true" : "false")
           << ",\"uplink_allowed\":"
           << (gs.uplink_allowed ? "true" : "false")
           << ",\"command_pending\":"
           << (gs.command_pending ? "true" : "false")
           << ",\"telemetry_age_ms\":";
    append_number(output, gs.telemetry_age_ms);
    output << ",\"heartbeat_age_ms\":";
    append_number(output, gs.heartbeat_age_ms);
    output << ",\"session_id\":" << gs.session_id
           << ",\"state\":\"" << gs_link::to_string(gs.heartbeat.state)
           << "\",\"error_code\":"
           << static_cast<unsigned int>(gs.heartbeat.error_code)
           << ",\"heartbeat_status_flags\":"
           << gs.heartbeat.status_flags
           << ",\"telemetry\":{\"valid\":"
           << (gs.telemetry.valid ? "true" : "false")
           << ",\"fc_timestamp_ms\":" << gs.telemetry.timestamp_ms
           << ",\"status_flags\":" << gs.telemetry.status_flags
           << ",\"roll_cdeg\":" << gs.telemetry.roll_cdeg
           << ",\"pitch_cdeg\":" << gs.telemetry.pitch_cdeg
           << ",\"yaw_cdeg\":" << gs.telemetry.yaw_cdeg
           << ",\"motor1_rpm\":" << gs.telemetry.motor1_rpm
           << ",\"motor2_rpm\":" << gs.telemetry.motor2_rpm
           << ",\"battery_voltage_mv\":"
           << gs.telemetry.battery_voltage_mv
           << ",\"battery_current_ca\":"
           << gs.telemetry.battery_current_ca
           << ",\"battery_remaining\":"
           << static_cast<unsigned int>(
                  gs.telemetry.battery_remaining)
           << ",\"rc_channel_count\":"
           << static_cast<unsigned int>(gs.telemetry.rc_channel_count)
           << ",\"rc_channels\":[";
    for (std::size_t index = 0;
         index < gs.telemetry.rc_channel_count; ++index) {
        if (index != 0U) {
            output << ',';
        }
        output << static_cast<int>(gs.telemetry.rc_channels[index]);
    }
    output << "]}";
    output << ",\"heartbeat\":{\"valid\":"
           << (gs.heartbeat.valid ? "true" : "false")
           << ",\"uptime_ms\":" << gs.heartbeat.uptime_ms
           << ",\"session_id\":" << gs.heartbeat.session_id
           << ",\"status_flags\":" << gs.heartbeat.status_flags
           << ",\"protocol_version\":"
           << gs.heartbeat.protocol_version
           << ",\"state\":\""
           << gs_link::to_string(gs.heartbeat.state)
           << "\",\"error_code\":"
           << static_cast<unsigned int>(gs.heartbeat.error_code) << '}';
    output << ",\"last_ack\":{\"valid\":"
           << (gs.last_ack.valid ? "true" : "false")
           << ",\"acked_class\":"
           << static_cast<unsigned int>(gs.last_ack.acked_class)
           << ",\"acked_id\":"
           << static_cast<unsigned int>(gs.last_ack.acked_id)
           << ",\"acked_sequence\":" << gs.last_ack.acked_sequence
           << ",\"result\":\""
           << gs_link::to_string(gs.last_ack.result)
           << "\",\"detail\":"
           << static_cast<unsigned int>(gs.last_ack.detail)
           << ",\"echo_token\":" << gs.last_ack.echo_token << '}';
    output << ",\"counters\":{\"rx_packets\":"
           << gs.counters.rx_packets
           << ",\"tx_packets\":" << gs.counters.tx_packets
           << ",\"crc_errors\":" << gs.counters.crc_errors
           << ",\"length_errors\":" << gs.counters.length_errors
           << ",\"version_errors\":" << gs.counters.version_errors
           << ",\"unknown_data\":" << gs.counters.unknown_data
           << ",\"unknown_commands\":"
           << gs.counters.unknown_commands
           << ",\"duplicates\":" << gs.counters.duplicates
           << ",\"sequence_gaps\":" << gs.counters.sequence_gaps
           << ",\"tx_retries\":" << gs.counters.tx_retries
           << ",\"ack_timeouts\":" << gs.counters.ack_timeouts
           << ",\"send_errors\":" << gs.counters.send_errors
           << ",\"last_send_errno\":" << gs.last_send_errno
           << "}}";

    Impl::BindingState binding;
    {
        const std::lock_guard lock(impl_->binding_mutex);
        binding = impl_->binding;
    }
    const auto bodies = impl_->pose_source->discovered_sensors();
    const std::uint64_t now_ns = monotonic_ns();
    output << ",\"aircrafts\":[{\"id\":\""
           << json::escape(impl_->config.aircraft_id)
           << "\",\"name\":\"" << json::escape(impl_->config.aircraft_name)
           << "\"}]";
    output << ",\"binding\":{\"aircraft_id\":\""
           << json::escape(binding.aircraft_id) << "\",\"body_id\":\""
           << json::escape(binding.body_id) << "\",\"sensor_id\":"
           << binding.sensor_id << ",\"active\":"
           << (state.source_status.connected &&
                       state.source_status.sensor_id == binding.sensor_id
                   ? "true"
                   : "false")
           << '}';
    output << ",\"rigid_bodies\":[";
    for (std::size_t index = 0; index < bodies.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        const TrackerBodyInfo& body = bodies[index];
        const double age_ms =
            body.last_seen_mono_ns != 0 && now_ns >= body.last_seen_mono_ns
                ? static_cast<double>(now_ns - body.last_seen_mono_ns) /
                      1'000'000.0
                : -1.0;
        output << "{\"body_id\":\""
               << json::escape(body_label(body.sensor_id))
               << "\",\"sensor_id\":" << body.sensor_id
               << ",\"messages_received\":" << body.messages_received
               << ",\"last_seen_ms\":";
        append_number(output, age_ms);
        output << ",\"bound\":"
               << (body.sensor_id == binding.sensor_id ? "true" : "false")
               << '}';
    }
    output << ']';

    output << ",\"pose\":{\"valid\":"
           << (state.pose.valid ? "true" : "false");
    output << ",\"sequence\":" << state.pose.sequence;
    output << ",\"source_sensor_id\":" << state.pose.source_sensor_id;
    output << ",\"received_mono_ns\":" << state.pose.received_mono_ns;
    output << ",\"source_time_ns\":" << state.pose.source_mono_ns;
    output << ",\"source_time_present\":"
           << (state.pose.source_time_present ? "true" : "false");
    output << ",\"source_time_measured\":"
           << (state.pose.source_time_valid ? "true" : "false");
    output << ",\"source_clock_delta_ms\":";
    append_number(
        output,
        static_cast<double>(state.pose.source_clock_delta_ns) /
            1'000'000.0);
    output << ",\"source_age_us_at_receive\":"
           << state.pose.source_age_at_receive_ns / 1'000U;
    output << ",\"arrival_interval_ms\":";
    append_number(
        output,
        static_cast<double>(state.pose.arrival_interval_ns) /
            1'000'000.0);
    output << ",\"source_time_interval_ms\":";
    append_number(
        output,
        static_cast<double>(state.pose.source_time_interval_ns) /
            1'000'000.0);
    output << ",\"source_time_regression\":"
           << (state.pose.source_time_regression ? "true" : "false");
    output << ",\"age_ms\":";
    append_number(output, state.pose_age_ms);
    output << ",\"position_xyz_m\":";
    append_vec3(output, state.pose.position_xyz_m);
    output << ",\"orientation_wxyz\":";
    append_quaternion(output, state.pose.q_body_to_world);
    output << ",\"raw_position\":";
    append_vec3(output, state.pose.raw_position);
    output << ",\"raw_orientation_wxyz\":";
    append_quaternion(output, state.pose.raw_orientation);
    output << '}';

    output << ",\"target\":{\"position_xyz_m\":";
    append_vec3(output, state.target.position_xyz_m);
    output << ",\"yaw_rad\":";
    append_number(output, state.target.yaw_rad);
    output << ",\"deadman\":"
           << (state.target.deadman ? "true" : "false");
    output << ",\"client_sequence\":" << state.target.client_sequence;
    output << '}';

    output << ",\"command\":{\"valid\":"
           << (state.command_valid ? "true" : "false");
    output << ",\"sequence\":" << state.command.sequence;
    output << ",\"pose_sequence\":" << state.command.pose_sequence;
    output << ",\"pose_age_us\":" << state.command.pose_age_us;
    output << ",\"orientation_wxyz\":";
    append_quaternion(output, state.command.q_body_to_world_desired);
    output << ",\"thrust\":";
    append_number(output, state.command.collective_thrust);
    output << ",\"upper_throttle\":";
    append_number(output, state.command.collective_thrust);
    output << ",\"manual\":{\"valid\":"
           << (state.command.manual.valid ? "true" : "false")
           << ",\"roll\":";
    append_number(output, state.command.manual.roll);
    output << ",\"pitch\":";
    append_number(output, state.command.manual.pitch);
    output << ",\"yaw\":";
    append_number(output, state.command.manual.yaw);
    output << ",\"throttle\":";
    append_number(output, state.command.manual.throttle);
    output << "},\"position_loop_output_xyz\":";
    append_vec3(output, state.command.position_loop_output_xyz);
    output << ",\"roll_right_deg\":";
    append_number(
        output,
        state.command.roll_right_command_rad * 180.0 /
            3.14159265358979323846);
    output << ",\"pitch_down_deg\":";
    append_number(
        output,
        state.command.pitch_down_command_rad * 180.0 /
            3.14159265358979323846);
    output << ",\"yaw_rate_right_deg_s\":";
    append_number(
        output,
        state.command.yaw_rate_right_command_rad_s * 180.0 /
            3.14159265358979323846);
    output << '}';

    output << ",\"link\":{\"mode\":\""
           << json::escape(state.link.mode) << "\",\"protocol\":\""
           << json::escape(state.link.protocol) << "\",\"connected\":"
           << (state.link.connected ? "true" : "false");
    output << ",\"commands_sent\":" << state.link.commands_sent;
    output << ",\"send_errors\":" << state.link.send_errors;
    output << ",\"suppressed_sends\":"
           << state.link.suppressed_sends;
    output << ",\"target_rate_hz\":";
    append_number(output, state.link.target_rate_hz);
    output << ",\"actual_rate_hz\":";
    append_number(output, state.link.actual_rate_hz);
    output << ",\"last_send_interval_ms\":";
    append_number(output, state.link.last_send_interval_ms);
    output << ",\"last_error\":\""
           << json::escape(state.link.last_error) << "\"";
    output << ",\"last_errno\":" << state.link.last_errno;
    output << '}';

    output << ",\"metrics\":{\"target_period_ms\":";
    append_number(output, state.metrics.target_period_ms);
    output << ",\"last_period_ms\":";
    append_number(output, state.metrics.last_period_ms);
    output << ",\"work_ms\":";
    append_number(output, state.metrics.work_ms);
    output << ",\"jitter_ms\":";
    append_number(output, state.metrics.jitter_ms);
    output << ",\"p99_work_ms\":";
    append_number(output, state.metrics.p99_work_ms);
    output << ",\"p99_jitter_ms\":";
    append_number(output, state.metrics.p99_jitter_ms);
    output << ",\"overruns\":" << state.metrics.overruns;
    output << "}}";
    return output.str();
}

}  // namespace dock_ground
