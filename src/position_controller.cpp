#include "dock_ground/position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dock_ground {
namespace {

constexpr double kVelocityFilterAlpha = 0.25;
constexpr double kYawRateFilterAlpha = 0.25;
constexpr double kDefaultControlDtS = 0.01;

[[nodiscard]] double yaw_from_quaternion(const Quaternion& quaternion) {
    const Quaternion q = normalized(quaternion);
    return normalized_angle(std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
}

[[nodiscard]] double roll_from_quaternion(const Quaternion& quaternion) {
    const Quaternion q = normalized(quaternion);
    return std::atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y));
}

[[nodiscard]] double pitch_from_quaternion(
    const Quaternion& quaternion) {
    const Quaternion q = normalized(quaternion);
    return std::asin(clamp(
        2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
}

[[nodiscard]] Quaternion attitude_for_acceleration(
    const Vec3& acceleration_xyz_mps2, double heading_yaw_rad,
    double gravity_mps2) {
    const Vec3 thrust_specific_force{
        acceleration_xyz_mps2.x,
        acceleration_xyz_mps2.y,
        gravity_mps2,
    };
    const Vec3 body_z_up = normalized(thrust_specific_force);
    const Vec3 heading{
        std::cos(heading_yaw_rad), std::sin(heading_yaw_rad), 0.0};
    Vec3 body_x_forward = heading - body_z_up * dot(heading, body_z_up);
    if (norm_squared(body_x_forward) < 1e-10) {
        const Vec3 fallback{1.0, 0.0, 0.0};
        body_x_forward =
            fallback - body_z_up * dot(fallback, body_z_up);
    }
    body_x_forward = normalized(body_x_forward);
    const Vec3 body_y_left =
        normalized(cross(body_z_up, body_x_forward));
    body_x_forward = normalized(cross(body_y_left, body_z_up));
    return quaternion_from_body_axes(
        body_x_forward, body_y_left, body_z_up);
}

[[nodiscard]] PositionProfile profile_for_mode(
    PositionRegulationMode mode,
    const PositionControllerConfig& config) {
    if (mode == PositionRegulationMode::PrecisionHold) {
        return config.precision_hold;
    }
    if (mode == PositionRegulationMode::Approach) {
        return config.approach;
    }
    return {
        .x = config.x,
        .y = config.y,
        .z = config.z,
    };
}

}  // namespace

PositionController::PositionController(PositionControllerConfig config)
    : config_(config) {
    validate(config_);
}

void PositionController::validate(
    const PositionControllerConfig& config) {
    const auto valid_axis = [](const AxisPidGains& axis) {
        return std::isfinite(axis.kp) && std::isfinite(axis.ki) &&
               std::isfinite(axis.kd) && axis.kp >= 0.0 &&
               axis.ki >= 0.0 && axis.kd >= 0.0 &&
               axis.kp + axis.ki + axis.kd > 0.0;
    };
    const auto valid_profile = [&valid_axis](
                                   const PositionProfile& profile) {
        return valid_axis(profile.x) && valid_axis(profile.y) &&
               valid_axis(profile.z);
    };
    const auto& scheduling = config.mode_scheduling;
    if (!valid_axis(config.x) || !valid_axis(config.y) ||
        !valid_axis(config.z) ||
        !valid_profile(config.approach) ||
        !valid_profile(config.precision_hold) ||
        !std::isfinite(scheduling.brake_enter_distance_m) ||
        !std::isfinite(scheduling.brake_exit_distance_m) ||
        !std::isfinite(scheduling.brake_enter_vertical_error_m) ||
        !std::isfinite(scheduling.brake_exit_vertical_error_m) ||
        scheduling.brake_enter_vertical_error_m <= 0.0 ||
        scheduling.brake_enter_vertical_error_m >=
            scheduling.brake_exit_vertical_error_m ||
        !std::isfinite(scheduling.hold_enter_distance_m) ||
        !std::isfinite(scheduling.hold_exit_distance_m) ||
        scheduling.hold_enter_distance_m <= 0.0 ||
        scheduling.hold_enter_distance_m >=
            scheduling.hold_exit_distance_m ||
        scheduling.hold_exit_distance_m >=
            scheduling.brake_enter_distance_m ||
        scheduling.brake_enter_distance_m >=
            scheduling.brake_exit_distance_m ||
        !std::isfinite(scheduling.hold_enter_vertical_error_m) ||
        !std::isfinite(scheduling.hold_exit_vertical_error_m) ||
        scheduling.hold_enter_vertical_error_m <= 0.0 ||
        scheduling.hold_enter_vertical_error_m >=
            scheduling.hold_exit_vertical_error_m ||
        !std::isfinite(scheduling.hold_enter_speed_mps) ||
        !std::isfinite(scheduling.hold_exit_speed_mps) ||
        scheduling.hold_enter_speed_mps <= 0.0 ||
        scheduling.hold_enter_speed_mps >=
            scheduling.hold_exit_speed_mps ||
        !std::isfinite(
            scheduling.hold_enter_vertical_speed_mps) ||
        !std::isfinite(
            scheduling.hold_exit_vertical_speed_mps) ||
        scheduling.hold_enter_vertical_speed_mps <= 0.0 ||
        scheduling.hold_enter_vertical_speed_mps >=
            scheduling.hold_exit_vertical_speed_mps ||
        !std::isfinite(scheduling.target_motion_speed_mps) ||
        scheduling.target_motion_speed_mps < 0.0 ||
        !std::isfinite(scheduling.hold_enter_dwell_s) ||
        !std::isfinite(scheduling.hold_exit_dwell_s) ||
        !std::isfinite(scheduling.gain_transition_time_s) ||
        scheduling.hold_enter_dwell_s < 0.0 ||
        scheduling.hold_exit_dwell_s < 0.0 ||
        scheduling.gain_transition_time_s < 0.0 ||
        !std::isfinite(config.max_accel_mps2) ||
        config.max_accel_mps2 <= 0.0 ||
        !std::isfinite(config.max_tilt_rad) ||
        config.max_tilt_rad <= 0.0 || config.max_tilt_rad >= 1.4 ||
        !std::isfinite(config.min_throttle) ||
        !std::isfinite(config.hover_throttle) ||
        !std::isfinite(config.max_throttle) ||
        config.min_throttle < 0.0 ||
        config.hover_throttle < config.min_throttle ||
        config.hover_throttle > config.max_throttle ||
        config.max_throttle > 1.0 ||
        !std::isfinite(config.z_velocity_damping) ||
        config.z_velocity_damping < 0.0 ||
        config.z_velocity_damping > 0.5 ||
        !std::isfinite(config.max_yaw_rate_rad_s) ||
        config.max_yaw_rate_rad_s <= 0.0 ||
        config.max_yaw_rate_rad_s > 1.5 ||
        !std::isfinite(config.yaw_rate_kp) ||
        config.yaw_rate_kp < 0.0 || config.yaw_rate_kp > 10.0 ||
        !std::isfinite(config.yaw_rate_kd) ||
        config.yaw_rate_kd < 0.0 || config.yaw_rate_kd > 5.0 ||
        !std::isfinite(config.yaw_rate_slew_rate_rad_s2) ||
        config.yaw_rate_slew_rate_rad_s2 <= 0.0 ||
        config.yaw_rate_slew_rate_rad_s2 > 20.0 ||
        !std::isfinite(config.fc_yaw_rate_scale_rad_s) ||
        config.fc_yaw_rate_scale_rad_s <= 0.0 ||
        config.fc_yaw_rate_scale_rad_s > 3.0 ||
        !std::isfinite(config.max_position_error_m) ||
        config.max_position_error_m <= 0.0 ||
        !std::isfinite(config.gravity_mps2) ||
        config.gravity_mps2 <= 0.0) {
        throw std::invalid_argument(
            "invalid position PID configuration");
    }
}

double PositionController::update_position_axis(
    double error, double measured_velocity,
    double target_velocity, double dt_s, const AxisPidGains& gains,
    double initial_output, double minimum_output, double maximum_output,
    double additional_velocity_damping, bool integrate,
    AxisState& state) {
    if (!state.initialized) {
        state.integral =
            clamp(initial_output, minimum_output, maximum_output);
        state.initialized = true;
    }

    const auto axis_output =
        [&](double integral) {
            return gains.kp * error + integral +
                   gains.kd * target_velocity -
                   (gains.kd + additional_velocity_damping) *
                       measured_velocity;
        };
    const double proposed_integral =
        integrate
            ? state.integral + gains.ki * error * dt_s
            : state.integral;
    double candidate = axis_output(proposed_integral);
    const bool saturated_against_error =
        (candidate > maximum_output && error > 0.0) ||
        (candidate < minimum_output && error < 0.0);
    if (!std::isfinite(candidate) || saturated_against_error) {
        candidate = axis_output(state.integral);
    } else {
        state.integral = proposed_integral;
    }
    if (!std::isfinite(candidate)) {
        return clamp(axis_output(0.0), minimum_output, maximum_output);
    }
    return clamp(candidate, minimum_output, maximum_output);
}

void PositionController::smooth_toward(
    AxisPidGains& current, const AxisPidGains& target,
    double alpha) {
    current.kp += (target.kp - current.kp) * alpha;
    current.ki += (target.ki - current.ki) * alpha;
    current.kd += (target.kd - current.kd) * alpha;
}

void PositionController::set_config(
    const PositionControllerConfig& config) {
    validate(config);
    const std::lock_guard lock(config_mutex_);
    config_ = config;
}

PositionControllerConfig PositionController::config() const {
    return snapshot_config();
}

PositionControllerConfig PositionController::snapshot_config() const {
    const std::lock_guard lock(config_mutex_);
    return config_;
}

void PositionController::reset() {
    x_axis_ = {};
    y_axis_ = {};
    z_axis_ = {};
    previous_pose_.reset();
    velocity_xyz_mps_ = {};
    yaw_rate_dock_rad_s_ = 0.0;
    yaw_rate_command_dock_rad_s_ = 0.0;
    regulation_mode_ = PositionRegulationMode::FarTrack;
    active_x_gains_ = {};
    active_y_gains_ = {};
    active_z_gains_ = {};
    active_gains_initialized_ = false;
    hold_enter_since_ns_ = 0;
    hold_exit_since_ns_ = 0;
    previous_control_time_ns_ = 0;
}

PositionControllerOutput PositionController::step(
    const PoseSample& pose, const OperatorTarget& target,
    std::uint64_t control_time_ns,
    const ManualControlSetpoint& manual_base) {
    PositionControllerOutput output;
    const PositionControllerConfig config = snapshot_config();
    if (!pose.valid || !target.deadman ||
        !is_finite(pose.position_xyz_m) ||
        !is_finite(pose.q_body_to_world) ||
        !is_finite(target.position_xyz_m) ||
        !std::isfinite(target.yaw_rad) || !manual_base.valid ||
        !std::isfinite(manual_base.roll) ||
        !std::isfinite(manual_base.pitch) ||
        !std::isfinite(manual_base.yaw) ||
        !std::isfinite(manual_base.throttle)) {
        reset();
        return output;
    }

    const double current_yaw =
        yaw_from_quaternion(pose.q_body_to_world);
    if (previous_pose_.has_value() &&
        pose.received_mono_ns > previous_pose_->received_mono_ns) {
        const double dt_s =
            static_cast<double>(
                pose.received_mono_ns -
                previous_pose_->received_mono_ns) /
            1'000'000'000.0;
        if (dt_s >= 0.001 && dt_s <= 0.25) {
            const Vec3 raw_velocity =
                (pose.position_xyz_m -
                 previous_pose_->position_xyz_m) /
                dt_s;
            velocity_xyz_mps_ =
                velocity_xyz_mps_ * (1.0 - kVelocityFilterAlpha) +
                raw_velocity * kVelocityFilterAlpha;
            const double previous_yaw =
                yaw_from_quaternion(
                    previous_pose_->q_body_to_world);
            const double raw_yaw_rate =
                normalized_angle(current_yaw - previous_yaw) / dt_s;
            yaw_rate_dock_rad_s_ =
                yaw_rate_dock_rad_s_ *
                    (1.0 - kYawRateFilterAlpha) +
                raw_yaw_rate * kYawRateFilterAlpha;
        }
    }
    previous_pose_ = pose;

    double dt_s = kDefaultControlDtS;
    const std::uint64_t now_ns =
        control_time_ns != 0U ? control_time_ns : pose.received_mono_ns;
    if (previous_control_time_ns_ != 0U &&
        now_ns > previous_control_time_ns_) {
        dt_s = static_cast<double>(
                   now_ns - previous_control_time_ns_) /
               1'000'000'000.0;
        dt_s = clamp(dt_s, 0.001, 0.05);
    }
    previous_control_time_ns_ = now_ns;

    const Vec3 raw_error = target.position_xyz_m - pose.position_xyz_m;
    const Vec3 error{
        clamp(
            raw_error.x, -config.max_position_error_m,
            config.max_position_error_m),
        clamp(
            raw_error.y, -config.max_position_error_m,
            config.max_position_error_m),
        clamp(
            raw_error.z, -config.max_position_error_m,
            config.max_position_error_m),
    };

    const auto& scheduling = config.mode_scheduling;
    const double horizontal_error_m =
        std::hypot(raw_error.x, raw_error.y);
    const double vertical_error_m = std::abs(raw_error.z);
    const double horizontal_speed_mps =
        std::hypot(velocity_xyz_mps_.x, velocity_xyz_mps_.y);
    const double vertical_speed_mps =
        std::abs(velocity_xyz_mps_.z);
    const bool target_moving =
        norm(target.target_velocity_xyz_mps) >
        scheduling.target_motion_speed_mps;

    const auto elapsed_since = [now_ns](std::uint64_t since_ns) {
        if (since_ns == 0U || now_ns <= since_ns) {
            return 0.0;
        }
        return static_cast<double>(now_ns - since_ns) /
               1'000'000'000.0;
    };
    if (!scheduling.enabled) {
        regulation_mode_ = PositionRegulationMode::FarTrack;
        hold_enter_since_ns_ = 0;
        hold_exit_since_ns_ = 0;
    } else if (target_moving) {
        regulation_mode_ = PositionRegulationMode::FarTrack;
        hold_enter_since_ns_ = 0;
        hold_exit_since_ns_ = 0;
    } else {
        switch (regulation_mode_) {
            case PositionRegulationMode::FarTrack:
                hold_enter_since_ns_ = 0;
                hold_exit_since_ns_ = 0;
                if (horizontal_error_m <=
                        scheduling.brake_enter_distance_m &&
                    vertical_error_m <=
                        scheduling.brake_enter_vertical_error_m) {
                    regulation_mode_ =
                        PositionRegulationMode::Approach;
                }
                break;
            case PositionRegulationMode::Approach: {
                hold_exit_since_ns_ = 0;
                if (horizontal_error_m >
                        scheduling.brake_exit_distance_m ||
                    vertical_error_m >
                        scheduling.brake_exit_vertical_error_m) {
                    regulation_mode_ =
                        PositionRegulationMode::FarTrack;
                    hold_enter_since_ns_ = 0;
                    break;
                }
                const bool enter_hold =
                    horizontal_error_m <=
                        scheduling.hold_enter_distance_m &&
                    vertical_error_m <=
                        scheduling.hold_enter_vertical_error_m &&
                    horizontal_speed_mps <=
                        scheduling.hold_enter_speed_mps &&
                    vertical_speed_mps <=
                        scheduling.hold_enter_vertical_speed_mps;
                if (!enter_hold) {
                    hold_enter_since_ns_ = 0;
                } else if (hold_enter_since_ns_ == 0U) {
                    hold_enter_since_ns_ = now_ns;
                } else if (
                    elapsed_since(hold_enter_since_ns_) >=
                    scheduling.hold_enter_dwell_s) {
                    regulation_mode_ =
                        PositionRegulationMode::PrecisionHold;
                    hold_enter_since_ns_ = 0;
                }
                break;
            }
            case PositionRegulationMode::PrecisionHold: {
                hold_enter_since_ns_ = 0;
                if (horizontal_error_m >
                        scheduling.brake_exit_distance_m ||
                    vertical_error_m >
                        scheduling.brake_exit_vertical_error_m) {
                    regulation_mode_ =
                        PositionRegulationMode::FarTrack;
                    hold_exit_since_ns_ = 0;
                    break;
                }
                const bool exit_hold =
                    horizontal_error_m >
                        scheduling.hold_exit_distance_m ||
                    vertical_error_m >
                        scheduling.hold_exit_vertical_error_m ||
                    horizontal_speed_mps >
                        scheduling.hold_exit_speed_mps ||
                    vertical_speed_mps >
                        scheduling.hold_exit_vertical_speed_mps;
                if (!exit_hold) {
                    hold_exit_since_ns_ = 0;
                } else if (hold_exit_since_ns_ == 0U) {
                    hold_exit_since_ns_ = now_ns;
                } else if (
                    elapsed_since(hold_exit_since_ns_) >=
                    scheduling.hold_exit_dwell_s) {
                    regulation_mode_ =
                        PositionRegulationMode::Approach;
                    hold_exit_since_ns_ = 0;
                }
                break;
            }
        }
    }

    const PositionProfile target_profile =
        profile_for_mode(regulation_mode_, config);
    if (!active_gains_initialized_) {
        active_x_gains_ = target_profile.x;
        active_y_gains_ = target_profile.y;
        active_z_gains_ = target_profile.z;
        active_gains_initialized_ = true;
    } else {
        const double transition_time_s =
            scheduling.gain_transition_time_s;
        const double alpha =
            transition_time_s <= 1e-9
                ? 1.0
                : clamp(dt_s / transition_time_s, 0.0, 1.0);
        smooth_toward(active_x_gains_, target_profile.x, alpha);
        smooth_toward(active_y_gains_, target_profile.y, alpha);
        smooth_toward(active_z_gains_, target_profile.z, alpha);
    }
    const auto max_gain_difference = [](const AxisPidGains& active,
                                        const AxisPidGains& desired) {
        return std::max({
            std::abs(active.kp - desired.kp),
            std::abs(active.ki - desired.ki),
            std::abs(active.kd - desired.kd),
        });
    };
    const bool gain_transition_active =
        max_gain_difference(active_x_gains_, target_profile.x) > 1e-5 ||
        max_gain_difference(active_y_gains_, target_profile.y) > 1e-5 ||
        max_gain_difference(active_z_gains_, target_profile.z) > 1e-5;
    const bool integrate =
        !scheduling.enabled || !target_moving;

    const double x_output = update_position_axis(
        error.x, velocity_xyz_mps_.x, target.target_velocity_xyz_mps.x,
        dt_s, active_x_gains_, 0.0,
        -config.max_accel_mps2, config.max_accel_mps2,
        0.0, integrate, x_axis_);
    const double y_output = update_position_axis(
        error.y, velocity_xyz_mps_.y, target.target_velocity_xyz_mps.y,
        dt_s, active_y_gains_, 0.0,
        -config.max_accel_mps2, config.max_accel_mps2,
        0.0, integrate, y_axis_);
    const double z_output = update_position_axis(
        error.z, velocity_xyz_mps_.z, target.target_velocity_xyz_mps.z,
        dt_s, active_z_gains_,
        config.hover_throttle,
        config.min_throttle, config.max_throttle,
        config.z_velocity_damping, integrate, z_axis_);

    output.regulation.mode = regulation_mode_;
    output.regulation.scheduling_enabled = scheduling.enabled;
    output.regulation.gain_transition_active =
        gain_transition_active;
    output.regulation.horizontal_error_m = horizontal_error_m;
    output.regulation.vertical_error_m = vertical_error_m;
    output.regulation.horizontal_speed_mps = horizontal_speed_mps;
    output.regulation.vertical_speed_mps = vertical_speed_mps;
    output.regulation.active_x = active_x_gains_;
    output.regulation.active_y = active_y_gains_;
    output.regulation.active_z = active_z_gains_;

    Vec3 acceleration{x_output, y_output, 0.0};
    const double horizontal_limit = std::min(
        config.max_accel_mps2,
        config.gravity_mps2 * std::tan(config.max_tilt_rad));
    const double horizontal_norm = std::hypot(
        acceleration.x, acceleration.y);
    if (horizontal_norm > horizontal_limit &&
        horizontal_norm > 1e-12) {
        acceleration =
            acceleration * (horizontal_limit / horizontal_norm);
    }

    // DATA_RC roll/pitch are body-relative manual commands. Decompose the
    // world acceleration using the absolute VRPN heading, not the target
    // heading and not the FC's integrated heading estimate.
    const Quaternion body_relative_attitude =
        attitude_for_acceleration(
            acceleration, current_yaw, config.gravity_mps2);
    if (!is_finite(body_relative_attitude)) {
        reset();
        return output;
    }

    const double roll_flu =
        roll_from_quaternion(body_relative_attitude);
    const double pitch_flu =
        pitch_from_quaternion(body_relative_attitude);
    // Positive roll tilts thrust toward world -Y; positive pitch toward +X.
    const double roll_right = roll_flu;
    const double pitch_down = pitch_flu;

    const double yaw_error =
        normalized_angle(target.yaw_rad - current_yaw);
    const double requested_yaw_rate_dock_rad_s = clamp(
        config.yaw_rate_kp * yaw_error -
            config.yaw_rate_kd * yaw_rate_dock_rad_s_,
        -config.max_yaw_rate_rad_s, config.max_yaw_rate_rad_s);
    const double maximum_yaw_rate_step =
        config.yaw_rate_slew_rate_rad_s2 * dt_s;
    yaw_rate_command_dock_rad_s_ = clamp(
        yaw_rate_command_dock_rad_s_ +
            clamp(
                requested_yaw_rate_dock_rad_s -
                    yaw_rate_command_dock_rad_s_,
                -maximum_yaw_rate_step,
                maximum_yaw_rate_step),
        -config.max_yaw_rate_rad_s, config.max_yaw_rate_rad_s);

    const double pid_roll =
        clamp(roll_right / config.max_tilt_rad, -1.0, 1.0);
    const double pid_pitch =
        clamp(pitch_down / config.max_tilt_rad, -1.0, 1.0);
    // Dock yaw is left-positive; PX4 DATA_RC yaw is right-positive.
    const double pid_yaw = clamp(
        -yaw_rate_command_dock_rad_s_ /
            config.fc_yaw_rate_scale_rad_s,
        -1.0, 1.0);
    // The height loop owns the upper-throttle channel. Adding it to the
    // captured RC throttle can leave too little downward authority when
    // handoff occurs with an elevated stick. Runtime applies a bounded
    // transition from that captured value to this absolute command.
    const double z_command_output =
        clamp(z_output, config.min_throttle, config.max_throttle);
    const double absolute_throttle =
        clamp(2.0 * z_command_output - 1.0, -1.0, 1.0);

    output.manual.roll =
        clamp(manual_base.roll + pid_roll, -1.0, 1.0);
    output.manual.pitch =
        clamp(manual_base.pitch + pid_pitch, -1.0, 1.0);
    output.manual.yaw =
        clamp(manual_base.yaw + pid_yaw, -1.0, 1.0);
    output.manual.throttle = absolute_throttle;
    output.manual.valid =
        std::isfinite(output.manual.roll) &&
        std::isfinite(output.manual.pitch) &&
        std::isfinite(output.manual.yaw) &&
        std::isfinite(output.manual.throttle);
    output.upper_throttle =
        clamp(0.5 * (output.manual.throttle + 1.0), 0.0, 1.0);
    output.q_body_to_world_desired =
        attitude_for_acceleration(
            acceleration, target.yaw_rad, config.gravity_mps2);
    output.acceleration_xyz_mps2 = acceleration;
    output.pid_output_xyz = {
        acceleration.x, acceleration.y, z_command_output};
    output.velocity_xyz_mps = velocity_xyz_mps_;
    output.roll_right_command_rad =
        output.manual.roll * config.max_tilt_rad;
    output.pitch_down_command_rad =
        output.manual.pitch * config.max_tilt_rad;
    output.yaw_rate_right_command_rad_s =
        -yaw_rate_command_dock_rad_s_;
    output.valid =
        output.manual.valid &&
        std::isfinite(output.upper_throttle) &&
        is_finite(output.acceleration_xyz_mps2);
    output.regulation.active = output.valid;
    return output;
}

}  // namespace dock_ground
