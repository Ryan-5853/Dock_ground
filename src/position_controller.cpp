#include "dock_ground/position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dock_ground {
namespace {

constexpr double kVelocityFilterAlpha = 0.25;
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
    const Vec3& acceleration_xyz_mps2, double target_yaw_rad,
    double gravity_mps2) {
    const Vec3 thrust_specific_force{
        acceleration_xyz_mps2.x,
        acceleration_xyz_mps2.y,
        gravity_mps2,
    };
    const Vec3 body_z_up = normalized(thrust_specific_force);
    const Vec3 heading{
        std::cos(target_yaw_rad), std::sin(target_yaw_rad), 0.0};
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

[[nodiscard]] Quaternion quaternion_from_euler_zyx(
    double yaw_rad, double pitch_rad, double roll_rad) {
    const double cr = std::cos(0.5 * roll_rad);
    const double sr = std::sin(0.5 * roll_rad);
    const double cp = std::cos(0.5 * pitch_rad);
    const double sp = std::sin(0.5 * pitch_rad);
    const double cy = std::cos(0.5 * yaw_rad);
    const double sy = std::sin(0.5 * yaw_rad);
    return normalized({
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    });
}

void update_incremental_axis(
    double error, double dt_s, const AxisPidGains& gains,
    double initial_output, double minimum_output, double maximum_output,
    double& previous_error, double& previous_previous_error,
    bool& initialized, double& output) {
    const bool first_update = !initialized;
    double proportional = 0.0;
    double integral = 0.0;
    double derivative = 0.0;
    if (!initialized) {
        initialized = true;
        output = clamp(initial_output, minimum_output, maximum_output);
        proportional = gains.kp * error;
        previous_error = error;
        previous_previous_error = error;
    } else {
        proportional = gains.kp * (error - previous_error);
        derivative =
            gains.kd *
            (error - 2.0 * previous_error +
             previous_previous_error) /
            dt_s;
    }
    integral = gains.ki * error * dt_s;
    const double candidate =
        output + proportional + integral + derivative;
    output = std::isfinite(candidate)
                 ? clamp(candidate, minimum_output, maximum_output)
                 : output;
    if (!first_update) {
        previous_previous_error = previous_error;
        previous_error = error;
    }
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
    if (!valid_axis(config.x) || !valid_axis(config.y) ||
        !valid_axis(config.z) ||
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
        !std::isfinite(config.yaw_rate_kp) ||
        config.yaw_rate_kp < 0.0 ||
        !std::isfinite(config.max_position_error_m) ||
        config.max_position_error_m <= 0.0 ||
        !std::isfinite(config.gravity_mps2) ||
        config.gravity_mps2 <= 0.0) {
        throw std::invalid_argument(
            "invalid incremental position PID configuration");
    }
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

    update_incremental_axis(
        error.x, dt_s, config.x, 0.0,
        -config.max_accel_mps2, config.max_accel_mps2,
        x_axis_.previous_error, x_axis_.previous_previous_error,
        x_axis_.initialized, x_axis_.output);
    update_incremental_axis(
        error.y, dt_s, config.y, 0.0,
        -config.max_accel_mps2, config.max_accel_mps2,
        y_axis_.previous_error, y_axis_.previous_previous_error,
        y_axis_.initialized, y_axis_.output);
    update_incremental_axis(
        error.z, dt_s, config.z,
        clamp(
            0.5 * (manual_base.throttle + 1.0),
            config.min_throttle, config.max_throttle),
        config.min_throttle, config.max_throttle,
        z_axis_.previous_error, z_axis_.previous_previous_error,
        z_axis_.initialized, z_axis_.output);

    Vec3 acceleration{x_axis_.output, y_axis_.output, 0.0};
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
    x_axis_.output = acceleration.x;
    y_axis_.output = acceleration.y;

    const Quaternion incremental_attitude =
        attitude_for_acceleration(
            acceleration, target.yaw_rad, config.gravity_mps2);
    if (!is_finite(incremental_attitude)) {
        reset();
        return output;
    }

    const double roll_flu =
        roll_from_quaternion(incremental_attitude);
    const double pitch_flu =
        pitch_from_quaternion(incremental_attitude);
    // Positive roll tilts thrust toward world -Y; positive pitch toward +X.
    const double roll_right = roll_flu;
    const double pitch_down = pitch_flu;

    const double current_yaw =
        yaw_from_quaternion(pose.q_body_to_world);
    const double yaw_error =
        normalized_angle(target.yaw_rad - current_yaw);
    const double yaw_rate_dock_rad_s = clamp(
        config.yaw_rate_kp * yaw_error,
        -config.max_yaw_rate_rad_s, config.max_yaw_rate_rad_s);
    // Dock yaw is left-positive; PX4 DATA_RC yaw is right-positive.
    const double yaw_rate_right_rad_s = -yaw_rate_dock_rad_s;

    const double pid_roll =
        clamp(roll_right / config.max_tilt_rad, -1.0, 1.0);
    const double pid_pitch =
        clamp(pitch_down / config.max_tilt_rad, -1.0, 1.0);
    const double pid_yaw =
        clamp(
            yaw_rate_right_rad_s /
                config.max_yaw_rate_rad_s,
            -1.0, 1.0);
    // The height loop owns the upper-throttle channel. Adding it to the
    // captured RC throttle can leave too little downward authority when
    // handoff occurs with an elevated stick. Runtime applies a bounded
    // transition from that captured value to this absolute command.
    const double z_command_output = clamp(
        z_axis_.output -
            config.z_velocity_damping * velocity_xyz_mps_.z,
        config.min_throttle, config.max_throttle);
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
    const double final_roll_flu =
        output.manual.roll * config.max_tilt_rad;
    const double final_pitch_flu =
        output.manual.pitch * config.max_tilt_rad;
    output.q_body_to_world_desired =
        quaternion_from_euler_zyx(
            target.yaw_rad, final_pitch_flu, final_roll_flu);
    output.acceleration_xyz_mps2 = acceleration;
    output.pid_output_xyz = {
        x_axis_.output, y_axis_.output, z_command_output};
    output.velocity_xyz_mps = velocity_xyz_mps_;
    output.roll_right_command_rad =
        output.manual.roll * config.max_tilt_rad;
    output.pitch_down_command_rad =
        output.manual.pitch * config.max_tilt_rad;
    output.yaw_rate_right_command_rad_s =
        output.manual.yaw * config.max_yaw_rate_rad_s;
    output.valid =
        output.manual.valid &&
        std::isfinite(output.upper_throttle) &&
        is_finite(output.acceleration_xyz_mps2);
    return output;
}

}  // namespace dock_ground
