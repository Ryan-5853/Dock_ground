#pragma once

#include "dock_ground/types.hpp"

#include <mutex>
#include <optional>

namespace dock_ground {

using PositionControllerConfig = PositionLoopTuning;

struct PositionControllerOutput {
    Quaternion q_body_to_world_desired{};
    double upper_throttle{0.0};
    Vec3 acceleration_xyz_mps2{};
    Vec3 pid_output_xyz{};
    Vec3 velocity_xyz_mps{};
    ManualControlSetpoint manual{};
    double roll_right_command_rad{0.0};
    double pitch_down_command_rad{0.0};
    double yaw_rate_right_command_rad_s{0.0};
    PositionRegulationStatus regulation{};
    bool valid{false};
};

class PositionController {
public:
    explicit PositionController(PositionControllerConfig config);

    [[nodiscard]] PositionControllerOutput step(
        const PoseSample& pose, const OperatorTarget& target,
        std::uint64_t control_time_ns,
        const ManualControlSetpoint& manual_base);

    void reset();

    void set_config(const PositionControllerConfig& config);
    [[nodiscard]] PositionControllerConfig config() const;

private:
    struct AxisState {
        double integral{0.0};
        bool initialized{false};
    };

    static double update_position_axis(
        double error, double measured_velocity,
        double target_velocity, double dt_s, const AxisPidGains& gains,
        double initial_output, double minimum_output,
        double maximum_output, double additional_velocity_damping,
        bool integrate, AxisState& state);

    static void validate(const PositionControllerConfig& config);
    static void smooth_toward(
        AxisPidGains& current, const AxisPidGains& target,
        double alpha);

    PositionControllerConfig snapshot_config() const;

    PositionControllerConfig config_;
    mutable std::mutex config_mutex_;
    AxisState x_axis_;
    AxisState y_axis_;
    AxisState z_axis_;
    std::optional<PoseSample> previous_pose_;
    Vec3 velocity_xyz_mps_{};
    double yaw_rate_dock_rad_s_{0.0};
    double yaw_rate_command_dock_rad_s_{0.0};
    PositionRegulationMode regulation_mode_{
        PositionRegulationMode::FarTrack};
    AxisPidGains active_x_gains_{};
    AxisPidGains active_y_gains_{};
    AxisPidGains active_z_gains_{};
    bool active_gains_initialized_{false};
    std::uint64_t hold_enter_since_ns_{0};
    std::uint64_t hold_exit_since_ns_{0};
    std::uint64_t previous_control_time_ns_{0};
};

}  // namespace dock_ground
