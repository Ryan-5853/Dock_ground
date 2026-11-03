#pragma once

#include "dock_ground/types.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace dock_ground {

class IPoseSource {
public:
    virtual ~IPoseSource() = default;
    virtual void start() = 0;
    virtual void stop() = 0;
    [[nodiscard]] virtual std::optional<PoseSample> latest() const = 0;
    [[nodiscard]] virtual PoseSourceStatus status() const = 0;
    virtual bool select_sensor(int sensor_id) {
        (void)sensor_id;
        return false;
    }
    [[nodiscard]] virtual std::vector<TrackerBodyInfo> discovered_sensors() const {
        return {};
    }
};

class SimulatedPoseSource final : public IPoseSource {
public:
    explicit SimulatedPoseSource(double rate_hz = 100.0);
    ~SimulatedPoseSource() override;

    SimulatedPoseSource(const SimulatedPoseSource&) = delete;
    SimulatedPoseSource& operator=(const SimulatedPoseSource&) = delete;

    void start();
    void stop();

    [[nodiscard]] std::optional<PoseSample> latest() const override;
    [[nodiscard]] PoseSourceStatus status() const override;
    [[nodiscard]] std::vector<TrackerBodyInfo> discovered_sensors() const override;
    [[nodiscard]] std::uint64_t missed_reads() const;

private:
    void run();
    [[nodiscard]] PoseSample sample(std::uint64_t elapsed_ns);

    double rate_hz_;
    std::atomic_bool running_{false};
    mutable std::atomic<std::uint64_t> missed_reads_{0};
    mutable std::mutex mutex_;
    PoseSample latest_{};
    bool has_sample_{false};
    std::uint32_t next_sequence_{1};
    std::atomic<std::uint64_t> messages_received_{0};
    std::atomic<std::uint64_t> last_received_ns_{0};
    std::thread thread_;
};

struct VrpnPoseSourceOptions {
    std::string endpoint{"MCAvatar@192.168.31.221:3883"};
    int sensor_id{-1};
    double position_scale{0.001};
    double expected_rate_hz{120.0};
    bool auto_origin{true};
    bool trust_source_timestamp{false};
    Mat3 axes_from_vrpn{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    };
};

class VrpnPoseSource final : public IPoseSource {
public:
    explicit VrpnPoseSource(VrpnPoseSourceOptions options);
    ~VrpnPoseSource() override;

    VrpnPoseSource(const VrpnPoseSource&) = delete;
    VrpnPoseSource& operator=(const VrpnPoseSource&) = delete;

    void start() override;
    void stop() override;
    [[nodiscard]] std::optional<PoseSample> latest() const override;
    [[nodiscard]] PoseSourceStatus status() const override;
    bool select_sensor(int sensor_id) override;
    [[nodiscard]] std::vector<TrackerBodyInfo> discovered_sensors() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dock_ground
