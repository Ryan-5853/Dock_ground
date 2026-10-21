#include "dock_ground/pose_source.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace dock_ground {
namespace {

using namespace std::chrono_literals;

constexpr double kPi = 3.14159265358979323846;

}  // namespace

SimulatedPoseSource::SimulatedPoseSource(double rate_hz) : rate_hz_(rate_hz) {
    if (!std::isfinite(rate_hz_) || rate_hz_ < 10.0 || rate_hz_ > 500.0) {
        throw std::invalid_argument(
            "simulated pose rate must be in [10, 500] Hz");
    }
}

SimulatedPoseSource::~SimulatedPoseSource() {
    stop();
}

void SimulatedPoseSource::start() {
    if (running_.exchange(true)) {
        return;
    }
    thread_ = std::thread([this] { run(); });
}

void SimulatedPoseSource::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (thread_.joinable()) {
        thread_.join();
    }
}

std::optional<PoseSample> SimulatedPoseSource::latest() const {
    std::unique_lock lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock() || !has_sample_) {
        missed_reads_.fetch_add(1, std::memory_order_relaxed);
        return std::nullopt;
    }
    return latest_;
}

PoseSourceStatus SimulatedPoseSource::status() const {
    PoseSourceStatus result;
    result.connected = messages_received_.load(std::memory_order_relaxed) > 0;
    result.transport_connected = true;
    result.name = "simulated";
    result.sensor_id = 0;
    result.messages_received =
        messages_received_.load(std::memory_order_relaxed);
    return result;
}

std::vector<TrackerBodyInfo> SimulatedPoseSource::discovered_sensors() const {
    const std::uint64_t count =
        messages_received_.load(std::memory_order_relaxed);
    if (count == 0) {
        return {};
    }
    return {TrackerBodyInfo{
        .sensor_id = 0,
        .messages_received = count,
        .last_seen_mono_ns =
            last_received_ns_.load(std::memory_order_relaxed),
    }};
}

std::uint64_t SimulatedPoseSource::missed_reads() const {
    return missed_reads_.load(std::memory_order_relaxed);
}

PoseSample SimulatedPoseSource::sample(std::uint64_t elapsed_ns) {
    const double time_s =
        static_cast<double>(elapsed_ns) / 1'000'000'000.0;
    const double x = 0.55 * std::sin(0.45 * time_s);
    const double y = 0.35 * std::sin(0.31 * time_s + 0.6);
    const double z = 1.5 + 0.08 * std::sin(0.7 * time_s);
    const double yaw = 0.18 * std::sin(0.22 * time_s);

    PoseSample result;
    result.received_mono_ns = monotonic_ns();
    result.source_mono_ns = result.received_mono_ns;
    result.sequence = next_sequence_++;
    result.position_xyz_m = {x, y, z};
    result.q_body_to_world = {
        std::cos(yaw * 0.5), 0.0, 0.0, std::sin(yaw * 0.5)};
    result.source_time_valid = true;
    result.valid = true;
    return result;
}

void SimulatedPoseSource::run() {
    const auto period = std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(1.0 / rate_hz_));
    const auto started = SteadyClock::now();
    auto deadline = started;

    while (running_.load(std::memory_order_relaxed)) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
            SteadyClock::now() - started);
        const PoseSample next =
            sample(static_cast<std::uint64_t>(elapsed.count()));
        {
            const std::lock_guard lock(mutex_);
            latest_ = next;
            has_sample_ = true;
        }
        messages_received_.fetch_add(1, std::memory_order_relaxed);
        last_received_ns_.store(
            next.received_mono_ns, std::memory_order_relaxed);

        deadline += period;
        const auto now = SteadyClock::now();
        if (deadline <= now) {
            deadline = now + period;
        }
        std::this_thread::sleep_until(deadline);
    }
}

}  // namespace dock_ground
