#include "dock_ground/pose_source.hpp"

#include <vrpn_Tracker.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <map>
#include <mutex>
#include <thread>
#include <utility>

namespace dock_ground {
namespace {

using namespace std::chrono_literals;

constexpr std::uint64_t kConnectedTimeoutNs = 500'000'000ULL;
constexpr int kAutoSensor = -1;
constexpr int kNoActiveSensor = -2;

[[nodiscard]] Mat3 rotation_matrix(const Quaternion& quaternion) {
    const Quaternion q = normalized(quaternion);
    const double xx = q.x * q.x;
    const double yy = q.y * q.y;
    const double zz = q.z * q.z;
    const double xy = q.x * q.y;
    const double xz = q.x * q.z;
    const double yz = q.y * q.z;
    const double wx = q.w * q.x;
    const double wy = q.w * q.y;
    const double wz = q.w * q.z;
    return {
        1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
        2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
        2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy),
    };
}

[[nodiscard]] Quaternion transform_orientation(
    const Quaternion& source, const Mat3& from_source) {
    const Mat3 world_rotation = from_source * rotation_matrix(source) *
                                transpose(from_source);
    return quaternion_from_body_axes(
        {world_rotation[0], world_rotation[3], world_rotation[6]},
        {world_rotation[1], world_rotation[4], world_rotation[7]},
        {world_rotation[2], world_rotation[5], world_rotation[8]});
}

[[nodiscard]] std::uint64_t source_time_ns(const timeval& timestamp) {
    if (timestamp.tv_sec < 0 || timestamp.tv_usec < 0) {
        return 0;
    }
    return static_cast<std::uint64_t>(timestamp.tv_sec) * 1'000'000'000ULL +
           static_cast<std::uint64_t>(timestamp.tv_usec) * 1'000ULL;
}

[[nodiscard]] std::uint64_t wall_clock_ns() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

}  // namespace

struct VrpnPoseSource::Impl {
    explicit Impl(VrpnPoseSourceOptions input_options)
        : options(std::move(input_options)) {}

    void start() {
        if (running.exchange(true)) {
            return;
        }
        thread = std::thread([this] { run(); });
    }

    void stop() {
        if (!running.exchange(false)) {
            return;
        }
        if (thread.joinable()) {
            thread.join();
        }
    }

    void run() {
        vrpn_Tracker_Remote tracker(options.endpoint.c_str());
        if (tracker.register_change_handler(
                this, &Impl::tracker_callback, vrpn_ALL_SENSORS) != 0) {
            std::fprintf(
                stderr, "VRPN: failed to register tracker callback for %s\n",
                options.endpoint.c_str());
        }

        while (running.load(std::memory_order_relaxed)) {
            tracker.mainloop();
            vrpn_Connection* connection = tracker.connectionPtr();
            const bool connected =
                connection != nullptr && connection->connected() != 0;
            if (transport_connected.exchange(
                    connected, std::memory_order_relaxed) != connected) {
                if (connected) {
                    connect_count.fetch_add(
                        1U, std::memory_order_relaxed);
                } else {
                    disconnect_count.fetch_add(
                        1U, std::memory_order_relaxed);
                }
            }
            std::this_thread::sleep_for(1ms);
        }
    }

    static void VRPN_CALLBACK tracker_callback(
        void* userdata, const vrpn_TRACKERCB report) {
        static_cast<Impl*>(userdata)->on_tracker(report);
    }

    void on_tracker(const vrpn_TRACKERCB& report) {
        const std::uint64_t received_ns = monotonic_ns();
        callbacks_received.fetch_add(1U, std::memory_order_relaxed);
        {
            const std::lock_guard lock(body_mutex);
            TrackerBodyInfo& body = discovered_sensors[report.sensor];
            body.sensor_id = report.sensor;
            ++body.messages_received;
            body.last_seen_mono_ns = received_ns;
        }

        const std::uint64_t generation =
            binding_generation.load(std::memory_order_relaxed);
        if (generation != active_generation) {
            active_generation = generation;
            const int preferred =
                preferred_sensor.load(std::memory_order_relaxed);
            active_sensor.store(
                preferred == kAutoSensor ? kNoActiveSensor : preferred,
                std::memory_order_relaxed);
            origin_initialized = false;
            if (preferred >= 0) {
                std::fprintf(
                    stderr, "VRPN: binding switched to tracker sensor %d\n",
                    preferred);
            }
        }

        int active = active_sensor.load(std::memory_order_relaxed);
        const int preferred =
            preferred_sensor.load(std::memory_order_relaxed);
        if (active == kNoActiveSensor) {
            active = preferred >= 0 ? preferred : report.sensor;
            active_sensor.store(active, std::memory_order_relaxed);
            std::fprintf(
                stderr, "VRPN: selected tracker sensor %d from %s\n",
                active, options.endpoint.c_str());
        }
        if (preferred >= 0 && report.sensor != preferred) {
            unselected_sensor_samples.fetch_add(
                1U, std::memory_order_relaxed);
            return;
        }
        if (report.sensor != active) {
            unselected_sensor_samples.fetch_add(
                1U, std::memory_order_relaxed);
            return;
        }

        const Vec3 raw_position{
            report.pos[0], report.pos[1], report.pos[2]};
        if (!is_finite(raw_position)) {
            samples_rejected.fetch_add(1U, std::memory_order_relaxed);
            invalid_position_samples.fetch_add(
                1U, std::memory_order_relaxed);
            return;
        }

        Vec3 position_xyz =
            transform(options.axes_from_vrpn, raw_position) *
            options.position_scale;
        if (options.auto_origin && !origin_initialized) {
            // Keep the mapped vertical coordinate so the 3D ground plane and
            // target Z retain physical meaning; only level X/Y at the first
            // sample of the selected body.
            origin_ = {position_xyz.x, position_xyz.y, 0.0};
            origin_initialized = true;
            std::fprintf(
                stderr,
                "VRPN: local X/Y origin set; Z preserved at %.3f m\n",
                position_xyz.z);
        }
        position_xyz = position_xyz - origin_;
        if (!is_finite(position_xyz)) {
            samples_rejected.fetch_add(1U, std::memory_order_relaxed);
            invalid_position_samples.fetch_add(
                1U, std::memory_order_relaxed);
            return;
        }

        const Quaternion raw_orientation{
            report.quat[3],
            report.quat[0],
            report.quat[1],
            report.quat[2],
        };
        const double quaternion_norm = std::sqrt(
            raw_orientation.w * raw_orientation.w +
            raw_orientation.x * raw_orientation.x +
            raw_orientation.y * raw_orientation.y +
            raw_orientation.z * raw_orientation.z);
        if (!std::isfinite(quaternion_norm) || quaternion_norm < 1e-9) {
            samples_rejected.fetch_add(1U, std::memory_order_relaxed);
            invalid_orientation_samples.fetch_add(
                1U, std::memory_order_relaxed);
            return;
        }
        const Quaternion orientation =
            transform_orientation(raw_orientation, options.axes_from_vrpn);
        if (!is_finite(orientation)) {
            samples_rejected.fetch_add(1U, std::memory_order_relaxed);
            invalid_orientation_samples.fetch_add(
                1U, std::memory_order_relaxed);
            return;
        }

        PoseSample sample;
        sample.received_mono_ns = received_ns;
        sample.source_mono_ns = source_time_ns(report.msg_time);
        sample.source_time_present = sample.source_mono_ns != 0U;
        sample.sequence = next_sequence.fetch_add(1, std::memory_order_relaxed);
        sample.source_sensor_id = report.sensor;
        sample.position_xyz_m = position_xyz;
        sample.q_body_to_world = orientation;
        sample.raw_position = raw_position;
        sample.raw_orientation = raw_orientation;

        const std::uint64_t previous_received_ns =
            last_received_ns.load(std::memory_order_relaxed);
        if (previous_received_ns != 0U &&
            received_ns > previous_received_ns) {
            sample.arrival_interval_ns =
                received_ns - previous_received_ns;
            const double expected_period_ns =
                1'000'000'000.0 /
                std::max(1.0, options.expected_rate_hz);
            const std::uint64_t long_gap_threshold_ns =
                static_cast<std::uint64_t>(
                    std::max(20'000'000.0, 2.5 * expected_period_ns));
            last_arrival_gap_ns.store(
                sample.arrival_interval_ns,
                std::memory_order_relaxed);
            if (sample.arrival_interval_ns > long_gap_threshold_ns) {
                long_gap_count.fetch_add(
                    1U, std::memory_order_relaxed);
            }
            const std::uint64_t previous_max =
                max_arrival_gap_ns.load(std::memory_order_relaxed);
            if (sample.arrival_interval_ns > previous_max) {
                max_arrival_gap_ns.store(
                    sample.arrival_interval_ns,
                    std::memory_order_relaxed);
            }
        }

        if (sample.source_time_present) {
            sample.source_clock_delta_ns =
                static_cast<std::int64_t>(wall_clock_ns()) -
                static_cast<std::int64_t>(sample.source_mono_ns);
            const std::uint64_t previous_source_ns =
                last_source_time_ns.load(std::memory_order_relaxed);
            if (previous_source_ns != 0U) {
                if (sample.source_mono_ns == previous_source_ns) {
                    duplicate_timestamp_samples.fetch_add(
                        1U, std::memory_order_relaxed);
                } else if (sample.source_mono_ns < previous_source_ns) {
                    sample.source_time_regression = true;
                    timestamp_regression_samples.fetch_add(
                        1U, std::memory_order_relaxed);
                } else {
                    sample.source_time_interval_ns =
                        sample.source_mono_ns - previous_source_ns;
                }
            }
            last_source_time_ns.store(
                sample.source_mono_ns, std::memory_order_relaxed);
        }

        if (options.trust_source_timestamp && sample.source_time_present) {
            const std::int64_t signed_age_ns =
                sample.source_clock_delta_ns;
            // A timestamp is trusted only when both hosts are demonstrably in
            // the same clock domain. Large offsets fall back to receive age.
            if (signed_age_ns >= -50'000'000LL &&
                signed_age_ns <= 2'000'000'000LL) {
                sample.source_age_at_receive_ns =
                    signed_age_ns > 0
                        ? static_cast<std::uint64_t>(signed_age_ns)
                        : 0;
                sample.source_time_valid = true;
            }
        }
        sample.valid = true;

        {
            const std::lock_guard lock(mutex);
            latest_sample = sample;
            has_sample = true;
        }
        last_received_ns.store(received_ns, std::memory_order_relaxed);
        messages_received.fetch_add(1, std::memory_order_relaxed);
    }

    [[nodiscard]] std::optional<PoseSample> latest() const {
        std::unique_lock lock(mutex, std::try_to_lock);
        if (!lock.owns_lock() || !has_sample) {
            return std::nullopt;
        }
        return latest_sample;
    }

    [[nodiscard]] PoseSourceStatus status() const {
        PoseSourceStatus result;
        const std::uint64_t last =
            last_received_ns.load(std::memory_order_relaxed);
        result.connected =
            last != 0 && monotonic_ns() - last <= kConnectedTimeoutNs;
        result.transport_connected =
            transport_connected.load(std::memory_order_relaxed);
        result.name = "vrpn";
        const int preferred =
            preferred_sensor.load(std::memory_order_relaxed);
        const int active = active_sensor.load(std::memory_order_relaxed);
        result.sensor_id =
            preferred >= 0
                ? static_cast<std::int32_t>(preferred)
                : (active == kNoActiveSensor
                       ? -1
                       : static_cast<std::int32_t>(active));
        result.messages_received =
            messages_received.load(std::memory_order_relaxed);
        result.callbacks_received =
            callbacks_received.load(std::memory_order_relaxed);
        result.samples_rejected =
            samples_rejected.load(std::memory_order_relaxed);
        result.unselected_sensor_samples =
            unselected_sensor_samples.load(std::memory_order_relaxed);
        result.invalid_position_samples =
            invalid_position_samples.load(std::memory_order_relaxed);
        result.invalid_orientation_samples =
            invalid_orientation_samples.load(std::memory_order_relaxed);
        result.duplicate_timestamp_samples =
            duplicate_timestamp_samples.load(std::memory_order_relaxed);
        result.timestamp_regression_samples =
            timestamp_regression_samples.load(std::memory_order_relaxed);
        result.long_gap_count =
            long_gap_count.load(std::memory_order_relaxed);
        result.connect_count =
            connect_count.load(std::memory_order_relaxed);
        result.disconnect_count =
            disconnect_count.load(std::memory_order_relaxed);
        result.last_received_mono_ns = last;
        result.expected_rate_hz = options.expected_rate_hz;
        const std::uint64_t last_gap_ns =
            last_arrival_gap_ns.load(std::memory_order_relaxed);
        result.last_arrival_gap_ms =
            last_gap_ns == 0U
                ? -1.0
                : static_cast<double>(last_gap_ns) / 1'000'000.0;
        result.max_arrival_gap_ms =
            static_cast<double>(
                max_arrival_gap_ns.load(std::memory_order_relaxed)) /
            1'000'000.0;
        return result;
    }

    bool select_sensor(int sensor_id) {
        if (sensor_id < 0) {
            return false;
        }
        preferred_sensor.store(sensor_id, std::memory_order_relaxed);
        binding_generation.fetch_add(1, std::memory_order_relaxed);
        last_received_ns.store(0, std::memory_order_relaxed);
        last_source_time_ns.store(0, std::memory_order_relaxed);
        last_arrival_gap_ns.store(0, std::memory_order_relaxed);
        {
            const std::lock_guard lock(mutex);
            has_sample = false;
            latest_sample = {};
        }
        return true;
    }

    [[nodiscard]] std::vector<TrackerBodyInfo> bodies() const {
        std::vector<TrackerBodyInfo> result;
        const std::lock_guard lock(body_mutex);
        result.reserve(discovered_sensors.size());
        for (const auto& [sensor_id, body] : discovered_sensors) {
            (void)sensor_id;
            result.push_back(body);
        }
        return result;
    }

    VrpnPoseSourceOptions options;
    std::atomic_bool running{false};
    std::thread thread;

    mutable std::mutex mutex;
    PoseSample latest_sample{};
    bool has_sample{false};

    std::atomic<std::int32_t> preferred_sensor{options.sensor_id};
    std::atomic<std::int32_t> active_sensor{kNoActiveSensor};
    std::atomic<std::uint64_t> binding_generation{0};
    std::uint64_t active_generation{0};
    std::atomic<std::uint64_t> last_received_ns{0};
    std::atomic<std::uint64_t> messages_received{0};
    std::atomic<std::uint64_t> callbacks_received{0};
    std::atomic<std::uint64_t> samples_rejected{0};
    std::atomic<std::uint64_t> unselected_sensor_samples{0};
    std::atomic<std::uint64_t> invalid_position_samples{0};
    std::atomic<std::uint64_t> invalid_orientation_samples{0};
    std::atomic<std::uint64_t> duplicate_timestamp_samples{0};
    std::atomic<std::uint64_t> timestamp_regression_samples{0};
    std::atomic<std::uint64_t> long_gap_count{0};
    std::atomic<std::uint64_t> last_arrival_gap_ns{0};
    std::atomic<std::uint64_t> max_arrival_gap_ns{0};
    std::atomic<std::uint64_t> last_source_time_ns{0};
    std::atomic<std::uint64_t> connect_count{0};
    std::atomic<std::uint64_t> disconnect_count{0};
    std::atomic<std::uint32_t> next_sequence{1};
    std::atomic_bool transport_connected{false};

    Vec3 origin_{};
    bool origin_initialized{false};

    mutable std::mutex body_mutex;
    std::map<int, TrackerBodyInfo> discovered_sensors;
};

VrpnPoseSource::VrpnPoseSource(VrpnPoseSourceOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}

VrpnPoseSource::~VrpnPoseSource() {
    stop();
}

void VrpnPoseSource::start() {
    impl_->start();
}

void VrpnPoseSource::stop() {
    impl_->stop();
}

std::optional<PoseSample> VrpnPoseSource::latest() const {
    return impl_->latest();
}

PoseSourceStatus VrpnPoseSource::status() const {
    return impl_->status();
}

bool VrpnPoseSource::select_sensor(int sensor_id) {
    return impl_->select_sensor(sensor_id);
}

std::vector<TrackerBodyInfo> VrpnPoseSource::discovered_sensors() const {
    return impl_->bodies();
}

}  // namespace dock_ground
