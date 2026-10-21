#include <vrpn_Tracker.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <thread>

namespace {

using namespace std::chrono_literals;

struct SensorState {
    std::uint64_t count{0};
    double last_x{0.0};
    double last_y{0.0};
    double last_z{0.0};
    double qx{0.0};
    double qy{0.0};
    double qz{0.0};
    double qw{1.0};
    bool source_clock_delta_valid{false};
    double source_clock_delta_ms{0.0};
};

class Probe {
public:
    static void VRPN_CALLBACK callback(
        void* userdata, const vrpn_TRACKERCB report) {
        static_cast<Probe*>(userdata)->record(report);
    }

    void record(const vrpn_TRACKERCB& report) {
        SensorState& state = sensors[report.sensor];
        ++state.count;
        state.last_x = report.pos[0];
        state.last_y = report.pos[1];
        state.last_z = report.pos[2];
        state.qx = report.quat[0];
        state.qy = report.quat[1];
        state.qz = report.quat[2];
        state.qw = report.quat[3];
        const auto source_ns =
            static_cast<std::int64_t>(report.msg_time.tv_sec) * 1'000'000'000LL +
            static_cast<std::int64_t>(report.msg_time.tv_usec) * 1'000LL;
        const auto now_ns = static_cast<std::int64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count());
        const auto age_ns = now_ns - source_ns;
        state.source_clock_delta_valid =
            age_ns >= -50'000'000LL && age_ns <= 2'000'000'000LL;
        state.source_clock_delta_ms =
            state.source_clock_delta_valid
                ? static_cast<double>(std::max<std::int64_t>(0, age_ns)) /
                      1'000'000.0
                : 0.0;
    }

    void print(std::uint64_t previous_count) const {
        for (const auto& [sensor, state] : sensors) {
            std::cout << "sensor " << sensor << " total=" << state.count
                      << " pos_raw_xyz=[" << state.last_x << ", "
                      << state.last_y << ", " << state.last_z << "]"
                      << " quat_xyzw=[" << state.qx << ", " << state.qy << ", "
                      << state.qz << ", " << state.qw << "]";
            if (state.source_clock_delta_valid) {
                // This is a clock-domain delta, not one-way delay, unless PTP
                // or NTP synchronization has been verified independently.
                std::cout << " source_clock_delta_ms="
                          << state.source_clock_delta_ms;
            }
            std::cout << '\n';
        }
        if (sensors.empty()) {
            std::cout << "no tracker samples"
                      << " (previous_total=" << previous_count << ")\n";
        }
    }

    std::uint64_t total() const {
        std::uint64_t result = 0;
        for (const auto& [sensor, state] : sensors) {
            (void)sensor;
            result += state.count;
        }
        return result;
    }

    std::map<int, SensorState> sensors;
};

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        std::cerr << "Usage: " << argv[0]
                  << " device@host:port [seconds]\n";
        return 2;
    }
    double duration_s = 5.0;
    if (argc == 3) {
        duration_s = std::strtod(argv[2], nullptr);
        if (!std::isfinite(duration_s) || duration_s <= 0.0 ||
            duration_s > 3600.0) {
            std::cerr << "invalid duration\n";
            return 2;
        }
    }

    vrpn_Tracker_Remote tracker(argv[1]);
    Probe probe;
    if (tracker.register_change_handler(
            &probe, &Probe::callback, vrpn_ALL_SENSORS) != 0) {
        std::cerr << "failed to register VRPN tracker callback\n";
        return 1;
    }

    std::cout << "listening to " << argv[1] << " for " << duration_s
              << " seconds\n";
    const auto started = std::chrono::steady_clock::now();
    auto next_report = started + 1s;
    std::uint64_t previous_total = 0;

    while (std::chrono::steady_clock::now() - started <
           std::chrono::duration<double>(duration_s)) {
        tracker.mainloop();
        std::this_thread::sleep_for(1ms);
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_report) {
            const std::uint64_t total = probe.total();
            std::cout << "--- t="
                      << std::chrono::duration<double>(now - started).count()
                      << "s samples/s=" << (total - previous_total) << " ---\n";
            probe.print(previous_total);
            previous_total = total;
            next_report += 1s;
        }
    }
    return 0;
}
