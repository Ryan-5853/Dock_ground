#include <vrpn_Tracker.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

using namespace std::chrono_literals;

struct Options {
    std::string endpoint;
    double duration_s{5.0};
    int sensor{-1};
    double expected_rate_hz{120.0};
    double gap_warning_ms{25.0};
    std::string csv_path;
    bool quiet{false};
};

struct LastSample {
    std::uint64_t received_mono_ns{0};
    std::uint64_t source_time_ns{0};
};

void emit_double(std::ostream& output, double value) {
    if (std::isfinite(value)) {
        output << value;
    } else {
        output << "nan";
    }
}

std::uint64_t wall_clock_ns() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

std::uint64_t source_time_ns(const timeval& timestamp) {
    if (timestamp.tv_sec < 0 || timestamp.tv_usec < 0) {
        return 0;
    }
    return static_cast<std::uint64_t>(timestamp.tv_sec) * 1'000'000'000ULL +
           static_cast<std::uint64_t>(timestamp.tv_usec) * 1'000ULL;
}

bool parse_double(
    std::string_view text, double& destination,
    double minimum, double maximum) {
    char* end = nullptr;
    const std::string input(text);
    const double value = std::strtod(input.c_str(), &end);
    if (end == input.c_str() || *end != '\0' ||
        !std::isfinite(value) || value < minimum || value > maximum) {
        return false;
    }
    destination = value;
    return true;
}

bool parse_int(
    std::string_view text, int& destination, int minimum, int maximum) {
    char* end = nullptr;
    const std::string input(text);
    const long value = std::strtol(input.c_str(), &end, 10);
    if (end == input.c_str() || *end != '\0' ||
        value < minimum || value > maximum) {
        return false;
    }
    destination = static_cast<int>(value);
    return true;
}

void print_usage(const char* program) {
    std::cerr
        << "Usage: " << program << " ENDPOINT [SECONDS]\n"
        << "Options:\n"
        << "  --duration SEC       Capture duration, default 5\n"
        << "  --sensor ID          Tracker sensor; -1 records every sensor\n"
        << "  --expected-rate-hz H Nominal source rate, default 120\n"
        << "  --gap-warning-ms MS  Live long-gap threshold, default 25\n"
        << "  --csv PATH           Write every callback to a CSV file\n"
        << "  --quiet              Suppress one-second live reports\n"
        << "ENDPOINT uses device@host:port, for example "
           "MCAvatar@192.168.31.221:3883\n";
}

std::optional<Options> parse_options(int argc, char** argv) {
    Options options;
    bool endpoint_set = false;
    for (int index = 1; index < argc; ++index) {
        const std::string_view argument(argv[index]);
        const auto next_value = [&](const char* name) -> const char* {
            if (index + 1 >= argc) {
                std::cerr << name << " requires a value\n";
                return nullptr;
            }
            return argv[++index];
        };

        if (argument == "--duration") {
            const char* value = next_value("--duration");
            if (value == nullptr ||
                !parse_double(value, options.duration_s, 0.1, 86'400.0)) {
                std::cerr << "invalid --duration\n";
                return std::nullopt;
            }
        } else if (argument == "--sensor") {
            const char* value = next_value("--sensor");
            if (value == nullptr ||
                !parse_int(value, options.sensor, -1, 1024)) {
                std::cerr << "invalid --sensor\n";
                return std::nullopt;
            }
        } else if (argument == "--expected-rate-hz") {
            const char* value = next_value("--expected-rate-hz");
            if (value == nullptr ||
                !parse_double(
                    value, options.expected_rate_hz, 1.0, 1'000.0)) {
                std::cerr << "invalid --expected-rate-hz\n";
                return std::nullopt;
            }
        } else if (argument == "--gap-warning-ms") {
            const char* value = next_value("--gap-warning-ms");
            if (value == nullptr ||
                !parse_double(
                    value, options.gap_warning_ms, 1.0, 60'000.0)) {
                std::cerr << "invalid --gap-warning-ms\n";
                return std::nullopt;
            }
        } else if (argument == "--csv") {
            const char* value = next_value("--csv");
            if (value == nullptr || *value == '\0') {
                std::cerr << "invalid --csv\n";
                return std::nullopt;
            }
            options.csv_path = value;
        } else if (argument == "--quiet") {
            options.quiet = true;
        } else if (argument.starts_with("--")) {
            std::cerr << "unknown option: " << argument << '\n';
            return std::nullopt;
        } else if (!endpoint_set) {
            options.endpoint = argument;
            endpoint_set = true;
        } else if (parse_double(
                       argument, options.duration_s, 0.1, 86'400.0)) {
            // Backward-compatible positional duration.
        } else {
            std::cerr << "unexpected argument: " << argument << '\n';
            return std::nullopt;
        }
    }
    if (!endpoint_set || options.endpoint.empty()) {
        std::cerr << "endpoint is required\n";
        return std::nullopt;
    }
    return options;
}

class Probe {
public:
    explicit Probe(const Options& options) : options_(options) {
        if (!options_.csv_path.empty()) {
            csv_.open(options_.csv_path, std::ios::out | std::ios::trunc);
            if (!csv_) {
                throw std::runtime_error(
                    "cannot open VRPN probe CSV: " + options_.csv_path);
            }
            csv_ << "schema,record_index,received_mono_ns,wall_time_ns,"
                    "sensor,source_time_ns,source_time_present,"
                    "source_clock_delta_ms,arrival_interval_ms,"
                    "source_time_interval_ms,source_time_regression,"
                    "raw_x,raw_y,raw_z,qx,qy,qz,qw,qnorm\n";
        }
    }

    static void VRPN_CALLBACK callback(
        void* userdata, const vrpn_TRACKERCB report) {
        static_cast<Probe*>(userdata)->record(report);
    }

    void record(const vrpn_TRACKERCB& report) {
        if (options_.sensor >= 0 && report.sensor != options_.sensor) {
            return;
        }
        const std::uint64_t received_ns = monotonic_ns();
        const std::uint64_t wall_ns = wall_clock_ns();
        const std::uint64_t source_ns = source_time_ns(report.msg_time);
        const bool source_present = source_ns != 0U;
        LastSample& previous = last_by_sensor_[report.sensor];

        double arrival_interval_ms = -1.0;
        if (previous.received_mono_ns != 0U &&
            received_ns >= previous.received_mono_ns) {
            arrival_interval_ms =
                static_cast<double>(
                    received_ns - previous.received_mono_ns) /
                1'000'000.0;
            intervals_ms_.push_back(arrival_interval_ms);
            max_gap_ms_ = std::max(max_gap_ms_, arrival_interval_ms);
            if (arrival_interval_ms > options_.gap_warning_ms) {
                ++long_gaps_;
            }
        }

        double source_interval_ms = -1.0;
        bool source_regression = false;
        if (source_present && previous.source_time_ns != 0U) {
            if (source_ns < previous.source_time_ns) {
                source_regression = true;
                ++source_regressions_;
            } else {
                source_interval_ms =
                    static_cast<double>(source_ns - previous.source_time_ns) /
                    1'000'000.0;
            }
        }

        const double source_clock_delta_ms =
            source_present
                ? (static_cast<double>(wall_ns) -
                   static_cast<double>(source_ns)) /
                      1'000'000.0
                : std::numeric_limits<double>::quiet_NaN();
        const double qnorm = std::sqrt(
            report.quat[0] * report.quat[0] +
            report.quat[1] * report.quat[1] +
            report.quat[2] * report.quat[2] +
            report.quat[3] * report.quat[3]);

        if (csv_) {
            csv_ << "1," << record_index_ << ',' << received_ns << ','
                 << wall_ns << ',' << report.sensor << ',' << source_ns
                 << ',' << source_present << ',';
            emit_double(csv_, source_clock_delta_ms);
            csv_ << ',';
            emit_double(csv_, arrival_interval_ms);
            csv_ << ',';
            emit_double(csv_, source_interval_ms);
            csv_ << ',' << source_regression << ',';
            emit_double(csv_, report.pos[0]);
            csv_ << ',';
            emit_double(csv_, report.pos[1]);
            csv_ << ',';
            emit_double(csv_, report.pos[2]);
            csv_ << ',';
            emit_double(csv_, report.quat[0]);
            csv_ << ',';
            emit_double(csv_, report.quat[1]);
            csv_ << ',';
            emit_double(csv_, report.quat[2]);
            csv_ << ',';
            emit_double(csv_, report.quat[3]);
            csv_ << ',';
            emit_double(csv_, qnorm);
            csv_ << '\n';
        }

        ++record_index_;
        ++total_samples_;
        ++interval_samples_;
        last_by_sensor_[report.sensor] =
            LastSample{received_ns, source_ns};
        last_sensor_ = report.sensor;
        last_received_ns_ = received_ns;
        last_source_time_ns_ = source_ns;
        last_clock_delta_ms_ = source_clock_delta_ms;
        last_position_ = {
            report.pos[0], report.pos[1], report.pos[2]};
        last_quaternion_ = {
            report.quat[3], report.quat[0], report.quat[1], report.quat[2]};
        last_qnorm_ = qnorm;
    }

    void one_second_report(
        double elapsed_s, double interval_elapsed_s) {
        const double rate =
            static_cast<double>(interval_samples_) /
            std::max(1e-9, interval_elapsed_s);
        std::cout << "t=" << std::fixed << std::setprecision(1)
                  << elapsed_s << "s samples/s=" << std::setprecision(1)
                  << rate << " total=" << total_samples_
                  << " max_gap_ms=" << std::setprecision(2)
                  << max_gap_ms_ << " long_gaps=" << long_gaps_ << '\n';
        std::cout << "  sensor=" << last_sensor_
                  << " source_clock_delta_ms=" << last_clock_delta_ms_
                  << " pos_raw_xyz=[" << last_position_[0] << ", "
                  << last_position_[1] << ", " << last_position_[2]
                  << "] qnorm=" << last_qnorm_ << '\n';
        interval_samples_ = 0;
    }

    void final_report(double elapsed_s) {
        if (csv_.is_open()) {
            csv_.flush();
            csv_.close();
        }
        std::vector<double> sorted = intervals_ms_;
        std::sort(sorted.begin(), sorted.end());
        const auto percentile = [&sorted](double value) {
            if (sorted.empty()) {
                return 0.0;
            }
            const double index =
                value * static_cast<double>(sorted.size() - 1U);
            return sorted[static_cast<std::size_t>(std::llround(index))];
        };
        std::cout << "RESULT"
                  << " duration_s=" << std::setprecision(3) << elapsed_s
                  << " samples=" << total_samples_
                  << " avg_rate_hz="
                  << (elapsed_s > 0.0
                          ? static_cast<double>(total_samples_) / elapsed_s
                          : 0.0)
                  << " gap_p50_ms=" << std::setprecision(3)
                  << percentile(0.50)
                  << " gap_p95_ms=" << percentile(0.95)
                  << " gap_p99_ms=" << percentile(0.99)
                  << " gap_max_ms=" << percentile(1.0)
                  << " long_gaps=" << long_gaps_
                  << " source_regressions=" << source_regressions_
                  << " last_source_clock_delta_ms="
                  << last_clock_delta_ms_ << '\n';
        if (!options_.csv_path.empty()) {
            std::cout << "CSV=" << options_.csv_path << '\n';
        }
    }

    [[nodiscard]] std::uint64_t total_samples() const {
        return total_samples_;
    }

private:
    [[nodiscard]] static std::uint64_t monotonic_ns() {
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
    }

    Options options_;
    std::ofstream csv_;
    std::map<int, LastSample> last_by_sensor_;
    std::vector<double> intervals_ms_;
    std::uint64_t record_index_{0};
    std::uint64_t total_samples_{0};
    std::uint64_t interval_samples_{0};
    std::uint64_t long_gaps_{0};
    std::uint64_t source_regressions_{0};
    std::uint64_t last_received_ns_{0};
    std::uint64_t last_source_time_ns_{0};
    double last_clock_delta_ms_{0.0};
    double max_gap_ms_{0.0};
    int last_sensor_{-1};
    std::array<double, 3> last_position_{};
    std::array<double, 4> last_quaternion_{1.0, 0.0, 0.0, 0.0};
    double last_qnorm_{1.0};
};

}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 &&
        (std::string_view(argv[1]) == "--help" ||
         std::string_view(argv[1]) == "-h")) {
        print_usage(argv[0]);
        return 0;
    }
    const auto options = parse_options(argc, argv);
    if (!options) {
        print_usage(argv[0]);
        return 2;
    }

    try {
        vrpn_Tracker_Remote tracker(options->endpoint.c_str());
        Probe probe(*options);
        if (tracker.register_change_handler(
                &probe, &Probe::callback, vrpn_ALL_SENSORS) != 0) {
            std::cerr << "failed to register VRPN tracker callback\n";
            return 1;
        }

        std::cout << "listening to " << options->endpoint
                  << " for " << options->duration_s
                  << " seconds; sensor=" << options->sensor
                  << " expected_rate_hz=" << options->expected_rate_hz
                  << " gap_warning_ms=" << options->gap_warning_ms << '\n';
        const auto started = std::chrono::steady_clock::now();
        auto next_report = started + 1s;
        auto report_started = started;
        while (std::chrono::steady_clock::now() - started <
               std::chrono::duration<double>(options->duration_s)) {
            tracker.mainloop();
            std::this_thread::sleep_for(1ms);
            const auto now = std::chrono::steady_clock::now();
            if (!options->quiet && now >= next_report) {
                const double report_elapsed =
                    std::chrono::duration<double>(
                        now - report_started)
                        .count();
                const double elapsed =
                    std::chrono::duration<double>(now - started).count();
                probe.one_second_report(elapsed, report_elapsed);
                report_started = now;
                next_report = now + 1s;
            }
        }
        const double elapsed =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - started)
                .count();
        probe.final_report(elapsed);
        return probe.total_samples() == 0U ? 3 : 0;
    } catch (const std::exception& exception) {
        std::cerr << "VRPN probe failed: " << exception.what() << '\n';
        return 1;
    }
}
