#include "dock_ground/web_server.hpp"

#include "dock_ground/json.hpp"
#include "dock_ground/web_protocol.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace dock_ground {
namespace {

using namespace std::chrono_literals;

constexpr std::size_t kMaxHttpHeaderBytes = 16U * 1024U;
constexpr std::size_t kMaxClientInputBytes = 256U * 1024U;
constexpr std::size_t kMaxClientOutputBytes = 1024U * 1024U;
constexpr std::size_t kMaxStaticFileBytes = 16U * 1024U * 1024U;
constexpr std::size_t kMaxWebSocketClients = 32U;

constexpr std::uint8_t kOpcodeText = 0x1U;
constexpr std::uint8_t kOpcodeClose = 0x8U;
constexpr std::uint8_t kOpcodePing = 0x9U;
constexpr std::uint8_t kOpcodePong = 0xAU;

std::string trim(std::string_view value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return std::string(value.substr(first, last - first + 1U));
}

std::string lowercase(std::string value) {
    std::transform(
        value.begin(), value.end(), value.begin(),
        [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return value;
}

std::string url_decode(std::string_view input, bool& valid) {
    valid = true;
    std::string output;
    output.reserve(input.size());
    const auto hex_value = [](char character) -> int {
        if (character >= '0' && character <= '9') {
            return character - '0';
        }
        if (character >= 'a' && character <= 'f') {
            return 10 + character - 'a';
        }
        if (character >= 'A' && character <= 'F') {
            return 10 + character - 'A';
        }
        return -1;
    };

    for (std::size_t index = 0; index < input.size(); ++index) {
        if (input[index] != '%') {
            output.push_back(input[index]);
            continue;
        }
        if (index + 2U >= input.size()) {
            valid = false;
            return {};
        }
        const int high = hex_value(input[index + 1U]);
        const int low = hex_value(input[index + 2U]);
        if (high < 0 || low < 0) {
            valid = false;
            return {};
        }
        output.push_back(static_cast<char>((high << 4) | low));
        index += 2U;
    }
    return output;
}

const char* mime_type(const std::filesystem::path& path) {
    const std::string extension = lowercase(path.extension().string());
    if (extension == ".html") {
        return "text/html; charset=utf-8";
    }
    if (extension == ".js" || extension == ".mjs") {
        return "text/javascript; charset=utf-8";
    }
    if (extension == ".css") {
        return "text/css; charset=utf-8";
    }
    if (extension == ".json") {
        return "application/json; charset=utf-8";
    }
    if (extension == ".svg") {
        return "image/svg+xml";
    }
    if (extension == ".png") {
        return "image/png";
    }
    if (extension == ".ico") {
        return "image/x-icon";
    }
    if (extension == ".wasm") {
        return "application/wasm";
    }
    if (extension == ".map") {
        return "application/json";
    }
    return "application/octet-stream";
}

std::string http_response(
    int status, std::string_view reason, std::string content_type,
    std::string body, std::string_view cache_control = "no-store") {
    std::ostringstream response;
    response << "HTTP/1.1 " << status << ' ' << reason << "\r\n";
    response << "Content-Type: " << content_type << "\r\n";
    response << "Content-Length: " << body.size() << "\r\n";
    response << "Cache-Control: " << cache_control << "\r\n";
    response << "X-Content-Type-Options: nosniff\r\n";
    response << "Connection: close\r\n\r\n";
    response << body;
    return response.str();
}

std::string simple_json_error(std::string_view message) {
    return "{\"v\":1,\"type\":\"error\",\"message\":\"" +
           json::escape(message) + "\"}";
}

bool read_file(
    const std::filesystem::path& path, std::string& contents) {
    std::error_code error;
    const auto size = std::filesystem::file_size(path, error);
    if (error || size > kMaxStaticFileBytes) {
        return false;
    }
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        return false;
    }
    contents.resize(static_cast<std::size_t>(size));
    if (!contents.empty()) {
        stream.read(contents.data(), static_cast<std::streamsize>(size));
    }
    return stream.good() || stream.eof();
}

struct HttpRequest {
    std::string method;
    std::string target;
    std::string version;
    std::map<std::string, std::string, std::less<>> headers;
};

std::optional<HttpRequest> parse_http_request(
    std::string_view header_block, std::string& error) {
    HttpRequest result;
    const std::size_t line_end = header_block.find("\r\n");
    if (line_end == std::string_view::npos) {
        error = "invalid HTTP request line";
        return std::nullopt;
    }
    std::istringstream request_line(
        std::string(header_block.substr(0, line_end)));
    if (!(request_line >> result.method >> result.target >> result.version)) {
        error = "invalid HTTP request line";
        return std::nullopt;
    }
    if (result.version != "HTTP/1.1" && result.version != "HTTP/1.0") {
        error = "unsupported HTTP version";
        return std::nullopt;
    }

    std::size_t position = line_end + 2U;
    while (position < header_block.size()) {
        const std::size_t end = header_block.find("\r\n", position);
        if (end == position) {
            break;
        }
        const std::string_view line =
            end == std::string_view::npos
                ? header_block.substr(position)
                : header_block.substr(position, end - position);
        const std::size_t colon = line.find(':');
        if (colon == std::string_view::npos) {
            error = "invalid HTTP header";
            return std::nullopt;
        }
        const std::string name = lowercase(trim(line.substr(0, colon)));
        const std::string value = trim(line.substr(colon + 1U));
        result.headers[name] = value;
        if (end == std::string_view::npos) {
            break;
        }
        position = end + 2U;
    }
    return result;
}

enum class ClientMode {
    Http,
    WebSocket,
};

struct Client {
    int fd{-1};
    ClientMode mode{ClientMode::Http};
    std::string input;
    std::string output;
    bool close_after_write{false};
    std::uint64_t intent_count{0};
};

}  // namespace

struct WebServer::Impl {
    Impl(
        WebServerOptions input_options, IntentHandler input_intent_handler,
        BindingHandler input_binding_handler,
        UplinkHandler input_uplink_handler,
        TuningHandler input_tuning_handler,
        SnapshotProvider input_snapshot_provider)
        : options(std::move(input_options)),
          intent_handler(std::move(input_intent_handler)),
          binding_handler(std::move(input_binding_handler)),
          uplink_handler(std::move(input_uplink_handler)),
          tuning_handler(std::move(input_tuning_handler)),
          snapshot_provider(std::move(input_snapshot_provider)) {
        std::error_code error;
        const auto root = std::filesystem::weakly_canonical(
            std::filesystem::absolute(options.web_root), error);
        if (!error) {
            web_root = root;
        }
    }

    ~Impl() {
        stop();
    }

    bool start(std::string& error) {
        if (running_.load()) {
            return true;
        }

        listen_fd = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
        if (listen_fd < 0) {
            error = "socket(): " + std::string(std::strerror(errno));
            return false;
        }

        const int enabled = 1;
        if (::setsockopt(
                listen_fd, SOL_SOCKET, SO_REUSEADDR, &enabled,
                sizeof(enabled)) < 0) {
            error = "setsockopt(SO_REUSEADDR): " +
                    std::string(std::strerror(errno));
            ::close(listen_fd);
            listen_fd = -1;
            return false;
        }

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(options.port);
        if (::inet_pton(AF_INET, options.bind_address.c_str(), &address.sin_addr) !=
            1) {
            error = "invalid IPv4 bind address: " + options.bind_address;
            ::close(listen_fd);
            listen_fd = -1;
            return false;
        }
        if (::bind(
                listen_fd, reinterpret_cast<sockaddr*>(&address),
                sizeof(address)) < 0) {
            error = "bind(" + options.bind_address + ":" +
                    std::to_string(options.port) + "): " +
                    std::string(std::strerror(errno));
            ::close(listen_fd);
            listen_fd = -1;
            return false;
        }
        if (::listen(listen_fd, 32) < 0) {
            error = "listen(): " + std::string(std::strerror(errno));
            ::close(listen_fd);
            listen_fd = -1;
            return false;
        }
        if (!set_nonblocking(listen_fd)) {
            error = "cannot set listener nonblocking";
            ::close(listen_fd);
            listen_fd = -1;
            return false;
        }

        sockaddr_in bound{};
        socklen_t bound_length = sizeof(bound);
        if (::getsockname(
                listen_fd, reinterpret_cast<sockaddr*>(&bound),
                &bound_length) == 0) {
            bound_port = ntohs(bound.sin_port);
        }
        started_at = monotonic_ns();
        running_.store(true);
        thread = std::thread([this] { run(); });
        return true;
    }

    void stop() {
        if (!running_.exchange(false)) {
            return;
        }
        if (thread.joinable()) {
            thread.join();
        }
        if (listen_fd >= 0) {
            ::close(listen_fd);
            listen_fd = -1;
        }
        for (auto& client : clients) {
            close_client(*client);
        }
        clients.clear();
    }

    static bool set_nonblocking(int fd) {
        const int flags = ::fcntl(fd, F_GETFL, 0);
        return flags >= 0 && ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
    }

    void close_client(Client& client) {
        if (client.fd >= 0) {
            ::close(client.fd);
            client.fd = -1;
        }
    }

    void accept_clients() {
        while (true) {
            sockaddr_in peer{};
            socklen_t length = sizeof(peer);
            const int fd = ::accept4(
                listen_fd, reinterpret_cast<sockaddr*>(&peer), &length,
                SOCK_NONBLOCK | SOCK_CLOEXEC);
            if (fd < 0) {
                if (errno == EINTR) {
                    continue;
                }
                return;
            }
            if (clients.size() >= kMaxWebSocketClients) {
                ::close(fd);
                continue;
            }
            const int enabled = 1;
            ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
            auto client = std::make_unique<Client>();
            client->fd = fd;
            clients.push_back(std::move(client));
        }
    }

    void queue(Client& client, std::string bytes) {
        if (client.output.size() + bytes.size() > kMaxClientOutputBytes) {
            client.close_after_write = false;
            close_client(client);
            return;
        }
        client.output.append(std::move(bytes));
    }

    void queue_websocket(
        Client& client, std::string_view payload,
        std::uint8_t opcode = kOpcodeText) {
        queue(client, web::encode_websocket_frame(payload, opcode));
    }

    void close_websocket(Client& client, std::uint16_t code) {
        std::array<char, 2> payload{
            static_cast<char>((code >> 8U) & 0xFFU),
            static_cast<char>(code & 0xFFU),
        };
        queue_websocket(
            client, std::string_view(payload.data(), payload.size()),
            kOpcodeClose);
        client.close_after_write = true;
    }

    bool route_http(Client& client, const HttpRequest& request) {
        if (request.method != "GET") {
            queue(
                client, http_response(
                            405, "Method Not Allowed", "application/json",
                            simple_json_error("only GET is supported")));
            client.close_after_write = true;
            return true;
        }

        const std::string_view path =
            std::string_view(request.target).substr(
                0, request.target.find('?'));
        if (path == "/ws/v1/realtime") {
            const auto upgrade = request.headers.find("upgrade");
            const auto key = request.headers.find("sec-websocket-key");
            const auto version = request.headers.find("sec-websocket-version");
            if (upgrade == request.headers.end() ||
                lowercase(upgrade->second) != "websocket" ||
                key == request.headers.end() || version == request.headers.end() ||
                version->second != "13") {
                queue(
                    client,
                    http_response(
                        400, "Bad Request", "application/json",
                        simple_json_error("invalid WebSocket handshake")));
                client.close_after_write = true;
                return false;
            }
            std::ostringstream response;
            response << "HTTP/1.1 101 Switching Protocols\r\n";
            response << "Upgrade: websocket\r\n";
            response << "Connection: Upgrade\r\n";
            response << "Sec-WebSocket-Accept: "
                     << web::websocket_accept(key->second) << "\r\n\r\n";
            queue(client, response.str());
            client.mode = ClientMode::WebSocket;
            return true;
        }

        if (path == "/api/v1/health") {
            std::ostringstream body;
            body << "{\"v\":1,\"status\":\"ok\","
                 << "\"service\":\"dock-ground-station\","
                 << "\"version\":\"0.1.0\",\"uptime_ms\":"
                 << (monotonic_ns() - started_at) / 1'000'000U << '}';
            queue(
                client,
                http_response(
                    200, "OK", "application/json; charset=utf-8",
                    body.str()));
            client.close_after_write = true;
            return true;
        }

        if (path == "/api/v1/status") {
            queue(
                client,
                http_response(
                    200, "OK", "application/json; charset=utf-8",
                    snapshot_provider ? snapshot_provider() : "{}"));
            client.close_after_write = true;
            return true;
        }

        if (path == "/api/v1/events") {
            queue(
                client,
                http_response(
                    200, "OK", "application/json; charset=utf-8",
                    "{\"v\":1,\"events\":[]}"));
            client.close_after_write = true;
            return true;
        }

        bool valid_path = false;
        std::string decoded = url_decode(path, valid_path);
        if (!valid_path || decoded.find("..") != std::string::npos) {
            queue(
                client,
                http_response(
                    400, "Bad Request", "application/json",
                    simple_json_error("invalid path")));
            client.close_after_write = true;
            return true;
        }
        if (decoded == "/" || decoded.empty()) {
            decoded = "/index.html";
        }

        std::error_code path_error;
        const auto request_path = std::filesystem::weakly_canonical(
            web_root / std::filesystem::path(decoded.substr(1U)), path_error);
        const auto relative_path =
            request_path.lexically_relative(web_root);
        const bool outside_web_root =
            relative_path.empty() ||
            *relative_path.begin() == std::filesystem::path("..");
        if (path_error || web_root.empty() || outside_web_root ||
            !std::filesystem::is_regular_file(request_path, path_error)) {
            queue(
                client,
                http_response(
                    404, "Not Found", "application/json",
                    simple_json_error("resource not found")));
            client.close_after_write = true;
            return true;
        }

        std::string contents;
        if (!read_file(request_path, contents)) {
            queue(
                client,
                http_response(
                    500, "Internal Server Error", "application/json",
                    simple_json_error("cannot read static file")));
            client.close_after_write = true;
            return true;
        }
        const std::string_view cache_control =
            request_path.extension() == ".html"
                ? std::string_view("no-store")
                : std::string_view("public, max-age=31536000, immutable");
        queue(
            client,
            http_response(
                200, "OK", mime_type(request_path), std::move(contents),
                cache_control));
        client.close_after_write = true;
        return true;
    }

    void handle_position_tuning(
        Client& client, const json::Value& root) {
        const auto* tuning_value = root.find("tuning");
        if (tuning_value == nullptr ||
            tuning_value->type != json::Value::Type::Object) {
            queue_websocket(
                client,
                simple_json_error(
                    "position_tuning.tuning object is required"));
            return;
        }

        PositionLoopTuning tuning;
        std::string error;
        const auto read_number =
            [&tuning_value](
                std::string_view name, double& destination,
                std::string& read_error) -> bool {
            const auto* value = tuning_value->find(name);
            if (value == nullptr ||
                value->type != json::Value::Type::Number ||
                !std::isfinite(value->number)) {
                read_error = "missing or invalid number '" +
                             std::string(name) + "'";
                return false;
            }
            destination = value->number;
            return true;
        };
        const auto read_axis_from =
            [](const json::Value* object, std::string_view name,
               AxisPidGains& axis, std::string& read_error) -> bool {
            const auto* value =
                object == nullptr ? nullptr : object->find(name);
            if (value == nullptr ||
                value->type != json::Value::Type::Object) {
                read_error = "missing PID axis '" + std::string(name) +
                             "'";
                return false;
            }
            const auto* kp = value->find("kp");
            const auto* ki = value->find("ki");
            const auto* kd = value->find("kd");
            if (kp == nullptr || ki == nullptr || kd == nullptr ||
                kp->type != json::Value::Type::Number ||
                ki->type != json::Value::Type::Number ||
                kd->type != json::Value::Type::Number ||
                !std::isfinite(kp->number) ||
                !std::isfinite(ki->number) ||
                !std::isfinite(kd->number)) {
                read_error = "PID axis '" + std::string(name) +
                             "' requires finite kp/ki/kd";
                return false;
            }
            axis = {kp->number, ki->number, kd->number};
            return true;
        };
        const auto read_axis =
            [&tuning_value, &read_axis_from](
                std::string_view name, AxisPidGains& axis,
                std::string& read_error) -> bool {
            return read_axis_from(
                tuning_value, name, axis, read_error);
        };
        const auto read_profile =
            [&tuning_value, &read_axis_from](
                std::string_view name, PositionProfile& profile,
                std::string& read_error) -> bool {
            const auto* value = tuning_value->find(name);
            if (value == nullptr ||
                value->type != json::Value::Type::Object) {
                read_error = "missing PID profile '" +
                             std::string(name) + "'";
                return false;
            }
            return read_axis_from(
                       value, "x", profile.x, read_error) &&
                   read_axis_from(
                       value, "y", profile.y, read_error) &&
                   read_axis_from(
                       value, "z", profile.z, read_error);
        };
        const auto read_scheduling =
            [&tuning_value](
                PositionModeScheduling& scheduling,
                std::string& read_error) -> bool {
            const auto* value = tuning_value->find("mode_scheduling");
            if (value == nullptr ||
                value->type != json::Value::Type::Object) {
                read_error =
                    "missing PID mode_scheduling object";
                return false;
            }
            const auto number =
                [value](std::string_view name, double& destination) {
                    const auto* entry = value->find(name);
                    if (entry == nullptr ||
                        entry->type != json::Value::Type::Number ||
                        !std::isfinite(entry->number)) {
                        return false;
                    }
                    destination = entry->number;
                    return true;
                };
            const auto* enabled = value->find("enabled");
            if (enabled == nullptr ||
                enabled->type != json::Value::Type::Boolean) {
                read_error =
                    "mode_scheduling.enabled must be boolean";
                return false;
            }
            scheduling.enabled = enabled->boolean;
            if (!number(
                    "brake_enter_distance_m",
                    scheduling.brake_enter_distance_m) ||
                !number(
                    "brake_exit_distance_m",
                    scheduling.brake_exit_distance_m) ||
                !number(
                    "brake_enter_vertical_error_m",
                    scheduling.brake_enter_vertical_error_m) ||
                !number(
                    "brake_exit_vertical_error_m",
                    scheduling.brake_exit_vertical_error_m) ||
                !number(
                    "hold_enter_distance_m",
                    scheduling.hold_enter_distance_m) ||
                !number(
                    "hold_exit_distance_m",
                    scheduling.hold_exit_distance_m) ||
                !number(
                    "hold_enter_vertical_error_m",
                    scheduling.hold_enter_vertical_error_m) ||
                !number(
                    "hold_exit_vertical_error_m",
                    scheduling.hold_exit_vertical_error_m) ||
                !number(
                    "hold_enter_speed_mps",
                    scheduling.hold_enter_speed_mps) ||
                !number(
                    "hold_exit_speed_mps",
                    scheduling.hold_exit_speed_mps) ||
                !number(
                    "hold_enter_vertical_speed_mps",
                    scheduling.hold_enter_vertical_speed_mps) ||
                !number(
                    "hold_exit_vertical_speed_mps",
                    scheduling.hold_exit_vertical_speed_mps) ||
                !number(
                    "target_motion_speed_mps",
                    scheduling.target_motion_speed_mps) ||
                !number(
                    "hold_enter_dwell_s",
                    scheduling.hold_enter_dwell_s) ||
                !number(
                    "hold_exit_dwell_s",
                    scheduling.hold_exit_dwell_s) ||
                !number(
                    "gain_transition_time_s",
                    scheduling.gain_transition_time_s)) {
                read_error =
                    "mode_scheduling contains a missing or invalid number";
                return false;
            }
            return true;
        };

        if (!read_axis("x", tuning.x, error) ||
            !read_axis("y", tuning.y, error) ||
            !read_axis("z", tuning.z, error) ||
            !read_profile("approach", tuning.approach, error) ||
            !read_profile(
                "precision_hold", tuning.precision_hold, error) ||
            !read_scheduling(
                tuning.mode_scheduling, error) ||
            !read_number(
                "max_accel_mps2", tuning.max_accel_mps2, error) ||
            !read_number(
                "max_tilt_rad", tuning.max_tilt_rad, error) ||
            !read_number("min_throttle", tuning.min_throttle, error) ||
            !read_number(
                "hover_throttle", tuning.hover_throttle, error) ||
            !read_number("max_throttle", tuning.max_throttle, error) ||
            !read_number(
                "z_velocity_damping", tuning.z_velocity_damping,
                error) ||
            !read_number(
                "max_yaw_rate_rad_s", tuning.max_yaw_rate_rad_s,
                error) ||
            !read_number("yaw_rate_kp", tuning.yaw_rate_kp, error) ||
            !read_number("yaw_rate_kd", tuning.yaw_rate_kd, error) ||
            !read_number(
                "yaw_rate_slew_rate_rad_s2",
                tuning.yaw_rate_slew_rate_rad_s2, error) ||
            !read_number(
                "fc_yaw_rate_scale_rad_s",
                tuning.fc_yaw_rate_scale_rad_s, error) ||
            !read_number(
                "max_position_error_m", tuning.max_position_error_m,
                error) ||
            !read_number("gravity_mps2", tuning.gravity_mps2, error)) {
            queue_websocket(client, simple_json_error(error));
            return;
        }

        if (!tuning_handler || !tuning_handler(tuning, error)) {
            std::ostringstream response;
            response << "{\"v\":1,\"type\":\"position_tuning_error\","
                     << "\"message\":\""
                     << json::escape(
                            error.empty()
                                ? "position tuning was rejected"
                                : error)
                     << "\"}";
            queue_websocket(client, response.str());
            return;
        }

        const auto append_axis = [](std::ostringstream& output,
                                    const AxisPidGains& axis) {
            output << "{\"kp\":" << axis.kp << ",\"ki\":" << axis.ki
                   << ",\"kd\":" << axis.kd << '}';
        };
        const auto append_profile =
            [&append_axis](
                std::ostringstream& output,
                const PositionProfile& profile) {
                output << "{\"x\":";
                append_axis(output, profile.x);
                output << ",\"y\":";
                append_axis(output, profile.y);
                output << ",\"z\":";
                append_axis(output, profile.z);
                output << '}';
            };
        std::ostringstream response;
        response << std::setprecision(10)
                 << "{\"v\":1,\"type\":\"position_tuning_ack\","
                 << "\"tuning\":{\"x\":";
        append_axis(response, tuning.x);
        response << ",\"y\":";
        append_axis(response, tuning.y);
        response << ",\"z\":";
        append_axis(response, tuning.z);
        response << ",\"approach\":";
        append_profile(response, tuning.approach);
        response << ",\"precision_hold\":";
        append_profile(response, tuning.precision_hold);
        response << ",\"mode_scheduling\":{\"enabled\":"
                 << (tuning.mode_scheduling.enabled ? "true" : "false")
                 << ",\"brake_enter_distance_m\":"
                 << tuning.mode_scheduling.brake_enter_distance_m
                 << ",\"brake_exit_distance_m\":"
                 << tuning.mode_scheduling.brake_exit_distance_m
                 << ",\"brake_enter_vertical_error_m\":"
                 << tuning.mode_scheduling.brake_enter_vertical_error_m
                 << ",\"brake_exit_vertical_error_m\":"
                 << tuning.mode_scheduling.brake_exit_vertical_error_m
                 << ",\"hold_enter_distance_m\":"
                 << tuning.mode_scheduling.hold_enter_distance_m
                 << ",\"hold_exit_distance_m\":"
                 << tuning.mode_scheduling.hold_exit_distance_m
                 << ",\"hold_enter_vertical_error_m\":"
                 << tuning.mode_scheduling.hold_enter_vertical_error_m
                 << ",\"hold_exit_vertical_error_m\":"
                 << tuning.mode_scheduling.hold_exit_vertical_error_m
                 << ",\"hold_enter_speed_mps\":"
                 << tuning.mode_scheduling.hold_enter_speed_mps
                 << ",\"hold_exit_speed_mps\":"
                 << tuning.mode_scheduling.hold_exit_speed_mps
                 << ",\"hold_enter_vertical_speed_mps\":"
                 << tuning.mode_scheduling
                        .hold_enter_vertical_speed_mps
                 << ",\"hold_exit_vertical_speed_mps\":"
                 << tuning.mode_scheduling
                        .hold_exit_vertical_speed_mps
                 << ",\"target_motion_speed_mps\":"
                 << tuning.mode_scheduling.target_motion_speed_mps
                 << ",\"hold_enter_dwell_s\":"
                 << tuning.mode_scheduling.hold_enter_dwell_s
                 << ",\"hold_exit_dwell_s\":"
                 << tuning.mode_scheduling.hold_exit_dwell_s
                 << ",\"gain_transition_time_s\":"
                 << tuning.mode_scheduling.gain_transition_time_s
                 << '}';
        response << ",\"max_accel_mps2\":" << tuning.max_accel_mps2
                 << ",\"max_tilt_rad\":" << tuning.max_tilt_rad
                 << ",\"min_throttle\":" << tuning.min_throttle
                 << ",\"hover_throttle\":" << tuning.hover_throttle
                 << ",\"max_throttle\":" << tuning.max_throttle
                 << ",\"z_velocity_damping\":"
                 << tuning.z_velocity_damping
                 << ",\"max_yaw_rate_rad_s\":"
                 << tuning.max_yaw_rate_rad_s
                 << ",\"yaw_rate_kp\":" << tuning.yaw_rate_kp
                 << ",\"yaw_rate_kd\":" << tuning.yaw_rate_kd
                 << ",\"yaw_rate_slew_rate_rad_s2\":"
                 << tuning.yaw_rate_slew_rate_rad_s2
                 << ",\"fc_yaw_rate_scale_rad_s\":"
                 << tuning.fc_yaw_rate_scale_rad_s
                 << ",\"max_position_error_m\":"
                 << tuning.max_position_error_m
                 << ",\"gravity_mps2\":" << tuning.gravity_mps2
                 << "}}";
        queue_websocket(client, response.str());
    }

    void handle_gs_uplink(
        Client& client, const json::Value& root) {
        const auto* kind = root.find("kind");
        if (kind == nullptr ||
            kind->type != json::Value::Type::String) {
            queue_websocket(
                client,
                simple_json_error("gs_link_uplink.kind is required"));
            return;
        }

        const auto required_number =
            [&root](
                std::string_view name, double& destination,
                std::string& error) -> bool {
            const auto* value = root.find(name);
            if (value == nullptr ||
                value->type != json::Value::Type::Number ||
                !std::isfinite(value->number)) {
                error = "missing or invalid numeric field '" +
                        std::string(name) + "'";
                return false;
            }
            destination = value->number;
            return true;
        };
        const auto optional_boolean =
            [&root](
                std::string_view name, bool fallback,
                bool& destination, std::string& error) -> bool {
            const auto* value = root.find(name);
            if (value == nullptr) {
                destination = fallback;
                return true;
            }
            if (value->type != json::Value::Type::Boolean) {
                error = "field '" + std::string(name) +
                        "' must be boolean";
                return false;
            }
            destination = value->boolean;
            return true;
        };
        const auto optional_integer =
            [&root](
                std::string_view name, double minimum, double maximum,
                double fallback, double& destination,
                std::string& error) -> bool {
            const auto* value = root.find(name);
            if (value == nullptr) {
                destination = fallback;
                return true;
            }
            if (value->type != json::Value::Type::Number ||
                !std::isfinite(value->number) ||
                value->number < minimum || value->number > maximum ||
                std::floor(value->number) != value->number) {
                error = "field '" + std::string(name) +
                        "' must be an integer in range";
                return false;
            }
            destination = value->number;
            return true;
        };

        gs_link::UplinkRequest request;
        std::string error;
        const std::string uplink_kind = kind->string;
        if (uplink_kind == "rc") {
            request.kind = gs_link::UplinkKind::Rc;
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            double throttle = -1.0;
            if (!required_number("roll", roll, error) ||
                !required_number("pitch", pitch, error) ||
                !required_number("yaw", yaw, error) ||
                !required_number("throttle", throttle, error)) {
                queue_websocket(client, simple_json_error(error));
                return;
            }
            for (const double axis : {roll, pitch, yaw, throttle}) {
                if (axis < -1.0 || axis > 1.0) {
                    queue_websocket(
                        client,
                        simple_json_error(
                            "RC roll/pitch/yaw/throttle must be in "
                            "[-1, 1]"));
                    return;
                }
            }
            double link_quality = 100.0;
            if (!optional_boolean(
                    "valid", false, request.rc.valid, error) ||
                !optional_boolean(
                    "gs_ready", false, request.rc.gs_ready, error) ||
                !optional_integer(
                    "link_quality", 0.0, 255.0, 100.0,
                    link_quality, error)) {
                queue_websocket(client, simple_json_error(error));
                return;
            }
            request.rc.roll = static_cast<std::int16_t>(
                std::llround(roll * 10000.0));
            request.rc.pitch = static_cast<std::int16_t>(
                std::llround(pitch * 10000.0));
            request.rc.yaw = static_cast<std::int16_t>(
                std::llround(yaw * 10000.0));
            request.rc.throttle = static_cast<std::int16_t>(
                std::llround(throttle * 10000.0));
            request.rc.link_quality =
                static_cast<std::uint8_t>(link_quality);
        } else if (uplink_kind == "ping") {
            request.kind = gs_link::UplinkKind::Ping;
            double token = 0.0;
            if (!optional_integer(
                    "token", 0.0,
                    static_cast<double>(
                        std::numeric_limits<std::uint32_t>::max()),
                    0.0, token, error)) {
                queue_websocket(client, simple_json_error(error));
                return;
            }
            request.token = static_cast<std::uint32_t>(token);
        } else if (uplink_kind == "takeoff") {
            request.kind = gs_link::UplinkKind::Takeoff;
            double altitude = 0.0;
            if (!required_number(
                    "target_altitude_mm", altitude, error)) {
                queue_websocket(client, simple_json_error(error));
                return;
            }
            if (altitude < 0.0 ||
                altitude >
                    static_cast<double>(
                        std::numeric_limits<std::int32_t>::max()) ||
                std::floor(altitude) != altitude) {
                queue_websocket(
                    client,
                    simple_json_error(
                        "target_altitude_mm must be a positive integer"));
                return;
            }
            request.target_altitude_mm =
                static_cast<std::int32_t>(altitude);
        } else if (uplink_kind == "land") {
            request.kind = gs_link::UplinkKind::Land;
        } else if (uplink_kind == "abort") {
            request.kind = gs_link::UplinkKind::Abort;
        } else {
            queue_websocket(
                client,
                simple_json_error("unsupported gs_link_uplink.kind"));
            return;
        }

        if (request.kind != gs_link::UplinkKind::Rc) {
            double token = 0.0;
            double timeout_ms = 5000.0;
            if (!optional_integer(
                    "token", 0.0,
                    static_cast<double>(
                        std::numeric_limits<std::uint32_t>::max()),
                    0.0, token, error) ||
                !optional_integer(
                    "timeout_ms", 0.0, 65535.0, 5000.0,
                    timeout_ms, error)) {
                queue_websocket(client, simple_json_error(error));
                return;
            }
            request.token = static_cast<std::uint32_t>(token);
            request.timeout_ms =
                static_cast<std::uint16_t>(timeout_ms);
        }
        if (request.kind == gs_link::UplinkKind::Takeoff) {
            double climb = 100.0;
            if (!optional_integer(
                    "max_climb_cm_s", 0.0, 65535.0, 100.0,
                    climb, error)) {
                queue_websocket(client, simple_json_error(error));
                return;
            }
            request.max_climb_cm_s =
                static_cast<std::uint16_t>(climb);
        } else if (request.kind == gs_link::UplinkKind::Land) {
            double descent = 50.0;
            if (!optional_integer(
                    "max_descent_cm_s", 0.0, 65535.0, 50.0,
                    descent, error)) {
                queue_websocket(client, simple_json_error(error));
                return;
            }
            request.max_descent_cm_s =
                static_cast<std::uint16_t>(descent);
        } else if (request.kind == gs_link::UplinkKind::Abort) {
            double reason = 1.0;
            if (!optional_integer(
                    "reason_code", 0.0, 65535.0, 1.0,
                    reason, error)) {
                queue_websocket(client, simple_json_error(error));
                return;
            }
            request.reason_code = static_cast<std::uint32_t>(reason);
        }

        if (!uplink_handler || !uplink_handler(request, error)) {
            std::ostringstream response;
            response << "{\"v\":1,\"type\":\"gs_link_error\",\"kind\":\""
                     << json::escape(uplink_kind) << "\",\"message\":\""
                     << json::escape(
                            error.empty()
                                ? "GS-Link uplink was rejected"
                                : error)
                     << "\"}";
            queue_websocket(client, response.str());
            return;
        }
        std::ostringstream response;
        response << "{\"v\":1,\"type\":\"gs_link_accept\",\"kind\":\""
                 << json::escape(uplink_kind) << "\"}";
        queue_websocket(client, response.str());
    }

    void handle_operator_intent(Client& client, std::string_view payload) {
        if (payload.size() > 4096U) {
            close_websocket(client, 1009);
            return;
        }
        std::string parse_error;
        const auto root = json::parse(payload, parse_error);
        if (!root || root->type != json::Value::Type::Object) {
            queue_websocket(client, simple_json_error(parse_error));
            return;
        }

        const auto* type = root->find("type");
        if (type == nullptr || type->type != json::Value::Type::String) {
            queue_websocket(
                client, simple_json_error("unsupported message type"));
            return;
        }

        if (type->string == "gs_link_uplink") {
            handle_gs_uplink(client, *root);
            return;
        }

        if (type->string == "position_tuning") {
            handle_position_tuning(client, *root);
            return;
        }

        if (type->string == "bind_body") {
            const auto* aircraft_id = root->find("aircraft_id");
            const auto* body_id = root->find("body_id");
            const auto* sensor_id = root->find("sensor_id");
            if (aircraft_id == nullptr ||
                aircraft_id->type != json::Value::Type::String ||
                body_id == nullptr ||
                body_id->type != json::Value::Type::String ||
                sensor_id == nullptr ||
                sensor_id->type != json::Value::Type::Number ||
                sensor_id->number < 0.0 ||
                sensor_id->number >
                    static_cast<double>(std::numeric_limits<int>::max()) ||
                std::floor(sensor_id->number) != sensor_id->number) {
                queue_websocket(
                    client, simple_json_error("invalid bind_body fields"));
                return;
            }

            std::string binding_error;
            const int requested_sensor =
                static_cast<int>(sensor_id->number);
            if (!binding_handler ||
                !binding_handler(
                    aircraft_id->string, body_id->string, requested_sensor,
                    binding_error)) {
                queue_websocket(
                    client,
                    simple_json_error(
                        binding_error.empty()
                            ? "body binding was rejected"
                            : binding_error));
                return;
            }
            std::ostringstream acknowledgement;
            acknowledgement
                << "{\"v\":1,\"type\":\"binding_ack\",\"aircraft_id\":\""
                << json::escape(aircraft_id->string) << "\",\"body_id\":\""
                << json::escape(body_id->string)
                << "\",\"sensor_id\":" << requested_sensor << '}';
            queue_websocket(client, acknowledgement.str());
            return;
        }

        if (type->string != "operator_intent") {
            queue_websocket(
                client, simple_json_error("unsupported message type"));
            return;
        }

        const auto* target_array = root->find("target_xyz_m");
        const auto* yaw = root->find("target_yaw_rad");
        const auto* deadman = root->find("deadman");
        const auto* client_sequence = root->find("client_seq");
        if (target_array == nullptr ||
            target_array->type != json::Value::Type::Array ||
            target_array->array.size() != 3U || yaw == nullptr ||
            yaw->type != json::Value::Type::Number || deadman == nullptr ||
            deadman->type != json::Value::Type::Boolean) {
            queue_websocket(
                client, simple_json_error("invalid operator_intent fields"));
            return;
        }
        for (const auto& component : target_array->array) {
            if (component->type != json::Value::Type::Number ||
                !std::isfinite(component->number)) {
                queue_websocket(
                    client,
                    simple_json_error("target_xyz_m must contain numbers"));
                return;
            }
        }

        OperatorTarget target;
        target.position_xyz_m = {
            target_array->array[0]->number,
            target_array->array[1]->number,
            target_array->array[2]->number,
        };
        target.yaw_rad = yaw->number;
        target.deadman = deadman->boolean;
        if (client_sequence != nullptr) {
            if (client_sequence->type != json::Value::Type::Number ||
                client_sequence->number < 0.0 ||
                std::floor(client_sequence->number) !=
                    client_sequence->number) {
                queue_websocket(
                    client, simple_json_error("invalid client_seq"));
                return;
            }
            target.client_sequence = static_cast<std::uint64_t>(
                client_sequence->number);
        }

        ++client.intent_count;
        if (!intent_handler || !intent_handler(target)) {
            queue_websocket(
                client, simple_json_error("operator intent was rejected"));
        }
    }

    void process_websocket_frames(Client& client) {
        while (client.fd >= 0 && !client.input.empty()) {
            web::DecodedFrame frame = web::decode_client_frame(client.input);
            if (frame.status == web::FrameDecodeStatus::Incomplete) {
                return;
            }
            if (frame.status == web::FrameDecodeStatus::Error) {
                queue_websocket(client, simple_json_error(frame.error));
                close_websocket(client, 1002);
                return;
            }
            client.input.erase(0, frame.consumed_bytes);
            if (!frame.final) {
                close_websocket(client, 1003);
                return;
            }
            switch (frame.opcode) {
                case kOpcodeText:
                    handle_operator_intent(client, frame.payload);
                    break;
                case kOpcodePing:
                    queue_websocket(client, frame.payload, kOpcodePong);
                    break;
                case kOpcodePong:
                    break;
                case kOpcodeClose:
                    close_websocket(client, 1000);
                    return;
                default:
                    close_websocket(client, 1003);
                    return;
            }
        }
    }

    void process_http(Client& client) {
        if (client.fd < 0) {
            return;
        }
        const std::size_t header_end = client.input.find("\r\n\r\n");
        if (header_end == std::string::npos) {
            if (client.input.size() > kMaxHttpHeaderBytes) {
                queue(
                    client,
                    http_response(
                        431, "Request Header Fields Too Large",
                        "application/json",
                        simple_json_error("HTTP header is too large")));
                client.close_after_write = true;
            }
            return;
        }

        std::string parse_error;
        const auto request = parse_http_request(
            std::string_view(client.input).substr(0, header_end), parse_error);
        if (!request) {
            queue(
                client,
                http_response(
                    400, "Bad Request", "application/json",
                    simple_json_error(parse_error)));
            client.close_after_write = true;
            return;
        }
        client.input.erase(0, header_end + 4U);
        route_http(client, *request);
        if (client.mode == ClientMode::WebSocket) {
            process_websocket_frames(client);
        }
    }

    void read_client(Client& client) {
        std::array<char, 65536> buffer{};
        while (client.fd >= 0) {
            const ssize_t received =
                ::recv(client.fd, buffer.data(), buffer.size(), 0);
            if (received > 0) {
                client.input.append(
                    buffer.data(), static_cast<std::size_t>(received));
                if (client.input.size() > kMaxClientInputBytes) {
                    close_client(client);
                    return;
                }
                if (client.mode == ClientMode::Http) {
                    process_http(client);
                } else {
                    process_websocket_frames(client);
                }
                if (client.close_after_write) {
                    return;
                }
                continue;
            }
            if (received == 0) {
                close_client(client);
                return;
            }
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return;
            }
            close_client(client);
            return;
        }
    }

    void write_client(Client& client) {
        while (client.fd >= 0 && !client.output.empty()) {
            const ssize_t sent = ::send(
                client.fd, client.output.data(), client.output.size(),
                MSG_NOSIGNAL);
            if (sent > 0) {
                client.output.erase(0, static_cast<std::size_t>(sent));
                continue;
            }
            if (sent < 0 && errno == EINTR) {
                continue;
            }
            if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                return;
            }
            close_client(client);
            return;
        }
        if (client.fd >= 0 && client.close_after_write) {
            close_client(client);
        }
    }

    void broadcast(std::string payload) {
        for (auto& client : clients) {
            if (client->fd >= 0 && client->mode == ClientMode::WebSocket) {
                queue_websocket(*client, payload);
            }
        }
    }

    void run() {
        auto next_broadcast = SteadyClock::now();
        while (running_.load(std::memory_order_relaxed)) {
            bool accept_pending = false;
            std::vector<pollfd> descriptors;
            std::vector<Client*> descriptor_clients;
            descriptors.push_back({listen_fd, POLLIN, 0});
            descriptor_clients.push_back(nullptr);
            for (auto& client : clients) {
                if (client->fd >= 0) {
                    short events = POLLIN;
                    if (!client->output.empty()) {
                        events = static_cast<short>(events | POLLOUT);
                    }
                    descriptors.push_back({client->fd, events, 0});
                    descriptor_clients.push_back(client.get());
                }
            }

            const auto now = SteadyClock::now();
            int timeout_ms = 33;
            if (now >= next_broadcast) {
                timeout_ms = 0;
            } else {
                timeout_ms = static_cast<int>(
                    static_cast<long long>(
                        std::max<long long>(
                            0LL,
                            static_cast<long long>(
                                std::chrono::duration_cast<
                                    std::chrono::milliseconds>(
                                    next_broadcast - now)
                                    .count()))));
            }

            const int poll_result =
                ::poll(descriptors.data(), descriptors.size(), timeout_ms);
            if (poll_result < 0 && errno != EINTR) {
                break;
            }

            if (poll_result > 0) {
                for (std::size_t index = 0; index < descriptors.size();
                     ++index) {
                    const short revents = descriptors[index].revents;
                    if (revents == 0) {
                        continue;
                    }
                    if (index == 0U) {
                        if ((revents & POLLIN) != 0) {
                            accept_pending = true;
                        }
                        continue;
                    }
                    Client* client = descriptor_clients[index];
                    if (client == nullptr || client->fd < 0) {
                        continue;
                    }
                    if ((revents & (POLLERR | POLLHUP | POLLNVAL)) != 0 &&
                        (revents & POLLIN) == 0) {
                        close_client(*client);
                        continue;
                    }
                    if ((revents & POLLIN) != 0) {
                        read_client(*client);
                    }
                    if (client->fd >= 0 && (revents & POLLOUT) != 0) {
                        write_client(*client);
                    }
                }
            }

            // Accept after walking the current clients so vector growth cannot
            // invalidate Client* entries used by this poll iteration.
            if (accept_pending) {
                accept_clients();
            }

            const auto after_io = SteadyClock::now();
            if (after_io >= next_broadcast) {
                const bool has_websocket_client = std::any_of(
                    clients.begin(), clients.end(), [](const auto& client) {
                        return client->fd >= 0 &&
                               client->mode == ClientMode::WebSocket;
                    });
                if (has_websocket_client && snapshot_provider) {
                    broadcast(snapshot_provider());
                }
                next_broadcast = after_io + 33ms;
            }

            clients.erase(
                std::remove_if(
                    clients.begin(), clients.end(),
                    [](const auto& client) {
                        return client->fd < 0 ||
                               (client->close_after_write &&
                                client->output.empty());
                    }),
                clients.end());
        }
    }

    WebServerOptions options;
    IntentHandler intent_handler;
    BindingHandler binding_handler;
    UplinkHandler uplink_handler;
    TuningHandler tuning_handler;
    SnapshotProvider snapshot_provider;
    std::filesystem::path web_root;
    std::atomic_bool running_{false};
    std::thread thread;
    int listen_fd{-1};
    std::uint16_t bound_port{0};
    std::uint64_t started_at{0};
    std::vector<std::unique_ptr<Client>> clients;
};

WebServer::WebServer(
    WebServerOptions options, IntentHandler intent_handler,
    BindingHandler binding_handler, UplinkHandler uplink_handler,
    TuningHandler tuning_handler, SnapshotProvider snapshot_provider)
    : impl_(std::make_unique<Impl>(
          std::move(options), std::move(intent_handler),
          std::move(binding_handler), std::move(uplink_handler),
          std::move(tuning_handler),
          std::move(snapshot_provider))) {}

WebServer::~WebServer() {
    stop();
}

bool WebServer::start(std::string& error) {
    return impl_->start(error);
}

void WebServer::stop() {
    impl_->stop();
}

std::uint16_t WebServer::bound_port() const {
    return impl_->bound_port;
}

}  // namespace dock_ground
