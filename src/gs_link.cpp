#include "dock_ground/gs_link.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <mutex>
#include <netinet/in.h>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <string_view>
#include <sys/socket.h>
#include <thread>
#include <type_traits>
#include <unistd.h>

namespace dock_ground::gs_link {
namespace {

constexpr std::uint8_t kMagic0 = 0xA5U;
constexpr std::uint8_t kMagic1 = 0x5AU;
constexpr std::uint16_t kCrcInitial = 0xFFFFU;
constexpr std::uint16_t kCrcPolynomial = 0x1021U;
constexpr auto kAckRetryInterval = std::chrono::milliseconds(20);
constexpr int kMaximumTransmissions = 4;

using Clock = std::chrono::steady_clock;

template <typename Integer>
void append_le(std::vector<std::uint8_t>& output, Integer value) {
    using Unsigned = std::make_unsigned_t<Integer>;
    const auto bits = static_cast<Unsigned>(value);
    for (std::size_t byte = 0; byte < sizeof(Integer); ++byte) {
        output.push_back(static_cast<std::uint8_t>(
            (bits >> static_cast<unsigned int>(byte * 8U)) & 0xFFU));
    }
}

template <typename Integer>
Integer read_le(std::span<const std::uint8_t> input,
                std::size_t offset) {
    using Unsigned = std::make_unsigned_t<Integer>;
    Unsigned value = 0;
    for (std::size_t byte = 0; byte < sizeof(Integer); ++byte) {
        value |= static_cast<Unsigned>(input[offset + byte])
                 << static_cast<unsigned int>(byte * 8U);
    }
    return static_cast<Integer>(value);
}

[[nodiscard]] std::uint64_t duration_ms(
    Clock::duration duration) noexcept {
    return static_cast<std::uint64_t>(
        std::max<std::int64_t>(
            0, std::chrono::duration_cast<std::chrono::milliseconds>(
                   duration)
                   .count()));
}

[[nodiscard]] bool fresh(
    Clock::time_point now, Clock::time_point received,
    std::uint16_t timeout_ms, bool valid) noexcept {
    if (!valid) {
        return false;
    }
    if (now < received) {
        return true;
    }
    return now - received <=
           std::chrono::milliseconds(timeout_ms);
}

[[nodiscard]] double age_ms(
    Clock::time_point now, Clock::time_point received,
    bool valid) noexcept {
    if (!valid) {
        return -1.0;
    }
    const auto age =
        now >= received ? now - received : Clock::duration::zero();
    return std::chrono::duration<double, std::milli>(age).count();
}

[[nodiscard]] std::uint32_t make_session_id() {
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         Clock::now().time_since_epoch())
                         .count();
    std::uint32_t value =
        static_cast<std::uint32_t>(now) ^
        static_cast<std::uint32_t>(::getpid());
    return value == 0U ? 1U : value;
}

}  // namespace

std::uint16_t crc16(std::span<const std::uint8_t> input) noexcept {
    std::uint16_t crc = kCrcInitial;
    for (const std::uint8_t byte : input) {
        crc ^= static_cast<std::uint16_t>(byte) << 8U;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) != 0U
                      ? static_cast<std::uint16_t>(
                            (crc << 1U) ^ kCrcPolynomial)
                      : static_cast<std::uint16_t>(crc << 1U);
        }
    }
    return crc;
}

std::vector<std::uint8_t> encode_frame(
    PacketClass packet_class, std::uint8_t message_id,
    std::uint8_t flags, std::uint16_t sequence,
    std::span<const std::uint8_t> payload) {
    if (payload.size() > kMaximumPayloadBytes) {
        throw std::length_error("GS-Link payload exceeds 512 bytes");
    }
    std::vector<std::uint8_t> frame;
    frame.reserve(kHeaderBytes + payload.size() + kCrcBytes);
    frame.push_back(kMagic0);
    frame.push_back(kMagic1);
    frame.push_back(kVersion);
    frame.push_back(static_cast<std::uint8_t>(packet_class));
    frame.push_back(message_id);
    frame.push_back(flags);
    append_le<std::uint16_t>(frame, sequence);
    append_le<std::uint16_t>(
        frame, static_cast<std::uint16_t>(payload.size()));
    frame.insert(frame.end(), payload.begin(), payload.end());
    const std::uint16_t crc = crc16(std::span<const std::uint8_t>(
        frame.data() + 2U, frame.size() - 2U));
    append_le<std::uint16_t>(frame, crc);
    return frame;
}

std::vector<std::uint8_t> encode_data_rc(
    std::uint16_t sequence, const RcCommand& command,
    std::uint32_t sender_time_ms, std::uint8_t packet_flags) {
    std::vector<std::uint8_t> payload;
    payload.reserve(16U);
    append_le<std::uint32_t>(payload, sender_time_ms);
    append_le<std::int16_t>(payload, command.roll);
    append_le<std::int16_t>(payload, command.pitch);
    append_le<std::int16_t>(payload, command.yaw);
    append_le<std::int16_t>(payload, command.throttle);
    std::uint8_t rc_flags = 0;
    if (command.valid) {
        rc_flags |= 1U << 0U;
    }
    if (command.gs_ready) {
        rc_flags |= 1U << 1U;
    }
    payload.push_back(rc_flags);
    payload.push_back(command.link_quality);
    append_le<std::uint16_t>(payload, 0U);
    return encode_frame(
        PacketClass::Data, static_cast<std::uint8_t>(DataMessage::Rc),
        packet_flags, sequence, payload);
}

std::vector<std::uint8_t> encode_telemetry(
    std::uint16_t sequence, const FlightTelemetry& telemetry,
    std::uint8_t packet_flags) {
    const std::uint8_t channel_count = std::min<std::uint8_t>(
        telemetry.rc_channel_count, kMaximumRcChannels);
    std::vector<std::uint8_t> payload;
    payload.reserve(22U + channel_count);
    append_le<std::uint32_t>(payload, telemetry.timestamp_ms);
    append_le<std::uint16_t>(payload, telemetry.status_flags);
    append_le<std::int16_t>(payload, telemetry.roll_cdeg);
    append_le<std::int16_t>(payload, telemetry.pitch_cdeg);
    append_le<std::int16_t>(payload, telemetry.yaw_cdeg);
    append_le<std::uint16_t>(payload, telemetry.motor1_rpm);
    append_le<std::uint16_t>(payload, telemetry.motor2_rpm);
    append_le<std::uint16_t>(payload, telemetry.battery_voltage_mv);
    append_le<std::int16_t>(payload, telemetry.battery_current_ca);
    payload.push_back(telemetry.battery_remaining);
    payload.push_back(channel_count);
    for (std::size_t index = 0; index < channel_count; ++index) {
        payload.push_back(static_cast<std::uint8_t>(
            telemetry.rc_channels[index]));
    }
    return encode_frame(
        PacketClass::Data,
        static_cast<std::uint8_t>(DataMessage::Telemetry),
        packet_flags, sequence, payload);
}

std::vector<std::uint8_t> encode_heartbeat(
    std::uint16_t sequence, const Heartbeat& heartbeat,
    std::uint8_t packet_flags) {
    std::vector<std::uint8_t> payload;
    payload.reserve(16U);
    append_le<std::uint32_t>(payload, heartbeat.uptime_ms);
    append_le<std::uint32_t>(payload, heartbeat.session_id);
    append_le<std::uint16_t>(payload, heartbeat.status_flags);
    append_le<std::uint16_t>(payload, heartbeat.protocol_version);
    payload.push_back(static_cast<std::uint8_t>(heartbeat.state));
    payload.push_back(heartbeat.error_code);
    append_le<std::uint16_t>(payload, 0U);
    return encode_frame(
        PacketClass::Command,
        static_cast<std::uint8_t>(CommandMessage::Heartbeat),
        packet_flags, sequence, payload);
}

std::vector<std::uint8_t> encode_ping(
    std::uint16_t sequence, std::uint32_t token,
    std::uint8_t packet_flags) {
    std::vector<std::uint8_t> payload;
    payload.reserve(4U);
    append_le<std::uint32_t>(payload, token);
    return encode_frame(
        PacketClass::Command,
        static_cast<std::uint8_t>(CommandMessage::Ping),
        packet_flags, sequence, payload);
}

std::vector<std::uint8_t> encode_ack(
    std::uint16_t sequence, const Ack& ack,
    std::uint8_t packet_flags) {
    std::vector<std::uint8_t> payload;
    payload.reserve(12U);
    payload.push_back(ack.acked_class);
    payload.push_back(ack.acked_id);
    append_le<std::uint16_t>(payload, ack.acked_sequence);
    payload.push_back(static_cast<std::uint8_t>(ack.result));
    payload.push_back(ack.detail);
    append_le<std::uint16_t>(payload, 0U);
    append_le<std::uint32_t>(payload, ack.echo_token);
    packet_flags |= static_cast<std::uint8_t>(PacketFlag::Response);
    return encode_frame(
        PacketClass::Command,
        static_cast<std::uint8_t>(CommandMessage::Ack),
        packet_flags, sequence, payload);
}

DecodeResult decode_frame(std::span<const std::uint8_t> frame) {
    DecodeResult result;
    if (frame.size() < kHeaderBytes + kCrcBytes) {
        result.failure = DecodeFailure::Length;
        result.error = "GS-Link frame is shorter than header plus CRC";
        return result;
    }
    if (frame[0] != kMagic0 || frame[1] != kMagic1) {
        result.failure = DecodeFailure::Magic;
        result.error = "invalid GS-Link magic";
        return result;
    }
    if (frame[2] != kVersion) {
        result.failure = DecodeFailure::Version;
        result.error = "unsupported GS-Link version";
        return result;
    }
    if (frame[3] != static_cast<std::uint8_t>(PacketClass::Data) &&
        frame[3] != static_cast<std::uint8_t>(PacketClass::Command)) {
        result.failure = DecodeFailure::PacketClass;
        result.error = "invalid GS-Link packet class";
        return result;
    }
    const std::uint16_t payload_length = read_le<std::uint16_t>(frame, 8U);
    if (payload_length > kMaximumPayloadBytes ||
        frame.size() !=
            kHeaderBytes + static_cast<std::size_t>(payload_length) +
                kCrcBytes) {
        result.failure = DecodeFailure::Length;
        result.error = "invalid GS-Link payload length";
        return result;
    }
    const std::uint16_t received_crc =
        read_le<std::uint16_t>(frame, frame.size() - kCrcBytes);
    const std::uint16_t calculated_crc = crc16(frame.subspan(
        2U, frame.size() - 2U - kCrcBytes));
    if (received_crc != calculated_crc) {
        result.failure = DecodeFailure::Crc;
        result.error = "GS-Link CRC mismatch";
        return result;
    }

    result.ok = true;
    result.packet.packet_class =
        static_cast<PacketClass>(frame[3]);
    result.packet.message_id = frame[4];
    result.packet.flags = frame[5];
    result.packet.sequence = read_le<std::uint16_t>(frame, 6U);
    const auto payload_begin = frame.begin() + kHeaderBytes;
    result.packet.payload.assign(
        payload_begin,
        payload_begin + static_cast<std::ptrdiff_t>(payload_length));
    return result;
}

bool decode_data_rc(
    const Packet& packet, RcCommand& command,
    std::uint32_t& sender_time_ms, std::string& error) {
    if (packet.packet_class != PacketClass::Data ||
        packet.message_id != static_cast<std::uint8_t>(DataMessage::Rc)) {
        error = "packet is not DATA_RC";
        return false;
    }
    if (packet.payload.size() != 16U) {
        error = "DATA_RC payload must be 16 bytes";
        return false;
    }
    const auto payload =
        std::span<const std::uint8_t>(packet.payload);
    sender_time_ms = read_le<std::uint32_t>(payload, 0U);
    command.roll = read_le<std::int16_t>(payload, 4U);
    command.pitch = read_le<std::int16_t>(payload, 6U);
    command.yaw = read_le<std::int16_t>(payload, 8U);
    command.throttle = read_le<std::int16_t>(payload, 10U);
    const std::uint8_t flags = payload[12];
    command.valid = (flags & (1U << 0U)) != 0U;
    command.gs_ready = (flags & (1U << 1U)) != 0U;
    command.link_quality = payload[13];
    error.clear();
    return true;
}

bool decode_telemetry(
    const Packet& packet, FlightTelemetry& telemetry,
    std::string& error) {
    if (packet.packet_class != PacketClass::Data ||
        packet.message_id !=
            static_cast<std::uint8_t>(DataMessage::Telemetry)) {
        error = "packet is not DATA_TELEMETRY";
        return false;
    }
    if (packet.payload.size() < 22U ||
        packet.payload.size() > 22U + kMaximumRcChannels) {
        error = "DATA_TELEMETRY payload has invalid length";
        return false;
    }
    const std::uint8_t channel_count = packet.payload[21];
    if (channel_count > kMaximumRcChannels ||
        packet.payload.size() != 22U + channel_count) {
        error = "DATA_TELEMETRY RC channel count does not match length";
        return false;
    }

    const auto payload =
        std::span<const std::uint8_t>(packet.payload);
    telemetry.timestamp_ms = read_le<std::uint32_t>(payload, 0U);
    telemetry.status_flags = read_le<std::uint16_t>(payload, 4U);
    telemetry.roll_cdeg = read_le<std::int16_t>(payload, 6U);
    telemetry.pitch_cdeg = read_le<std::int16_t>(payload, 8U);
    telemetry.yaw_cdeg = read_le<std::int16_t>(payload, 10U);
    if (telemetry.roll_cdeg < -18000 ||
        telemetry.roll_cdeg > 18000 ||
        telemetry.pitch_cdeg < -18000 ||
        telemetry.pitch_cdeg > 18000 ||
        telemetry.yaw_cdeg < -18000 ||
        telemetry.yaw_cdeg > 18000) {
        error = "DATA_TELEMETRY attitude is outside [-18000, 18000] cdeg";
        return false;
    }
    telemetry.motor1_rpm = read_le<std::uint16_t>(payload, 12U);
    telemetry.motor2_rpm = read_le<std::uint16_t>(payload, 14U);
    telemetry.battery_voltage_mv =
        read_le<std::uint16_t>(payload, 16U);
    telemetry.battery_current_ca =
        read_le<std::int16_t>(payload, 18U);
    telemetry.battery_remaining = payload[20];
    telemetry.rc_channel_count = channel_count;
    telemetry.rc_channels.fill(0);
    for (std::size_t index = 0; index < channel_count; ++index) {
        telemetry.rc_channels[index] =
            static_cast<std::int8_t>(payload[22U + index]);
    }
    telemetry.valid = true;
    error.clear();
    return true;
}

bool decode_heartbeat(
    const Packet& packet, Heartbeat& heartbeat,
    std::string& error) {
    if (packet.packet_class != PacketClass::Command ||
        packet.message_id !=
            static_cast<std::uint8_t>(CommandMessage::Heartbeat)) {
        error = "packet is not CMD_HEARTBEAT";
        return false;
    }
    if (packet.payload.size() != 16U) {
        error = "CMD_HEARTBEAT payload must be 16 bytes";
        return false;
    }
    const auto payload =
        std::span<const std::uint8_t>(packet.payload);
    heartbeat.uptime_ms = read_le<std::uint32_t>(payload, 0U);
    heartbeat.session_id = read_le<std::uint32_t>(payload, 4U);
    heartbeat.status_flags = read_le<std::uint16_t>(payload, 8U);
    heartbeat.protocol_version =
        read_le<std::uint16_t>(payload, 10U);
    if (heartbeat.protocol_version != kVersion) {
        error = "CMD_HEARTBEAT protocol version is not v1";
        return false;
    }
    heartbeat.state = static_cast<FlightState>(payload[12]);
    heartbeat.error_code = payload[13];
    heartbeat.valid = true;
    error.clear();
    return true;
}

bool decode_ack(
    const Packet& packet, Ack& ack, std::string& error) {
    if (packet.packet_class != PacketClass::Command ||
        packet.message_id !=
            static_cast<std::uint8_t>(CommandMessage::Ack)) {
        error = "packet is not CMD_ACK";
        return false;
    }
    if (packet.payload.size() != 12U) {
        error = "CMD_ACK payload must be 12 bytes";
        return false;
    }
    const auto payload =
        std::span<const std::uint8_t>(packet.payload);
    ack.acked_class = payload[0];
    ack.acked_id = payload[1];
    ack.acked_sequence = read_le<std::uint16_t>(payload, 2U);
    ack.result = static_cast<AckResult>(payload[4]);
    ack.detail = payload[5];
    ack.echo_token = read_le<std::uint32_t>(payload, 8U);
    ack.valid = true;
    error.clear();
    return true;
}

const char* to_string(FlightState state) noexcept {
    switch (state) {
        case FlightState::Boot:
            return "BOOT";
        case FlightState::Ready:
            return "READY";
        case FlightState::Auto:
            return "AUTO";
        case FlightState::Failsafe:
            return "FAILSAFE";
        case FlightState::Error:
            return "ERROR";
    }
    return "RESERVED";
}

const char* to_string(AckResult result) noexcept {
    switch (result) {
        case AckResult::Ok:
            return "OK";
        case AckResult::Unsupported:
            return "UNSUPPORTED";
        case AckResult::BadLength:
            return "BAD_LENGTH";
        case AckResult::BadArgument:
            return "BAD_ARGUMENT";
        case AckResult::RejectedSafety:
            return "REJECTED_SAFETY";
        case AckResult::Busy:
            return "BUSY";
        case AckResult::Timeout:
            return "TIMEOUT";
        case AckResult::Duplicate:
            return "DUPLICATE";
        case AckResult::BadVersion:
            return "BAD_VERSION";
        case AckResult::InternalError:
            return "INTERNAL_ERROR";
        case AckResult::StaleSession:
            return "STALE_SESSION";
        case AckResult::LinkStale:
            return "LINK_STALE";
    }
    return "RESERVED";
}

struct UdpLink::Impl {
    explicit Impl(UdpConfig input_config)
        : config(std::move(input_config)),
          session_id(make_session_id()) {}

    ~Impl() {
        stop();
    }

    bool configure_socket(std::string& error) {
        fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
        if (fd < 0) {
            error = "cannot create GS-Link UDP socket: " +
                    std::string(std::strerror(errno));
            return false;
        }
        int reuse = 1;
        (void)::setsockopt(
            fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        const int flags = ::fcntl(fd, F_GETFL, 0);
        if (flags < 0 ||
            ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
            error = "cannot make GS-Link UDP socket non-blocking: " +
                    std::string(std::strerror(errno));
            close_socket();
            return false;
        }

        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_port = htons(config.bind_port);
        if (::inet_pton(AF_INET, config.bind_ip.c_str(),
                        &local.sin_addr) != 1) {
            error = "invalid GS-Link local IPv4 address: " +
                    config.bind_ip;
            close_socket();
            return false;
        }
        if (::bind(
                fd, reinterpret_cast<const sockaddr*>(&local),
                sizeof(local)) != 0) {
            error = "cannot bind GS-Link UDP socket: " +
                    std::string(std::strerror(errno));
            close_socket();
            return false;
        }

        sockaddr_in remote{};
        remote.sin_family = AF_INET;
        remote.sin_port = htons(config.remote_port);
        if (::inet_pton(AF_INET, config.remote_ip.c_str(),
                        &remote.sin_addr) != 1) {
            error = "invalid GS-Link remote IPv4 address: " +
                    config.remote_ip;
            close_socket();
            return false;
        }
        if (::connect(
                fd, reinterpret_cast<const sockaddr*>(&remote),
                sizeof(remote)) != 0) {
            error = "cannot connect GS-Link UDP socket: " +
                    std::string(std::strerror(errno));
            close_socket();
            return false;
        }

        sockaddr_in bound{};
        socklen_t bound_length = sizeof(bound);
        if (::getsockname(
                fd, reinterpret_cast<sockaddr*>(&bound),
                &bound_length) == 0) {
            bound_port = ntohs(bound.sin_port);
        }
        {
            const std::lock_guard lock(mutex);
            transport_connected = true;
        }
        error.clear();
        return true;
    }

    void close_socket() noexcept {
        if (fd >= 0) {
            (void)::shutdown(fd, SHUT_RDWR);
            (void)::close(fd);
            fd = -1;
        }
    }

    bool start(std::string& error) {
        if (running.load(std::memory_order_relaxed)) {
            return true;
        }
        if (!configure_socket(error)) {
            return false;
        }
        started_at = Clock::now();
        next_heartbeat = started_at;
        running.store(true, std::memory_order_relaxed);
        try {
            thread = std::thread([this] { run(); });
        } catch (const std::exception& exception) {
            running.store(false, std::memory_order_relaxed);
            close_socket();
            error = "cannot start GS-Link UDP thread: " +
                    std::string(exception.what());
            return false;
        }
        error.clear();
        return true;
    }

    void stop() noexcept {
        if (!running.exchange(false, std::memory_order_relaxed)) {
            close_socket();
            return;
        }
        if (thread.joinable()) {
            thread.join();
        }
        close_socket();
        {
            const std::lock_guard lock(mutex);
            transport_connected = false;
            pending.active = false;
        }
    }

    [[nodiscard]] std::uint16_t next_sequence_locked() noexcept {
        ++tx_sequence;
        return tx_sequence;
    }

    bool raw_send_locked(
        std::span<const std::uint8_t> frame) noexcept {
        const ssize_t sent = ::send(
            fd, frame.data(), frame.size(), MSG_NOSIGNAL);
        if (sent != static_cast<ssize_t>(frame.size())) {
            last_send_errno = errno;
            ++counters.send_errors;
            return false;
        }
        last_send_errno = 0;
        ++counters.tx_packets;
        return true;
    }

    bool link_ready_locked(Clock::time_point now) const noexcept {
        return transport_connected &&
               fresh(now, last_telemetry_rx,
                     config.telemetry_timeout_ms,
                     telemetry.valid) &&
               fresh(now, last_heartbeat_rx,
                     config.heartbeat_timeout_ms,
                     heartbeat.valid);
    }

    void send_heartbeat_locked(Clock::time_point now) {
        Heartbeat outbound;
        outbound.uptime_ms = static_cast<std::uint32_t>(
            duration_ms(now - started_at));
        outbound.session_id = session_id;
        if (link_ready_locked(now)) {
            outbound.status_flags =
                static_cast<std::uint16_t>(StatusFlag::LinkOk);
            outbound.state = FlightState::Ready;
        } else if (
            fresh(now, last_heartbeat_rx,
                  config.heartbeat_timeout_ms, heartbeat.valid) ||
            fresh(now, last_telemetry_rx,
                  config.telemetry_timeout_ms, telemetry.valid)) {
            outbound.status_flags =
                static_cast<std::uint16_t>(StatusFlag::LinkOk);
            outbound.state = FlightState::Boot;
        }
        const auto frame = encode_heartbeat(
            next_sequence_locked(), outbound);
        (void)raw_send_locked(frame);
    }

    void send_ack_locked(
        const Packet& packet, AckResult result,
        std::uint32_t token) {
        Ack ack;
        ack.acked_class = static_cast<std::uint8_t>(
            packet.packet_class);
        ack.acked_id = packet.message_id;
        ack.acked_sequence = packet.sequence;
        ack.result = result;
        ack.echo_token = token;
        const auto frame = encode_ack(tx_sequence, ack);
        (void)raw_send_locked(frame);
    }

    void update_rx_sequence_locked(std::uint16_t sequence) {
        if (!have_rx_sequence) {
            have_rx_sequence = true;
            last_rx_sequence = sequence;
            return;
        }
        if (sequence == last_rx_sequence) {
            ++counters.duplicates;
            return;
        }
        const std::uint16_t expected =
            static_cast<std::uint16_t>(last_rx_sequence + 1U);
        if (sequence != expected) {
            counters.sequence_gaps +=
                static_cast<std::uint16_t>(sequence - expected);
        }
        last_rx_sequence = sequence;
    }

    void handle_packet_locked(
        const Packet& packet, Clock::time_point now) {
        ++counters.rx_packets;
        const bool is_ack =
            packet.packet_class == PacketClass::Command &&
            packet.message_id ==
                static_cast<std::uint8_t>(CommandMessage::Ack);
        if (!is_ack) {
            update_rx_sequence_locked(packet.sequence);
        }

        if (packet.packet_class == PacketClass::Data) {
            if (packet.message_id ==
                static_cast<std::uint8_t>(DataMessage::Telemetry)) {
                if (packet.payload.size() < 22U ||
                    packet.payload.size() >
                        22U + kMaximumRcChannels ||
                    (packet.payload.size() >= 22U &&
                     (packet.payload[21] > kMaximumRcChannels ||
                      packet.payload.size() !=
                          22U + packet.payload[21]))) {
                    ++counters.length_errors;
                    return;
                }
                FlightTelemetry decoded;
                std::string error;
                if (!decode_telemetry(packet, decoded, error)) {
                    ++counters.unknown_data;
                    return;
                }
                decoded.received_mono_ns =
                    static_cast<std::uint64_t>(
                        std::chrono::duration_cast<
                            std::chrono::nanoseconds>(
                            now.time_since_epoch())
                            .count());
                telemetry = decoded;
                last_telemetry_rx = now;
                return;
            }
            ++counters.unknown_data;
            return;
        }

        std::string error;
        if (packet.message_id ==
            static_cast<std::uint8_t>(CommandMessage::Heartbeat)) {
            if (packet.payload.size() != 16U) {
                ++counters.length_errors;
                return;
            }
            Heartbeat decoded;
            if (!decode_heartbeat(packet, decoded, error)) {
                ++counters.version_errors;
                return;
            }
            decoded.received_mono_ns =
                static_cast<std::uint64_t>(
                    std::chrono::duration_cast<
                        std::chrono::nanoseconds>(
                        now.time_since_epoch())
                        .count());
            heartbeat = decoded;
            last_heartbeat_rx = now;
            return;
        }
        if (is_ack) {
            if (packet.payload.size() != 12U) {
                ++counters.length_errors;
                return;
            }
            Ack decoded;
            if (!decode_ack(packet, decoded, error)) {
                ++counters.length_errors;
                return;
            }
            decoded.received_mono_ns =
                static_cast<std::uint64_t>(
                    std::chrono::duration_cast<
                        std::chrono::nanoseconds>(
                        now.time_since_epoch())
                        .count());
            last_ack = decoded;
            if (pending.active &&
                decoded.acked_class ==
                    static_cast<std::uint8_t>(pending.packet_class) &&
                decoded.acked_id == pending.message_id &&
                decoded.acked_sequence == pending.sequence) {
                pending.active = false;
            }
            return;
        }
        if (packet.message_id ==
            static_cast<std::uint8_t>(CommandMessage::Ping)) {
            if (packet.payload.size() != 4U) {
                ++counters.length_errors;
                send_ack_locked(packet, AckResult::BadLength, 0U);
                return;
            }
            const std::uint32_t token =
                read_le<std::uint32_t>(
                    std::span<const std::uint8_t>(packet.payload), 0U);
            send_ack_locked(packet, AckResult::Ok, token);
            return;
        }

        std::optional<std::size_t> expected_length;
        std::optional<std::size_t> token_offset;
        if (packet.message_id ==
            static_cast<std::uint8_t>(CommandMessage::Takeoff)) {
            expected_length = 16U;
            token_offset = 8U;
        } else if (
            packet.message_id ==
            static_cast<std::uint8_t>(CommandMessage::Land)) {
            expected_length = 12U;
            token_offset = 4U;
        } else if (
            packet.message_id ==
            static_cast<std::uint8_t>(CommandMessage::Abort)) {
            expected_length = 12U;
            token_offset = 4U;
        }
        if (expected_length.has_value()) {
            std::uint32_t token = 0;
            if (packet.payload.size() >= token_offset.value() + 4U) {
                token = read_le<std::uint32_t>(
                    std::span<const std::uint8_t>(packet.payload),
                    token_offset.value());
            }
            if (packet.payload.size() != expected_length.value()) {
                ++counters.length_errors;
                send_ack_locked(
                    packet, AckResult::BadLength, token);
                return;
            }
            send_ack_locked(
                packet, AckResult::Unsupported, token);
            return;
        }

        ++counters.unknown_commands;
        send_ack_locked(packet, AckResult::Unsupported, 0U);
    }

    void receive_available() {
        std::array<std::uint8_t, kMaximumFrameBytes + 1U> buffer{};
        while (running.load(std::memory_order_relaxed)) {
            const ssize_t received =
                ::recv(fd, buffer.data(), buffer.size(), 0);
            if (received < 0) {
                if (errno == EINTR) {
                    continue;
                }
                return;
            }
            if (received == 0) {
                return;
            }
            const auto now = Clock::now();
            const auto decoded = decode_frame(
                std::span<const std::uint8_t>(
                    buffer.data(),
                    static_cast<std::size_t>(received)));
            const std::lock_guard lock(mutex);
            if (!decoded.ok) {
                switch (decoded.failure) {
                    case DecodeFailure::Crc:
                        ++counters.crc_errors;
                        break;
                    case DecodeFailure::Length:
                        ++counters.length_errors;
                        break;
                    case DecodeFailure::Version:
                        ++counters.version_errors;
                        break;
                    case DecodeFailure::Magic:
                    case DecodeFailure::PacketClass:
                    case DecodeFailure::None:
                        break;
                }
                continue;
            }
            handle_packet_locked(decoded.packet, now);
        }
    }

    void retry_pending_locked(Clock::time_point now) {
        if (!pending.active || now < pending.next_retry) {
            return;
        }
        if (pending.transmissions >= kMaximumTransmissions) {
            pending.active = false;
            ++counters.ack_timeouts;
            return;
        }
        const std::uint8_t flags =
            static_cast<std::uint8_t>(
                PacketFlag::AckRequest) |
            static_cast<std::uint8_t>(PacketFlag::Retry) |
            (pending.urgent
                 ? static_cast<std::uint8_t>(PacketFlag::Urgent)
                 : 0U);
        const auto frame = encode_frame(
            pending.packet_class, pending.message_id, flags,
            pending.sequence,
            std::span<const std::uint8_t>(
                pending.payload.data(), pending.payload.size()));
        (void)raw_send_locked(frame);
        ++pending.transmissions;
        ++counters.tx_retries;
        pending.next_retry = now + kAckRetryInterval;
    }

    void run() {
        while (running.load(std::memory_order_relaxed)) {
            std::array<pollfd, 1> descriptor{{
                {fd, POLLIN, 0},
            }};
            const int poll_result =
                ::poll(descriptor.data(), descriptor.size(), 5);
            if (poll_result < 0 && errno != EINTR) {
                break;
            }
            if (poll_result > 0 &&
                (descriptor[0].revents & POLLIN) != 0) {
                receive_available();
            }
            if (!running.load(std::memory_order_relaxed)) {
                break;
            }
            const auto now = Clock::now();
            {
                const std::lock_guard lock(mutex);
                if (now >= next_heartbeat) {
                    send_heartbeat_locked(now);
                    const double rate =
                        std::max(1.0, config.heartbeat_rate_hz);
                    next_heartbeat =
                        now + std::chrono::duration_cast<Clock::duration>(
                                  std::chrono::duration<double>(
                                      1.0 / rate));
                }
                retry_pending_locked(now);
            }
        }
    }

    bool send_rc_locked(
        const RcCommand& command, Clock::time_point now) {
        if (!link_ready_locked(now)) {
            return false;
        }
        const std::uint32_t sender_time_ms =
            static_cast<std::uint32_t>(
                duration_ms(now - started_at));
        const auto frame = encode_data_rc(
            next_sequence_locked(), command, sender_time_ms);
        return raw_send_locked(frame);
    }

    bool send_command_locked(
        const UplinkRequest& request, Clock::time_point now,
        std::string& error) {
        if (!link_ready_locked(now)) {
            error =
                "GS-Link uplink is disabled until FC telemetry and "
                "heartbeat are fresh";
            return false;
        }
        if (pending.active) {
            error = "another GS-Link command is awaiting ACK";
            return false;
        }

        PacketClass packet_class = PacketClass::Command;
        std::uint8_t message_id = 0;
        std::uint8_t flags =
            static_cast<std::uint8_t>(PacketFlag::AckRequest);
        std::vector<std::uint8_t> payload;
        std::uint32_t token = request.token;
        switch (request.kind) {
            case UplinkKind::Ping:
                message_id =
                    static_cast<std::uint8_t>(CommandMessage::Ping);
                append_le<std::uint32_t>(payload, request.token);
                break;
            case UplinkKind::Takeoff:
                message_id =
                    static_cast<std::uint8_t>(CommandMessage::Takeoff);
                append_le<std::int32_t>(
                    payload, request.target_altitude_mm);
                append_le<std::uint16_t>(
                    payload, request.max_climb_cm_s);
                append_le<std::uint16_t>(payload, request.timeout_ms);
                append_le<std::uint32_t>(payload, request.token);
                append_le<std::uint32_t>(payload, heartbeat.session_id);
                break;
            case UplinkKind::Land:
                message_id =
                    static_cast<std::uint8_t>(CommandMessage::Land);
                append_le<std::uint16_t>(
                    payload, request.max_descent_cm_s);
                append_le<std::uint16_t>(payload, request.timeout_ms);
                append_le<std::uint32_t>(payload, request.token);
                append_le<std::uint32_t>(payload, heartbeat.session_id);
                break;
            case UplinkKind::Abort:
                message_id =
                    static_cast<std::uint8_t>(CommandMessage::Abort);
                append_le<std::uint32_t>(
                    payload, request.reason_code);
                append_le<std::uint32_t>(payload, request.token);
                append_le<std::uint32_t>(payload, heartbeat.session_id);
                flags |= static_cast<std::uint8_t>(
                    PacketFlag::Urgent);
                break;
            case UplinkKind::Rc:
                error = "RC data is not a command packet";
                return false;
        }

        const std::uint16_t sequence = next_sequence_locked();
        const auto frame = encode_frame(
            packet_class, message_id, flags, sequence, payload);
        if (!raw_send_locked(frame)) {
            error = "GS-Link UDP send failed";
            return false;
        }
        pending.active = true;
        pending.packet_class = packet_class;
        pending.message_id = message_id;
        pending.sequence = sequence;
        pending.payload = std::move(payload);
        pending.token = token;
        pending.transmissions = 1;
        pending.next_retry = now + kAckRetryInterval;
        pending.urgent =
            request.kind == UplinkKind::Abort;
        error.clear();
        return true;
    }

    bool send(const UplinkRequest& request, std::string& error) {
        if (!running.load(std::memory_order_relaxed)) {
            error = "GS-Link UDP transport is not running";
            return false;
        }
        if (request.kind == UplinkKind::Rc) {
            if (request.rc.roll < -10000 ||
                request.rc.roll > 10000 ||
                request.rc.pitch < -10000 ||
                request.rc.pitch > 10000 ||
                request.rc.yaw < -10000 ||
                request.rc.yaw > 10000 ||
                request.rc.throttle < -10000 ||
                request.rc.throttle > 10000) {
                error = "DATA_RC axis values must be in [-10000, 10000]";
                return false;
            }
        } else if (request.kind == UplinkKind::Takeoff) {
            if (request.target_altitude_mm <= 0 ||
                request.max_climb_cm_s < 10U ||
                request.max_climb_cm_s > 500U ||
                request.timeout_ms < 1000U ||
                request.timeout_ms > 30000U) {
                error = "invalid CMD_TAKEOFF argument";
                return false;
            }
        } else if (request.kind == UplinkKind::Land) {
            if (request.max_descent_cm_s < 10U ||
                request.max_descent_cm_s > 200U ||
                request.timeout_ms < 1000U ||
                request.timeout_ms > 60000U) {
                error = "invalid CMD_LAND argument";
                return false;
            }
        } else if (request.kind == UplinkKind::Abort) {
            if (request.reason_code < 1U ||
                request.reason_code > 4U) {
                error = "CMD_ABORT reason_code must be in [1, 4]";
                return false;
            }
        }

        if (request.kind != UplinkKind::Rc &&
            request.kind != UplinkKind::Ping &&
            request.token == 0U) {
            error = "flight command token must be non-zero";
            return false;
        }

        const auto now = Clock::now();
        const std::lock_guard lock(mutex);
        if (!transport_connected) {
            error = "GS-Link UDP transport is not connected";
            return false;
        }
        if (request.kind == UplinkKind::Rc) {
            if (!link_ready_locked(now)) {
                error =
                    "DATA_RC is blocked while FC telemetry/heartbeat "
                    "are stale";
                return false;
            }
            if (!send_rc_locked(request.rc, now)) {
                error =
                    "GS-Link UDP DATA_RC send failed: " +
                    std::string(std::strerror(last_send_errno));
                return false;
            }
            error.clear();
            return true;
        }
        return send_command_locked(request, now, error);
    }

    [[nodiscard]] Status status() const {
        const auto now = Clock::now();
        const std::lock_guard lock(mutex);
        Status result;
        result.enabled = true;
        result.transport = "udp";
        result.protocol = "gs-link-v1";
        result.transport_connected = transport_connected;
        result.telemetry = telemetry;
        result.heartbeat = heartbeat;
        result.last_ack = last_ack;
        result.counters = counters;
        result.command_pending = pending.active;
        result.session_id = heartbeat.valid ? heartbeat.session_id : 0U;
        result.telemetry_fresh = fresh(
            now, last_telemetry_rx, config.telemetry_timeout_ms,
            telemetry.valid);
        result.heartbeat_fresh = fresh(
            now, last_heartbeat_rx, config.heartbeat_timeout_ms,
            heartbeat.valid);
        result.uplink_allowed =
            result.transport_connected && result.telemetry_fresh &&
            result.heartbeat_fresh;
        result.last_send_errno = last_send_errno;
        result.telemetry_age_ms =
            age_ms(now, last_telemetry_rx, telemetry.valid);
        result.heartbeat_age_ms =
            age_ms(now, last_heartbeat_rx, heartbeat.valid);
        return result;
    }

    UdpConfig config;
    int fd{-1};
    std::uint16_t bound_port{0};
    std::uint32_t session_id{0};
    std::atomic_bool running{false};
    std::thread thread;
    Clock::time_point started_at{};
    Clock::time_point next_heartbeat{};

    mutable std::mutex mutex;
    bool transport_connected{false};
    int last_send_errno{0};
    std::uint16_t tx_sequence{0};
    std::uint16_t last_rx_sequence{0};
    bool have_rx_sequence{false};
    FlightTelemetry telemetry;
    Heartbeat heartbeat;
    Ack last_ack;
    Clock::time_point last_telemetry_rx{};
    Clock::time_point last_heartbeat_rx{};
    Counters counters;

    struct PendingCommand {
        bool active{false};
        bool urgent{false};
        PacketClass packet_class{PacketClass::Command};
        std::uint8_t message_id{0};
        std::uint16_t sequence{0};
        std::vector<std::uint8_t> payload;
        std::uint32_t token{0};
        int transmissions{0};
        Clock::time_point next_retry{};
    };
    PendingCommand pending;
};

UdpLink::UdpLink(UdpConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

UdpLink::~UdpLink() = default;

bool UdpLink::start(std::string& error) {
    return impl_->start(error);
}

void UdpLink::stop() {
    impl_->stop();
}

bool UdpLink::send(
    const UplinkRequest& request, std::string& error) {
    return impl_->send(request, error);
}

Status UdpLink::status() const {
    return impl_->status();
}

std::uint16_t UdpLink::bound_port() const {
    return impl_->bound_port;
}

}  // namespace dock_ground::gs_link
