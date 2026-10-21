#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace dock_ground::gs_link {

inline constexpr std::uint8_t kVersion = 1;
inline constexpr std::size_t kHeaderBytes = 10;
inline constexpr std::size_t kCrcBytes = 2;
inline constexpr std::size_t kMaximumPayloadBytes = 512;
inline constexpr std::size_t kMaximumFrameBytes =
    kHeaderBytes + kMaximumPayloadBytes + kCrcBytes;
inline constexpr std::uint8_t kMaximumRcChannels = 18;
inline constexpr std::uint16_t kC5ProxyUdpPort = 14550;
inline constexpr std::uint16_t kDefaultGcsProxyUdpPort = 39050;

enum class PacketClass : std::uint8_t {
    Data = 0x01,
    Command = 0x02,
};

enum class DataMessage : std::uint8_t {
    Rc = 0x01,
    Telemetry = 0x02,
};

enum class CommandMessage : std::uint8_t {
    Ping = 0x01,
    Heartbeat = 0x02,
    Takeoff = 0x03,
    Land = 0x04,
    Abort = 0x05,
    Ack = 0x7F,
};

enum class PacketFlag : std::uint8_t {
    AckRequest = 1U << 0U,
    Retry = 1U << 1U,
    Urgent = 1U << 2U,
    Response = 1U << 3U,
};

enum class StatusFlag : std::uint16_t {
    Armed = 1U << 0U,
    GsModeActive = 1U << 1U,
    RcValid = 1U << 2U,
    RcSwitchSelected = 1U << 3U,
    Failsafe = 1U << 4U,
    AttitudeValid = 1U << 5U,
    MotorRpmValid = 1U << 6U,
    PowerValid = 1U << 7U,
    BatteryWarning = 1U << 8U,
    CommandBusy = 1U << 9U,
    TakeoffActive = 1U << 10U,
    LandingActive = 1U << 11U,
    LinkOk = 1U << 12U,
    RcSwitchValid = 1U << 13U,
    RcChannelsValid = 1U << 14U,
};

enum class FlightState : std::uint8_t {
    Boot = 0,
    Ready = 1,
    Auto = 2,
    Failsafe = 3,
    Error = 4,
};

enum class AckResult : std::uint8_t {
    Ok = 0,
    Unsupported = 1,
    BadLength = 2,
    BadArgument = 3,
    RejectedSafety = 4,
    Busy = 5,
    Timeout = 6,
    Duplicate = 7,
    BadVersion = 8,
    InternalError = 9,
    StaleSession = 10,
    LinkStale = 11,
};

enum class DecodeFailure {
    None,
    Magic,
    Version,
    PacketClass,
    Length,
    Crc,
};

struct RcCommand {
    std::int16_t roll{0};
    std::int16_t pitch{0};
    std::int16_t yaw{0};
    std::int16_t throttle{-10000};
    bool valid{false};
    bool gs_ready{false};
    std::uint8_t link_quality{100};
};

struct FlightTelemetry {
    std::uint32_t timestamp_ms{0};
    std::uint16_t status_flags{0};
    std::int16_t roll_cdeg{0};
    std::int16_t pitch_cdeg{0};
    std::int16_t yaw_cdeg{0};
    std::uint16_t motor1_rpm{0xFFFFU};
    std::uint16_t motor2_rpm{0xFFFFU};
    std::uint16_t battery_voltage_mv{0};
    std::int16_t battery_current_ca{-32768};
    std::uint8_t battery_remaining{255};
    std::uint8_t rc_channel_count{0};
    std::array<std::int8_t, kMaximumRcChannels> rc_channels{};
    std::uint64_t received_mono_ns{0};
    bool valid{false};
};

struct Heartbeat {
    std::uint32_t uptime_ms{0};
    std::uint32_t session_id{0};
    std::uint16_t status_flags{0};
    std::uint16_t protocol_version{kVersion};
    FlightState state{FlightState::Boot};
    std::uint8_t error_code{0};
    std::uint64_t received_mono_ns{0};
    bool valid{false};
};

struct Ack {
    std::uint8_t acked_class{0};
    std::uint8_t acked_id{0};
    std::uint16_t acked_sequence{0};
    AckResult result{AckResult::Ok};
    std::uint8_t detail{0};
    std::uint32_t echo_token{0};
    std::uint64_t received_mono_ns{0};
    bool valid{false};
};

struct Counters {
    std::uint64_t rx_packets{0};
    std::uint64_t tx_packets{0};
    std::uint64_t crc_errors{0};
    std::uint64_t length_errors{0};
    std::uint64_t version_errors{0};
    std::uint64_t unknown_data{0};
    std::uint64_t unknown_commands{0};
    std::uint64_t duplicates{0};
    std::uint64_t sequence_gaps{0};
    std::uint64_t tx_retries{0};
    std::uint64_t ack_timeouts{0};
    std::uint64_t send_errors{0};
};

struct Status {
    bool enabled{false};
    const char* transport{"none"};
    const char* protocol{"none"};
    bool transport_connected{false};
    bool telemetry_fresh{false};
    bool heartbeat_fresh{false};
    bool uplink_allowed{false};
    double telemetry_age_ms{-1.0};
    double heartbeat_age_ms{-1.0};
    FlightTelemetry telemetry{};
    Heartbeat heartbeat{};
    Ack last_ack{};
    Counters counters{};
    bool command_pending{false};
    std::uint32_t session_id{0};
    int last_send_errno{0};
};

struct Packet {
    PacketClass packet_class{PacketClass::Data};
    std::uint8_t message_id{0};
    std::uint8_t flags{0};
    std::uint16_t sequence{0};
    std::vector<std::uint8_t> payload;
};

struct DecodeResult {
    bool ok{false};
    DecodeFailure failure{DecodeFailure::None};
    std::string error;
    Packet packet{};
};

struct UdpConfig {
    std::string bind_ip{"0.0.0.0"};
    std::uint16_t bind_port{kDefaultGcsProxyUdpPort};
    std::string remote_ip{"192.168.31.100"};
    std::uint16_t remote_port{kC5ProxyUdpPort};
    std::uint16_t telemetry_timeout_ms{250};
    std::uint16_t heartbeat_timeout_ms{300};
    double heartbeat_rate_hz{10.0};
};

enum class UplinkKind {
    Rc,
    Ping,
    Takeoff,
    Land,
    Abort,
};

struct UplinkRequest {
    UplinkKind kind{UplinkKind::Rc};
    RcCommand rc{};
    std::uint32_t token{0};
    std::int32_t target_altitude_mm{0};
    std::uint16_t max_climb_cm_s{100};
    std::uint16_t max_descent_cm_s{50};
    std::uint16_t timeout_ms{5000};
    std::uint32_t reason_code{1};
};

[[nodiscard]] std::uint16_t crc16(
    std::span<const std::uint8_t> input) noexcept;

[[nodiscard]] std::vector<std::uint8_t> encode_frame(
    PacketClass packet_class, std::uint8_t message_id,
    std::uint8_t flags, std::uint16_t sequence,
    std::span<const std::uint8_t> payload);

[[nodiscard]] std::vector<std::uint8_t> encode_data_rc(
    std::uint16_t sequence, const RcCommand& command,
    std::uint32_t sender_time_ms, std::uint8_t packet_flags = 0);

[[nodiscard]] std::vector<std::uint8_t> encode_telemetry(
    std::uint16_t sequence, const FlightTelemetry& telemetry,
    std::uint8_t packet_flags = 0);

[[nodiscard]] std::vector<std::uint8_t> encode_heartbeat(
    std::uint16_t sequence, const Heartbeat& heartbeat,
    std::uint8_t packet_flags = 0);

[[nodiscard]] std::vector<std::uint8_t> encode_ping(
    std::uint16_t sequence, std::uint32_t token,
    std::uint8_t packet_flags = 0);

[[nodiscard]] std::vector<std::uint8_t> encode_ack(
    std::uint16_t sequence, const Ack& ack,
    std::uint8_t packet_flags = 0);

[[nodiscard]] DecodeResult decode_frame(
    std::span<const std::uint8_t> frame);

[[nodiscard]] bool decode_data_rc(
    const Packet& packet, RcCommand& command,
    std::uint32_t& sender_time_ms, std::string& error);

[[nodiscard]] bool decode_telemetry(
    const Packet& packet, FlightTelemetry& telemetry,
    std::string& error);

[[nodiscard]] bool decode_heartbeat(
    const Packet& packet, Heartbeat& heartbeat,
    std::string& error);

[[nodiscard]] bool decode_ack(
    const Packet& packet, Ack& ack, std::string& error);

[[nodiscard]] const char* to_string(FlightState state) noexcept;
[[nodiscard]] const char* to_string(AckResult result) noexcept;

class UdpLink {
public:
    explicit UdpLink(UdpConfig config);
    ~UdpLink();

    UdpLink(const UdpLink&) = delete;
    UdpLink& operator=(const UdpLink&) = delete;

    [[nodiscard]] bool start(std::string& error);
    void stop();

    [[nodiscard]] bool send(
        const UplinkRequest& request, std::string& error);
    [[nodiscard]] Status status() const;
    [[nodiscard]] std::uint16_t bound_port() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dock_ground::gs_link
