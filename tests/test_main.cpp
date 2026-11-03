#include "dock_ground/config.hpp"
#include "dock_ground/flight_logger.hpp"
#include "dock_ground/gs_link.hpp"
#include "dock_ground/json.hpp"
#include "dock_ground/position_controller.hpp"
#include "dock_ground/runtime.hpp"
#include "dock_ground/web_server.hpp"
#include "dock_ground/web_protocol.hpp"

#include <algorithm>
#include <array>
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <optional>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

int g_failures = 0;

void check(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++g_failures;
    }
}

bool status_flag_set(
    std::uint16_t flags, dock_ground::gs_link::StatusFlag flag) {
    return (flags & static_cast<std::uint16_t>(flag)) != 0U;
}

bool wait_for_packet(
    int socket_fd, std::optional<dock_ground::gs_link::PacketClass> wanted_class,
    std::optional<std::uint8_t> wanted_id,
    dock_ground::gs_link::Packet& packet, sockaddr_in& peer,
    int timeout_ms, std::string& error) {
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        pollfd descriptor{socket_fd, POLLIN, 0};
        const int result = ::poll(&descriptor, 1, 20);
        if (result <= 0) {
            continue;
        }
        std::array<std::uint8_t, 1024> buffer{};
        sockaddr_in source{};
        socklen_t source_length = sizeof(source);
        const ssize_t received = ::recvfrom(
            socket_fd, buffer.data(), buffer.size(), 0,
            reinterpret_cast<sockaddr*>(&source), &source_length);
        if (received <= 0) {
            continue;
        }
        const auto decoded = dock_ground::gs_link::decode_frame(
            std::span<const std::uint8_t>(
                buffer.data(), static_cast<std::size_t>(received)));
        if (!decoded.ok) {
            error = decoded.error;
            continue;
        }
        packet = decoded.packet;
        peer = source;
        if (wanted_class.has_value() &&
            packet.packet_class != *wanted_class) {
            continue;
        }
        if (wanted_id.has_value() && packet.message_id != *wanted_id) {
            continue;
        }
        return true;
    }
    error = "timed out waiting for UDP packet";
    return false;
}

bool send_packet(
    int socket_fd, const sockaddr_in& destination,
    const std::vector<std::uint8_t>& frame, std::string& error) {
    const ssize_t sent = ::sendto(
        socket_fd, frame.data(), frame.size(), 0,
        reinterpret_cast<const sockaddr*>(&destination),
        sizeof(destination));
    if (sent != static_cast<ssize_t>(frame.size())) {
        error = "sendto failed";
        return false;
    }
    return true;
}

std::vector<std::uint8_t> masked_text_frame(std::string_view payload) {
    std::vector<std::uint8_t> frame;
    frame.push_back(0x81U);
    if (payload.size() < 126U) {
        frame.push_back(static_cast<std::uint8_t>(0x80U | payload.size()));
    } else {
        frame.push_back(0x80U | 126U);
        frame.push_back(
            static_cast<std::uint8_t>((payload.size() >> 8U) & 0xFFU));
        frame.push_back(
            static_cast<std::uint8_t>(payload.size() & 0xFFU));
    }
    const std::array<std::uint8_t, 4> mask{
        0x12U, 0x34U, 0x56U, 0x78U};
    frame.insert(frame.end(), mask.begin(), mask.end());
    for (std::size_t index = 0; index < payload.size(); ++index) {
        frame.push_back(
            static_cast<std::uint8_t>(payload[index]) ^
            mask[index % mask.size()]);
    }
    return frame;
}

void test_json() {
    std::string error;
    const auto value = dock_ground::json::parse(
        R"({"name":"dock\nstation","value":1.25,"ok":true,"items":[1,2]})",
        error);
    check(value != nullptr, "valid JSON should parse: " + error);
    if (!value) {
        return;
    }
    const auto* name = value->find("name");
    check(
        name != nullptr && name->string == "dock\nstation",
        "JSON string escape should decode");
    const auto* value_field = value->find("value");
    check(
        value_field != nullptr && value_field->number == 1.25,
        "JSON number should decode");
    const auto* ok = value->find("ok");
    check(
        ok != nullptr && ok->type == dock_ground::json::Value::Type::Boolean &&
            ok->boolean,
        "JSON true should decode as true");

    const auto false_value =
        dock_ground::json::parse(R"({"enabled":false})", error);
    const auto* enabled =
        false_value ? false_value->find("enabled") : nullptr;
    check(
        enabled != nullptr &&
            enabled->type == dock_ground::json::Value::Type::Boolean &&
            !enabled->boolean,
        "JSON false should decode as false");

    check(
        dock_ground::json::parse(R"({"a":1} trailing)", error) == nullptr,
        "trailing JSON data should be rejected");
    check(
        dock_ground::json::parse(R"({"a":1,"a":2})", error) == nullptr,
        "duplicate JSON keys should be rejected");
}

void test_flight_logger_arm_session() {
    const auto path = std::filesystem::temp_directory_path() /
                      ("dock_ground_flight_log_test_" +
                       std::to_string(::getpid()));
    std::error_code filesystem_error;
    std::filesystem::remove_all(path, filesystem_error);

    dock_ground::FlightLogger logger(
        dock_ground::FlightLoggerOptions{
            .enabled = true,
            .directory = path.string(),
            .aircraft_id = "aircraft-01",
            .aircraft_name = "测试飞行器",
            .rc_mapping = {},
            .telemetry_stale_timeout_ms = 2000,
            .udp_flight_transport = true,
            .control_rate_hz = 100.0,
            .warning_pose_age_ms = 30.0,
            .max_pose_age_ms = 50.0,
            .pose_source = "vrpn",
            .pose_endpoint = "MCAvatar@192.168.31.221:3883",
            .pose_sensor_id = 1,
            .pose_position_scale = 0.001,
            .pose_expected_rate_hz = 120.0,
            .pose_auto_origin = true,
            .pose_trust_source_timestamp = false,
        });
    std::string error;
    check(logger.start(error), "flight logger should start: " + error);

    dock_ground::FlightLogSample sample;
    sample.mono_ns = 1'000'000'000ULL;
    sample.wall_time_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    sample.control_state = dock_ground::ControlState::Track;
    sample.pose.valid = true;
    sample.pose.sequence = 7;
    sample.pose.received_mono_ns = 900'000'000ULL;
    sample.pose.source_mono_ns = 1'700'000'000'000'000'000ULL;
    sample.pose.source_time_present = true;
    sample.pose.arrival_interval_ns = 8'333'333ULL;
    sample.pose.source_time_interval_ns = 8'000'000ULL;
    sample.pose.source_clock_delta_ns = 12'000'000LL;
    sample.pose.source_sensor_id = 1;
    sample.pose.position_xyz_m = {1.2, -0.3, 1.5};
    sample.pose.q_body_to_world = {};
    sample.pose.raw_position = {1200.0, -300.0, 1500.0};
    sample.pose.raw_orientation = {};
    sample.pose_age_ms = 8.0;
    sample.pose_fresh = true;
    sample.target.position_xyz_m = {1.4, -0.3, 1.5};
    sample.target.deadman = false;
    sample.regulation.mode =
        dock_ground::PositionRegulationMode::PrecisionHold;
    sample.regulation.active = true;
    sample.regulation.scheduling_enabled = true;
    sample.regulation.horizontal_error_m = 0.083;
    sample.regulation.vertical_error_m = 0.021;
    sample.regulation.active_x = {0.30, 0.005, 0.90};
    sample.regulation.active_y = {0.34, 0.005, 1.00};
    sample.regulation.active_z = {0.08, 0.003, 0.14};
    sample.telemetry.valid = true;
    sample.telemetry_fresh = true;
    sample.heartbeat_fresh = true;
    sample.gs_transport_connected = true;
    sample.uplink_allowed = true;
    sample.telemetry_age_ms = 4.0;
    sample.heartbeat_age_ms = 20.0;
    sample.command_link.connected = true;
    sample.command_link.commands_sent = 3;
    sample.command_link.send_errors = 1;
    sample.command_link.last_error = "udp_send_error";
    sample.command_link.last_errno = 11;
    sample.telemetry.status_flags =
        static_cast<std::uint16_t>(
            dock_ground::gs_link::StatusFlag::RcValid) |
        static_cast<std::uint16_t>(
            dock_ground::gs_link::StatusFlag::RcChannelsValid);
    sample.telemetry.rc_channel_count = 4;
    sample.telemetry.rc_channels[0] = 10;
    sample.telemetry.rc_channels[1] = -12;
    sample.telemetry.rc_channels[2] = 35;
    sample.telemetry.rc_channels[3] = 4;
    sample.telemetry.roll_cdeg = 500;
    sample.telemetry.battery_voltage_mv = 16800;
    sample.heartbeat.valid = true;
    sample.heartbeat.state =
        dock_ground::gs_link::FlightState::Ready;
    sample.source_status.connected = true;
    sample.source_status.transport_connected = true;
    sample.source_status.name = "vrpn";
    sample.source_status.sensor_id = 1;
    sample.source_status.messages_received = 120;
    sample.source_status.callbacks_received = 240;
    sample.source_status.samples_rejected = 1;
    sample.source_status.unselected_sensor_samples = 1;
    sample.source_status.long_gap_count = 2;
    sample.source_status.connect_count = 1;
    sample.source_status.last_received_mono_ns = 900'000'000ULL;
    sample.source_status.expected_rate_hz = 120.0;
    sample.source_status.last_arrival_gap_ms = 8.3;
    sample.source_status.max_arrival_gap_ms = 57.5;
    sample.loop_metrics.target_period_ms = 10.0;
    sample.loop_metrics.last_period_ms = 10.1;
    sample.loop_metrics.work_ms = 1.2;
    sample.loop_metrics.jitter_ms = 0.1;
    sample.loop_metrics.p99_work_ms = 2.3;
    sample.loop_metrics.p99_jitter_ms = 0.4;
    sample.loop_metrics.overruns = 3;
    logger.observe(sample);
    sample.mono_ns += 10'000'000ULL;
    sample.wall_time_ns += 10'000'000LL;
    sample.telemetry.status_flags |= static_cast<std::uint16_t>(
        dock_ground::gs_link::StatusFlag::Armed);
    logger.observe(sample);

    ++sample.mono_ns;
    sample.command_valid = true;
    sample.deadman_active = true;
    sample.handoff_reset = true;
    sample.handoff_reset_reason = "pose_stale";
    sample.handoff_error =
        "waiting for stable handoff, throttle delta=0.606";
    logger.observe(sample);

    ++sample.mono_ns;
    sample.command_valid = false;
    sample.deadman_active = false;
    sample.handoff_reset = false;
    sample.handoff_reset_reason.clear();
    logger.observe(sample);

    ++sample.mono_ns;
    sample.telemetry.status_flags = static_cast<std::uint16_t>(
        sample.telemetry.status_flags &
        ~static_cast<std::uint16_t>(
            dock_ground::gs_link::StatusFlag::Armed));
    sample.telemetry_fresh = true;
    logger.observe(sample);

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(1000);
    while (std::chrono::steady_clock::now() < deadline &&
           logger.status().sessions_written == 0U) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto first_status = logger.status();
    check(
        first_status.sessions_written == 1U &&
            first_status.frames_written >= 4U &&
            !first_status.recording,
        "pre-roll + ARM -> disarm should produce one complete flight log");
    check(
        std::filesystem::is_regular_file(first_status.current_file),
        "flight logger should create a CSV file");
    const std::filesystem::path csv_path(first_status.current_file);
    const std::filesystem::path metadata_path =
        csv_path.parent_path() / (csv_path.stem().string() + ".json");
    const std::filesystem::path summary_path =
        csv_path.parent_path() /
        (csv_path.stem().string() + ".summary.json");
    {
        std::ifstream metadata_input(metadata_path);
        const std::string metadata{
            std::istreambuf_iterator<char>(metadata_input),
            std::istreambuf_iterator<char>()};
        check(
            metadata.find("\"schema\": 2") != std::string::npos &&
                metadata.find("\"pose_source\":") != std::string::npos &&
                metadata.find("\"expected_rate_hz\": 120") !=
                    std::string::npos &&
                metadata.find("\"max_pose_age_ms\": 50") !=
                    std::string::npos &&
                metadata.find("\"pre_roll_s\": 5") !=
                    std::string::npos,
            "flight metadata should capture motion-capture configuration");
    }
    {
        std::ifstream summary_input(summary_path);
        const std::string summary{
            std::istreambuf_iterator<char>(summary_input),
            std::istreambuf_iterator<char>()};
        check(
            summary.find("\"handoff_resets\": 1") != std::string::npos &&
                summary.find("\"first_handoff_reset_reason\": "
                             "\"pose_stale\"") != std::string::npos &&
                summary.find("\"long_gaps\": 0") != std::string::npos,
            "flight summary should retain handoff and VRPN failure evidence");
    }

    std::ifstream input(first_status.current_file);
    std::string header;
    std::string first_row;
    std::string second_row;
    std::getline(input, header);
    std::getline(input, first_row);
    std::getline(input, second_row);
    std::string all_rows = first_row + '\n' + second_row;
    std::string row;
    while (std::getline(input, row)) {
        all_rows += '\n';
        all_rows += row;
    }
    const auto commas = [](std::string_view line) {
        std::size_t count = 0;
        bool quoted = false;
        for (const char character : line) {
            if (character == '"') {
                quoted = !quoted;
            } else if (character == ',' && !quoted) {
                ++count;
            }
        }
        return count;
    };
    check(
        !header.empty() && !first_row.empty() &&
            commas(header) == commas(first_row) &&
            commas(first_row) == commas(second_row) &&
            header.find("rc_ch1") != std::string::npos &&
            header.find("rc_base_roll") != std::string::npos &&
            header.find("heartbeat_fresh") != std::string::npos &&
            header.find("uplink_allowed") != std::string::npos &&
            header.find("command_send_errors") != std::string::npos &&
            header.find("command_last_error") != std::string::npos &&
            header.find("position_mode") != std::string::npos &&
            header.find("active_pid_x_kp") != std::string::npos &&
            header.find("command_actual_rate_hz") != std::string::npos &&
            header.find("pose_received_mono_ns") != std::string::npos &&
            header.find("pose_raw_x") != std::string::npos &&
            header.find("vrpn_callbacks_received") != std::string::npos &&
            header.find("vrpn_long_gap_count") != std::string::npos &&
            header.find("handoff_reset_reason") != std::string::npos &&
            header.find("loop_jitter_ms") != std::string::npos,
        "flight CSV should have consistent columns and RC data fields");
    check(
        all_rows.find(",PRECISION_HOLD,1,1,") != std::string::npos,
        "flight CSV should record the active position regulation mode");
    check(
        all_rows.find(",RC_PASSTHROUGH,") != std::string::npos,
        "flight CSV should distinguish pre-confirmation RC passthrough");
    check(
        all_rows.find(",\"udp_send_error\",11,") != std::string::npos,
        "flight CSV should record command send failure details");
    check(
        all_rows.find(",\"pose_stale\",") != std::string::npos,
        "flight CSV should record why position handoff was reset");

    sample.mono_ns += 1'000'000'000ULL;
    sample.wall_time_ns += 1'000'000'000LL;
    sample.telemetry.status_flags |= static_cast<std::uint16_t>(
        dock_ground::gs_link::StatusFlag::Armed);
    logger.observe(sample);
    ++sample.mono_ns;
    sample.telemetry.status_flags = static_cast<std::uint16_t>(
        sample.telemetry.status_flags &
        ~static_cast<std::uint16_t>(
            dock_ground::gs_link::StatusFlag::Armed));
    logger.observe(sample);

    const auto second_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(1000);
    while (std::chrono::steady_clock::now() < second_deadline &&
           logger.status().sessions_written < 2U) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto second_status = logger.status();
    check(
        second_status.sessions_written == 2U &&
            second_status.current_file != first_status.current_file &&
            std::filesystem::is_regular_file(
                second_status.current_file),
        "each new ARM should create a separate flight log session");

    logger.stop();
    std::filesystem::remove_all(path, filesystem_error);
}

void test_gs_link_codec() {
    using namespace dock_ground::gs_link;
    const std::string crc_vector = "123456789";
    check(
        crc16(std::span<const std::uint8_t>(
            reinterpret_cast<const std::uint8_t*>(crc_vector.data()),
            crc_vector.size())) == 0x29B1U,
        "GS-Link CRC-16/CCITT-FALSE golden vector");

    RcCommand rc;
    rc.throttle = -10000;
    rc.valid = true;
    rc.link_quality = 100;
    const auto frame = encode_data_rc(1U, rc, 1000U);
    const std::array<std::uint8_t, 28> expected{
        0xA5U, 0x5AU, 0x01U, 0x01U, 0x01U, 0x00U, 0x01U, 0x00U,
        0x10U, 0x00U, 0xE8U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x00U, 0xF0U, 0xD8U, 0x01U, 0x64U,
        0x00U, 0x00U, 0x97U, 0xB2U,
    };
    check(
        frame == std::vector<std::uint8_t>(expected.begin(), expected.end()),
        "DATA_RC should match the GS-Link v1 reference frame");
    const auto decoded = decode_frame(frame);
    check(decoded.ok, "reference DATA_RC should decode: " + decoded.error);
    RcCommand decoded_rc;
    std::uint32_t sender_time = 0;
    std::string error;
    check(
        decoded.ok &&
            decode_data_rc(decoded.packet, decoded_rc, sender_time, error) &&
            sender_time == 1000U && decoded_rc.throttle == -10000 &&
            decoded_rc.valid && !decoded_rc.gs_ready,
        "DATA_RC payload should round trip");

    FlightTelemetry telemetry;
    telemetry.timestamp_ms = 123456U;
    telemetry.status_flags =
        static_cast<std::uint16_t>(StatusFlag::Armed) |
        static_cast<std::uint16_t>(StatusFlag::AttitudeValid);
    telemetry.roll_cdeg = 1250;
    telemetry.pitch_cdeg = -250;
    telemetry.yaw_cdeg = 9000;
    telemetry.motor1_rpm = 4200;
    telemetry.motor2_rpm = 3900;
    telemetry.battery_voltage_mv = 16800;
    telemetry.battery_current_ca = -1250;
    telemetry.battery_remaining = 77;
    telemetry.rc_channel_count = 8;
    telemetry.rc_channels[0] = 12;
    telemetry.rc_channels[2] = 64;
    telemetry.rc_channels[3] = -128;
    const auto telemetry_decoded =
        decode_frame(encode_telemetry(9U, telemetry));
    FlightTelemetry round_trip;
    check(
        telemetry_decoded.ok &&
            decode_telemetry(
                telemetry_decoded.packet, round_trip, error) &&
            round_trip.roll_cdeg == 1250 &&
            round_trip.battery_current_ca == -1250 &&
            round_trip.rc_channel_count == 8 &&
            round_trip.rc_channels[2] == 64 &&
            round_trip.rc_channels[3] == -128,
        "DATA_TELEMETRY should round trip");

    auto corrupt = encode_data_rc(2U, rc, 1001U);
    corrupt[15] ^= 0x01U;
    check(
        decode_frame(corrupt).failure == DecodeFailure::Crc,
        "corrupt GS-Link frame should fail CRC");
}

void test_gs_link_udp() {
    using namespace dock_ground::gs_link;
    const int fake_fc = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    check(
        fake_fc >= 0,
        "fake FC UDP socket should be created: " +
            std::string(std::strerror(errno)));
    if (fake_fc < 0) {
        return;
    }
    sockaddr_in fake_address{};
    fake_address.sin_family = AF_INET;
    fake_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    fake_address.sin_port = 0;
    check(
        ::bind(
            fake_fc, reinterpret_cast<const sockaddr*>(&fake_address),
            sizeof(fake_address)) == 0,
        "fake FC UDP socket should bind");
    socklen_t address_length = sizeof(fake_address);
    (void)::getsockname(
        fake_fc, reinterpret_cast<sockaddr*>(&fake_address),
        &address_length);

    UdpLink link(UdpConfig{
        .bind_ip = "127.0.0.1",
        .bind_port = 0,
        .remote_ip = "127.0.0.1",
        .remote_port = ntohs(fake_address.sin_port),
        .telemetry_timeout_ms = 250,
        .heartbeat_timeout_ms = 300,
        .heartbeat_rate_hz = 20.0,
    });
    std::string error;
    check(link.start(error), "GS-Link UDP link should start: " + error);

    Packet packet;
    sockaddr_in peer{};
    check(
        wait_for_packet(
            fake_fc, std::nullopt, std::nullopt,
            packet, peer, 500, error),
        "first GS-Link packet should reach fake C5: " + error);
    check(
        packet.packet_class == PacketClass::Command &&
            packet.message_id ==
                static_cast<std::uint8_t>(CommandMessage::Heartbeat),
        "first packet must be valid CMD_HEARTBEAT for C5 auto-learning");
    check(
        ntohs(peer.sin_port) == link.bound_port(),
        "C5 should learn the same bound source port used for replies");

    Heartbeat heartbeat;
    heartbeat.uptime_ms = 5000U;
    heartbeat.session_id = 0x12345678U;
    heartbeat.status_flags =
        static_cast<std::uint16_t>(StatusFlag::LinkOk);
    heartbeat.state = FlightState::Ready;
    check(
        send_packet(
            fake_fc, peer,
            encode_heartbeat(100U, heartbeat), error),
        "fake FC heartbeat should send: " + error);

    FlightTelemetry telemetry;
    telemetry.timestamp_ms = 6000U;
    telemetry.status_flags =
        static_cast<std::uint16_t>(StatusFlag::AttitudeValid) |
        static_cast<std::uint16_t>(StatusFlag::RcChannelsValid);
    telemetry.roll_cdeg = 1000;
    telemetry.rc_channel_count = 4;
    telemetry.rc_channels[0] = 10;
    telemetry.rc_channels[1] = -20;
    check(
        send_packet(
            fake_fc, peer,
            encode_telemetry(101U, telemetry), error),
        "fake FC telemetry should send: " + error);

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto status = link.status();
        if (status.telemetry_fresh && status.heartbeat_fresh &&
            status.uplink_allowed) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto status = link.status();
    check(status.telemetry_fresh, "UDP telemetry should become fresh");
    check(status.heartbeat_fresh, "UDP heartbeat should become fresh");
    check(status.uplink_allowed, "fresh session should allow uplink");
    check(
        status.session_id == 0x12345678U,
        "FC session should be retained for command generation");

    UplinkRequest rc_request;
    rc_request.kind = UplinkKind::Rc;
    rc_request.rc.roll = 2000;
    rc_request.rc.pitch = -1000;
    rc_request.rc.yaw = 300;
    rc_request.rc.throttle = 4000;
    rc_request.rc.valid = true;
    rc_request.rc.gs_ready = true;
    check(
        link.send(rc_request, error),
        "DATA_RC should send after handshake: " + error);
    check(
        wait_for_packet(
            fake_fc, PacketClass::Data,
            static_cast<std::uint8_t>(DataMessage::Rc),
            packet, peer, 300, error),
        "fake FC should receive DATA_RC: " + error);
    RcCommand received_rc;
    std::uint32_t sender_time = 0;
    check(
        decode_data_rc(packet, received_rc, sender_time, error) &&
            received_rc.roll == 2000 &&
            received_rc.throttle == 4000 && received_rc.valid &&
            received_rc.gs_ready,
        "UDP DATA_RC should preserve uplink fields");

    UplinkRequest ping;
    ping.kind = UplinkKind::Ping;
    ping.token = 77U;
    check(
        link.send(ping, error),
        "CMD_PING should be accepted: " + error);
    check(
        wait_for_packet(
            fake_fc, PacketClass::Command,
            static_cast<std::uint8_t>(CommandMessage::Ping),
            packet, peer, 300, error),
        "fake FC should receive CMD_PING: " + error);
    Ack ack;
    ack.acked_class =
        static_cast<std::uint8_t>(packet.packet_class);
    ack.acked_id = packet.message_id;
    ack.acked_sequence = packet.sequence;
    ack.result = AckResult::Ok;
    ack.echo_token = 77U;
    check(
        send_packet(fake_fc, peer, encode_ack(102U, ack), error),
        "fake FC ACK should send: " + error);
    const auto ack_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < ack_deadline &&
           !link.status().last_ack.valid) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    status = link.status();
    check(
        status.last_ack.valid &&
            status.last_ack.echo_token == 77U &&
            status.last_ack.result == AckResult::Ok &&
            !status.command_pending,
        "CMD_ACK should complete the pending uplink command");

    std::this_thread::sleep_for(std::chrono::milliseconds(350));
    telemetry.timestamp_ms += 1U;
    check(
        send_packet(
            fake_fc, peer,
            encode_telemetry(103U, telemetry), error),
        "fresh telemetry should send after heartbeat expiry: " + error);
    const auto telemetry_only_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(100);
    while (std::chrono::steady_clock::now() < telemetry_only_deadline &&
           !link.status().telemetry_fresh) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    status = link.status();
    check(
        status.telemetry_fresh && !status.heartbeat_fresh &&
            status.uplink_allowed,
        "DATA_RC uplink should survive a missing low-rate heartbeat");
    check(
        link.send(rc_request, error),
        "DATA_RC should send with fresh telemetry and stale heartbeat: " +
            error);
    check(
        wait_for_packet(
            fake_fc, PacketClass::Data,
            static_cast<std::uint8_t>(DataMessage::Rc),
            packet, peer, 300, error),
        "heartbeat-stale DATA_RC should reach fake FC: " + error);
    check(
        !link.send(ping, error),
        "session commands must remain blocked while heartbeat is stale");

    std::this_thread::sleep_for(std::chrono::milliseconds(350));
    status = link.status();
    check(
        !status.telemetry_fresh && !status.heartbeat_fresh &&
            !status.uplink_allowed,
        "UDP link freshness gates should expire");
    check(
        !link.send(rc_request, error),
        "DATA_RC must stop when telemetry is stale");

    link.stop();
    (void)::close(fake_fc);
}

void test_websocket_protocol() {
    check(
        dock_ground::web::websocket_accept("dGhlIHNhbXBsZSBub25jZQ==") ==
            "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=",
        "RFC 6455 handshake accept value");

    const std::string payload = R"({"v":1})";
    const std::array<std::uint8_t, 4> mask{0x12U, 0x34U, 0x56U, 0x78U};
    std::string frame;
    frame.push_back(static_cast<char>(0x81U));
    frame.push_back(static_cast<char>(0x80U | payload.size()));
    for (const std::uint8_t byte : mask) {
        frame.push_back(static_cast<char>(byte));
    }
    for (std::size_t index = 0; index < payload.size(); ++index) {
        frame.push_back(static_cast<char>(
            static_cast<std::uint8_t>(payload[index]) ^
            mask[index % mask.size()]));
    }

    const auto decoded = dock_ground::web::decode_client_frame(frame);
    check(
        decoded.status == dock_ground::web::FrameDecodeStatus::Complete,
        "masked client frame should decode");
    check(decoded.payload == payload, "WebSocket payload should round trip");

    const std::string server_frame =
        dock_ground::web::encode_websocket_frame(payload);
    check(
        !server_frame.empty() &&
            static_cast<std::uint8_t>(server_frame[0]) == 0x81U,
        "server frame should be final text");
}

void test_controller() {
    dock_ground::PositionController controller(
        dock_ground::PositionControllerConfig{});
    dock_ground::PoseSample pose;
    pose.received_mono_ns = 1'000'000'000ULL;
    pose.sequence = 1;
    pose.position_xyz_m = {0.0, 0.0, 1.5};
    pose.q_body_to_world = {};
    pose.valid = true;

    dock_ground::OperatorTarget target;
    target.position_xyz_m = pose.position_xyz_m;
    target.deadman = true;
    const dock_ground::ManualControlSetpoint hover_base{
        0.0, 0.0, 0.0, -0.4, true};
    auto output = controller.step(
        pose, target, pose.received_mono_ns, hover_base);
    check(output.valid, "hover controller output should be valid");
    check(
        std::abs(
            output.upper_throttle -
            controller.config().hover_throttle) < 1e-6,
        "zero height error should use calibrated hover feedforward");
    check(
        std::abs(output.q_body_to_world_desired.w - 1.0) < 1e-6,
        "hover desired attitude should be identity");
    check(
        std::abs(output.manual.roll) < 1e-9 &&
            std::abs(output.manual.pitch) < 1e-9 &&
            std::abs(output.manual.yaw) < 1e-9,
        "hover should center roll/pitch/yaw channels");

    controller.reset();
    target.position_xyz_m = pose.position_xyz_m;
    target.yaw_rad = 3.14159265358979323846 / 2.0;
    output = controller.step(
        pose, target, pose.received_mono_ns, hover_base);
    check(
        std::abs(output.yaw_rate_right_command_rad_s) <=
            controller.config().yaw_rate_slew_rate_rad_s2 * 0.01 + 1e-9,
        "yaw-rate command must respect the configured slew limit");
    for (int index = 1; index < 40; ++index) {
        output = controller.step(
            pose, target,
            pose.received_mono_ns +
                static_cast<std::uint64_t>(index) * 10'000'000ULL,
            hover_base);
    }
    check(
        output.q_body_to_world_desired.z > 0.7,
        "positive yaw around +Z should rotate nose from +X toward +Y");
    check(
        output.manual.yaw < -0.30 &&
            output.manual.yaw >=
                -controller.config().max_yaw_rate_rad_s /
                    controller.config().fc_yaw_rate_scale_rad_s - 1e-6,
        "Dock +Z yaw target must use bounded PX4 right-yaw command");

    controller.reset();
    target.yaw_rad = 0.0;
    dock_ground::PoseSample rotating_pose = pose;
    for (int index = 0; index < 40; ++index) {
        const double yaw = 0.002 * static_cast<double>(index);
        rotating_pose.q_body_to_world = {
            std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
        rotating_pose.received_mono_ns =
            pose.received_mono_ns +
            static_cast<std::uint64_t>(index) * 10'000'000ULL;
        target.yaw_rad = yaw;
        output = controller.step(
            rotating_pose, target, rotating_pose.received_mono_ns,
            hover_base);
    }
    check(
        output.yaw_rate_right_command_rad_s > 0.05,
        "measured Dock yaw rate should produce an opposing right-yaw command");

    controller.reset();
    target.position_xyz_m = {1.0, 0.0, 1.5};
    target.yaw_rad = 0.0;
    output = controller.step(
        pose, target, pose.received_mono_ns, hover_base);
    check(output.valid, "+X target should produce a valid output");
    check(
        output.acceleration_xyz_mps2.x > 0.0,
        "+X position error should request +X acceleration");
    check(
        output.manual.pitch > 0.0 &&
            std::abs(output.manual.roll) < 1e-6,
        "+X/nose-forward command must use positive nose-down pitch");

    controller.reset();
    const dock_ground::ManualControlSetpoint captured_rc_base{
        -0.15, -0.20, 0.10, 0.25, true};
    output = controller.step(
        pose, target, pose.received_mono_ns, captured_rc_base);
    check(
        std::abs(output.manual.roll - captured_rc_base.roll) < 1e-9 &&
            output.manual.pitch > captured_rc_base.pitch &&
            std::abs(output.manual.yaw - captured_rc_base.yaw) < 1e-9 &&
            std::abs(
                output.upper_throttle -
                controller.config().hover_throttle) < 1e-6,
        "height control should use hover feedforward after the handoff hold");

    controller.reset();
    target.position_xyz_m = pose.position_xyz_m;
    const dock_ground::ManualControlSetpoint elevated_rc_base{
        0.0, 0.0, 0.0, 0.9, true};
    output = controller.step(
        pose, target, pose.received_mono_ns, elevated_rc_base);
    check(
        std::abs(
            output.upper_throttle -
            controller.config().hover_throttle) < 1e-6,
        "zero height error must not retain an elevated RC throttle");

    controller.reset();
    target.position_xyz_m = {0.0, 0.0, 1.0};
    output = controller.step(
        pose, target, pose.received_mono_ns, hover_base);
    check(
        output.upper_throttle < controller.config().hover_throttle,
        "over-target height must command below the calibrated hover throttle");

    auto velocity_damped_config = dock_ground::PositionControllerConfig{};
    velocity_damped_config.z_velocity_damping = 0.08;
    auto velocity_undamped_config = dock_ground::PositionControllerConfig{};
    velocity_undamped_config.z_velocity_damping = 0.0;
    dock_ground::PositionController velocity_damped(
        velocity_damped_config);
    dock_ground::PositionController velocity_undamped(
        velocity_undamped_config);
    target.position_xyz_m = pose.position_xyz_m;
    dock_ground::PoseSample descending_pose = pose;
    dock_ground::PositionControllerOutput damped_output;
    dock_ground::PositionControllerOutput undamped_output;
    for (int index = 0; index < 12; ++index) {
        descending_pose.received_mono_ns =
            pose.received_mono_ns +
            static_cast<std::uint64_t>(index) * 10'000'000ULL;
        descending_pose.position_xyz_m.z =
            pose.position_xyz_m.z - 0.02 * index;
        damped_output = velocity_damped.step(
            descending_pose, target, descending_pose.received_mono_ns,
            hover_base);
        undamped_output = velocity_undamped.step(
            descending_pose, target, descending_pose.received_mono_ns,
            hover_base);
    }
    check(
        damped_output.manual.throttle >
            undamped_output.manual.throttle + 0.10,
        "downward vertical velocity must add braking throttle");

    controller.reset();
    target.position_xyz_m = {0.0, 1.0, 1.5};
    output = controller.step(
        pose, target, pose.received_mono_ns, hover_base);
    check(
        output.manual.roll < 0.0 &&
            std::abs(output.manual.pitch) < 1e-6,
        "+Y/left command must use negative right-roll channel");

    controller.reset();
    target.position_xyz_m = {1.0, 0.0, 1.5};
    target.yaw_rad = 3.14159265358979323846 / 2.0;
    output = controller.step(
        pose, target, pose.received_mono_ns, hover_base);
    const auto& forward_q = output.q_body_to_world_desired;
    const double forward_body_z_x =
        2.0 * (forward_q.x * forward_q.z + forward_q.w * forward_q.y);
    const double forward_body_z_y =
        2.0 * (forward_q.y * forward_q.z - forward_q.w * forward_q.x);
    check(
        forward_body_z_x > 0.05 &&
            std::abs(forward_body_z_y) < 1e-6,
        "rotated-yaw +X command must tilt thrust toward world +X");

    controller.reset();
    target.position_xyz_m = {0.0, 1.0, 1.5};
    output = controller.step(
        pose, target, pose.received_mono_ns, hover_base);
    const auto& left_q = output.q_body_to_world_desired;
    const double left_body_z_x =
        2.0 * (left_q.x * left_q.z + left_q.w * left_q.y);
    const double left_body_z_y =
        2.0 * (left_q.y * left_q.z - left_q.w * left_q.x);
    check(
        left_body_z_y > 0.05 &&
            std::abs(left_body_z_x) < 1e-6,
        "rotated-yaw +Y command must tilt thrust toward world +Y");

    controller.reset();
    target.yaw_rad = 0.0;
    target.position_xyz_m = {0.0, 0.0, 2.0};
    output = controller.step(
        pose, target, pose.received_mono_ns, hover_base);
    check(
        output.manual.throttle > -0.4 &&
            output.upper_throttle > 0.30,
        "+Z climb error must increase the upper throttle");
    check(
        std::abs(output.manual.roll) <= 1.0 &&
            std::abs(output.manual.pitch) <= 1.0 &&
            std::abs(output.manual.yaw) <= 1.0 &&
            output.manual.throttle >= -1.0 &&
            output.manual.throttle <= 1.0,
        "four DATA_RC channels must remain normalized");
    const double q_norm =
        std::sqrt(
            output.q_body_to_world_desired.w *
                output.q_body_to_world_desired.w +
            output.q_body_to_world_desired.x *
                output.q_body_to_world_desired.x +
            output.q_body_to_world_desired.y *
                output.q_body_to_world_desired.y +
            output.q_body_to_world_desired.z *
                output.q_body_to_world_desired.z);
    check(
        std::abs(q_norm - 1.0) < 1e-9,
        "desired quaternion must stay normalized");

    auto tuning = controller.config();
    tuning.x.kp = 2.5;
    controller.set_config(tuning);
    check(
        std::abs(controller.config().x.kp - 2.5) < 1e-12,
        "position PID X gains should update independently");
    check(
        std::abs(controller.config().y.kp - 0.60) < 1e-12,
        "updating X gains must not overwrite Y gains");

    auto position_config = dock_ground::PositionControllerConfig{};
    position_config.x = {1.0, 0.0, 0.0};
    position_config.y = {1.0, 0.0, 0.0};
    dock_ground::PositionController position_controller(position_config);
    target.position_xyz_m = {1.0, 0.0, 1.5};
    const auto first = position_controller.step(
        pose, target, 2'000'000'000ULL, hover_base);
    target.position_xyz_m = pose.position_xyz_m;
    const auto second =
        position_controller.step(
            pose, target, 2'010'000'000ULL, hover_base);
    check(
        first.acceleration_xyz_mps2.x > 0.9 &&
            std::abs(second.acceleration_xyz_mps2.x) < 1e-9,
        "position X PID should return to zero when the error is removed");

    dock_ground::PositionController constant_error(position_config);
    target.position_xyz_m = {1.0, 0.0, 1.5};
    const auto constant_first = constant_error.step(
        pose, target, 4'000'000'000ULL, hover_base);
    const auto constant_second = constant_error.step(
        pose, target, 4'010'000'000ULL, hover_base);
    check(
        constant_first.acceleration_xyz_mps2.x > 0.9 &&
            constant_second.acceleration_xyz_mps2.x > 0.9,
        "position PID must retain its absolute Kp correction at constant error");

    auto low_derivative_config = dock_ground::PositionControllerConfig{};
    low_derivative_config.x = {1.2, 0.0, 0.4};
    dock_ground::PositionController low_derivative(low_derivative_config);
    target.position_xyz_m = pose.position_xyz_m;
    (void)low_derivative.step(
        pose, target, 5'000'000'000ULL, hover_base);
    target.position_xyz_m.x += 0.25;
    const auto stepped = low_derivative.step(
        pose, target, 5'010'000'000ULL, hover_base);
    check(
        std::abs(stepped.acceleration_xyz_mps2.x) < 0.5,
        "a target step must not create a derivative-on-error acceleration kick");

    dock_ground::PositionController velocity_feedforward(
        low_derivative_config);
    target.position_xyz_m = {
        pose.position_xyz_m.x + 1.0,
        pose.position_xyz_m.y,
        pose.position_xyz_m.z,
    };
    target.target_velocity_xyz_mps = {0.5, 0.0, 0.0};
    const auto fed_forward = velocity_feedforward.step(
        pose, target, 5'020'000'000ULL, hover_base);
    check(
        std::abs(fed_forward.acceleration_xyz_mps2.x - 1.4) < 1e-6,
        "target velocity should add the original Kd-scaled feedforward");
    target.target_velocity_xyz_mps = {};

    auto damped_config = dock_ground::PositionControllerConfig{};
    damped_config.x = {0.60, 0.01, 0.30};
    dock_ground::PositionController damped(damped_config);
    target.position_xyz_m = {1.0, 0.0, 1.5};
    const auto damped_first = damped.step(
        pose, target, 3'000'000'000ULL, hover_base);
    const auto damped_second = damped.step(
        pose, target, 3'010'000'000ULL, hover_base);
    check(
        damped_first.acceleration_xyz_mps2.x > 0.55 &&
            damped_second.acceleration_xyz_mps2.x > 0.55 &&
            std::abs(
                damped_second.acceleration_xyz_mps2.x -
                damped_first.acceleration_xyz_mps2.x) < 0.01 &&
            damped_second.manual.pitch > 0.0,
        "D-state initialization must not kick the second PID sample");

    const double half_sqrt_two =
        std::sqrt(2.0) / 2.0;
    dock_ground::PoseSample yawed_pose = pose;
    yawed_pose.q_body_to_world = {
        half_sqrt_two, 0.0, 0.0, half_sqrt_two};

    controller.reset();
    target.position_xyz_m = {
        yawed_pose.position_xyz_m.x + 1.0,
        yawed_pose.position_xyz_m.y,
        yawed_pose.position_xyz_m.z,
    };
    target.yaw_rad = 0.0;
    output = controller.step(
        yawed_pose, target, 6'000'000'000ULL, hover_base);
    check(
        output.manual.roll > 0.0 &&
            std::abs(output.manual.pitch) < 1e-6,
        "world +X acceleration at absolute +90 deg VRPN yaw must use body roll");

    controller.reset();
    target.position_xyz_m = {
        yawed_pose.position_xyz_m.x,
        yawed_pose.position_xyz_m.y + 1.0,
        yawed_pose.position_xyz_m.z,
    };
    output = controller.step(
        yawed_pose, target, 6'010'000'000ULL, hover_base);
    check(
        output.manual.pitch > 0.0 &&
            std::abs(output.manual.roll) < 1e-6,
        "world +Y acceleration at absolute +90 deg VRPN yaw must use body pitch");
}

void test_position_mode_scheduling() {
    dock_ground::PositionControllerConfig config{};
    config.mode_scheduling.enabled = true;
    config.mode_scheduling.hold_enter_dwell_s = 0.03;
    config.mode_scheduling.hold_exit_dwell_s = 0.01;
    config.mode_scheduling.gain_transition_time_s = 0.0;
    dock_ground::PositionController controller(config);

    dock_ground::PoseSample pose;
    pose.valid = true;
    pose.position_xyz_m = {0.0, 0.0, 1.5};
    pose.q_body_to_world = {};
    dock_ground::OperatorTarget target;
    target.position_xyz_m = pose.position_xyz_m;
    target.deadman = true;
    const dock_ground::ManualControlSetpoint base{
        0.0, 0.0, 0.0, 0.2, true};

    const auto step_at =
        [&controller, &pose, &target, &base](
            std::uint64_t time_ns) {
            return controller.step(pose, target, time_ns, base);
        };

    auto output = step_at(1'000'000'000ULL);
    check(
        output.regulation.mode ==
            dock_ground::PositionRegulationMode::Approach,
        "an initially near target should select APPROACH immediately");
    output = step_at(1'010'000'000ULL);
    check(
        output.regulation.mode ==
            dock_ground::PositionRegulationMode::Approach,
        "near target should enter APPROACH");
    (void)step_at(1'020'000'000ULL);
    (void)step_at(1'030'000'000ULL);
    output = step_at(1'040'000'000ULL);
    check(
        output.regulation.mode ==
            dock_ground::PositionRegulationMode::PrecisionHold,
        "stable near-target motion should enter PRECISION_HOLD after dwell");
    check(
        std::abs(output.regulation.active_x.kp -
                 config.precision_hold.x.kp) < 1e-9 &&
            std::abs(output.regulation.active_y.kd -
                     config.precision_hold.y.kd) < 1e-9,
        "precision mode should use the configured hold gains");

    target.target_velocity_xyz_mps = {0.30, 0.0, 0.0};
    output = step_at(1'050'000'000ULL);
    check(
        output.regulation.mode ==
            dock_ground::PositionRegulationMode::FarTrack,
        "moving target should leave precision hold immediately");
    check(
        std::abs(output.regulation.active_x.kp -
                 config.x.kp) < 1e-9,
        "moving target should restore far-track gains");
    target.target_velocity_xyz_mps = {};

    pose.received_mono_ns = 1'060'000'000ULL;
    output = step_at(1'060'000'000ULL);
    output = step_at(1'070'000'000ULL);
    check(
        output.regulation.mode ==
            dock_ground::PositionRegulationMode::Approach,
        "stationary near-target position should re-enter APPROACH");
    (void)step_at(1'080'000'000ULL);
    (void)step_at(1'090'000'000ULL);
    output = step_at(1'100'000'000ULL);
    check(
        output.regulation.mode ==
            dock_ground::PositionRegulationMode::PrecisionHold,
        "stable position should return to PRECISION_HOLD");

    pose.position_xyz_m.x = 0.23;
    pose.received_mono_ns = 1'110'000'000ULL;
    output = step_at(1'110'000'000ULL);
    check(
        output.regulation.mode ==
            dock_ground::PositionRegulationMode::PrecisionHold,
        "hold mode should remain latched during the exit dwell");
    (void)step_at(1'120'000'000ULL);
    output = step_at(1'130'000'000ULL);
    check(
        output.regulation.mode ==
            dock_ground::PositionRegulationMode::Approach,
        "sustained hold-window violation should enter APPROACH");

    pose.position_xyz_m.x = 0.80;
    pose.received_mono_ns = 1'140'000'000ULL;
    output = step_at(1'140'000'000ULL);
    check(
        output.regulation.mode ==
            dock_ground::PositionRegulationMode::FarTrack,
        "large position excursion should restore FAR_TRACK");

    pose.position_xyz_m.x = 0.0;
    pose.received_mono_ns = 1'150'000'000ULL;
    target.position_xyz_m.z = 2.0;
    output = step_at(1'150'000'000ULL);
    check(
        output.regulation.mode ==
            dock_ground::PositionRegulationMode::FarTrack,
        "large vertical error should keep the fast far-track profile");

    target.position_xyz_m.z = 1.7;
    output = step_at(1'160'000'000ULL);
    check(
        output.regulation.mode ==
            dock_ground::PositionRegulationMode::Approach,
        "zero horizontal error and a near vertical error should enter APPROACH");

    auto smooth_config = config;
    smooth_config.mode_scheduling.gain_transition_time_s = 0.20;
    dock_ground::PositionController smooth_controller(smooth_config);
    dock_ground::PoseSample smooth_pose;
    smooth_pose.valid = true;
    smooth_pose.position_xyz_m = {1.0, 0.0, 1.5};
    smooth_pose.q_body_to_world = {};
    smooth_pose.received_mono_ns = 2'000'000'000ULL;
    dock_ground::OperatorTarget smooth_target;
    smooth_target.position_xyz_m = {0.0, 0.0, 1.5};
    smooth_target.deadman = true;
    (void)smooth_controller.step(
        smooth_pose, smooth_target, 2'000'000'000ULL, base);
    smooth_pose.position_xyz_m.x = 0.40;
    smooth_pose.received_mono_ns = 2'010'000'000ULL;
    const auto transitioning = smooth_controller.step(
        smooth_pose, smooth_target, 2'010'000'000ULL, base);
    check(
        transitioning.regulation.mode ==
            dock_ground::PositionRegulationMode::Approach,
        "mode change should select APPROACH before gain blending");
    check(
        transitioning.regulation.gain_transition_active,
        "gain blending should report an active transition");
    check(
        transitioning.regulation.active_x.kp >
                std::min(
                    smooth_config.approach.x.kp,
                    smooth_config.x.kp) &&
            transitioning.regulation.active_x.kp <
                std::max(
                    smooth_config.approach.x.kp,
                    smooth_config.x.kp),
        "mode changes should blend gains without an instantaneous jump: "
        "expected (" +
            std::to_string(smooth_config.approach.x.kp) + ", " +
            std::to_string(smooth_config.x.kp) + "), got " +
            std::to_string(transitioning.regulation.active_x.kp));
}

void test_config() {
    const std::string path =
        std::string(DOCK_GROUND_SOURCE_DIR) + "/config/ground.json";
    std::string error;
    const auto config = dock_ground::load_config_file(path, error);
    check(config.has_value(), "example config should load: " + error);
    if (config) {
        check(config->http_port == 8080, "config HTTP port");
        check(
            config->pose_source == "vrpn",
            "example config should use live VRPN");
        check(
            config->vrpn_endpoint == "MCAvatar@192.168.31.221:3883",
            "example config VRPN endpoint");
        check(
            config->vrpn_sensor_id == 1,
            "example config should bind body1/sensor 1");
        check(
            std::abs(config->vrpn_expected_rate_hz - 120.0) < 1e-9,
            "example config should declare the expected VRPN rate");
        check(
            std::abs(config->flight_log_pre_roll_s - 5.0) < 1e-9,
            "example config should retain five seconds before ARM");
        const std::array<double, 9> identity{
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        };
        check(
            config->vrpn_axes_from_vrpn == identity,
            "VRPN source XYZ should map one-to-one to station XYZ");
        check(
            config->aircraft_id == "aircraft-01",
            "example config aircraft instance");
        check(
            config->initial_target_use_current_pose,
            "example config should initialize target from VRPN pose");
        check(
            !config->vrpn_trust_source_timestamp,
            "example config must not trust unsynchronized VRPN timestamps");
        check(
            config->flight_protocol == "null",
            "real flight protocol must remain disabled");
        check(
            config->flight_transport == "loopback",
            "phase 1 transport should be loopback");
        check(
            std::abs(config->position_loop.max_tilt_rad -
                     0.3141592654) < 1e-9 &&
            std::abs(config->position_loop.hover_throttle - 0.606) <
                    1e-9,
            "position loop should use FC-matched 18 deg and calibrated hover");
        check(
            std::abs(config->position_loop.x.kp - 1.35) < 1e-9 &&
                std::abs(config->position_loop.x.ki - 0.05) < 1e-9 &&
                std::abs(config->position_loop.x.kd - 0.60) < 1e-9 &&
                std::abs(config->position_loop.y.kp - 1.50) < 1e-9 &&
                std::abs(config->position_loop.y.ki - 0.06) < 1e-9 &&
                std::abs(config->position_loop.y.kd - 0.72) < 1e-9,
            "position X/Y far gains should match the latest flight");
        check(
            std::abs(config->position_loop.z.kp - 0.22) < 1e-9 &&
                std::abs(config->position_loop.z.ki - 0.015) < 1e-9 &&
                std::abs(config->position_loop.z.kd - 0.10) < 1e-9,
            "position Z far gains should match the latest flight");
        check(
            config->rc_mapping.roll == 1 &&
                config->rc_mapping.pitch == 2 &&
                config->rc_mapping.throttle == 3 &&
                config->rc_mapping.yaw == 4 &&
                config->rc_mapping.handoff_hold_ms == 40 &&
                config->rc_mapping.throttle_transition_ms == 300 &&
                std::abs(
                    config->rc_mapping
                        .handoff_max_vertical_speed_mps -
                    0.20) < 1e-9 &&
                std::abs(
                    config->rc_mapping.handoff_max_throttle_delta -
                    0.10) < 1e-9,
            "RC mapping and safe handoff windows should load");
        check(
            std::abs(config->position_loop.z_velocity_damping - 0.35) <
                1e-9,
            "vertical velocity damping should load");
        check(
            config->position_loop.mode_scheduling.enabled &&
                std::abs(
                    config->position_loop.mode_scheduling
                        .hold_enter_distance_m -
                    0.12) < 1e-9 &&
                std::abs(
                    config->position_loop.mode_scheduling
                        .hold_exit_distance_m -
                    0.22) < 1e-9 &&
                std::abs(
                    config->position_loop.approach.x.kd -
                    1.10) < 1e-9 &&
                std::abs(
                    config->position_loop.precision_hold.y.kp -
                    0.34) < 1e-9,
            "three-state position scheduling parameters should load");
        check(
            std::abs(config->target_slew_rate_mps - 0.30) < 1e-9,
            "target position slew rate should load");
        check(
            std::abs(config->data_rc_rate_hz - 100.0) < 1e-9,
            "DATA_RC output rate should load at 100 Hz");
        check(
            std::abs(config->position_loop.max_yaw_rate_rad_s - 0.50) <
                    1e-9 &&
                std::abs(config->position_loop.yaw_rate_kp - 1.0) < 1e-9 &&
                std::abs(config->position_loop.yaw_rate_kd - 0.50) < 1e-9 &&
                std::abs(
                    config->position_loop.yaw_rate_slew_rate_rad_s2 -
                    1.5) < 1e-9 &&
                std::abs(
                    config->position_loop.fc_yaw_rate_scale_rad_s -
                    1.5) < 1e-9,
            "yaw damping, rate limit, slew and FC scale should load");
        check(
            config->flight_log_enabled &&
                config->flight_log_directory == "logs/flights" &&
                config->flight_log_stale_timeout_ms == 2000,
            "ARM-triggered flight logging config should load");
    }

    const std::string gs_path =
        std::string(DOCK_GROUND_SOURCE_DIR) +
        "/config/ground.gs-link.example.json";
    error.clear();
    const auto gs_config =
        dock_ground::load_config_file(gs_path, error);
    check(
        gs_config.has_value(),
        "GS-Link example config should load: " + error);
    if (gs_config) {
        check(
            gs_config->flight_transport == "udp",
            "GS-Link example should select UDP");
        check(
            gs_config->flight_protocol == "gs_link_v1",
            "GS-Link example should select the v1 protocol");
        check(
            std::abs(gs_config->data_rc_rate_hz - 100.0) < 1e-9,
            "GS-Link example should send DATA_RC at 100 Hz");
        check(
            gs_config->esp32_port.has_value() &&
                gs_config->gs_link_bind_port.has_value(),
            "GS-Link example should define both UDP ports");
        check(
            gs_config->esp32_port == 14550U &&
                gs_config->gs_link_bind_port == 39050U,
            "GS-Link example should use C5 14550 and GCS 39050");
        check(
            gs_config->gs_link_heartbeat_rate_hz == 20.0 &&
                gs_config->gs_link_heartbeat_timeout_ms == 500U,
            "GS-Link example should use resilient heartbeat settings");
        check(
            std::abs(
                gs_config->position_loop.hover_throttle - 0.606) < 1e-9 &&
                std::abs(
                    gs_config->position_loop.z_velocity_damping -
                    0.35) < 1e-9,
            "GS-Link example should use calibrated hover feedforward");
        check(
            std::abs(gs_config->position_loop.x.kp - 1.35) < 1e-9 &&
                std::abs(gs_config->position_loop.x.ki - 0.05) < 1e-9 &&
                std::abs(gs_config->position_loop.x.kd - 0.60) < 1e-9 &&
                std::abs(gs_config->position_loop.y.kp - 1.50) < 1e-9 &&
                std::abs(gs_config->position_loop.y.ki - 0.06) < 1e-9 &&
                std::abs(gs_config->position_loop.y.kd - 0.72) < 1e-9,
            "GS-Link example should use damped horizontal gains");
    }
}

void test_runtime() {
    dock_ground::AppConfig config;
    config.flight_log_enabled = false;
    config.control_rate_hz = 100.0;
    config.simulated_pose_rate_hz = 100.0;
    config.warning_pose_age_ms = 30.0;
    config.max_pose_age_ms = 50.0;
    config.deadman_timeout_ms = 500.0;
    config.pose_source = "simulated";
    config.initial_target_xyz_m = {0.0, 0.0, 1.5};

    dock_ground::Runtime runtime(config);
    runtime.start();
    dock_ground::OperatorTarget target;
    target.position_xyz_m = {0.0, 0.0, 1.5};
    target.deadman = true;
    check(
        runtime.apply_operator_target(target),
        "runtime should accept a finite target");

    std::this_thread::sleep_for(std::chrono::milliseconds(180));
    const auto state = runtime.snapshot();
    check(state.pose.valid, "simulated pose should be available");
    check(
        state.state == dock_ground::ControlState::Track,
        "runtime should track while deadman and pose are valid");
    check(
        state.link.commands_sent > 0,
        "runtime should send loopback four-channel commands");
    check(
        state.metrics.last_period_ms > 5.0 &&
            state.metrics.last_period_ms < 50.0,
        "runtime should run near the configured period");
    check(
        runtime.snapshot_json().find("\"mode\":\"simulation\"") !=
            std::string::npos,
        "snapshot JSON should identify simulation mode");
    std::string json_error;
    const auto state_json =
        dock_ground::json::parse(runtime.snapshot_json(), json_error);
    check(
        state_json != nullptr,
        "runtime snapshot should be valid JSON: " + json_error);
    const auto* tuning =
        state_json ? state_json->find("position_tuning") : nullptr;
    check(
        tuning != nullptr,
        "runtime snapshot should expose live position tuning");
    const auto* position_mode =
        state_json ? state_json->find("position_mode") : nullptr;
    check(
        position_mode != nullptr &&
            position_mode->find("mode") != nullptr &&
            position_mode->find("active") != nullptr &&
            position_mode->find("active_gains") != nullptr,
        "runtime snapshot should expose position regulation mode and gains");
    runtime.stop();
}

void test_runtime_gs_link_snapshot() {
    using namespace dock_ground::gs_link;
    const auto log_path = std::filesystem::temp_directory_path() /
                          ("dock_ground_runtime_log_test_" +
                           std::to_string(::getpid()));
    std::error_code log_filesystem_error;
    std::filesystem::remove_all(log_path, log_filesystem_error);
    const int fake_fc = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    check(
        fake_fc >= 0,
        "runtime fake FC socket should be created: " +
            std::string(std::strerror(errno)));
    if (fake_fc < 0) {
        return;
    }
    sockaddr_in fake_address{};
    fake_address.sin_family = AF_INET;
    fake_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    check(
        ::bind(
            fake_fc, reinterpret_cast<const sockaddr*>(&fake_address),
            sizeof(fake_address)) == 0,
        "runtime fake FC socket should bind");
    socklen_t address_length = sizeof(fake_address);
    (void)::getsockname(
        fake_fc, reinterpret_cast<sockaddr*>(&fake_address),
        &address_length);

    dock_ground::AppConfig config;
    config.flight_log_enabled = true;
    config.flight_log_directory = log_path.string();
    config.pose_source = "simulated";
    config.control_rate_hz = 100.0;
    config.simulated_pose_rate_hz = 100.0;
    config.deadman_timeout_ms = 3000.0;
    config.flight_transport = "udp";
    config.flight_protocol = "gs_link_v1";
    config.gs_link_bind_ip = "127.0.0.1";
    config.gs_link_bind_port = 0;
    config.esp32_ip = "127.0.0.1";
    config.esp32_port =
        static_cast<std::uint16_t>(ntohs(fake_address.sin_port));
    config.gs_link_heartbeat_rate_hz = 20.0;
    config.data_rc_rate_hz = 100.0;
    config.gs_link_telemetry_timeout_ms = 3000;
    config.gs_link_heartbeat_timeout_ms = 3000;

    dock_ground::Runtime runtime(config);
    runtime.start();
    Packet packet;
    sockaddr_in peer{};
    std::string error;
    check(
        wait_for_packet(
            fake_fc, PacketClass::Command,
            static_cast<std::uint8_t>(CommandMessage::Heartbeat),
            packet, peer, 500, error),
        "runtime should send GS-Link heartbeat: " + error);

    Heartbeat heartbeat;
    heartbeat.uptime_ms = 7000U;
    heartbeat.session_id = 0x87654321U;
    heartbeat.status_flags =
        static_cast<std::uint16_t>(StatusFlag::LinkOk);
    heartbeat.state = FlightState::Ready;
    (void)send_packet(
        fake_fc, peer, encode_heartbeat(200U, heartbeat), error);

    FlightTelemetry telemetry;
    telemetry.timestamp_ms = 8000U;
    telemetry.status_flags =
        static_cast<std::uint16_t>(StatusFlag::Armed) |
        static_cast<std::uint16_t>(StatusFlag::GsModeActive) |
        static_cast<std::uint16_t>(StatusFlag::RcValid) |
        static_cast<std::uint16_t>(StatusFlag::RcSwitchValid) |
        static_cast<std::uint16_t>(StatusFlag::RcSwitchSelected) |
        static_cast<std::uint16_t>(StatusFlag::RcChannelsValid) |
        static_cast<std::uint16_t>(StatusFlag::AttitudeValid);
    telemetry.roll_cdeg = 321;
    telemetry.pitch_cdeg = -123;
    telemetry.yaw_cdeg = 4500;
    telemetry.battery_voltage_mv = 16750U;
    telemetry.battery_remaining = 88U;
    telemetry.rc_channel_count = 4;
    telemetry.rc_channels[0] = 12;   // roll
    telemetry.rc_channels[1] = -7;   // pitch
    telemetry.rc_channels[2] = 40;   // throttle
    telemetry.rc_channels[3] = 5;    // yaw
    FlightTelemetry awaiting_gs_mode = telemetry;
    awaiting_gs_mode.status_flags = static_cast<std::uint16_t>(
        awaiting_gs_mode.status_flags &
        ~static_cast<std::uint16_t>(StatusFlag::GsModeActive));
    (void)send_packet(
        fake_fc, peer, encode_telemetry(201U, awaiting_gs_mode), error);

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto state = runtime.snapshot();
        if (state.gs_link.telemetry_fresh &&
            state.gs_link.heartbeat_fresh) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto state = runtime.snapshot();
    const std::string snapshot = runtime.snapshot_json();
    check(
        state.gs_link.telemetry_fresh &&
            state.gs_link.heartbeat_fresh,
        "runtime snapshot should expose fresh FC telemetry");
    const auto logging_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < logging_deadline &&
           !runtime.snapshot().flight_log.recording) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto logging_state = runtime.snapshot();
    check(
        logging_state.flight_log.recording &&
            !logging_state.flight_log.current_file.empty(),
        "first armed telemetry should automatically start flight logging: " +
            logging_state.flight_log.last_error + " / " +
            logging_state.flight_log.current_file);
    check(
        snapshot.find("\"gs_link\":{\"enabled\":true") !=
            std::string::npos &&
            snapshot.find("\"roll_cdeg\":321") != std::string::npos &&
            snapshot.find("\"battery_remaining\":88") !=
                std::string::npos &&
            snapshot.find("\"rc_channels\":[12,-7,40,5]") !=
                std::string::npos,
        "runtime JSON should expose GS-Link telemetry to WebUI");
    std::string snapshot_error;
    const auto snapshot_json =
        dock_ground::json::parse(snapshot, snapshot_error);
    check(
        snapshot_json != nullptr,
        "GS-Link runtime snapshot should be valid JSON: " +
            snapshot_error);

    dock_ground::OperatorTarget target;
    target.position_xyz_m = state.pose.position_xyz_m;
    target.position_xyz_m.x += 0.5;
    target.yaw_rad = 0.0;
    target.deadman = true;
    check(
        runtime.apply_operator_target(target),
        "runtime should accept a position target for DATA_RC output");
    check(
        wait_for_packet(
            fake_fc, PacketClass::Data,
            static_cast<std::uint8_t>(DataMessage::Rc),
            packet, peer, 500, error),
        "runtime position loop should emit DATA_RC: " + error);
    RcCommand initial_passthrough_rc;
    std::uint32_t sender_time = 0;
    check(
        decode_data_rc(
            packet, initial_passthrough_rc, sender_time, error) &&
            initial_passthrough_rc.valid &&
            initial_passthrough_rc.gs_ready &&
            initial_passthrough_rc.roll == 945 &&
            initial_passthrough_rc.pitch == -551 &&
            initial_passthrough_rc.throttle == 3150 &&
            initial_passthrough_rc.yaw == 394,
        "pre-confirmation DATA_RC should pass through the current RC");
    const auto passthrough_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(100);
    while (std::chrono::steady_clock::now() < passthrough_deadline &&
           !runtime.snapshot().rc_passthrough_active) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const auto passthrough_state = runtime.snapshot();
    check(
        passthrough_state.command_valid &&
            passthrough_state.rc_passthrough_active &&
            !passthrough_state.rc_handoff_active &&
            !passthrough_state.rc_handoff_hold,
        "RC passthrough must not start the handoff before FC confirmation");

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    FlightTelemetry moved_rc = awaiting_gs_mode;
    moved_rc.timestamp_ms += 20U;
    moved_rc.rc_channels[0] = 20;
    moved_rc.rc_channels[1] = -10;
    moved_rc.rc_channels[2] = 20;
    moved_rc.rc_channels[3] = -5;
    (void)send_packet(
        fake_fc, peer, encode_telemetry(202U, moved_rc), error);
    const auto moved_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < moved_deadline &&
           std::abs(
               runtime.snapshot().rc_handoff_base.throttle -
               20.0 / 127.0) > 1e-12) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    RcCommand moved_passthrough_rc;
    bool moved_passthrough_seen = false;
    for (int attempt = 0; attempt < 100 && !moved_passthrough_seen;
         ++attempt) {
        if (!wait_for_packet(
                fake_fc, PacketClass::Data,
                static_cast<std::uint8_t>(DataMessage::Rc),
                packet, peer, 10, error)) {
            break;
        }
        moved_passthrough_seen =
            decode_data_rc(
                packet, moved_passthrough_rc, sender_time, error) &&
            moved_passthrough_rc.roll == 1575 &&
            moved_passthrough_rc.pitch == -787 &&
            moved_passthrough_rc.throttle == 1575 &&
            moved_passthrough_rc.yaw == -394;
    }
    const auto unconfirmed_state = runtime.snapshot();
    check(
        moved_passthrough_seen &&
            unconfirmed_state.rc_passthrough_active &&
            !unconfirmed_state.rc_handoff_active,
        "ground station must pass through RC movement until FC confirms GS");
    FlightTelemetry confirmed_telemetry = moved_rc;
    confirmed_telemetry.timestamp_ms += 20U;
    confirmed_telemetry.status_flags |=
        static_cast<std::uint16_t>(StatusFlag::GsModeActive);
    (void)send_packet(
        fake_fc, peer, encode_telemetry(203U, confirmed_telemetry), error);
    const auto confirmation_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < confirmation_deadline &&
           !(runtime.snapshot().rc_handoff_active &&
             runtime.snapshot().rc_handoff_hold)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const auto handoff_state = runtime.snapshot();
    check(
        handoff_state.rc_handoff_active &&
            handoff_state.rc_handoff_hold &&
            !handoff_state.rc_passthrough_active &&
            std::abs(
                handoff_state.rc_handoff_base.roll -
                20.0 / 127.0) < 1e-12 &&
            std::abs(
                handoff_state.command.manual.throttle -
                handoff_state.rc_handoff_base.throttle) < 1e-12,
        "confirmation must recapture and then hold the latest RC baseline");
    std::string handoff_json_error;
    const auto handoff_json =
        dock_ground::json::parse(
            runtime.snapshot_json(), handoff_json_error);
    const auto* control_output =
        handoff_json ? handoff_json->find("control_output") : nullptr;
    const auto* highlighted_source =
        control_output ? control_output->find("source") : nullptr;
    const auto* highlighted_active =
        control_output
            ? control_output->find("ground_station_active")
            : nullptr;
    check(
        handoff_json != nullptr && highlighted_source != nullptr &&
            highlighted_source->string == "GROUND_STATION" &&
            highlighted_active != nullptr &&
            highlighted_active->boolean,
        "runtime JSON should prominently identify the ground control source");

    RcCommand after_hold_rc;
    bool post_hold_packet_seen = false;
    const auto transition_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(800);
    while (std::chrono::steady_clock::now() < transition_deadline &&
           !post_hold_packet_seen) {
        if (wait_for_packet(
                fake_fc, PacketClass::Data,
                static_cast<std::uint8_t>(DataMessage::Rc),
                packet, peer, 50, error) &&
            decode_data_rc(
                packet, after_hold_rc, sender_time, error)) {
            const auto transition_state = runtime.snapshot();
            post_hold_packet_seen =
                !transition_state.rc_handoff_hold &&
                transition_state.rc_handoff_throttle_blend >= 0.999;
        }
    }
    const auto post_hold_state = runtime.snapshot();
    check(
        post_hold_packet_seen && post_hold_state.command_valid &&
            !post_hold_state.rc_handoff_hold &&
            std::isfinite(
                post_hold_state.handoff_vertical_speed_mps) &&
            std::abs(
                after_hold_rc.throttle -
                (2.0 * config.position_loop.hover_throttle - 1.0) *
                    10000.0) <= 1500.0,
        "height control should transition from RC to hover feedforward");
    check(
        std::abs(post_hold_state.link.target_rate_hz - 100.0) < 1e-9 &&
            post_hold_state.link.actual_rate_hz > 50.0 &&
            post_hold_state.link.last_send_interval_ms < 30.0,
        "DATA_RC deadline scheduler should report the effective 100 Hz rate");

    FlightTelemetry fc_still_rc = telemetry;
    fc_still_rc.timestamp_ms += 100U;
    fc_still_rc.status_flags = static_cast<std::uint16_t>(
        fc_still_rc.status_flags &
        ~static_cast<std::uint16_t>(StatusFlag::GsModeActive));
    (void)send_packet(
        fake_fc, peer, encode_telemetry(204U, fc_still_rc), error);
    const auto fc_still_rc_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < fc_still_rc_deadline &&
           status_flag_set(
               runtime.snapshot().gs_link.telemetry.status_flags,
               StatusFlag::GsModeActive)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::string fc_still_rc_error;
    const auto fc_still_rc_json =
        dock_ground::json::parse(runtime.snapshot_json(), fc_still_rc_error);
    const auto* fc_still_rc_control =
        fc_still_rc_json ? fc_still_rc_json->find("control_output") : nullptr;
    const auto* fc_still_rc_source =
        fc_still_rc_control ? fc_still_rc_control->find("source") : nullptr;
    const auto* fc_still_rc_active =
        fc_still_rc_control
            ? fc_still_rc_control->find("ground_station_active")
            : nullptr;
    const auto* fc_still_rc_commanding =
        fc_still_rc_control
            ? fc_still_rc_control->find("ground_station_commanding")
            : nullptr;
    const auto* fc_still_rc_passthrough =
        fc_still_rc_control
            ? fc_still_rc_control->find("rc_passthrough")
            : nullptr;
    check(
        fc_still_rc_json != nullptr && fc_still_rc_source != nullptr &&
            fc_still_rc_source->string == "RC" &&
            fc_still_rc_active != nullptr &&
            !fc_still_rc_active->boolean &&
            fc_still_rc_commanding != nullptr &&
            fc_still_rc_commanding->boolean &&
            fc_still_rc_passthrough != nullptr &&
            fc_still_rc_passthrough->boolean,
        "WebUI must report RC until the FC confirms GS_MODE_ACTIVE: " +
            fc_still_rc_error);

    FlightTelemetry switched_away = telemetry;
    switched_away.status_flags = static_cast<std::uint16_t>(
        switched_away.status_flags &
        ~static_cast<std::uint16_t>(StatusFlag::RcSwitchSelected));
    (void)send_packet(
        fake_fc, peer, encode_telemetry(205U, switched_away), error);
    const auto leave_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < leave_deadline &&
           runtime.snapshot().rc_handoff_active) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    check(
        !runtime.snapshot().rc_handoff_active,
        "leaving RC station endpoint should clear the RC handoff baseline");

    FlightTelemetry reentered = telemetry;
    reentered.timestamp_ms += 200U;
    reentered.rc_channels[0] = -20;
    reentered.rc_channels[1] = 15;
    reentered.rc_channels[2] = 10;
    reentered.rc_channels[3] = -6;
    (void)send_packet(
        fake_fc, peer, encode_telemetry(206U, reentered), error);
    bool reentry_base_seen = false;
    for (int attempt = 0; attempt < 20 && !reentry_base_seen;
         ++attempt) {
        if (!wait_for_packet(
                fake_fc, PacketClass::Data,
                static_cast<std::uint8_t>(DataMessage::Rc),
                packet, peer, 100, error)) {
            break;
        }
        if (decode_data_rc(
                packet, after_hold_rc, sender_time, error) &&
            after_hold_rc.roll == -1575 &&
            after_hold_rc.pitch == 1181 &&
            after_hold_rc.throttle == 787 &&
            after_hold_rc.yaw == -472) {
            reentry_base_seen = true;
        }
    }
    check(
        reentry_base_seen,
        "re-entering ground mode should recapture the new RC baseline");
    runtime.stop();
    bool runtime_csv_found = false;
    bool runtime_summary_found = false;
    for (const auto& entry :
         std::filesystem::directory_iterator(log_path)) {
        runtime_csv_found =
            runtime_csv_found ||
            entry.path().extension() == ".csv";
        runtime_summary_found =
            runtime_summary_found ||
            entry.path().filename().string().find(".summary.json") !=
                std::string::npos;
    }
    check(
        runtime_csv_found && runtime_summary_found,
        "Runtime should automatically close ARM session CSV with a summary");
    std::filesystem::remove_all(log_path, log_filesystem_error);
    (void)::close(fake_fc);
}

void test_position_tuning_websocket() {
    dock_ground::AppConfig config;
    config.flight_log_enabled = false;
    config.pose_source = "simulated";
    config.control_rate_hz = 100.0;
    config.simulated_pose_rate_hz = 100.0;

    dock_ground::Runtime runtime(config);
    runtime.start();
    dock_ground::WebServer server(
        dock_ground::WebServerOptions{
            .bind_address = "127.0.0.1",
            .port = 0,
            .web_root = std::string(DOCK_GROUND_SOURCE_DIR) +
                        "/frontend/dist",
        },
        [&runtime](const dock_ground::OperatorTarget& target) {
            return runtime.apply_operator_target(target);
        },
        [&runtime](
            const std::string& aircraft_id,
            const std::string& body_id, int sensor_id,
            std::string& error) {
            return runtime.bind_body(
                aircraft_id, body_id, sensor_id, error);
        },
        [](const dock_ground::gs_link::UplinkRequest&,
           std::string& error) {
            error = "loopback test has no GS-Link uplink";
            return false;
        },
        [&runtime](
            const dock_ground::PositionLoopTuning& tuning,
            std::string& error) {
            return runtime.apply_position_tuning(tuning, error);
        },
        [&runtime] { return runtime.snapshot_json(); });

    std::string error;
    check(
        server.start(error),
        "position tuning WebSocket server should start: " + error);

    const int client = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(server.bound_port());
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    check(
        client >= 0 &&
            ::connect(
                client, reinterpret_cast<const sockaddr*>(&address),
                sizeof(address)) == 0,
        "position tuning WebSocket client should connect");

    const std::string handshake =
        "GET /ws/v1/realtime HTTP/1.1\r\n"
        "Host: 127.0.0.1\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n";
    (void)::send(
        client, handshake.data(), handshake.size(), MSG_NOSIGNAL);
    std::array<char, 4096> buffer{};
    const ssize_t handshake_bytes =
        ::recv(client, buffer.data(), buffer.size(), 0);
    check(
        handshake_bytes > 0 &&
            std::string_view(buffer.data(), handshake_bytes)
                    .find("101 Switching Protocols") !=
                std::string_view::npos,
        "position tuning WebSocket handshake should succeed");

    std::ostringstream tuning_payload;
    tuning_payload
        << std::setprecision(10)
        << "{\"v\":1,\"type\":\"position_tuning\",\"tuning\":{"
        << "\"x\":{\"kp\":2.75,\"ki\":0.12,\"kd\":0.31},"
        << "\"y\":{\"kp\":1.25,\"ki\":0.09,\"kd\":0.22},"
        << "\"z\":{\"kp\":0.11,\"ki\":0.21,\"kd\":0.09},"
        << "\"approach\":{"
        << "\"x\":{\"kp\":0.81,\"ki\":0.02,\"kd\":1.11},"
        << "\"y\":{\"kp\":0.82,\"ki\":0.02,\"kd\":1.12},"
        << "\"z\":{\"kp\":0.13,\"ki\":0.006,\"kd\":0.15}},"
        << "\"precision_hold\":{"
        << "\"x\":{\"kp\":0.31,\"ki\":0.006,\"kd\":0.91},"
        << "\"y\":{\"kp\":0.32,\"ki\":0.006,\"kd\":0.92},"
        << "\"z\":{\"kp\":0.09,\"ki\":0.004,\"kd\":0.13}},"
        << "\"mode_scheduling\":{"
        << "\"enabled\":true,"
        << "\"brake_enter_distance_m\":0.54,"
        << "\"brake_exit_distance_m\":0.74,"
        << "\"brake_enter_vertical_error_m\":0.29,"
        << "\"brake_exit_vertical_error_m\":0.39,"
        << "\"hold_enter_distance_m\":0.11,"
        << "\"hold_exit_distance_m\":0.21,"
        << "\"hold_enter_vertical_error_m\":0.05,"
        << "\"hold_exit_vertical_error_m\":0.11,"
        << "\"hold_enter_speed_mps\":0.07,"
        << "\"hold_exit_speed_mps\":0.14,"
        << "\"hold_enter_vertical_speed_mps\":0.04,"
        << "\"hold_exit_vertical_speed_mps\":0.09,"
        << "\"target_motion_speed_mps\":0.02,"
        << "\"hold_enter_dwell_s\":0.45,"
        << "\"hold_exit_dwell_s\":0.18,"
        << "\"gain_transition_time_s\":0.55},"
        << "\"max_accel_mps2\":3.5,"
        << "\"max_tilt_rad\":0.3141592654,"
        << "\"min_throttle\":0.2,"
        << "\"hover_throttle\":0.56,"
        << "\"max_throttle\":0.96,"
        << "\"z_velocity_damping\":0.08,"
        << "\"max_yaw_rate_rad_s\":0.5,"
        << "\"yaw_rate_kp\":1.1,"
        << "\"yaw_rate_kd\":0.65,"
        << "\"yaw_rate_slew_rate_rad_s2\":2.0,"
        << "\"fc_yaw_rate_scale_rad_s\":1.5,"
        << "\"max_position_error_m\":10,"
        << "\"gravity_mps2\":9.80665}}";
    const auto frame = masked_text_frame(tuning_payload.str());
    (void)::send(
        client, frame.data(), frame.size(), MSG_NOSIGNAL);

    std::string received;
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(800);
    while (std::chrono::steady_clock::now() < deadline &&
           received.find("position_tuning_ack") == std::string::npos) {
        pollfd descriptor{client, POLLIN, 0};
        if (::poll(&descriptor, 1, 40) > 0) {
            const ssize_t bytes =
                ::recv(client, buffer.data(), buffer.size(), 0);
            if (bytes <= 0) {
                break;
            }
            received.append(buffer.data(), static_cast<std::size_t>(bytes));
        }
    }
    check(
        received.find("position_tuning_ack") != std::string::npos,
        "position tuning should return a WebSocket acknowledgement");
    check(
        std::abs(runtime.position_tuning().x.kp - 2.75) < 1e-9 &&
            std::abs(runtime.position_tuning().y.ki - 0.09) < 1e-9 &&
            std::abs(
                runtime.position_tuning().z_velocity_damping -
                0.08) < 1e-9 &&
            std::abs(runtime.position_tuning().yaw_rate_kd - 0.65) <
                1e-9 &&
            std::abs(
                runtime.position_tuning().yaw_rate_slew_rate_rad_s2 -
                2.0) < 1e-9,
        "WebSocket tuning should hot-update independent PID axes");
    check(
        std::abs(
            runtime.position_tuning().approach.x.kd -
            1.11) < 1e-9 &&
            std::abs(
                runtime.position_tuning().precision_hold.y.kp -
                0.32) < 1e-9 &&
            std::abs(
                runtime.position_tuning().mode_scheduling
                    .hold_enter_distance_m -
                0.11) < 1e-9,
        "WebSocket tuning should preserve editable mode profiles");

    (void)::close(client);
    server.stop();
    runtime.stop();
}

void test_initial_target_from_pose() {
    dock_ground::AppConfig config;
    config.flight_log_enabled = false;
    config.control_rate_hz = 100.0;
    config.simulated_pose_rate_hz = 100.0;
    config.pose_source = "simulated";
    config.initial_target_use_current_pose = true;
    config.initial_target_xyz_m = {9.0, 9.0, 9.0};
    config.initial_yaw_rad = 1.2;
    config.deadman_timeout_ms = 500.0;

    dock_ground::Runtime runtime(config);
    runtime.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(180));
    const auto state = runtime.snapshot();
    check(
        std::abs(state.target.position_xyz_m.x) < 0.4,
        "initial target X should follow the first simulated pose");
    check(
        std::abs(state.target.position_xyz_m.y) < 0.5,
        "initial target Y should follow the first simulated pose");
    check(
        state.target.position_xyz_m.z > 1.0 &&
            state.target.position_xyz_m.z < 2.0,
        "initial target Z should follow the first simulated pose");
    runtime.stop();
}

void test_target_position_slew_limit() {
    dock_ground::AppConfig config;
    config.flight_log_enabled = false;
    config.control_rate_hz = 100.0;
    config.simulated_pose_rate_hz = 100.0;
    config.pose_source = "simulated";
    config.initial_target_use_current_pose = true;
    config.deadman_timeout_ms = 2000.0;
    config.target_slew_rate_mps = 0.50;

    dock_ground::Runtime runtime(config);
    runtime.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    const auto before = runtime.snapshot();

    dock_ground::OperatorTarget target;
    target.position_xyz_m = before.target.position_xyz_m;
    target.position_xyz_m.y += 0.25;
    target.yaw_rad = before.target.yaw_rad;
    target.deadman = true;
    check(
        runtime.apply_operator_target(target),
        "runtime should accept a target for slew limiting");

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const auto ramping = runtime.snapshot();
    const double partial_step =
        std::abs(ramping.target.position_xyz_m.y -
                 before.target.position_xyz_m.y);
    check(
        partial_step > 0.01 && partial_step < 0.20,
        "target update should ramp instead of jumping in one control cycle");

    std::this_thread::sleep_for(std::chrono::milliseconds(550));
    const auto completed = runtime.snapshot();
    check(
        std::abs(completed.target.position_xyz_m.y -
                 target.position_xyz_m.y) < 1e-6,
        "target should reach the requested point after the slew interval");
    runtime.stop();
}

}  // namespace

int main() {
    test_json();
    test_flight_logger_arm_session();
    test_gs_link_codec();
    test_gs_link_udp();
    test_websocket_protocol();
    test_controller();
    test_position_mode_scheduling();
    test_config();
    test_runtime();
    test_runtime_gs_link_snapshot();
    test_position_tuning_websocket();
    test_initial_target_from_pose();
    test_target_position_slew_limit();

    if (g_failures != 0) {
        std::cerr << g_failures << " test(s) failed\n";
        return 1;
    }
    std::cout << "all dock_ground tests passed\n";
    return 0;
}
