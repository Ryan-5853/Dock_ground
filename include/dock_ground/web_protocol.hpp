#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

namespace dock_ground::web {

[[nodiscard]] std::string base64_encode(
    std::span<const std::uint8_t> input);

[[nodiscard]] std::array<std::uint8_t, 20> sha1(
    std::span<const std::uint8_t> input);

[[nodiscard]] std::string websocket_accept(std::string_view client_key);

[[nodiscard]] std::string encode_websocket_frame(
    std::string_view payload, std::uint8_t opcode = 0x1U);

enum class FrameDecodeStatus {
    Complete,
    Incomplete,
    Error,
};

struct DecodedFrame {
    FrameDecodeStatus status{FrameDecodeStatus::Incomplete};
    std::uint8_t opcode{0};
    bool final{false};
    std::string payload;
    std::size_t consumed_bytes{0};
    std::string error;
};

[[nodiscard]] DecodedFrame decode_client_frame(std::string_view input);

}  // namespace dock_ground::web
