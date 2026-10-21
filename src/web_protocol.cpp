#include "dock_ground/web_protocol.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <vector>

namespace dock_ground::web {
namespace {

constexpr std::size_t kMaximumFramePayload = 1024U * 1024U;

std::uint32_t rotate_left(std::uint32_t value, unsigned int bits) {
    return (value << bits) | (value >> (32U - bits));
}

}  // namespace

std::string base64_encode(std::span<const std::uint8_t> input) {
    static constexpr char kAlphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string output;
    output.reserve(((input.size() + 2U) / 3U) * 4U);

    std::size_t index = 0;
    while (index + 3U <= input.size()) {
        const std::uint32_t block =
            (static_cast<std::uint32_t>(input[index]) << 16U) |
            (static_cast<std::uint32_t>(input[index + 1U]) << 8U) |
            static_cast<std::uint32_t>(input[index + 2U]);
        output.push_back(kAlphabet[(block >> 18U) & 0x3FU]);
        output.push_back(kAlphabet[(block >> 12U) & 0x3FU]);
        output.push_back(kAlphabet[(block >> 6U) & 0x3FU]);
        output.push_back(kAlphabet[block & 0x3FU]);
        index += 3U;
    }

    const std::size_t remaining = input.size() - index;
    if (remaining == 1U) {
        const std::uint32_t block =
            static_cast<std::uint32_t>(input[index]) << 16U;
        output.push_back(kAlphabet[(block >> 18U) & 0x3FU]);
        output.push_back(kAlphabet[(block >> 12U) & 0x3FU]);
        output.push_back('=');
        output.push_back('=');
    } else if (remaining == 2U) {
        const std::uint32_t block =
            (static_cast<std::uint32_t>(input[index]) << 16U) |
            (static_cast<std::uint32_t>(input[index + 1U]) << 8U);
        output.push_back(kAlphabet[(block >> 18U) & 0x3FU]);
        output.push_back(kAlphabet[(block >> 12U) & 0x3FU]);
        output.push_back(kAlphabet[(block >> 6U) & 0x3FU]);
        output.push_back('=');
    }
    return output;
}

std::array<std::uint8_t, 20> sha1(
    std::span<const std::uint8_t> input) {
    std::vector<std::uint8_t> message(input.begin(), input.end());
    const std::uint64_t bit_length =
        static_cast<std::uint64_t>(message.size()) * 8U;
    message.push_back(0x80U);
    while ((message.size() % 64U) != 56U) {
        message.push_back(0U);
    }
    for (int shift = 56; shift >= 0; shift -= 8) {
        message.push_back(static_cast<std::uint8_t>(
            (bit_length >> static_cast<unsigned int>(shift)) & 0xFFU));
    }

    std::uint32_t h0 = 0x67452301U;
    std::uint32_t h1 = 0xEFCDAB89U;
    std::uint32_t h2 = 0x98BADCFEU;
    std::uint32_t h3 = 0x10325476U;
    std::uint32_t h4 = 0xC3D2E1F0U;

    for (std::size_t chunk = 0; chunk < message.size(); chunk += 64U) {
        std::array<std::uint32_t, 80> words{};
        for (std::size_t index = 0; index < 16U; ++index) {
            const std::size_t offset = chunk + index * 4U;
            words[index] =
                (static_cast<std::uint32_t>(message[offset]) << 24U) |
                (static_cast<std::uint32_t>(message[offset + 1U]) << 16U) |
                (static_cast<std::uint32_t>(message[offset + 2U]) << 8U) |
                static_cast<std::uint32_t>(message[offset + 3U]);
        }
        for (std::size_t index = 16U; index < 80U; ++index) {
            words[index] = rotate_left(
                words[index - 3U] ^ words[index - 8U] ^
                    words[index - 14U] ^ words[index - 16U],
                1U);
        }

        std::uint32_t a = h0;
        std::uint32_t b = h1;
        std::uint32_t c = h2;
        std::uint32_t d = h3;
        std::uint32_t e = h4;

        for (std::size_t index = 0; index < 80U; ++index) {
            std::uint32_t function = 0;
            std::uint32_t constant = 0;
            if (index < 20U) {
                function = (b & c) | ((~b) & d);
                constant = 0x5A827999U;
            } else if (index < 40U) {
                function = b ^ c ^ d;
                constant = 0x6ED9EBA1U;
            } else if (index < 60U) {
                function = (b & c) | (b & d) | (c & d);
                constant = 0x8F1BBCDCU;
            } else {
                function = b ^ c ^ d;
                constant = 0xCA62C1D6U;
            }
            const std::uint32_t temporary =
                rotate_left(a, 5U) + function + e + constant + words[index];
            e = d;
            d = c;
            c = rotate_left(b, 30U);
            b = a;
            a = temporary;
        }

        h0 += a;
        h1 += b;
        h2 += c;
        h3 += d;
        h4 += e;
    }

    std::array<std::uint8_t, 20> digest{};
    const std::array<std::uint32_t, 5> words{h0, h1, h2, h3, h4};
    for (std::size_t word = 0; word < words.size(); ++word) {
        for (std::size_t byte = 0; byte < 4U; ++byte) {
            digest[word * 4U + byte] = static_cast<std::uint8_t>(
                (words[word] >>
                 static_cast<unsigned int>((3U - byte) * 8U)) &
                0xFFU);
        }
    }
    return digest;
}

std::string websocket_accept(std::string_view client_key) {
    constexpr std::string_view kGuid =
        "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    std::string source(client_key);
    source.append(kGuid);
    const auto digest = sha1(std::span<const std::uint8_t>(
        reinterpret_cast<const std::uint8_t*>(source.data()),
        source.size()));
    return base64_encode(digest);
}

std::string encode_websocket_frame(
    std::string_view payload, std::uint8_t opcode) {
    std::string frame;
    frame.reserve(payload.size() + 10U);
    frame.push_back(static_cast<char>(0x80U | (opcode & 0x0FU)));
    if (payload.size() < 126U) {
        frame.push_back(static_cast<char>(payload.size()));
    } else if (payload.size() <= 0xFFFFU) {
        frame.push_back(static_cast<char>(126U));
        frame.push_back(
            static_cast<char>((payload.size() >> 8U) & 0xFFU));
        frame.push_back(static_cast<char>(payload.size() & 0xFFU));
    } else {
        frame.push_back(static_cast<char>(127U));
        const std::uint64_t length = payload.size();
        for (int shift = 56; shift >= 0; shift -= 8) {
            frame.push_back(static_cast<char>(
                (length >> static_cast<unsigned int>(shift)) & 0xFFU));
        }
    }
    frame.append(payload);
    return frame;
}

DecodedFrame decode_client_frame(std::string_view input) {
    DecodedFrame result;
    if (input.size() < 2U) {
        result.status = FrameDecodeStatus::Incomplete;
        return result;
    }

    const auto first =
        static_cast<std::uint8_t>(input[0]);
    const auto second =
        static_cast<std::uint8_t>(input[1]);
    result.final = (first & 0x80U) != 0U;
    result.opcode = first & 0x0FU;
    const bool masked = (second & 0x80U) != 0U;
    std::uint64_t payload_length = second & 0x7FU;
    std::size_t header_length = 2U;

    if (!masked) {
        result.status = FrameDecodeStatus::Error;
        result.error = "client WebSocket frames must be masked";
        return result;
    }
    if (payload_length == 126U) {
        if (input.size() < 4U) {
            return result;
        }
        payload_length =
            (static_cast<std::uint64_t>(
                 static_cast<std::uint8_t>(input[2]))
             << 8U) |
            static_cast<std::uint8_t>(input[3]);
        header_length = 4U;
    } else if (payload_length == 127U) {
        if (input.size() < 10U) {
            return result;
        }
        payload_length = 0;
        for (std::size_t index = 2U; index < 10U; ++index) {
            payload_length =
                (payload_length << 8U) |
                static_cast<std::uint8_t>(input[index]);
        }
        header_length = 10U;
    }

    if (payload_length > kMaximumFramePayload) {
        result.status = FrameDecodeStatus::Error;
        result.error = "WebSocket frame is too large";
        return result;
    }
    if (input.size() < header_length + 4U) {
        return result;
    }
    const std::size_t total_length =
        header_length + 4U + static_cast<std::size_t>(payload_length);
    if (input.size() < total_length) {
        return result;
    }

    const std::array<std::uint8_t, 4> mask{
        static_cast<std::uint8_t>(input[header_length]),
        static_cast<std::uint8_t>(input[header_length + 1U]),
        static_cast<std::uint8_t>(input[header_length + 2U]),
        static_cast<std::uint8_t>(input[header_length + 3U]),
    };
    result.payload.resize(static_cast<std::size_t>(payload_length));
    for (std::size_t index = 0; index < result.payload.size(); ++index) {
        result.payload[index] = static_cast<char>(
            static_cast<std::uint8_t>(
                input[header_length + 4U + index]) ^
            mask[index % mask.size()]);
    }
    result.status = FrameDecodeStatus::Complete;
    result.consumed_bytes = total_length;
    return result;
}

}  // namespace dock_ground::web
