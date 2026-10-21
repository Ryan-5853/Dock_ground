#include "dock_ground/json.hpp"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <sstream>

namespace dock_ground::json {

const Value* Value::find(std::string_view key) const {
    if (type != Type::Object) {
        return nullptr;
    }
    for (const auto& [entry_key, entry_value] : object) {
        if (entry_key == key) {
            return entry_value.get();
        }
    }
    return nullptr;
}

namespace {

constexpr std::size_t kMaxDepth = 32;
constexpr std::size_t kMaxStringBytes = 64 * 1024;

class Parser {
public:
    explicit Parser(std::string_view input) : input_(input) {}

    std::unique_ptr<Value> parse_document(std::string& error) {
        skip_whitespace();
        auto value = parse_value(error, 0);
        if (!value) {
            return nullptr;
        }
        skip_whitespace();
        if (position_ != input_.size()) {
            fail(error, "trailing data after JSON value");
            return nullptr;
        }
        return value;
    }

private:
    [[nodiscard]] bool at_end() const {
        return position_ >= input_.size();
    }

    [[nodiscard]] char peek() const {
        return at_end() ? '\0' : input_[position_];
    }

    void skip_whitespace() {
        while (!at_end()) {
            const char value = input_[position_];
            if (value != ' ' && value != '\n' && value != '\r' && value != '\t') {
                break;
            }
            ++position_;
        }
    }

    void fail(std::string& error, std::string_view message) const {
        std::ostringstream stream;
        stream << message << " at byte " << position_;
        error = stream.str();
    }

    bool consume(char expected) {
        if (peek() != expected) {
            return false;
        }
        ++position_;
        return true;
    }

    std::unique_ptr<Value> parse_value(std::string& error, std::size_t depth) {
        if (depth > kMaxDepth) {
            fail(error, "JSON nesting is too deep");
            return nullptr;
        }
        if (at_end()) {
            fail(error, "unexpected end of JSON");
            return nullptr;
        }

        switch (peek()) {
            case '{':
                return parse_object(error, depth);
            case '[':
                return parse_array(error, depth);
            case '"': {
                auto result = std::make_unique<Value>();
                result->type = Value::Type::String;
                if (!parse_string(result->string, error)) {
                    return nullptr;
                }
                return result;
            }
            case 't':
                return parse_literal("true", true, 1.0, error);
            case 'f':
                return parse_literal("false", false, 0.0, error);
            case 'n':
                return parse_literal("null", false, 0.0, error);
            default:
                return parse_number(error);
        }
    }

    std::unique_ptr<Value> parse_object(std::string& error, std::size_t depth) {
        consume('{');
        auto result = std::make_unique<Value>();
        result->type = Value::Type::Object;
        skip_whitespace();
        if (consume('}')) {
            return result;
        }

        while (true) {
            skip_whitespace();
            if (peek() != '"') {
                fail(error, "expected object key");
                return nullptr;
            }
            std::string key;
            if (!parse_string(key, error)) {
                return nullptr;
            }
            skip_whitespace();
            if (!consume(':')) {
                fail(error, "expected ':' after object key");
                return nullptr;
            }
            skip_whitespace();
            auto value = parse_value(error, depth + 1);
            if (!value) {
                return nullptr;
            }
            for (const auto& [existing_key, unused] : result->object) {
                (void)unused;
                if (existing_key == key) {
                    fail(error, "duplicate object key");
                    return nullptr;
                }
            }
            result->object.emplace_back(std::move(key), std::move(value));
            skip_whitespace();
            if (consume('}')) {
                return result;
            }
            if (!consume(',')) {
                fail(error, "expected ',' or '}' in object");
                return nullptr;
            }
        }
    }

    std::unique_ptr<Value> parse_array(std::string& error, std::size_t depth) {
        consume('[');
        auto result = std::make_unique<Value>();
        result->type = Value::Type::Array;
        skip_whitespace();
        if (consume(']')) {
            return result;
        }

        while (true) {
            skip_whitespace();
            auto value = parse_value(error, depth + 1);
            if (!value) {
                return nullptr;
            }
            result->array.push_back(std::move(value));
            skip_whitespace();
            if (consume(']')) {
                return result;
            }
            if (!consume(',')) {
                fail(error, "expected ',' or ']' in array");
                return nullptr;
            }
        }
    }

    static void append_utf8(std::string& output, std::uint32_t code_point) {
        if (code_point <= 0x7FU) {
            output.push_back(static_cast<char>(code_point));
        } else if (code_point <= 0x7FFU) {
            output.push_back(static_cast<char>(0xC0U | (code_point >> 6U)));
            output.push_back(static_cast<char>(0x80U | (code_point & 0x3FU)));
        } else if (code_point <= 0xFFFFU) {
            output.push_back(static_cast<char>(0xE0U | (code_point >> 12U)));
            output.push_back(
                static_cast<char>(0x80U | ((code_point >> 6U) & 0x3FU)));
            output.push_back(static_cast<char>(0x80U | (code_point & 0x3FU)));
        } else {
            output.push_back(static_cast<char>(0xF0U | (code_point >> 18U)));
            output.push_back(
                static_cast<char>(0x80U | ((code_point >> 12U) & 0x3FU)));
            output.push_back(
                static_cast<char>(0x80U | ((code_point >> 6U) & 0x3FU)));
            output.push_back(static_cast<char>(0x80U | (code_point & 0x3FU)));
        }
    }

    [[nodiscard]] static int hex_value(char value) {
        if (value >= '0' && value <= '9') {
            return value - '0';
        }
        if (value >= 'a' && value <= 'f') {
            return 10 + value - 'a';
        }
        if (value >= 'A' && value <= 'F') {
            return 10 + value - 'A';
        }
        return -1;
    }

    bool parse_unicode_escape(std::uint32_t& code_point, std::string& error) {
        if (position_ + 4 > input_.size()) {
            fail(error, "truncated unicode escape");
            return false;
        }
        code_point = 0;
        for (int index = 0; index < 4; ++index) {
            const int digit = hex_value(input_[position_++]);
            if (digit < 0) {
                fail(error, "invalid unicode escape");
                return false;
            }
            code_point = (code_point << 4U) | static_cast<std::uint32_t>(digit);
        }
        return true;
    }

    bool parse_string(std::string& output, std::string& error) {
        if (!consume('"')) {
            fail(error, "expected string");
            return false;
        }

        output.clear();
        while (!at_end()) {
            const char value = input_[position_++];
            if (value == '"') {
                return true;
            }
            if (static_cast<unsigned char>(value) < 0x20U) {
                fail(error, "unescaped control character in string");
                return false;
            }
            if (value != '\\') {
                output.push_back(value);
                if (output.size() > kMaxStringBytes) {
                    fail(error, "string is too large");
                    return false;
                }
                continue;
            }

            if (at_end()) {
                fail(error, "truncated string escape");
                return false;
            }
            const char escaped = input_[position_++];
            switch (escaped) {
                case '"':
                    output.push_back('"');
                    break;
                case '\\':
                    output.push_back('\\');
                    break;
                case '/':
                    output.push_back('/');
                    break;
                case 'b':
                    output.push_back('\b');
                    break;
                case 'f':
                    output.push_back('\f');
                    break;
                case 'n':
                    output.push_back('\n');
                    break;
                case 'r':
                    output.push_back('\r');
                    break;
                case 't':
                    output.push_back('\t');
                    break;
                case 'u': {
                    std::uint32_t code_point = 0;
                    if (!parse_unicode_escape(code_point, error)) {
                        return false;
                    }
                    if (code_point >= 0xD800U && code_point <= 0xDBFFU) {
                        if (position_ + 2 > input_.size() ||
                            input_[position_] != '\\' ||
                            input_[position_ + 1] != 'u') {
                            fail(error, "missing low surrogate");
                            return false;
                        }
                        position_ += 2;
                        std::uint32_t low = 0;
                        if (!parse_unicode_escape(low, error)) {
                            return false;
                        }
                        if (low < 0xDC00U || low > 0xDFFFU) {
                            fail(error, "invalid low surrogate");
                            return false;
                        }
                        code_point =
                            0x10000U + ((code_point - 0xD800U) << 10U) +
                            (low - 0xDC00U);
                    } else if (code_point >= 0xDC00U &&
                               code_point <= 0xDFFFU) {
                        fail(error, "unexpected low surrogate");
                        return false;
                    }
                    append_utf8(output, code_point);
                    break;
                }
                default:
                    fail(error, "invalid string escape");
                    return false;
            }
        }
        fail(error, "unterminated string");
        return false;
    }

    std::unique_ptr<Value> parse_literal(
        std::string_view literal, bool is_boolean, double number,
        std::string& error) {
        if (input_.substr(position_, literal.size()) != literal) {
            fail(error, "invalid JSON literal");
            return nullptr;
        }
        position_ += literal.size();
        auto result = std::make_unique<Value>();
        if (literal == "null") {
            result->type = Value::Type::Null;
        } else {
            result->type = Value::Type::Boolean;
            result->boolean = is_boolean;
            result->number = number;
        }
        return result;
    }

    std::unique_ptr<Value> parse_number(std::string& error) {
        const std::size_t start = position_;
        if (peek() == '-') {
            ++position_;
        }
        if (at_end() || peek() < '0' || peek() > '9') {
            fail(error, "invalid number");
            return nullptr;
        }
        if (peek() == '0') {
            ++position_;
        } else {
            while (!at_end() && peek() >= '0' && peek() <= '9') {
                ++position_;
            }
        }
        if (!at_end() && peek() == '.') {
            ++position_;
            const std::size_t fraction_start = position_;
            while (!at_end() && peek() >= '0' && peek() <= '9') {
                ++position_;
            }
            if (position_ == fraction_start) {
                fail(error, "invalid number fraction");
                return nullptr;
            }
        }
        if (!at_end() && (peek() == 'e' || peek() == 'E')) {
            ++position_;
            if (!at_end() && (peek() == '+' || peek() == '-')) {
                ++position_;
            }
            const std::size_t exponent_start = position_;
            while (!at_end() && peek() >= '0' && peek() <= '9') {
                ++position_;
            }
            if (position_ == exponent_start) {
                fail(error, "invalid number exponent");
                return nullptr;
            }
        }

        const std::string token(input_.substr(start, position_ - start));
        char* end = nullptr;
        const double number = std::strtod(token.c_str(), &end);
        if (end == nullptr || *end != '\0' || !std::isfinite(number)) {
            fail(error, "number is not finite or cannot be parsed");
            return nullptr;
        }

        auto result = std::make_unique<Value>();
        result->type = Value::Type::Number;
        result->number = number;
        return result;
    }

    std::string_view input_;
    std::size_t position_{0};
};

}  // namespace

std::unique_ptr<Value> parse(std::string_view input, std::string& error) {
    error.clear();
    Parser parser(input);
    return parser.parse_document(error);
}

std::string escape(std::string_view value) {
    std::ostringstream output;
    for (const char character : value) {
        switch (character) {
            case '"':
                output << "\\\"";
                break;
            case '\\':
                output << "\\\\";
                break;
            case '\b':
                output << "\\b";
                break;
            case '\f':
                output << "\\f";
                break;
            case '\n':
                output << "\\n";
                break;
            case '\r':
                output << "\\r";
                break;
            case '\t':
                output << "\\t";
                break;
            default:
                if (static_cast<unsigned char>(character) < 0x20U) {
                    output << "\\u" << std::hex << std::setw(4)
                           << std::setfill('0')
                           << static_cast<int>(
                                  static_cast<unsigned char>(character))
                           << std::dec;
                } else {
                    output << character;
                }
                break;
        }
    }
    return output.str();
}

}  // namespace dock_ground::json
