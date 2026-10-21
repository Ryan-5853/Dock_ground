#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace dock_ground::json {

struct Value {
    enum class Type {
        Null,
        Boolean,
        Number,
        String,
        Array,
        Object,
    };

    Type type{Type::Null};
    bool boolean{false};
    double number{0.0};
    std::string string;
    std::vector<std::unique_ptr<Value>> array;
    std::vector<std::pair<std::string, std::unique_ptr<Value>>> object;

    [[nodiscard]] const Value* find(std::string_view key) const;
};

[[nodiscard]] std::unique_ptr<Value> parse(
    std::string_view input, std::string& error);

[[nodiscard]] std::string escape(std::string_view value);

}  // namespace dock_ground::json
