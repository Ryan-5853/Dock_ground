#pragma once

#include "dock_ground/types.hpp"

#include <atomic>
#include <chrono>

namespace dock_ground {

class ICommandSink {
public:
    virtual ~ICommandSink() = default;
    virtual bool send(const ManualControlSetpoint& setpoint) noexcept = 0;
    [[nodiscard]] virtual LinkStatus status() const noexcept = 0;
};

class LoopbackCommandSink final : public ICommandSink {
public:
    bool send(const ManualControlSetpoint& setpoint) noexcept override;
    [[nodiscard]] LinkStatus status() const noexcept override;

private:
    std::atomic<std::uint64_t> commands_sent_{0};
    std::atomic<std::uint64_t> send_errors_{0};
    std::atomic<int> last_error_code_{0};
    std::atomic<int> last_errno_{0};
};

class GsLinkCommandSink final : public ICommandSink {
public:
    explicit GsLinkCommandSink(gs_link::UdpLink& link);

    bool send(const ManualControlSetpoint& setpoint) noexcept override;
    [[nodiscard]] LinkStatus status() const noexcept override;

private:
    gs_link::UdpLink& link_;
    std::atomic<std::uint64_t> commands_sent_{0};
    std::atomic<std::uint64_t> send_errors_{0};
    std::atomic<int> last_error_code_{0};
    std::atomic<int> last_errno_{0};
    std::chrono::steady_clock::time_point last_send_{};
};

}  // namespace dock_ground
