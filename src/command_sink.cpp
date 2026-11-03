#include "dock_ground/command_sink.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace dock_ground {
namespace {

constexpr int kCommandErrorNone = 0;
constexpr int kCommandErrorInvalidSetpoint = 1;
constexpr int kCommandErrorLinkNotReady = 2;
constexpr int kCommandErrorUdpSend = 3;

[[nodiscard]] const char* command_error_name(int code) noexcept {
    switch (code) {
        case kCommandErrorInvalidSetpoint:
            return "invalid_setpoint";
        case kCommandErrorLinkNotReady:
            return "link_not_ready";
        case kCommandErrorUdpSend:
            return "udp_send_error";
        default:
            return "none";
    }
}

[[nodiscard]] bool valid_manual_control(
    const ManualControlSetpoint& setpoint) noexcept {
    constexpr double kLimit = 1.0;
    return setpoint.valid && std::isfinite(setpoint.roll) &&
           std::isfinite(setpoint.pitch) &&
           std::isfinite(setpoint.yaw) &&
           std::isfinite(setpoint.throttle) &&
           setpoint.roll >= -kLimit && setpoint.roll <= kLimit &&
           setpoint.pitch >= -kLimit && setpoint.pitch <= kLimit &&
           setpoint.yaw >= -kLimit && setpoint.yaw <= kLimit &&
           setpoint.throttle >= -kLimit &&
           setpoint.throttle <= kLimit;
}

}  // namespace

bool LoopbackCommandSink::send(
    const ManualControlSetpoint& setpoint) noexcept {
    if (!valid_manual_control(setpoint)) {
        send_errors_.fetch_add(1, std::memory_order_relaxed);
        last_error_code_.store(
            kCommandErrorInvalidSetpoint, std::memory_order_relaxed);
        last_errno_.store(0, std::memory_order_relaxed);
        return false;
    }
    last_error_code_.store(kCommandErrorNone, std::memory_order_relaxed);
    last_errno_.store(0, std::memory_order_relaxed);
    commands_sent_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

LinkStatus LoopbackCommandSink::status() const noexcept {
    LinkStatus result;
    result.mode = "loopback";
    result.protocol = "none";
    result.connected = true;
    result.commands_sent =
        commands_sent_.load(std::memory_order_relaxed);
    result.send_errors = send_errors_.load(std::memory_order_relaxed);
    result.last_error = command_error_name(
        last_error_code_.load(std::memory_order_relaxed));
    result.last_errno =
        last_errno_.load(std::memory_order_relaxed);
    return result;
}

GsLinkCommandSink::GsLinkCommandSink(
    gs_link::UdpLink& link, double target_rate_hz)
    : link_(link), target_rate_hz_(target_rate_hz) {
    const double bounded_rate = std::max(1.0, target_rate_hz_);
    send_period_ =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / bounded_rate));
    const double tolerance_s =
        std::min(0.001, 0.25 / bounded_rate);
    send_tolerance_ =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(tolerance_s));
}

bool GsLinkCommandSink::send(
    const ManualControlSetpoint& setpoint) noexcept {
    if (!valid_manual_control(setpoint)) {
        send_errors_.fetch_add(1, std::memory_order_relaxed);
        last_error_code_.store(
            kCommandErrorInvalidSetpoint, std::memory_order_relaxed);
        last_errno_.store(0, std::memory_order_relaxed);
        return false;
    }
    const auto now = std::chrono::steady_clock::now();
    if (next_send_ != std::chrono::steady_clock::time_point{} &&
        now + send_tolerance_ < next_send_) {
        ++suppressed_sends_;
        return true;
    }

    gs_link::UplinkRequest request;
    request.kind = gs_link::UplinkKind::Rc;
    request.rc.roll = static_cast<std::int16_t>(
        std::llround(setpoint.roll * 10000.0));
    request.rc.pitch = static_cast<std::int16_t>(
        std::llround(setpoint.pitch * 10000.0));
    request.rc.yaw = static_cast<std::int16_t>(
        std::llround(setpoint.yaw * 10000.0));
    request.rc.throttle = static_cast<std::int16_t>(
        std::llround(setpoint.throttle * 10000.0));
    request.rc.valid = true;
    request.rc.gs_ready = true;
    request.rc.link_quality = 100;

    std::string error;
    if (!link_.send(request, error)) {
        send_errors_.fetch_add(1, std::memory_order_relaxed);
        const gs_link::Status gs = link_.status();
        last_error_code_.store(
            gs.uplink_allowed ? kCommandErrorUdpSend
                              : kCommandErrorLinkNotReady,
            std::memory_order_relaxed);
        last_errno_.store(
            gs.uplink_allowed ? gs.last_send_errno : 0,
            std::memory_order_relaxed);
        next_send_ = now;
        return false;
    }
    last_error_code_.store(kCommandErrorNone, std::memory_order_relaxed);
    last_errno_.store(0, std::memory_order_relaxed);
    if (last_send_ != std::chrono::steady_clock::time_point{}) {
        const double interval_s =
            std::chrono::duration<double>(now - last_send_).count();
        if (interval_s > 0.0 && interval_s < 1.0) {
            smoothed_send_interval_s_ =
                smoothed_send_interval_s_ <= 0.0
                    ? interval_s
                    : 0.90 * smoothed_send_interval_s_ +
                          0.10 * interval_s;
        }
    }
    last_send_ = now;
    if (next_send_ == std::chrono::steady_clock::time_point{}) {
        next_send_ = now + send_period_;
    } else {
        next_send_ += send_period_;
        if (next_send_ <= now) {
            next_send_ = now + send_period_;
        }
    }
    commands_sent_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

LinkStatus GsLinkCommandSink::status() const noexcept {
    const gs_link::Status gs = link_.status();
    LinkStatus result;
    result.mode = "udp";
    result.protocol = "gs-link-v1";
    result.connected = gs.uplink_allowed;
    result.commands_sent =
        commands_sent_.load(std::memory_order_relaxed);
    result.send_errors =
        send_errors_.load(std::memory_order_relaxed);
    result.suppressed_sends = suppressed_sends_;
    result.target_rate_hz = target_rate_hz_;
    result.actual_rate_hz =
        smoothed_send_interval_s_ > 0.0
            ? 1.0 / smoothed_send_interval_s_
            : 0.0;
    result.last_send_interval_ms =
        smoothed_send_interval_s_ * 1000.0;
    result.last_error = command_error_name(
        last_error_code_.load(std::memory_order_relaxed));
    result.last_errno =
        last_errno_.load(std::memory_order_relaxed);
    return result;
}

}  // namespace dock_ground
