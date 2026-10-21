#pragma once

#include "dock_ground/gs_link.hpp"
#include "dock_ground/types.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace dock_ground {

struct WebServerOptions {
    std::string bind_address{"127.0.0.1"};
    std::uint16_t port{8080};
    std::string web_root{"frontend/dist"};
};

class WebServer {
public:
    using IntentHandler = std::function<bool(const OperatorTarget&)>;
    using BindingHandler = std::function<bool(
        const std::string& aircraft_id, const std::string& body_id,
        int sensor_id, std::string& error)>;
    using UplinkHandler = std::function<bool(
        const gs_link::UplinkRequest& request, std::string& error)>;
    using TuningHandler = std::function<bool(
        const PositionLoopTuning& tuning, std::string& error)>;
    using SnapshotProvider = std::function<std::string()>;

    WebServer(
        WebServerOptions options, IntentHandler intent_handler,
        BindingHandler binding_handler, UplinkHandler uplink_handler,
        TuningHandler tuning_handler, SnapshotProvider snapshot_provider);
    ~WebServer();

    WebServer(const WebServer&) = delete;
    WebServer& operator=(const WebServer&) = delete;

    [[nodiscard]] bool start(std::string& error);
    void stop();

    [[nodiscard]] std::uint16_t bound_port() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dock_ground
