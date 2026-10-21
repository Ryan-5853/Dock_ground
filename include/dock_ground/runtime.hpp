#pragma once

#include "dock_ground/config.hpp"
#include "dock_ground/gs_link.hpp"
#include "dock_ground/types.hpp"

#include <memory>
#include <string>

namespace dock_ground {

class Runtime {
public:
    explicit Runtime(AppConfig config);
    ~Runtime();

    Runtime(const Runtime&) = delete;
    Runtime& operator=(const Runtime&) = delete;

    void start();
    void stop();

    [[nodiscard]] bool apply_operator_target(const OperatorTarget& target);
    [[nodiscard]] bool apply_position_tuning(
        const PositionLoopTuning& tuning, std::string& error);
    [[nodiscard]] PositionLoopTuning position_tuning() const;
    [[nodiscard]] bool bind_body(
        const std::string& aircraft_id, const std::string& body_id,
        int sensor_id, std::string& error);
    [[nodiscard]] bool apply_gs_uplink(
        const gs_link::UplinkRequest& request, std::string& error);
    [[nodiscard]] RuntimeSnapshot snapshot() const;
    [[nodiscard]] std::string snapshot_json() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dock_ground
