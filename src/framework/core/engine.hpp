#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "framework/core/tick_context.hpp"
#include "framework/core/world.hpp"

namespace rtr::framework::core {

struct EngineConfig {
    std::uint32_t window_width{800};
    std::uint32_t window_height{600};
    std::string window_title{"RTR2 Framework"};
    std::uint32_t max_frames_in_flight{2};

    double fixed_delta_seconds{1.0 / 60.0};
    std::uint32_t max_fixed_steps_per_frame{4};
    bool start_paused{false};
};

class Engine {
private:
    EngineConfig m_config{};
    std::unique_ptr<World> m_world{};

public:
    explicit Engine(EngineConfig config = {})
        : m_config(std::move(config)),
          m_world(std::make_unique<World>()) {}

    const EngineConfig& config() const {
        return m_config;
    }

    World& world() {
        return *m_world;
    }

    const World& world() const {
        return *m_world;
    }

    void fixed_tick(const FixedTickContext& ctx) {
        m_world->fixed_tick(ctx);
    }

    void tick(const FrameTickContext& ctx) {
        m_world->tick(ctx);
    }

    void late_tick(const FrameTickContext& ctx) {
        m_world->late_tick(ctx);
    }

    // Phase 1/feedback revision: keep run empty; full loop lands in Phase 2.
    void run() {}
};

} // namespace rtr::framework::core
