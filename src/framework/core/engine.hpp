#pragma once

#include <memory>
#include <utility>

#include "framework/core/engine_config.hpp"
#include "framework/core/tick_context.hpp"
#include "framework/core/world.hpp"

namespace rtr::framework::core {

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
