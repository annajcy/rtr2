#pragma once

#include <memory>
#include <utility>

#include "framework/core/engine_config.hpp"
#include "framework/world/world.hpp"

namespace rtr::framework::core {

class Engine {
private:
    EngineConfig m_config{};
    std::unique_ptr<world::World> m_world{};

public:
    explicit Engine(EngineConfig config = {})
        : m_config(std::move(config)),
          m_world(std::make_unique<world::World>()) {}

    const EngineConfig& config() const {
        return m_config;
    }

    world::World& world() {
        return *m_world;
    }

    const world::World& world() const {
        return *m_world;
    }

    // Phase 1 skeleton only. Tick execution is introduced in Phase 2.
    void run() {}
};

} // namespace rtr::framework::core
