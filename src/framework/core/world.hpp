#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "framework/core/scene.hpp"
#include "framework/core/tick_context.hpp"
#include "framework/core/types.hpp"

namespace rtr::framework::core {

class World {
private:
    WorldId m_id{core::kInvalidWorldId};
    bool m_enabled{true};
    SceneId m_next_scene_id{1};
    SceneId m_active_scene_id{core::kInvalidSceneId};
    std::vector<std::unique_ptr<Scene>> m_scenes{};

public:
    explicit World(WorldId id = core::kInvalidWorldId)
        : m_id(id) {}

    World(const World&) = delete;
    World& operator=(const World&) = delete;
    World(World&&) noexcept = default;
    World& operator=(World&&) noexcept = default;

    WorldId id() const {
        return m_id;
    }

    bool enabled() const {
        return m_enabled;
    }

    void set_enabled(bool enabled) {
        m_enabled = enabled;
    }

    Scene& create_scene(std::string name = "Scene") {
        auto scene = std::make_unique<Scene>(m_next_scene_id++, std::move(name));
        Scene* ptr = scene.get();
        m_scenes.emplace_back(std::move(scene));
        if (m_active_scene_id == core::kInvalidSceneId) {
            m_active_scene_id = ptr->id();
        }
        return *ptr;
    }

    Scene* find_scene(SceneId id) {
        for (const auto& scene : m_scenes) {
            if (scene && scene->id() == id) {
                return scene.get();
            }
        }
        return nullptr;
    }

    bool set_active_scene(SceneId id) {
        if (find_scene(id) == nullptr) {
            return false;
        }
        m_active_scene_id = id;
        return true;
    }

    SceneId active_scene_id() const {
        return m_active_scene_id;
    }

    Scene* active_scene() {
        return find_scene(m_active_scene_id);
    }

    const Scene* active_scene() const {
        for (const auto& scene : m_scenes) {
            if (scene && scene->id() == m_active_scene_id) {
                return scene.get();
            }
        }
        return nullptr;
    }

    const std::vector<std::unique_ptr<Scene>>& scenes() const {
        return m_scenes;
    }

    void fixed_tick(const FixedTickContext& ctx) {
        if (!m_enabled) {
            return;
        }
        Scene* scene = active_scene();
        if (scene != nullptr) {
            scene->fixed_tick(ctx);
        }
    }

    void tick(const FrameTickContext& ctx) {
        if (!m_enabled) {
            return;
        }
        Scene* scene = active_scene();
        if (scene != nullptr) {
            scene->tick(ctx);
        }
    }

    void late_tick(const FrameTickContext& ctx) {
        if (!m_enabled) {
            return;
        }
        Scene* scene = active_scene();
        if (scene != nullptr) {
            scene->late_tick(ctx);
        }
    }
};

} // namespace rtr::framework::core
