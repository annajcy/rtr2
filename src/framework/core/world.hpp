#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/framework/core/types.hpp"

namespace rtr::framework::core {

class World {
private:
    SceneId m_next_scene_id{1};
    SceneId m_active_scene_id{core::kInvalidSceneId};
    std::vector<std::unique_ptr<Scene>> m_scenes{};

public:
    World() = default;

    World(const World&) = delete;
    World& operator=(const World&) = delete;
    World(World&&) noexcept = default;
    World& operator=(World&&) noexcept = default;

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

    const Scene* find_scene(SceneId id) const {
        for (const auto& scene : m_scenes) {
            if (scene && scene->id() == id) {
                return scene.get();
            }
        }
        return nullptr;
    }

    bool has_scene(SceneId id) const {
        return find_scene(id) != nullptr;
    }

    bool set_active_scene(SceneId id) {
        if (find_scene(id) == nullptr) {
            return false;
        }
        m_active_scene_id = id;
        return true;
    }

    bool destroy_scene(SceneId id) {
        for (auto it = m_scenes.begin(); it != m_scenes.end(); ++it) {
            if (*it && (*it)->id() == id) {
                m_scenes.erase(it);
                if (m_active_scene_id == id) {
                    m_active_scene_id = m_scenes.empty() ? core::kInvalidSceneId : m_scenes.front()->id();
                }
                return true;
            }
        }
        return false;
    }

    SceneId active_scene_id() const {
        return m_active_scene_id;
    }

    Scene* active_scene() {
        return find_scene(m_active_scene_id);
    }

    const Scene* active_scene() const {
        return find_scene(m_active_scene_id);
    }

    const std::vector<std::unique_ptr<Scene>>& scenes() const {
        return m_scenes;
    }

    std::size_t scene_count() const {
        return m_scenes.size();
    }

    void fixed_tick(const FixedTickContext& ctx) {
        Scene* scene = active_scene();
        if (scene != nullptr) {
            scene->fixed_tick(ctx);
        }
    }

    void tick(const FrameTickContext& ctx) {
        Scene* scene = active_scene();
        if (scene != nullptr) {
            scene->tick(ctx);
        }
    }

    void late_tick(const FrameTickContext& ctx) {
        Scene* scene = active_scene();
        if (scene != nullptr) {
            scene->late_tick(ctx);
        }
    }
};

} // namespace rtr::framework::core
