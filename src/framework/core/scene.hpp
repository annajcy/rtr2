#pragma once

#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "framework/core/game_object.hpp"
#include "framework/core/scene_graph.hpp"
#include "framework/core/tick_context.hpp"
#include "framework/core/types.hpp"

namespace rtr::framework::core {

class Scene {
private:
    SceneId m_id{core::kInvalidSceneId};
    std::string m_name{"Scene"};
    bool m_enabled{true};

    GameObjectId m_next_game_object_id{1};
    std::vector<std::unique_ptr<GameObject>> m_game_objects{};
    SceneGraph m_scene_graph{};

    void sync_scene_graph_enabled_state() {
        for (const auto& game_object : m_game_objects) {
            if (!game_object) {
                continue;
            }
            m_scene_graph.set_self_enabled(game_object->id(), game_object->enabled());
        }
    }

public:
    explicit Scene(
        SceneId id = core::kInvalidSceneId,
        std::string name = "Scene"
    )
        : m_id(id), m_name(std::move(name)) {}

    Scene(const Scene&) = delete;
    Scene& operator=(const Scene&) = delete;
    Scene(Scene&&) noexcept = default;
    Scene& operator=(Scene&&) noexcept = default;

    SceneId id() const {
        return m_id;
    }

    const std::string& name() const {
        return m_name;
    }

    void set_name(std::string name) {
        m_name = std::move(name);
    }

    bool enabled() const {
        return m_enabled;
    }

    void set_enabled(bool enabled) {
        m_enabled = enabled;
    }

    GameObject& create_game_object(std::string name = "GameObject") {
        auto game_object = std::make_unique<GameObject>(m_next_game_object_id++, std::move(name));
        GameObject* ptr = game_object.get();
        m_scene_graph.register_node(ptr->id());
        m_game_objects.emplace_back(std::move(game_object));
        return *ptr;
    }

    GameObject* find_game_object(GameObjectId id) {
        for (const auto& game_object : m_game_objects) {
            if (game_object && game_object->id() == id) {
                return game_object.get();
            }
        }
        return nullptr;
    }

    const GameObject* find_game_object(GameObjectId id) const {
        for (const auto& game_object : m_game_objects) {
            if (game_object && game_object->id() == id) {
                return game_object.get();
            }
        }
        return nullptr;
    }

    bool has_game_object(GameObjectId id) const {
        return find_game_object(id) != nullptr;
    }

    bool destroy_game_object(GameObjectId id) {
        if (!m_scene_graph.has_node(id)) {
            return false;
        }
        const auto subtree_ids = m_scene_graph.collect_subtree_postorder(id);
        if (subtree_ids.empty()) {
            return false;
        }

        // First run destroy lifecycle hooks in subtree postorder. Any exception
        // is propagated to caller without rollback.
        for (const auto victim_id : subtree_ids) {
            GameObject* game_object = find_game_object(victim_id);
            if (game_object != nullptr) {
                game_object->destroy_components();
            }
        }

        for (const auto victim_id : subtree_ids) {
            const auto it = std::find_if(
                m_game_objects.begin(),
                m_game_objects.end(),
                [victim_id](const std::unique_ptr<GameObject>& game_object) {
                    return game_object && game_object->id() == victim_id;
                }
            );
            if (it != m_game_objects.end()) {
                m_game_objects.erase(it); // May throw; propagate by design.
            }
        }

        return m_scene_graph.unregister_subtree(id);
    }

    std::size_t game_object_count() const {
        return m_game_objects.size();
    }

    const std::vector<std::unique_ptr<GameObject>>& game_objects() const {
        return m_game_objects;
    }

    SceneGraph& scene_graph() {
        return m_scene_graph;
    }

    const SceneGraph& scene_graph() const {
        return m_scene_graph;
    }

    void fixed_tick(const FixedTickContext& ctx) {
        if (!m_enabled) {
            return;
        }
        for (const auto& game_object : m_game_objects) {
            if (game_object) {
                game_object->fixed_tick(ctx);
            }
        }
    }

    void tick(const FrameTickContext& ctx) {
        if (!m_enabled) {
            return;
        }
        sync_scene_graph_enabled_state();
        m_scene_graph.rebuild_active_set(m_enabled);
        m_scene_graph.update_world_transforms();
        for (const auto& game_object : m_game_objects) {
            if (game_object) {
                game_object->tick(ctx);
            }
        }
    }

    void late_tick(const FrameTickContext& ctx) {
        if (!m_enabled) {
            return;
        }
        for (const auto& game_object : m_game_objects) {
            if (game_object) {
                game_object->late_tick(ctx);
            }
        }
    }
};

} // namespace rtr::framework::core
