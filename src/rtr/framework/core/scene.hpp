#pragma once

#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rtr/framework/core/game_object.hpp"
#include "rtr/framework/core/scene_graph.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/framework/core/types.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::core {

class Scene final {
private:
    static std::shared_ptr<spdlog::logger> logger() { return utils::get_logger("framework.core.scene"); }

    SceneId m_id{core::kInvalidSceneId};
    bool    m_enabled{true};

    GameObjectId                             m_next_game_object_id{1};
    SceneGraph                               m_scene_graph{};
    std::vector<std::unique_ptr<GameObject>> m_game_objects{};
    std::unordered_map<GameObjectId, GameObject*> m_game_object_by_id{};
    std::unordered_map<GameObjectId, std::string> m_game_object_name_by_id{};
    std::unordered_map<std::string, GameObjectId> m_game_object_id_by_name{};

    bool is_name_occupied(std::string_view name, GameObjectId ignore_id = core::kInvalidGameObjectId) const {
        const auto it = m_game_object_id_by_name.find(std::string(name));
        if (it == m_game_object_id_by_name.end()) {
            return false;
        }
        return it->second != ignore_id;
    }

    std::string make_unique_game_object_name(std::string_view requested_name,
                                             GameObjectId ignore_id = core::kInvalidGameObjectId) const {
        std::string base_name = requested_name.empty() ? "GameObject" : std::string(requested_name);
        if (!is_name_occupied(base_name, ignore_id)) {
            return base_name;
        }
        for (std::size_t suffix = 1;; ++suffix) {
            std::string candidate = base_name + "_" + std::to_string(suffix);
            if (!is_name_occupied(candidate, ignore_id)) {
                return candidate;
            }
        }
    }

public:
    explicit Scene(SceneId id = core::kInvalidSceneId)
        : m_id(id) {}

    Scene(const Scene&) = delete;
    Scene& operator=(const Scene&) = delete;
    Scene(Scene&&) noexcept = delete;
    Scene& operator=(Scene&&) noexcept = delete;

    SceneId id() const { return m_id; }

    bool enabled() const { return m_enabled; }

    void set_enabled(bool enabled) {
        m_enabled = enabled;
        logger()->info("Scene {} enabled set to {}.", m_id, m_enabled);
    }

    GameObject& create_game_object(std::string name = "GameObject") {
        std::string unique_name = make_unique_game_object_name(name);
        auto game_object = std::make_unique<GameObject>(m_next_game_object_id++, m_scene_graph);
        auto* ptr        = game_object.get();
        m_scene_graph.register_node(ptr->id());
        m_game_objects.emplace_back(std::move(game_object));
        m_game_object_by_id.emplace(ptr->id(), ptr);
        m_game_object_name_by_id.emplace(ptr->id(), unique_name);
        m_game_object_id_by_name.emplace(unique_name, ptr->id());
        logger()->debug("GameObject created (scene_id={}, game_object_id={}, name='{}', count={})", m_id, ptr->id(),
                        unique_name, m_game_objects.size());
        return *ptr;
    }

    GameObject* find_game_object(GameObjectId id) {
        const auto it = m_game_object_by_id.find(id);
        return (it == m_game_object_by_id.end()) ? nullptr : it->second;
    }

    const GameObject* find_game_object(GameObjectId id) const {
        const auto it = m_game_object_by_id.find(id);
        return (it == m_game_object_by_id.end()) ? nullptr : it->second;
    }

    GameObject* find_game_object(std::string_view name) {
        const auto id_it = m_game_object_id_by_name.find(std::string(name));
        if (id_it == m_game_object_id_by_name.end()) {
            return nullptr;
        }
        return find_game_object(id_it->second);
    }

    const GameObject* find_game_object(std::string_view name) const {
        const auto id_it = m_game_object_id_by_name.find(std::string(name));
        if (id_it == m_game_object_id_by_name.end()) {
            return nullptr;
        }
        return find_game_object(id_it->second);
    }

    bool has_game_object(GameObjectId id) const { return find_game_object(id) != nullptr; }
    bool has_game_object(std::string_view name) const { return find_game_object(name) != nullptr; }
    std::optional<std::string_view> game_object_name(GameObjectId id) const noexcept {
        const auto it = m_game_object_name_by_id.find(id);
        if (it == m_game_object_name_by_id.end()) {
            return std::nullopt;
        }
        return std::string_view{it->second};
    }

    bool rename_game_object(GameObjectId id, std::string new_name) {
        auto* game_object = find_game_object(id);
        if (game_object == nullptr) {
            logger()->warn("rename_game_object ignored: game_object_id={} does not exist in Scene {}.", id, m_id);
            return false;
        }

        const auto name_it = m_game_object_name_by_id.find(id);
        if (name_it == m_game_object_name_by_id.end()) {
            logger()->error("rename_game_object failed: missing name entry for game_object_id={} in Scene {}.", id,
                            m_id);
            return false;
        }
        const std::string old_name = name_it->second;
        const std::string unique_name = make_unique_game_object_name(new_name, id);
        if (old_name == unique_name) {
            return true;
        }

        m_game_object_id_by_name.erase(old_name);
        name_it->second = unique_name;
        m_game_object_id_by_name[unique_name] = id;
        logger()->debug("GameObject renamed (scene_id={}, game_object_id={}, old_name='{}', new_name='{}').", m_id,
                        id, old_name, unique_name);
        return true;
    }

    bool destroy_game_object(GameObjectId id) {
        if (!m_scene_graph.has_node(id)) {
            logger()->warn("destroy_game_object ignored: node {} does not exist in Scene {}.", id, m_id);
            return false;
        }
        const auto subtree_ids = m_scene_graph.collect_subtree_postorder(id);
        if (subtree_ids.empty()) {
            logger()->warn("destroy_game_object ignored: subtree for node {} is empty in Scene {}.", id, m_id);
            return false;
        }

        for (const auto victim_id : subtree_ids) {
            auto* game_object = find_game_object(victim_id);
            if (game_object != nullptr) {
                game_object->destroy_components();
            }
        }

        for (const auto victim_id : subtree_ids) {
            if (auto by_id_it = m_game_object_by_id.find(victim_id); by_id_it != m_game_object_by_id.end()) {
                if (const auto name_it = m_game_object_name_by_id.find(victim_id);
                    name_it != m_game_object_name_by_id.end()) {
                    m_game_object_id_by_name.erase(name_it->second);
                    m_game_object_name_by_id.erase(name_it);
                }
                m_game_object_by_id.erase(by_id_it);
            }
            const auto it =
                std::find_if(m_game_objects.begin(), m_game_objects.end(),
                             [victim_id](const std::unique_ptr<GameObject>& game_object) {
                                 return game_object && game_object->id() == victim_id;
                             });
            if (it != m_game_objects.end()) {
                m_game_objects.erase(it);
            }
        }

        const bool unregistered = m_scene_graph.unregister_subtree(id);
        logger()->info(
            "GameObject subtree destroyed (scene_id={}, root_game_object_id={}, removed_count={}, success={}, remaining={})",
            m_id, id, subtree_ids.size(), unregistered, m_game_objects.size());
        return unregistered;
    }

    std::size_t game_object_count() const { return m_game_objects.size(); }

    const std::vector<std::unique_ptr<GameObject>>& game_objects() const { return m_game_objects; }

    SceneGraph& scene_graph() { return m_scene_graph; }
    const SceneGraph& scene_graph() const { return m_scene_graph; }

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
        m_scene_graph.update_world_transforms();
        for (const auto& game_object : m_game_objects) {
            if (game_object) {
                game_object->tick(ctx);
            }
        }
        m_scene_graph.update_world_transforms();
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

}  // namespace rtr::framework::core
