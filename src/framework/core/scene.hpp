#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <glm/mat4x4.hpp>

#include "framework/component/node_component.hpp"
#include "framework/core/game_object.hpp"
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

    void detach_child_from_current_parent(GameObjectId child_id, component::NodeComponent& child_node) {
        if (child_node.parent_id() == core::kInvalidGameObjectId) {
            return;
        }
        if (auto* parent_go = find_game_object(child_node.parent_id()); parent_go != nullptr) {
            if (auto* parent_node = parent_go->get_component<component::NodeComponent>(); parent_node != nullptr) {
                parent_node->remove_child_internal(child_id);
            }
        }
        child_node.set_parent_id_internal(core::kInvalidGameObjectId);
    }

    bool would_create_cycle(GameObjectId child_id, GameObjectId new_parent_id) const {
        GameObjectId current = new_parent_id;
        while (current != core::kInvalidGameObjectId) {
            if (current == child_id) {
                return true;
            }
            const auto* go = find_game_object(current);
            if (go == nullptr) {
                return false;
            }
            const auto* node = go->get_component<component::NodeComponent>();
            if (node == nullptr) {
                return false;
            }
            current = node->parent_id();
        }
        return false;
    }

    bool has_valid_parent_node(const component::NodeComponent& node) const {
        if (node.parent_id() == core::kInvalidGameObjectId) {
            return false;
        }
        const auto* parent_go = find_game_object(node.parent_id());
        if (parent_go == nullptr || !parent_go->enabled()) {
            return false;
        }
        return parent_go->get_component<component::NodeComponent>() != nullptr;
    }

    void mark_node_subtree_dirty(GameObjectId root_id, std::unordered_set<GameObjectId>& visited) {
        if (visited.contains(root_id)) {
            return;
        }
        visited.insert(root_id);

        auto* go = find_game_object(root_id);
        if (go == nullptr) {
            return;
        }
        auto* node = go->get_component<component::NodeComponent>();
        if (node == nullptr) {
            return;
        }
        node->mark_dirty();
        for (const auto child_id : node->children()) {
            mark_node_subtree_dirty(child_id, visited);
        }
    }

    void mark_node_subtree_dirty(GameObjectId root_id) {
        std::unordered_set<GameObjectId> visited;
        mark_node_subtree_dirty(root_id, visited);
    }

    void propagate_world_transform(
        GameObjectId game_object_id,
        const glm::mat4& parent_world,
        bool parent_dirty,
        std::unordered_set<GameObjectId>& visited,
        std::unordered_set<GameObjectId>& traversal_stack
    ) {
        if (traversal_stack.contains(game_object_id)) {
            return;
        }

        auto* game_object = find_game_object(game_object_id);
        if (game_object == nullptr || !game_object->enabled()) {
            return;
        }

        auto* node = game_object->get_component<component::NodeComponent>();
        if (node == nullptr) {
            return;
        }

        const bool dirty = parent_dirty || node->dirty();
        if (dirty) {
            node->set_world_matrix_internal(parent_world * node->local_matrix());
        }

        visited.insert(game_object_id);
        traversal_stack.insert(game_object_id);
        const glm::mat4 world_matrix = node->world_matrix();
        for (const auto child_id : node->children()) {
            propagate_world_transform(child_id, world_matrix, dirty, visited, traversal_stack);
        }
        traversal_stack.erase(game_object_id);
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
        for (auto it = m_game_objects.begin(); it != m_game_objects.end(); ++it) {
            if (*it && (*it)->id() == id) {
                if (auto* node = (*it)->get_component<component::NodeComponent>(); node != nullptr) {
                    detach_child_from_current_parent(id, *node);
                    const auto children = node->children();
                    for (const auto child_id : children) {
                        if (auto* child_go = find_game_object(child_id); child_go != nullptr) {
                            if (auto* child_node = child_go->get_component<component::NodeComponent>(); child_node != nullptr) {
                                child_node->set_parent_id_internal(core::kInvalidGameObjectId);
                                mark_node_subtree_dirty(child_id);
                            }
                        }
                    }
                }
                m_game_objects.erase(it);
                return true;
            }
        }
        return false;
    }

    std::size_t game_object_count() const {
        return m_game_objects.size();
    }

    bool set_parent(GameObjectId child_id, GameObjectId parent_id) {
        if (child_id == core::kInvalidGameObjectId || child_id == parent_id) {
            return false;
        }

        auto* child_go = find_game_object(child_id);
        if (child_go == nullptr) {
            return false;
        }
        auto* child_node = child_go->get_component<component::NodeComponent>();
        if (child_node == nullptr) {
            return false;
        }

        component::NodeComponent* parent_node = nullptr;
        if (parent_id != core::kInvalidGameObjectId) {
            auto* parent_go = find_game_object(parent_id);
            if (parent_go == nullptr) {
                return false;
            }
            parent_node = parent_go->get_component<component::NodeComponent>();
            if (parent_node == nullptr) {
                return false;
            }
            if (would_create_cycle(child_id, parent_id)) {
                return false;
            }
        }

        detach_child_from_current_parent(child_id, *child_node);
        if (parent_node != nullptr) {
            parent_node->add_child_internal(child_id);
            child_node->set_parent_id_internal(parent_id);
        }

        mark_node_subtree_dirty(child_id);
        return true;
    }

    bool clear_parent(GameObjectId child_id) {
        return set_parent(child_id, core::kInvalidGameObjectId);
    }

    void update_world_transforms() {
        std::unordered_set<GameObjectId> visited;
        std::unordered_set<GameObjectId> traversal_stack;

        for (const auto& game_object : m_game_objects) {
            if (!game_object || !game_object->enabled()) {
                continue;
            }
            auto* node = game_object->get_component<component::NodeComponent>();
            if (node == nullptr) {
                continue;
            }

            if (!has_valid_parent_node(*node)) {
                propagate_world_transform(
                    game_object->id(),
                    glm::mat4{1.0f},
                    false,
                    visited,
                    traversal_stack
                );
            }
        }

        for (const auto& game_object : m_game_objects) {
            if (!game_object || !game_object->enabled()) {
                continue;
            }
            auto* node = game_object->get_component<component::NodeComponent>();
            if (node == nullptr) {
                continue;
            }
            if (visited.contains(game_object->id())) {
                continue;
            }
            propagate_world_transform(
                game_object->id(),
                glm::mat4{1.0f},
                false,
                visited,
                traversal_stack
            );
        }
    }

    const std::vector<std::unique_ptr<GameObject>>& game_objects() const {
        return m_game_objects;
    }

    void fixed_tick(const FixedTickContext& ctx) {
        if (!m_enabled) {
            return;
        }
        update_world_transforms();
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
        update_world_transforms();
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
        update_world_transforms();
        for (const auto& game_object : m_game_objects) {
            if (game_object) {
                game_object->late_tick(ctx);
            }
        }
    }
};

} // namespace rtr::framework::core
