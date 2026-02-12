#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "framework/core/types.hpp"

namespace rtr::framework::core {

struct SceneGraphNodeSnapshot {
    GameObjectId id{core::kInvalidGameObjectId};
    GameObjectId parent_id{core::kInvalidGameObjectId};
    glm::vec3 local_position{0.0f, 0.0f, 0.0f};
    glm::quat local_rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 local_scale{1.0f, 1.0f, 1.0f};
    bool is_enabled{true};
    std::vector<GameObjectId> children{};
};

struct SceneGraphSnapshot {
    std::vector<GameObjectId> root_children{};
    std::vector<SceneGraphNodeSnapshot> nodes{};
};

class SceneGraph {
public:
    static constexpr GameObjectId kVirtualRootId = 0;

    struct NodeRecord {
        GameObjectId id{core::kInvalidGameObjectId};
        GameObjectId parent_id{kVirtualRootId};
        std::vector<GameObjectId> children{};

        glm::vec3 local_position{0.0f, 0.0f, 0.0f};
        glm::quat local_rotation{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 local_scale{1.0f, 1.0f, 1.0f};
        glm::mat4 world_matrix{1.0f};

        bool dirty{true};
        bool is_enabled{true};
    };

private:
    static constexpr float kEpsilon = 1e-6f;

    std::unordered_map<GameObjectId, NodeRecord> m_nodes{};

    static glm::mat4 compose_local_matrix(const NodeRecord& node) {
        glm::mat4 transform = glm::translate(glm::mat4{1.0f}, node.local_position);
        transform = transform * glm::mat4_cast(node.local_rotation);
        transform = glm::scale(transform, node.local_scale);
        return transform;
    }

    static glm::vec3 extract_world_position(const glm::mat4& world_matrix) {
        return {world_matrix[3][0], world_matrix[3][1], world_matrix[3][2]};
    }

    static glm::vec3 extract_world_scale(const glm::mat4& world_matrix) {
        return {
            glm::length(glm::vec3(world_matrix[0])),
            glm::length(glm::vec3(world_matrix[1])),
            glm::length(glm::vec3(world_matrix[2]))
        };
    }

    static glm::quat extract_world_rotation(const glm::mat4& world_matrix) {
        glm::vec3 scale = extract_world_scale(world_matrix);
        glm::mat3 rotation_matrix = glm::mat3(world_matrix);
        rotation_matrix[0] /= scale.x;
        rotation_matrix[1] /= scale.y;
        rotation_matrix[2] /= scale.z;
        return glm::quat_cast(rotation_matrix);
    }

    const NodeRecord& checked_record(GameObjectId id) const {
        const auto it = m_nodes.find(id);
        if (it == m_nodes.end()) {
            throw std::runtime_error("SceneGraph node id is invalid.");
        }
        return it->second;
    }

    NodeRecord& checked_record(GameObjectId id) {
        auto it = m_nodes.find(id);
        if (it == m_nodes.end()) {
            throw std::runtime_error("SceneGraph node id is invalid.");
        }
        return it->second;
    }

    void remove_child_link(GameObjectId parent_id, GameObjectId child_id) {
        auto parent_it = m_nodes.find(parent_id);
        if (parent_it == m_nodes.end()) {
            return;
        }
        auto& children = parent_it->second.children;
        const auto it = std::remove(children.begin(), children.end(), child_id);
        children.erase(it, children.end());
    }

    bool is_descendant(GameObjectId ancestor_id, GameObjectId candidate_descendant_id) const {
        GameObjectId current = candidate_descendant_id;
        while (current != kVirtualRootId) {
            if (current == ancestor_id) {
                return true;
            }
            const auto it = m_nodes.find(current);
            if (it == m_nodes.end()) {
                return false;
            }
            current = it->second.parent_id;
        }
        return false;
    }

    void mark_subtree_dirty_recursive(GameObjectId id) {
        auto it = m_nodes.find(id);
        if (it == m_nodes.end()) {
            return;
        }
        it->second.dirty = true;
        for (const auto child_id : it->second.children) {
            mark_subtree_dirty_recursive(child_id);
        }
    }

    void set_enabled_recursive(GameObjectId id, bool enabled) {
        auto it = m_nodes.find(id);
        if (it == m_nodes.end()) {
            return;
        }
        it->second.is_enabled = enabled;
        for (const auto child_id : it->second.children) {
            set_enabled_recursive(child_id, enabled);
        }
    }

    void collect_subtree_postorder_recursive(GameObjectId id, std::vector<GameObjectId>& out) const {
        const auto it = m_nodes.find(id);
        if (it == m_nodes.end()) {
            return;
        }
        for (const auto child_id : it->second.children) {
            collect_subtree_postorder_recursive(child_id, out);
        }
        out.emplace_back(id);
    }

    void update_world_recursive(GameObjectId id, const glm::mat4& parent_world, bool parent_dirty) {
        auto it = m_nodes.find(id);
        if (it == m_nodes.end()) {
            return;
        }

        NodeRecord& node = it->second;
        const bool dirty = parent_dirty || node.dirty;
        if (dirty) {
            node.world_matrix = parent_world * compose_local_matrix(node);
            node.dirty = false;
        }

        for (const auto child_id : node.children) {
            update_world_recursive(child_id, node.world_matrix, dirty);
        }
    }

    void collect_active_recursive(
        GameObjectId id,
        bool parent_active,
        std::vector<GameObjectId>& out
    ) const {
        auto it = m_nodes.find(id);
        if (it == m_nodes.end()) {
            return;
        }
        const NodeRecord& node = it->second;
        const bool active = parent_active && node.is_enabled;
        if (id != kVirtualRootId && active) {
            out.emplace_back(id);
        }
        for (const auto child_id : node.children) {
            collect_active_recursive(child_id, active, out);
        }
    }

    void set_world_position_internal(GameObjectId id, const glm::vec3& world_pos) {
        NodeRecord& node = checked_record(id);
        const glm::mat4 parent_world = (node.parent_id == kVirtualRootId)
                                           ? glm::mat4{1.0f}
                                           : checked_record(node.parent_id).world_matrix;
        const glm::vec4 local = glm::inverse(parent_world) * glm::vec4(world_pos, 1.0f);
        node.local_position = glm::vec3(local);
        mark_subtree_dirty_recursive(id);
    }

    void set_world_rotation_internal(GameObjectId id, const glm::quat& world_rot) {
        NodeRecord& node = checked_record(id);
        const glm::quat parent_rot = (node.parent_id == kVirtualRootId)
                                         ? glm::identity<glm::quat>()
                                         : extract_world_rotation(checked_record(node.parent_id).world_matrix);
        node.local_rotation = glm::inverse(parent_rot) * world_rot;
        mark_subtree_dirty_recursive(id);
    }

    void set_world_scale_internal(GameObjectId id, const glm::vec3& world_scale) {
        NodeRecord& node = checked_record(id);
        const glm::vec3 parent_scale = (node.parent_id == kVirtualRootId)
                                           ? glm::vec3{1.0f, 1.0f, 1.0f}
                                           : extract_world_scale(checked_record(node.parent_id).world_matrix);
        node.local_scale = world_scale / parent_scale;
        mark_subtree_dirty_recursive(id);
    }

public:
    SceneGraph() {
        NodeRecord root{};
        root.id = kVirtualRootId;
        root.parent_id = kVirtualRootId;
        root.world_matrix = glm::mat4{1.0f};
        root.dirty = false;
        root.is_enabled = true;
        m_nodes.emplace(kVirtualRootId, std::move(root));
    }

    bool register_node(GameObjectId id) {
        if (id == core::kInvalidGameObjectId || id == kVirtualRootId) {
            return false;
        }
        if (m_nodes.contains(id)) {
            return false;
        }

        NodeRecord record{};
        record.id = id;
        record.parent_id = kVirtualRootId;
        record.dirty = true;
        record.is_enabled = true;
        m_nodes.emplace(id, std::move(record));
        checked_record(kVirtualRootId).children.emplace_back(id);
        return true;
    }

    bool has_node(GameObjectId id) const {
        return m_nodes.contains(id);
    }

    std::vector<GameObjectId> collect_subtree_postorder(GameObjectId root_id) const {
        std::vector<GameObjectId> ids;
        if (!has_node(root_id) || root_id == kVirtualRootId) {
            return ids;
        }
        collect_subtree_postorder_recursive(root_id, ids);
        return ids;
    }

    bool unregister_subtree(GameObjectId root_id) {
        if (!has_node(root_id) || root_id == kVirtualRootId) {
            return false;
        }

        const auto subtree = collect_subtree_postorder(root_id);

        for (const auto id : subtree) {
            auto it = m_nodes.find(id);
            if (it == m_nodes.end()) {
                continue;
            }
            remove_child_link(it->second.parent_id, id);
            m_nodes.erase(it);
        }
        return true;
    }

    bool set_parent(GameObjectId child, GameObjectId parent, bool world_position_stays = true) {
        if (!has_node(child) || child == kVirtualRootId) {
            return false;
        }
        if (parent == core::kInvalidGameObjectId) {
            parent = kVirtualRootId;
        }
        if (!has_node(parent)) {
            return false;
        }
        if (child == parent) {
            return false;
        }
        if (is_descendant(child, parent)) {
            return false;
        }

        update_world_transforms();

        const glm::vec3 world_pos = extract_world_position(checked_record(child).world_matrix);
        const glm::quat world_rot = extract_world_rotation(checked_record(child).world_matrix);
        const glm::vec3 world_scale = extract_world_scale(checked_record(child).world_matrix);

        NodeRecord& child_node = checked_record(child);
        remove_child_link(child_node.parent_id, child);
        child_node.parent_id = parent;
        checked_record(parent).children.emplace_back(child);
        mark_subtree_dirty_recursive(child);

        if (world_position_stays) {
            set_world_position_internal(child, world_pos);
            set_world_rotation_internal(child, world_rot);
            set_world_scale_internal(child, world_scale);
        }

        return true;
    }

    bool clear_parent(GameObjectId child, bool world_position_stays = true) {
        return set_parent(child, kVirtualRootId, world_position_stays);
    }

    void set_enabled(GameObjectId id, bool enabled) {
        if (!has_node(id) || id == kVirtualRootId) {
            return;
        }
        set_enabled_recursive(id, enabled);
    }

    void update_world_transforms() {
        const NodeRecord& root = checked_record(kVirtualRootId);
        for (const auto child_id : root.children) {
            update_world_recursive(child_id, glm::mat4{1.0f}, false);
        }
    }

    std::vector<GameObjectId> active_nodes() const {
        std::vector<GameObjectId> result{};
        result.reserve(m_nodes.size() > 0 ? m_nodes.size() - 1 : 0);
        const NodeRecord& root = checked_record(kVirtualRootId);
        for (const auto child_id : root.children) {
            collect_active_recursive(child_id, true, result);
        }
        return result;
    }

    auto node(GameObjectId id);
    auto node(GameObjectId id) const;

    SceneGraphSnapshot to_snapshot() const {
        SceneGraphSnapshot snapshot{};
        snapshot.root_children = checked_record(kVirtualRootId).children;

        std::vector<GameObjectId> ids;
        ids.reserve(m_nodes.size());
        for (const auto& [id, _] : m_nodes) {
            if (id != kVirtualRootId) {
                ids.emplace_back(id);
            }
        }
        std::sort(ids.begin(), ids.end());

        snapshot.nodes.reserve(ids.size());
        for (const auto id : ids) {
            const NodeRecord& record = checked_record(id);
            SceneGraphNodeSnapshot node{};
            node.id = id;
            node.parent_id = record.parent_id;
            node.local_position = record.local_position;
            node.local_rotation = record.local_rotation;
            node.local_scale = record.local_scale;
            node.is_enabled = record.is_enabled;
            node.children = record.children;
            snapshot.nodes.emplace_back(std::move(node));
        }
        return snapshot;
    }

    static std::optional<SceneGraph> from_snapshot(const SceneGraphSnapshot& snapshot) {
        SceneGraph graph;
        graph.m_nodes.clear();

        NodeRecord root{};
        root.id = kVirtualRootId;
        root.parent_id = kVirtualRootId;
        root.world_matrix = glm::mat4{1.0f};
        root.dirty = false;
        root.is_enabled = true;
        graph.m_nodes.emplace(kVirtualRootId, std::move(root));

        for (const auto& item : snapshot.nodes) {
            if (item.id == kVirtualRootId || graph.m_nodes.contains(item.id)) {
                return std::nullopt;
            }
            NodeRecord record{};
            record.id = item.id;
            record.parent_id = item.parent_id;
            record.local_position = item.local_position;
            record.local_rotation = item.local_rotation;
            record.local_scale = item.local_scale;
            record.is_enabled = item.is_enabled;
            record.children = item.children;
            record.dirty = true;
            graph.m_nodes.emplace(item.id, std::move(record));
        }

        graph.checked_record(kVirtualRootId).children = snapshot.root_children;

        for (const auto& item : snapshot.nodes) {
            if (!graph.has_node(item.id)) {
                return std::nullopt;
            }
            if (item.parent_id != kVirtualRootId && !graph.has_node(item.parent_id)) {
                return std::nullopt;
            }
            for (const auto child_id : item.children) {
                if (!graph.has_node(child_id)) {
                    return std::nullopt;
                }
                if (graph.checked_record(child_id).parent_id != item.id) {
                    return std::nullopt;
                }
            }
        }

        for (const auto child_id : snapshot.root_children) {
            if (!graph.has_node(child_id)) {
                return std::nullopt;
            }
            if (graph.checked_record(child_id).parent_id != kVirtualRootId) {
                return std::nullopt;
            }
        }

        graph.update_world_transforms();
        return graph;
    }

private:
    friend class ConstNodeView;
    friend class NodeView;
};

} // namespace rtr::framework::core

#include "framework/core/scene_graph_view.hpp"
