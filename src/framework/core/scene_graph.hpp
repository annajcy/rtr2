#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
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
    bool self_enabled{true};
    std::vector<GameObjectId> children{};
};

struct SceneGraphSnapshot {
    std::vector<GameObjectId> root_children{};
    std::vector<SceneGraphNodeSnapshot> nodes{};
};

class SceneGraph;
class ConstNodeView;
class NodeView;

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
        bool self_enabled{true};
        bool hierarchy_active{true};
    };

private:
    static constexpr float kEpsilon = 1e-6f;

    std::unordered_map<GameObjectId, NodeRecord> m_nodes{};
    std::vector<GameObjectId> m_active_nodes{};

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

    void rebuild_active_recursive(GameObjectId id, bool parent_active) {
        auto it = m_nodes.find(id);
        if (it == m_nodes.end()) {
            return;
        }
        NodeRecord& node = it->second;
        const bool active = parent_active && node.self_enabled;
        node.hierarchy_active = active;
        if (id != kVirtualRootId && active) {
            m_active_nodes.emplace_back(id);
        }
        for (const auto child_id : node.children) {
            rebuild_active_recursive(child_id, active);
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
        root.self_enabled = true;
        root.hierarchy_active = true;
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
        record.self_enabled = true;
        record.hierarchy_active = true;
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
        std::unordered_set<GameObjectId> removed{subtree.begin(), subtree.end()};

        for (const auto id : subtree) {
            auto it = m_nodes.find(id);
            if (it == m_nodes.end()) {
                continue;
            }
            remove_child_link(it->second.parent_id, id);
            m_nodes.erase(it);
        }

        const auto active_it = std::remove_if(
            m_active_nodes.begin(),
            m_active_nodes.end(),
            [&removed](GameObjectId id) { return removed.contains(id); }
        );
        m_active_nodes.erase(active_it, m_active_nodes.end());
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

    void set_self_enabled(GameObjectId id, bool enabled) {
        if (!has_node(id) || id == kVirtualRootId) {
            return;
        }
        checked_record(id).self_enabled = enabled;
    }

    void rebuild_active_set(bool scene_enabled) {
        m_active_nodes.clear();
        NodeRecord& root = checked_record(kVirtualRootId);
        root.hierarchy_active = scene_enabled;
        for (const auto child_id : root.children) {
            rebuild_active_recursive(child_id, scene_enabled);
        }
    }

    void update_world_transforms() {
        const NodeRecord& root = checked_record(kVirtualRootId);
        for (const auto child_id : root.children) {
            update_world_recursive(child_id, glm::mat4{1.0f}, false);
        }
    }

    const std::vector<GameObjectId>& active_nodes() const {
        return m_active_nodes;
    }

    NodeView node(GameObjectId id);
    ConstNodeView node(GameObjectId id) const;

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
            node.self_enabled = record.self_enabled;
            node.children = record.children;
            snapshot.nodes.emplace_back(std::move(node));
        }
        return snapshot;
    }

    bool from_snapshot(const SceneGraphSnapshot& snapshot) {
        m_nodes.clear();
        m_active_nodes.clear();

        NodeRecord root{};
        root.id = kVirtualRootId;
        root.parent_id = kVirtualRootId;
        root.world_matrix = glm::mat4{1.0f};
        root.dirty = false;
        root.self_enabled = true;
        root.hierarchy_active = true;
        m_nodes.emplace(kVirtualRootId, std::move(root));

        for (const auto& item : snapshot.nodes) {
            if (item.id == kVirtualRootId || m_nodes.contains(item.id)) {
                return false;
            }
            NodeRecord record{};
            record.id = item.id;
            record.parent_id = item.parent_id;
            record.local_position = item.local_position;
            record.local_rotation = item.local_rotation;
            record.local_scale = item.local_scale;
            record.self_enabled = item.self_enabled;
            record.children = item.children;
            record.dirty = true;
            m_nodes.emplace(item.id, std::move(record));
        }

        checked_record(kVirtualRootId).children = snapshot.root_children;

        for (const auto& item : snapshot.nodes) {
            if (!has_node(item.id)) {
                return false;
            }
            if (item.parent_id != kVirtualRootId && !has_node(item.parent_id)) {
                return false;
            }
            for (const auto child_id : item.children) {
                if (!has_node(child_id)) {
                    return false;
                }
                if (checked_record(child_id).parent_id != item.id) {
                    return false;
                }
            }
        }

        for (const auto child_id : snapshot.root_children) {
            if (!has_node(child_id)) {
                return false;
            }
            if (checked_record(child_id).parent_id != kVirtualRootId) {
                return false;
            }
        }

        rebuild_active_set(true);
        update_world_transforms();
        return true;
    }

private:
    friend class ConstNodeView;
    friend class NodeView;
};

class ConstNodeView {
protected:
    const SceneGraph* m_graph{};
    GameObjectId m_id{core::kInvalidGameObjectId};

public:
    ConstNodeView() = default;
    ConstNodeView(const SceneGraph* graph, GameObjectId id)
        : m_graph(graph), m_id(id) {}

    bool valid() const {
        return m_graph != nullptr && m_graph->has_node(m_id);
    }

    GameObjectId id() const {
        return m_id;
    }

    GameObjectId parent_id() const {
        return m_graph->checked_record(m_id).parent_id;
    }

    const std::vector<GameObjectId>& children() const {
        return m_graph->checked_record(m_id).children;
    }

    const glm::vec3& local_position() const {
        return m_graph->checked_record(m_id).local_position;
    }

    const glm::quat& local_rotation() const {
        return m_graph->checked_record(m_id).local_rotation;
    }

    const glm::vec3& local_scale() const {
        return m_graph->checked_record(m_id).local_scale;
    }

    const glm::mat4& world_matrix() const {
        return m_graph->checked_record(m_id).world_matrix;
    }

    glm::vec3 world_position() const {
        return SceneGraph::extract_world_position(world_matrix());
    }

    glm::quat world_rotation() const {
        return SceneGraph::extract_world_rotation(world_matrix());
    }

    glm::vec3 world_scale() const {
        return SceneGraph::extract_world_scale(world_matrix());
    }

    bool dirty() const {
        return m_graph->checked_record(m_id).dirty;
    }

    bool self_enabled() const {
        return m_graph->checked_record(m_id).self_enabled;
    }

    bool hierarchy_active() const {
        return m_graph->checked_record(m_id).hierarchy_active;
    }
};

class NodeView final : public ConstNodeView {
public:
    NodeView() = default;
    NodeView(SceneGraph* graph, GameObjectId id)
        : ConstNodeView(graph, id) {}

    void set_local_position(const glm::vec3& value) {
        auto& record = const_cast<SceneGraph*>(m_graph)->checked_record(m_id);
        record.local_position = value;
        const_cast<SceneGraph*>(m_graph)->mark_subtree_dirty_recursive(m_id);
    }

    void set_local_rotation(const glm::quat& value) {
        auto& record = const_cast<SceneGraph*>(m_graph)->checked_record(m_id);
        record.local_rotation = value;
        const_cast<SceneGraph*>(m_graph)->mark_subtree_dirty_recursive(m_id);
    }

    void set_local_scale(const glm::vec3& value) {
        auto& record = const_cast<SceneGraph*>(m_graph)->checked_record(m_id);
        record.local_scale = value;
        const_cast<SceneGraph*>(m_graph)->mark_subtree_dirty_recursive(m_id);
    }

    void set_local_model_matrix(const glm::mat4& local_model_matrix) {
        glm::vec3 scale = {
            glm::length(glm::vec3(local_model_matrix[0])),
            glm::length(glm::vec3(local_model_matrix[1])),
            glm::length(glm::vec3(local_model_matrix[2]))
        };

        glm::mat3 rotation_matrix = glm::mat3(local_model_matrix);
        rotation_matrix[0] /= scale.x;
        rotation_matrix[1] /= scale.y;
        rotation_matrix[2] /= scale.z;
        glm::quat rotation = glm::quat_cast(rotation_matrix);
        glm::vec3 position = glm::vec3(local_model_matrix[3]);

        set_local_position(position);
        set_local_rotation(rotation);
        set_local_scale(scale);
    }

    void set_world_position(const glm::vec3& value) {
        const_cast<SceneGraph*>(m_graph)->set_world_position_internal(m_id, value);
    }

    void set_world_rotation(const glm::quat& value) {
        const_cast<SceneGraph*>(m_graph)->set_world_rotation_internal(m_id, value);
    }

    void set_world_scale(const glm::vec3& value) {
        const_cast<SceneGraph*>(m_graph)->set_world_scale_internal(m_id, value);
    }

    glm::vec3 position() const {
        return local_position();
    }

    glm::quat rotation() const {
        return local_rotation();
    }

    glm::vec3 scale() const {
        return local_scale();
    }

    glm::vec3 rotation_euler() const {
        return glm::degrees(glm::eulerAngles(local_rotation()));
    }

    glm::vec3 up() const {
        return local_rotation() * glm::vec3(0.0f, 1.0f, 0.0f);
    }

    glm::vec3 down() const {
        return local_rotation() * glm::vec3(0.0f, -1.0f, 0.0f);
    }

    glm::vec3 right() const {
        return local_rotation() * glm::vec3(1.0f, 0.0f, 0.0f);
    }

    glm::vec3 left() const {
        return local_rotation() * glm::vec3(-1.0f, 0.0f, 0.0f);
    }

    glm::vec3 front() const {
        return local_rotation() * glm::vec3(0.0f, 0.0f, 1.0f);
    }

    glm::vec3 back() const {
        return local_rotation() * glm::vec3(0.0f, 0.0f, -1.0f);
    }

    glm::vec3 world_up() const {
        return world_rotation() * glm::vec3(0.0f, 1.0f, 0.0f);
    }

    glm::vec3 world_down() const {
        return world_rotation() * glm::vec3(0.0f, -1.0f, 0.0f);
    }

    glm::vec3 world_right() const {
        return world_rotation() * glm::vec3(1.0f, 0.0f, 0.0f);
    }

    glm::vec3 world_left() const {
        return world_rotation() * glm::vec3(-1.0f, 0.0f, 0.0f);
    }

    glm::vec3 world_front() const {
        return world_rotation() * glm::vec3(0.0f, 0.0f, 1.0f);
    }

    glm::vec3 world_back() const {
        return world_rotation() * glm::vec3(0.0f, 0.0f, -1.0f);
    }

    glm::mat4 normal_matrix() const {
        return glm::transpose(glm::inverse(world_matrix()));
    }

    void look_at_direction(const glm::vec3& target_direction) {
        if (glm::length(target_direction) < SceneGraph::kEpsilon) {
            return;
        }

        const glm::vec3 direction = glm::normalize(target_direction);
        const glm::vec3 current_front = front();
        const float cross_len = glm::length(glm::cross(current_front, direction));
        glm::quat rotation = local_rotation();

        if (cross_len < SceneGraph::kEpsilon) {
            if (glm::dot(current_front, direction) < 0.0f) {
                rotation = glm::rotate(rotation, glm::radians(180.0f), up());
                set_local_rotation(rotation);
            }
            return;
        }

        const glm::quat delta = glm::rotation(current_front, direction);
        set_local_rotation(glm::normalize(delta * rotation));
    }

    void look_at_point(const glm::vec3& target_point) {
        look_at_direction(target_point - position());
    }

    void translate(const glm::vec3& direction, float distance) {
        set_local_position(position() + direction * distance);
    }

    void rotate(float angle, const glm::vec3& axis) {
        const glm::quat q = glm::angleAxis(glm::radians(angle), axis);
        set_local_rotation(q * local_rotation());
    }
};

inline NodeView SceneGraph::node(GameObjectId id) {
    return NodeView(this, id);
}

inline ConstNodeView SceneGraph::node(GameObjectId id) const {
    return ConstNodeView(this, id);
}

} // namespace rtr::framework::core
