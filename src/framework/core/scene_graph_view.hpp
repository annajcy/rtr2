#pragma once

#include "framework/core/scene_graph.hpp"

namespace rtr::framework::core {

class NodeView final {
private:
    const SceneGraph* m_graph{};
    SceneGraph* m_mut_graph{};
    GameObjectId m_id{core::kInvalidGameObjectId};

    SceneGraph& mutable_graph() {
        if (m_mut_graph == nullptr) {
            throw std::runtime_error("NodeView is read-only.");
        }
        return *m_mut_graph;
    }

public:
    NodeView() = default;
    NodeView(SceneGraph* graph, GameObjectId id)
        : m_graph(graph), m_mut_graph(graph), m_id(id) {}
    NodeView(const SceneGraph* graph, GameObjectId id)
        : m_graph(graph), m_mut_graph(nullptr), m_id(id) {}

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

    bool is_enabled() const {
        return m_graph->checked_record(m_id).is_enabled;
    }

    void set_local_position(const glm::vec3& value) {
        auto& graph = mutable_graph();
        auto& record = graph.checked_record(m_id);
        record.local_position = value;
        graph.mark_subtree_dirty_recursive(m_id);
    }

    void set_local_rotation(const glm::quat& value) {
        auto& graph = mutable_graph();
        auto& record = graph.checked_record(m_id);
        record.local_rotation = value;
        graph.mark_subtree_dirty_recursive(m_id);
    }

    void set_local_scale(const glm::vec3& value) {
        auto& graph = mutable_graph();
        auto& record = graph.checked_record(m_id);
        record.local_scale = value;
        graph.mark_subtree_dirty_recursive(m_id);
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
        mutable_graph().set_world_position_internal(m_id, value);
    }

    void set_world_rotation(const glm::quat& value) {
        mutable_graph().set_world_rotation_internal(m_id, value);
    }

    void set_world_scale(const glm::vec3& value) {
        mutable_graph().set_world_scale_internal(m_id, value);
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

inline auto SceneGraph::node(GameObjectId id) {
    return NodeView(this, id);
}

inline auto SceneGraph::node(GameObjectId id) const {
    return NodeView(this, id);
}

} // namespace rtr::framework::core
