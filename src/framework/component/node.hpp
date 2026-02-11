#pragma once

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

namespace rtr::framework::component {

class Node : public std::enable_shared_from_this<Node> {
private:
    static constexpr float kEpsilon = 1e-6f;

    glm::mat4 m_model_matrix{glm::identity<glm::mat4>()};
    bool m_is_dirty{true};

    glm::vec3 m_position{glm::zero<glm::vec3>()};
    glm::vec3 m_scale{glm::one<glm::vec3>()};
    glm::quat m_rotation{glm::identity<glm::quat>()};

    std::vector<std::shared_ptr<Node>> m_children{};
    std::weak_ptr<Node> m_parent{};

    bool contains_in_subtree(const Node* candidate) const {
        if (candidate == this) {
            return true;
        }
        for (const auto& child : m_children) {
            if (child && child->contains_in_subtree(candidate)) {
                return true;
            }
        }
        return false;
    }

    void remove_child_internal(const Node* node) {
        const auto it = std::remove_if(
            m_children.begin(),
            m_children.end(),
            [node](const std::shared_ptr<Node>& child) {
                return child.get() == node;
            }
        );
        m_children.erase(it, m_children.end());
    }

public:
    ~Node() {
        if (auto parent_node = m_parent.lock(); parent_node != nullptr) {
            parent_node->remove_child_internal(this);
        }

        for (auto& child : m_children) {
            if (!child) {
                continue;
            }
            child->m_parent.reset();
            child->set_dirty();
        }
    }

    static std::shared_ptr<Node> create() {
        return std::make_shared<Node>();
    }

    std::shared_ptr<Node> parent() const {
        return m_parent.lock();
    }

    const std::vector<std::shared_ptr<Node>>& children() const {
        return m_children;
    }

    bool is_dirty() const {
        return m_is_dirty;
    }

    void add_child(const std::shared_ptr<Node>& node, bool world_position_stays = false) {
        if (!node) {
            throw std::invalid_argument("Child node is null.");
        }
        if (node.get() == this) {
            throw std::invalid_argument("Cannot add self as child.");
        }
        if (node->contains_in_subtree(this)) {
            throw std::invalid_argument("Cannot create cycle in node hierarchy.");
        }

        if (auto existing_parent = node->parent(); existing_parent != nullptr) {
            existing_parent->remove_child(node);
        }

        const auto world_position = node->world_position();
        const auto world_rotation = node->world_rotation();
        const auto world_scale = node->world_scale();

        m_children.push_back(node);
        node->m_parent = shared_from_this();

        if (world_position_stays) {
            node->set_world_position(world_position);
            node->set_world_rotation(world_rotation);
            node->set_world_scale(world_scale);
        } else {
            node->set_dirty();
        }
    }

    void remove_child(const std::shared_ptr<Node>& node) {
        auto it = std::find(m_children.begin(), m_children.end(), node);
        if (it == m_children.end()) {
            throw std::invalid_argument("Node is not a child.");
        }
        m_children.erase(it);
        node->m_parent.reset();
        node->set_dirty();
    }

    void set_position(const glm::vec3& pos) {
        m_position = pos;
        set_dirty();
    }

    void set_world_position(const glm::vec3& pos) {
        if (auto parent_node = parent(); parent_node != nullptr) {
            m_position = pos - parent_node->world_position();
        } else {
            m_position = pos;
        }
        set_dirty();
    }

    void set_rotation(const glm::quat& rot) {
        m_rotation = rot;
        set_dirty();
    }

    void set_rotation_euler(const glm::vec3& rot) {
        m_rotation = glm::quat(glm::radians(rot));
        set_dirty();
    }

    void set_world_rotation(const glm::quat& rot) {
        if (auto parent_node = parent(); parent_node != nullptr) {
            m_rotation = rot * glm::inverse(parent_node->world_rotation());
        } else {
            m_rotation = rot;
        }
        set_dirty();
    }

    void set_scale(const glm::vec3& scale) {
        m_scale = scale;
        set_dirty();
    }

    void set_world_scale(const glm::vec3& scale) {
        if (auto parent_node = parent(); parent_node != nullptr) {
            m_scale = scale / parent_node->world_scale();
        } else {
            m_scale = scale;
        }
        set_dirty();
    }

    glm::vec3 position() const { return m_position; }
    glm::quat rotation() const { return m_rotation; }
    glm::vec3 scale() const { return m_scale; }
    glm::vec3 rotation_euler() const { return glm::degrees(glm::eulerAngles(m_rotation)); }
    glm::vec3 up() const { return m_rotation * glm::vec3(0.0f, 1.0f, 0.0f); }
    glm::vec3 down() const { return m_rotation * glm::vec3(0.0f, -1.0f, 0.0f); }
    glm::vec3 right() const { return m_rotation * glm::vec3(1.0f, 0.0f, 0.0f); }
    glm::vec3 left() const { return m_rotation * glm::vec3(-1.0f, 0.0f, 0.0f); }
    glm::vec3 front() const { return m_rotation * glm::vec3(0.0f, 0.0f, 1.0f); }
    glm::vec3 back() const { return m_rotation * glm::vec3(0.0f, 0.0f, -1.0f); }

    glm::vec3 world_up() { return world_rotation() * glm::vec3(0.0f, 1.0f, 0.0f); }
    glm::vec3 world_down() { return world_rotation() * glm::vec3(0.0f, -1.0f, 0.0f); }
    glm::vec3 world_right() { return world_rotation() * glm::vec3(1.0f, 0.0f, 0.0f); }
    glm::vec3 world_left() { return world_rotation() * glm::vec3(-1.0f, 0.0f, 0.0f); }
    glm::vec3 world_front() { return world_rotation() * glm::vec3(0.0f, 0.0f, 1.0f); }
    glm::vec3 world_back() { return world_rotation() * glm::vec3(0.0f, 0.0f, -1.0f); }

    glm::mat4 normal_matrix() { return glm::transpose(glm::inverse(model_matrix())); }

    glm::mat4 model_matrix() {
        if (!m_is_dirty) {
            return m_model_matrix;
        }

        m_is_dirty = false;

        glm::mat4 parent_matrix = glm::identity<glm::mat4>();
        if (auto parent_node = parent(); parent_node != nullptr) {
            parent_matrix = parent_node->model_matrix();
        }

        glm::mat4 transform = glm::identity<glm::mat4>();
        transform = glm::scale(transform, m_scale);
        transform *= glm::mat4_cast(m_rotation);
        transform = glm::translate(glm::identity<glm::mat4>(), m_position) * transform;

        m_model_matrix = parent_matrix * transform;
        return m_model_matrix;
    }

    void set_local_model_matrix(const glm::mat4& local_model_matrix) {
        glm::vec3 local_scale = glm::vec3(
            glm::length(glm::vec3(local_model_matrix[0])),
            glm::length(glm::vec3(local_model_matrix[1])),
            glm::length(glm::vec3(local_model_matrix[2]))
        );

        glm::mat3 rotation_matrix = glm::mat3(local_model_matrix);
        rotation_matrix[0] /= local_scale.x;
        rotation_matrix[1] /= local_scale.y;
        rotation_matrix[2] /= local_scale.z;
        const glm::quat local_rotation = glm::quat_cast(rotation_matrix);
        const glm::vec3 local_position = glm::vec3(local_model_matrix[3]);

        set_position(local_position);
        set_rotation(local_rotation);
        set_scale(local_scale);
        set_dirty();
    }

    void set_dirty() {
        if (m_is_dirty) {
            return;
        }
        m_is_dirty = true;
        for (auto& child : m_children) {
            if (child) {
                child->set_dirty();
            }
        }
    }

    glm::vec3 world_scale() {
        const auto matrix = model_matrix();
        return glm::vec3(
            glm::length(glm::vec3(matrix[0])),
            glm::length(glm::vec3(matrix[1])),
            glm::length(glm::vec3(matrix[2]))
        );
    }

    glm::vec3 world_position() {
        return glm::vec3(model_matrix()[3]);
    }

    glm::quat world_rotation() {
        auto scale_value = world_scale();
        auto matrix = glm::mat3(model_matrix());
        matrix[0] /= scale_value.x;
        matrix[1] /= scale_value.y;
        matrix[2] /= scale_value.z;
        return glm::quat_cast(matrix);
    }

    void look_at_direction(const glm::vec3& target_direction) {
        if (glm::length(target_direction) < kEpsilon) {
            return;
        }

        const glm::vec3 direction = glm::normalize(target_direction);
        const glm::vec3 current_front = front();
        const float cross_len = glm::length(glm::cross(current_front, direction));

        if (cross_len < kEpsilon) {
            if (glm::dot(current_front, direction) < 0.0f) {
                m_rotation = glm::rotate(m_rotation, glm::radians(180.0f), up());
                set_dirty();
            }
            return;
        }

        const glm::quat rotation_quat = glm::rotation(current_front, direction);
        set_rotation(glm::normalize(rotation_quat * m_rotation));
    }

    void look_at_point(const glm::vec3& target_point) {
        look_at_direction(target_point - position());
    }

    void translate(const glm::vec3& direction, float distance) {
        set_position(m_position + direction * distance);
    }

    void rotate(float angle, const glm::vec3& axis) {
        const glm::quat rotation_quat = glm::angleAxis(glm::radians(angle), axis);
        set_rotation(rotation_quat * m_rotation);
    }
};

} // namespace rtr::framework::component
