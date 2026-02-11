#pragma once

#include <algorithm>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>

#include "framework/component/component.hpp"
#include "framework/core/types.hpp"

namespace rtr::framework::core {
class Scene;
}

namespace rtr::framework::component {

class NodeComponent final : public Component {
private:
    glm::vec3 m_local_position{0.0f, 0.0f, 0.0f};
    glm::quat m_local_rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 m_local_scale{1.0f, 1.0f, 1.0f};

    glm::mat4 m_world_matrix{1.0f};
    core::GameObjectId m_parent_id{core::kInvalidGameObjectId};
    std::vector<core::GameObjectId> m_children{};
    bool m_dirty{true};

public:
    const glm::vec3& local_position() const { return m_local_position; }
    const glm::quat& local_rotation() const { return m_local_rotation; }
    const glm::vec3& local_scale() const { return m_local_scale; }

    void set_local_position(const glm::vec3& value) {
        m_local_position = value;
        m_dirty = true;
    }

    void set_local_rotation(const glm::quat& value) {
        m_local_rotation = value;
        m_dirty = true;
    }

    void set_local_scale(const glm::vec3& value) {
        m_local_scale = value;
        m_dirty = true;
    }

    glm::mat4 local_matrix() const {
        glm::mat4 transform = glm::translate(glm::mat4(1.0f), m_local_position);
        transform = transform * glm::mat4_cast(m_local_rotation);
        transform = glm::scale(transform, m_local_scale);
        return transform;
    }

    const glm::mat4& world_matrix() const {
        return m_world_matrix;
    }

    core::GameObjectId parent_id() const {
        return m_parent_id;
    }

    const std::vector<core::GameObjectId>& children() const {
        return m_children;
    }

    bool dirty() const {
        return m_dirty;
    }

    void mark_dirty() {
        m_dirty = true;
    }

private:
    void set_parent_id_internal(core::GameObjectId parent_id) {
        m_parent_id = parent_id;
        m_dirty = true;
    }

    bool add_child_internal(core::GameObjectId child_id) {
        if (std::find(m_children.begin(), m_children.end(), child_id) != m_children.end()) {
            return false;
        }
        m_children.emplace_back(child_id);
        return true;
    }

    bool remove_child_internal(core::GameObjectId child_id) {
        const auto it = std::remove(m_children.begin(), m_children.end(), child_id);
        if (it == m_children.end()) {
            return false;
        }
        m_children.erase(it, m_children.end());
        return true;
    }

    void set_world_matrix_internal(const glm::mat4& world) {
        m_world_matrix = world;
        m_dirty = false;
    }

    friend class rtr::framework::core::Scene;
};

} // namespace rtr::framework::component
