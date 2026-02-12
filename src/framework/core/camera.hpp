#pragma once

#include <algorithm>
#include <stdexcept>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>

#include "framework/core/scene_graph.hpp"
#include "framework/core/types.hpp"

namespace rtr::framework::core {

enum class CameraType {
    Perspective,
    Orthographic,
};

class CameraBase {
protected:
    GameObjectId m_owner_id{core::kInvalidGameObjectId};
    SceneGraph* m_scene_graph{nullptr};
    float m_near_bound{0.1f};
    float m_far_bound{100.0f};
    CameraType m_camera_type{};

    explicit CameraBase(
        CameraType camera_type,
        GameObjectId owner_id = core::kInvalidGameObjectId,
        SceneGraph* scene_graph = nullptr
    )
        : m_owner_id(owner_id), m_scene_graph(scene_graph), m_camera_type(camera_type) {}

public:
    virtual ~CameraBase() = default;

    GameObjectId owner_id() const {
        return m_owner_id;
    }

    void bind(GameObjectId owner_id, SceneGraph* scene_graph) {
        m_owner_id = owner_id;
        m_scene_graph = scene_graph;
    }

    float& near_bound() {
        return m_near_bound;
    }

    const float& near_bound() const {
        return m_near_bound;
    }

    float& far_bound() {
        return m_far_bound;
    }

    const float& far_bound() const {
        return m_far_bound;
    }

    CameraType camera_type() const {
        return m_camera_type;
    }

    SceneGraph::NodeView node() {
        if (m_scene_graph == nullptr ||
            m_owner_id == core::kInvalidGameObjectId ||
            !m_scene_graph->has_node(m_owner_id)) {
            throw std::runtime_error("Camera is not bound to a valid SceneGraph node.");
        }
        return m_scene_graph->node(m_owner_id);
    }

    SceneGraph::ConstNodeView node() const {
        if (m_scene_graph == nullptr ||
            m_owner_id == core::kInvalidGameObjectId ||
            !m_scene_graph->has_node(m_owner_id)) {
            throw std::runtime_error("Camera is not bound to a valid SceneGraph node.");
        }
        return m_scene_graph->node(m_owner_id);
    }

    glm::vec3 front() const {
        return node().world_front();
    }

    glm::mat4 view_matrix() const {
        const auto node_view = node();
        const glm::vec3 eye = node_view.world_position();
        const glm::vec3 center = eye + node_view.world_front();
        return glm::lookAt(eye, center, node_view.world_up());
    }

    virtual glm::mat4 projection_matrix() const = 0;
    virtual void adjust_zoom(float delta_zoom) = 0;
    virtual void set_aspect_ratio(float aspect_ratio) = 0;
};

class PerspectiveCamera final : public CameraBase {
private:
    float m_fov_degrees{45.0f};
    float m_aspect_ratio{16.0f / 9.0f};

public:
    explicit PerspectiveCamera(
        GameObjectId owner_id = core::kInvalidGameObjectId,
        SceneGraph* scene_graph = nullptr
    )
        : CameraBase(CameraType::Perspective, owner_id, scene_graph) {}

    float& fov_degrees() {
        return m_fov_degrees;
    }

    const float& fov_degrees() const {
        return m_fov_degrees;
    }

    float& aspect_ratio() {
        return m_aspect_ratio;
    }

    const float& aspect_ratio() const {
        return m_aspect_ratio;
    }

    glm::mat4 projection_matrix() const override {
        return glm::perspective(
            glm::radians(m_fov_degrees),
            m_aspect_ratio,
            m_near_bound,
            m_far_bound
        );
    }

    void adjust_zoom(float delta_zoom) override {
        auto node_view = node();
        node_view.set_world_position(
            node_view.world_position() + node_view.world_front() * delta_zoom
        );
    }

    void set_aspect_ratio(float aspect_ratio) override {
        if (aspect_ratio > 0.0f) {
            m_aspect_ratio = aspect_ratio;
        }
    }
};

class OrthographicCamera final : public CameraBase {
private:
    float m_left_bound{-5.0f};
    float m_right_bound{5.0f};
    float m_bottom_bound{-5.0f};
    float m_top_bound{5.0f};

public:
    explicit OrthographicCamera(
        GameObjectId owner_id = core::kInvalidGameObjectId,
        SceneGraph* scene_graph = nullptr
    )
        : CameraBase(CameraType::Orthographic, owner_id, scene_graph) {
        m_near_bound = -5.0f;
        m_far_bound = 5.0f;
    }

    float& left_bound() {
        return m_left_bound;
    }

    const float& left_bound() const {
        return m_left_bound;
    }

    float& right_bound() {
        return m_right_bound;
    }

    const float& right_bound() const {
        return m_right_bound;
    }

    float& bottom_bound() {
        return m_bottom_bound;
    }

    const float& bottom_bound() const {
        return m_bottom_bound;
    }

    float& top_bound() {
        return m_top_bound;
    }

    const float& top_bound() const {
        return m_top_bound;
    }

    void set_orthographic_size(float size) {
        const float clamped_size = std::max(size, 0.01f);
        const float half = clamped_size * 0.5f;
        m_left_bound = -half;
        m_right_bound = half;
        m_bottom_bound = -half;
        m_top_bound = half;
        m_near_bound = -half;
        m_far_bound = half;
    }

    glm::mat4 projection_matrix() const override {
        return glm::ortho(
            m_left_bound,
            m_right_bound,
            m_bottom_bound,
            m_top_bound,
            m_near_bound,
            m_far_bound
        );
    }

    void adjust_zoom(float delta_zoom) override {
        const glm::vec2 center = {
            (m_left_bound + m_right_bound) * 0.5f,
            (m_bottom_bound + m_top_bound) * 0.5f
        };
        const float half_w = std::max((m_right_bound - m_left_bound) * 0.5f + delta_zoom, 0.01f);
        const float half_h = std::max((m_top_bound - m_bottom_bound) * 0.5f + delta_zoom, 0.01f);
        m_left_bound = center.x - half_w;
        m_right_bound = center.x + half_w;
        m_bottom_bound = center.y - half_h;
        m_top_bound = center.y + half_h;
    }

    void set_aspect_ratio(float aspect_ratio) override {
        if (aspect_ratio <= 0.0f) {
            return;
        }
        const glm::vec2 center = {
            (m_left_bound + m_right_bound) * 0.5f,
            (m_bottom_bound + m_top_bound) * 0.5f
        };
        const float half_h = (m_top_bound - m_bottom_bound) * 0.5f;
        const float half_w = std::max(half_h * aspect_ratio, 0.01f);
        m_left_bound = center.x - half_w;
        m_right_bound = center.x + half_w;
        m_bottom_bound = center.y - half_h;
        m_top_bound = center.y + half_h;
    }
};

} // namespace rtr::framework::core
