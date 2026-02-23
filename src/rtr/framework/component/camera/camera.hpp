#pragma once

#include <pbpt/math/math.h>

#include <algorithm>
#include <stdexcept>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/core/game_object.hpp"

namespace rtr::framework::component {

class Camera : public Component {
private:
    bool  m_active{false};
    float m_near_bound{0.1f};
    float m_far_bound{100.0f};

protected:
    const core::GameObject& require_owner() const {
        const auto* go = owner();
        if (go == nullptr) {
            throw std::runtime_error("Camera owner is null.");
        }
        return *go;
    }

    core::GameObject& require_owner() {
        auto* go = owner();
        if (go == nullptr) {
            throw std::runtime_error("Camera owner is null.");
        }
        return *go;
    }

public:
    virtual ~Camera() = default;

    bool active() const { return m_active; }
    void set_active(bool active) { m_active = active; }

    float& near_bound() { return m_near_bound; }
    const float& near_bound() const { return m_near_bound; }

    float& far_bound() { return m_far_bound; }
    const float& far_bound() const { return m_far_bound; }

    pbpt::math::vec3 camera_world_front() const { return require_owner().node().world_back(); }
    pbpt::math::vec3 camera_world_back() const { return require_owner().node().world_front(); }
    pbpt::math::vec3 camera_local_front() const { return require_owner().node().local_back(); }
    pbpt::math::vec3 camera_local_back() const { return require_owner().node().local_front(); }

    void camera_look_at_direction_local(const pbpt::math::vec3& target_dir_local) {
        require_owner().node().look_at_direction_local(-target_dir_local);
    }

    void camera_look_at_direction_world(const pbpt::math::vec3& target_dir_world) {
        require_owner().node().look_at_direction_world(-target_dir_world);
    }

    void camera_look_at_point_local(const pbpt::math::vec3& target_pos_local) {
        auto node = require_owner().node();
        const auto to_target_local = target_pos_local - node.local_position();
        node.look_at_direction_local(-to_target_local);
    }

    void camera_look_at_point_world(const pbpt::math::vec3& target_pos_world) {
        auto node = require_owner().node();
        const auto to_target_world = target_pos_world - node.world_position();
        node.look_at_direction_world(-to_target_world);
    }

    pbpt::math::mat4 view_matrix() const { return pbpt::math::inverse(require_owner().node().world_matrix()); }

    virtual pbpt::math::mat4 projection_matrix() const = 0;
    virtual void adjust_zoom(float delta_zoom) = 0;

    void on_awake() override {
        auto& go = require_owner();
        if (go.get_component<Camera>() != nullptr) {
            throw std::runtime_error("GameObject already has a camera component.");
        }
    }
};

class PerspectiveCamera final : public Camera {
private:
    float m_fov_degrees{45.0f};
    float m_aspect_ratio{16.0f / 9.0f};

public:
    float& fov_degrees() { return m_fov_degrees; }
    const float& fov_degrees() const { return m_fov_degrees; }

    float& aspect_ratio() { return m_aspect_ratio; }
    const float& aspect_ratio() const { return m_aspect_ratio; }

    pbpt::math::mat4 projection_matrix() const override {
        return pbpt::math::perspective(pbpt::math::radians(m_fov_degrees), m_aspect_ratio, near_bound(), far_bound());
    }

    void adjust_zoom(float delta_zoom) override {
        auto node = require_owner().node();
        node.set_world_position(node.world_position() + camera_world_front() * delta_zoom);
    }
};

class OrthographicCamera final : public Camera {
private:
    float m_left_bound{-5.0f};
    float m_right_bound{5.0f};
    float m_bottom_bound{-5.0f};
    float m_top_bound{5.0f};

public:
    OrthographicCamera() {
        near_bound() = -5.0f;
        far_bound()  = 5.0f;
    }

    float& left_bound() { return m_left_bound; }
    const float& left_bound() const { return m_left_bound; }

    float& right_bound() { return m_right_bound; }
    const float& right_bound() const { return m_right_bound; }

    float& bottom_bound() { return m_bottom_bound; }
    const float& bottom_bound() const { return m_bottom_bound; }

    float& top_bound() { return m_top_bound; }
    const float& top_bound() const { return m_top_bound; }

    pbpt::math::mat4 projection_matrix() const override {
        return pbpt::math::ortho(m_left_bound, m_right_bound, m_bottom_bound, m_top_bound, near_bound(), far_bound());
    }

    void adjust_zoom(float delta_zoom) override {
        const pbpt::math::vec2 center = {(m_left_bound + m_right_bound) * 0.5f, (m_bottom_bound + m_top_bound) * 0.5f};
        const float half_w = std::max((m_right_bound - m_left_bound) * 0.5f + delta_zoom, 0.01f);
        const float half_h = std::max((m_top_bound - m_bottom_bound) * 0.5f + delta_zoom, 0.01f);
        m_left_bound       = center.x() - half_w;
        m_right_bound      = center.x() + half_w;
        m_bottom_bound     = center.y() - half_h;
        m_top_bound        = center.y() + half_h;
    }
};

}  // namespace rtr::framework::component
