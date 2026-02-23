#pragma once

#include <pbpt/math/math.h>

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
    explicit Camera(core::GameObject& owner)
        : Component(owner) {}

    const core::GameObject& require_owner() const { return owner(); }
    core::GameObject& require_owner() { return owner(); }

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

}  // namespace rtr::framework::component

#include "rtr/framework/component/camera/orthographic_camera.hpp"
#include "rtr/framework/component/camera/perspective_camera.hpp"
