#pragma once

#include "rtr/framework/component/camera/camera.hpp"

namespace rtr::framework::component {

class PerspectiveCamera final : public Camera {
private:
    float m_fov_degrees{45.0f};
    float m_aspect_ratio{16.0f / 9.0f};

public:
    explicit PerspectiveCamera(core::GameObject& owner)
        : Camera(owner) {}

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

}  // namespace rtr::framework::component
