#pragma once

#include <cmath>

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

    pbpt::math::Mat4 projection_matrix() const override {
        const float near = std::abs(near_bound());
        const float far = std::abs(far_bound());
        const float tan_half = std::tan(pbpt::math::radians(m_fov_degrees) * 0.5f);

        pbpt::math::Mat4 projection{};
        projection[0][0] = 1.0f / (m_aspect_ratio * tan_half);
        projection[1][1] = 1.0f / tan_half;
        projection[2][2] = -(far + near) / (far - near);
        projection[2][3] = -(2.0f * far * near) / (far - near);
        projection[3][2] = -1.0f;
        return projection;
    }

    void adjust_zoom(float delta_zoom) override {
        auto node = require_owner().node();
        node.set_world_position(node.world_position() + camera_world_front() * delta_zoom);
    }
};

}  // namespace rtr::framework::component
