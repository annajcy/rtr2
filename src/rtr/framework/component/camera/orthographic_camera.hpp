#pragma once

#include <algorithm>

#include "rtr/framework/component/camera/camera.hpp"

namespace rtr::framework::component {

class OrthographicCamera final : public Camera {
private:
    float m_left_bound{-5.0f};
    float m_right_bound{5.0f};
    float m_bottom_bound{-5.0f};
    float m_top_bound{5.0f};

public:
    explicit OrthographicCamera(core::GameObject& owner)
        : Camera(owner) {
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
