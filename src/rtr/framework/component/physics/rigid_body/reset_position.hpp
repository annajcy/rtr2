#pragma once

#include <cmath>
#include <stdexcept>

#include <pbpt/math/math.h>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/core/game_object.hpp"

namespace rtr::framework::component {

class ResetPosition final : public Component {
private:
    RigidBody*        m_rigid_body{nullptr};
    float             m_threshold_y{-1.0f};
    pbpt::math::Vec3  m_reset_position{0.0f, 2.0f, 0.0f};

    static float sanitize_threshold(float threshold_y) {
        if (!std::isfinite(threshold_y)) {
            throw std::invalid_argument("ResetPosition threshold_y must be finite.");
        }
        return threshold_y;
    }

    static pbpt::math::Vec3 sanitize_reset_position(const pbpt::math::Vec3& reset_position) {
        if (!std::isfinite(reset_position.x()) || !std::isfinite(reset_position.y()) ||
            !std::isfinite(reset_position.z())) {
            throw std::invalid_argument("ResetPosition reset_position must be finite.");
        }
        return reset_position;
    }

public:
    explicit ResetPosition(core::GameObject& owner) : Component(owner) {}

    void on_awake() override {}

    void on_enable() override { m_rigid_body = &owner().component_or_throw<RigidBody>(); }

    void on_fixed_update(const core::FixedTickContext& /*ctx*/) override {
        if (m_rigid_body == nullptr || !m_rigid_body->has_rigid_body()) {
            return;
        }

        if (m_rigid_body->position().y() <= m_threshold_y) {
            m_rigid_body->set_position(m_reset_position);
            m_rigid_body->reset_translation_dynamics();
        }
    }

    float threshold_y() const { return m_threshold_y; }
    void  set_threshold_y(float threshold_y) { m_threshold_y = sanitize_threshold(threshold_y); }

    pbpt::math::Vec3 reset_position() const { return m_reset_position; }
    void set_reset_position(const pbpt::math::Vec3& reset_position) {
        m_reset_position = sanitize_reset_position(reset_position);
    }
};

}  // namespace rtr::framework::component
