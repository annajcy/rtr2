#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/component/physics/rigid_body_component.hpp"
#include "rtr/framework/core/game_object.hpp"

namespace rtr::framework::component {

class PingPong final : public Component {
private:
    static constexpr float kVelocityEpsilon = 1e-5f;

    RigidBody* m_rigid_body{nullptr};

    float m_min_x{-2.0f};
    float m_max_x{2.0f};
    float m_speed{1.0f};
    bool  m_start_positive{true};

    static float sanitize_speed(float speed) {
        if (!std::isfinite(speed)) {
            throw std::invalid_argument("PingPongComponent speed must be finite.");
        }
        return std::abs(speed);
    }

    void normalize_bounds() {
        if (m_min_x > m_max_x) {
            std::swap(m_min_x, m_max_x);
        }
    }

public:
    explicit PingPong(core::GameObject& owner) : Component(owner) {}

    void on_awake() override {}

    void on_enable() override {
        m_rigid_body = &owner().component_or_throw<RigidBody>();
        apply_start_direction();
    }

    void on_fixed_update(const core::FixedTickContext& /*ctx*/) override {
        if (m_rigid_body == nullptr || !m_rigid_body->has_rigid_body()) {
            return;
        }

        pbpt::math::Vec3 position  = m_rigid_body->position();
        pbpt::math::Vec3 velocity  = m_rigid_body->linear_velocity();
        const float      abs_speed = std::abs(m_speed);

        if (position.x() >= m_max_x && velocity.x() > 0.0f) {
            position.x() = m_max_x;
            velocity.x() = -abs_speed;
        } else if (position.x() <= m_min_x && velocity.x() < 0.0f) {
            position.x() = m_min_x;
            velocity.x() = abs_speed;
        }

        if (std::abs(velocity.x()) <= kVelocityEpsilon) {
            velocity.x() = m_start_positive ? abs_speed : -abs_speed;
        }

        velocity.y() = 0.0f;
        velocity.z() = 0.0f;

        m_rigid_body->set_position(position);
        m_rigid_body->set_linear_velocity(velocity);
    }

    float min_x() const { return m_min_x; }
    float max_x() const { return m_max_x; }
    float speed() const { return m_speed; }
    bool  start_positive() const { return m_start_positive; }

    void set_min_x(float min_x) {
        m_min_x = min_x;
        normalize_bounds();
    }

    void set_max_x(float max_x) {
        m_max_x = max_x;
        normalize_bounds();
    }

    void set_bounds(float min_x, float max_x) {
        m_min_x = min_x;
        m_max_x = max_x;
        normalize_bounds();
    }

    void set_speed(float speed) { m_speed = sanitize_speed(speed); }

    void set_start_positive(bool start_positive) { m_start_positive = start_positive; }

    void apply_start_direction() {
        if (m_rigid_body == nullptr || !m_rigid_body->has_rigid_body()) {
            return;
        }
        pbpt::math::Vec3 velocity = m_rigid_body->linear_velocity();
        velocity.x()              = m_start_positive ? std::abs(m_speed) : -std::abs(m_speed);
        velocity.y()              = 0.0f;
        velocity.z()              = 0.0f;
        m_rigid_body->set_linear_velocity(velocity);
    }
};

}  // namespace rtr::framework::component
