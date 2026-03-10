#pragma once

#include <pbpt/math/math.h>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/component/physics/rigid_body.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/system/input/input_state.hpp"
#include "rtr/system/input/input_types.hpp"

namespace rtr::examples::games103_lab::lab1_angry_bunny {

class AngryBunnyController final : public framework::component::Component {
private:
    framework::component::RigidBody* m_rigid_body{nullptr};
    const system::input::InputState* m_input_state{nullptr};
    bool                             m_launched{false};

    pbpt::math::Vec3 m_reset_position{0.0f, 0.6f, 0.0f};
    pbpt::math::Quat m_reset_orientation{pbpt::math::Quat::identity()};
    pbpt::math::Vec3 m_launch_linear_velocity{3.5f, 5.0f, 0.0f};
    pbpt::math::Vec3 m_initial_angular_velocity{0.0f, 4.5f, 0.0f};

    void apply_waiting_state() {
        if (m_rigid_body == nullptr) {
            return;
        }
        m_rigid_body->set_position(m_reset_position);
        m_rigid_body->set_linear_velocity(pbpt::math::Vec3{0.0f});
        m_rigid_body->set_use_gravity(false);
        m_rigid_body->set_angular_velocity(m_initial_angular_velocity);
    }

    void launch() {
        if (m_rigid_body == nullptr) {
            return;
        }
        m_rigid_body->set_position(m_reset_position);
        m_rigid_body->set_linear_velocity(m_launch_linear_velocity);
        m_rigid_body->set_angular_velocity(m_initial_angular_velocity);
        m_rigid_body->set_use_gravity(true);
        m_launched = true;
    }

    void reset() {
        if (m_rigid_body == nullptr) {
            return;
        }
        m_rigid_body->set_position(m_reset_position);
        m_rigid_body->set_orientation(m_reset_orientation);
        m_rigid_body->reset_dynamics();
        m_rigid_body->set_use_gravity(false);
        m_rigid_body->set_linear_velocity(pbpt::math::Vec3{0.0f});
        m_rigid_body->set_angular_velocity(m_initial_angular_velocity);
        m_launched = false;
    }

public:
    explicit AngryBunnyController(framework::core::GameObject& owner,
                                  const system::input::InputState& input_state,
                                  const pbpt::math::Vec3& launch_linear_velocity = pbpt::math::Vec3{3.5f, 5.0f, 0.0f},
                                  const pbpt::math::Vec3& initial_angular_velocity = pbpt::math::Vec3{0.0f, 4.5f, 0.0f},
                                  const pbpt::math::Vec3& reset_position = pbpt::math::Vec3{0.0f, 0.6f, 0.0f})
        : Component(owner),
          m_input_state(&input_state),
          m_reset_position(reset_position),
          m_launch_linear_velocity(launch_linear_velocity),
          m_initial_angular_velocity(initial_angular_velocity) {}

    void on_enable() override {
        m_rigid_body = &owner().component_or_throw<framework::component::RigidBody>();
        m_reset_orientation = m_rigid_body->orientation();
        apply_waiting_state();
    }

    void on_fixed_update(const framework::core::FixedTickContext& /*ctx*/) override {
        if (m_rigid_body == nullptr || m_input_state == nullptr) {
            return;
        }

        const bool launch_pressed = m_input_state->key_action(system::input::KeyCode::L) ==
                                    system::input::KeyAction::PRESS;
        const bool reset_pressed = m_input_state->key_action(system::input::KeyCode::R) ==
                                   system::input::KeyAction::PRESS;

        if (reset_pressed) {
            reset();
        } else if (launch_pressed && !m_launched) {
            launch();
        } else if (!m_launched) {
            apply_waiting_state();
        }
    }

    bool launched() const { return m_launched; }
    const pbpt::math::Vec3& reset_position() const { return m_reset_position; }
    const pbpt::math::Quat& reset_orientation() const { return m_reset_orientation; }
    const pbpt::math::Vec3& launch_linear_velocity() const { return m_launch_linear_velocity; }
    const pbpt::math::Vec3& initial_angular_velocity() const { return m_initial_angular_velocity; }
};

}  // namespace rtr::examples::games103_lab::lab1_angry_bunny
