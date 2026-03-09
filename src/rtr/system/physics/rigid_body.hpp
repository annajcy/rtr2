#pragma once

#include "pbpt/math/basic/type_alias.hpp"
#include "pbpt/math/complex/quaternion.hpp"
#include "pbpt/math/matrix/matrix.hpp"
#include "pbpt/math/spatial/vector.hpp"

namespace rtr::system::physics {

enum class RigidBodyType { Static, Dynamic, Kinematic };

struct TranslationState {
    pbpt::math::Vec3 position{0.0f};
    pbpt::math::Vec3 linear_velocity{0.0f};
};

struct RotationalState {
    pbpt::math::Quat orientation{pbpt::math::Quat::identity()};
    pbpt::math::Vec3 angular_velocity{0.0f};
};

struct ForceAccumulator {
    pbpt::math::Vec3 accumulated_force{0.0f};
    pbpt::math::Vec3 accumulated_torque{0.0f};
};

struct RigidBodyState {
    TranslationState  translation{};
    RotationalState   rotation{};
    ForceAccumulator  forces{};
    pbpt::math::Float mass{1.0f};
};

class RigidBody {
private:
    RigidBodyType      m_type{RigidBodyType::Static};
    RigidBodyState     m_state{};
    bool               m_is_awake{false};
    bool               m_use_gravity{true};
    pbpt::math::Mat3   m_inverse_inertia_tensor_ref{pbpt::math::Mat3::zeros()};
    bool               m_linear_half_step_initialized{false};
    pbpt::math::Vec3   m_half_step_linear_velocity{0.0f};
    bool               m_angular_half_step_initialized{false};
    pbpt::math::Vec3   m_half_step_angular_velocity{0.0f};

public:
    RigidBody() = default;
    RigidBody(RigidBodyType type, const RigidBodyState& state, bool is_awake = true, bool use_gravity = true)
        : m_type(type), m_state(state), m_is_awake(is_awake), m_use_gravity(use_gravity) {}

    void set_awake(bool awake) { m_is_awake = awake; }
    bool is_awake() const { return m_is_awake; }

    void          set_type(RigidBodyType type) { m_type = type; }
    RigidBodyType type() const { return m_type; }

    void set_use_gravity(bool use_gravity) {
        m_use_gravity = use_gravity;
        invalidate_integrator_state();
    }
    bool use_gravity() const { return m_use_gravity; }

    void set_inverse_inertia_tensor_ref(const pbpt::math::Mat3& inverse_inertia_tensor_ref) {
        m_inverse_inertia_tensor_ref = inverse_inertia_tensor_ref;
    }
    const pbpt::math::Mat3& inverse_inertia_tensor_ref() const { return m_inverse_inertia_tensor_ref; }

    const RigidBodyState& state() const { return m_state; }
    RigidBodyState&       state() { return m_state; }

    void clear_forces() {
        m_state.forces.accumulated_force  = pbpt::math::Vec3(0.0f);
        m_state.forces.accumulated_torque = pbpt::math::Vec3(0.0f);
    }

    void clear_accumulated_force() { m_state.forces.accumulated_force = pbpt::math::Vec3(0.0f); }

    void clear_accumulated_torque() { m_state.forces.accumulated_torque = pbpt::math::Vec3(0.0f); }

    void reset_translation_dynamics() {
        m_state.translation.linear_velocity = pbpt::math::Vec3(0.0f);
        clear_accumulated_force();
        invalidate_linear_integrator_state();
    }

    void reset_rotational_dynamics() {
        m_state.rotation.angular_velocity = pbpt::math::Vec3(0.0f);
        clear_accumulated_torque();
        invalidate_angular_integrator_state();
    }

    void reset_dynamics() {
        reset_translation_dynamics();
        reset_rotational_dynamics();
    }

    void invalidate_integrator_state() {
        invalidate_linear_integrator_state();
        invalidate_angular_integrator_state();
    }

    void invalidate_linear_integrator_state() {
        m_half_step_linear_velocity     = pbpt::math::Vec3(0.0f);
        m_linear_half_step_initialized  = false;
    }

    void invalidate_angular_integrator_state() {
        m_half_step_angular_velocity    = pbpt::math::Vec3(0.0f);
        m_angular_half_step_initialized = false;
    }

    bool linear_half_step_initialized() const { return m_linear_half_step_initialized; }

    void initialize_half_step_linear_velocity(const pbpt::math::Vec3& linear_velocity) {
        m_half_step_linear_velocity   = linear_velocity;
        m_linear_half_step_initialized = true;
    }

    const pbpt::math::Vec3& half_step_linear_velocity() const { return m_half_step_linear_velocity; }
    pbpt::math::Vec3&       half_step_linear_velocity() { return m_half_step_linear_velocity; }

    bool angular_half_step_initialized() const { return m_angular_half_step_initialized; }

    void initialize_half_step_angular_velocity(const pbpt::math::Vec3& angular_velocity) {
        m_half_step_angular_velocity   = angular_velocity;
        m_angular_half_step_initialized = true;
    }

    const pbpt::math::Vec3& half_step_angular_velocity() const { return m_half_step_angular_velocity; }
    pbpt::math::Vec3&       half_step_angular_velocity() { return m_half_step_angular_velocity; }
};

}  // namespace rtr::system::physics
