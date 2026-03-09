#pragma once

#include <cmath>
#include <stdexcept>

#include "pbpt/math/basic/type_alias.hpp"
#include "pbpt/math/complex/quaternion.hpp"
#include "pbpt/math/matrix/matrix.hpp"
#include "pbpt/math/spatial/vector.hpp"
#include "rtr/system/physics/type.hpp"

namespace rtr::system::physics {

enum class RigidBodyType { Static, Dynamic, Kinematic };

struct TranslationState {
    pbpt::math::Vec3 position{0.0f};
    pbpt::math::Vec3 linear_velocity{0.0f};
};

struct RotationState {
    pbpt::math::Quat orientation{pbpt::math::Quat::identity()};
    pbpt::math::Vec3 angular_velocity{0.0f};
};

struct ForceAccumulator {
    pbpt::math::Vec3 accumulated_force{0.0f};
    pbpt::math::Vec3 accumulated_torque{0.0f};
};

struct RigidBodyState {
    TranslationState  translation{};
    RotationState   rotation{};
    ForceAccumulator  forces{};
    pbpt::math::Vec3 scale{1.0f};
    pbpt::math::Float mass{1.0f};

    bool is_mass_valid() const { return mass > 0.0f && std::isfinite(mass); }
    pbpt::math::Float inverse_mass() const { return is_mass_valid() ? 1.0f / mass : 0.0f; }
    PhysicsTransform to_transform() const {
        return PhysicsTransform{
            .position = translation.position,
            .rotation = rotation.orientation,
            .scale = scale
        };
    }
};

class RigidBody {
private:
    RigidBodyType      m_type{RigidBodyType::Static};
    RigidBodyState     m_state{};
    bool               m_is_awake{false};
    bool               m_use_gravity{true};
    pbpt::math::Float  m_restitution{0.0f};
    pbpt::math::Float  m_friction{0.0f};
    pbpt::math::Mat3   m_inverse_inertia_tensor_ref{pbpt::math::Mat3::zeros()};

    static pbpt::math::Float sanitize_restitution(pbpt::math::Float restitution) {
        if (!std::isfinite(restitution) || restitution < 0.0f || restitution > 1.0f) {
            throw std::invalid_argument("RigidBody restitution must be finite and within [0, 1].");
        }
        return restitution;
    }

    static pbpt::math::Float sanitize_friction(pbpt::math::Float friction) {
        if (!std::isfinite(friction) || friction < 0.0f) {
            throw std::invalid_argument("RigidBody friction must be finite and non-negative.");
        }
        return friction;
    }

public:
    RigidBody() = default;
    RigidBody(RigidBodyType type, const RigidBodyState& state, bool is_awake = true, bool use_gravity = true,
              pbpt::math::Float restitution = 0.0f, pbpt::math::Float friction = 0.0f)
        : m_type(type),
          m_state(state),
          m_is_awake(is_awake),
          m_use_gravity(use_gravity),
          m_restitution(sanitize_restitution(restitution)),
          m_friction(sanitize_friction(friction)) {}

    void set_awake(bool awake) { m_is_awake = awake; }
    bool is_awake() const { return m_is_awake; }

    void          set_type(RigidBodyType type) { m_type = type; }
    RigidBodyType type() const { return m_type; }

    void set_use_gravity(bool use_gravity) {
        m_use_gravity = use_gravity;
    }

    bool use_gravity() const { return m_use_gravity; }

    void set_restitution(pbpt::math::Float restitution) { m_restitution = sanitize_restitution(restitution); }
    pbpt::math::Float restitution() const { return m_restitution; }

    void set_friction(pbpt::math::Float friction) { m_friction = sanitize_friction(friction); }
    pbpt::math::Float friction() const { return m_friction; }

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
    }

    void reset_rotational_dynamics() {
        m_state.rotation.angular_velocity = pbpt::math::Vec3(0.0f);
        clear_accumulated_torque();
    }

    void reset_dynamics() {
        reset_translation_dynamics();
        reset_rotational_dynamics();
    }
};

}  // namespace rtr::system::physics
