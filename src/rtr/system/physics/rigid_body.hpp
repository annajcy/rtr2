#pragma once

#include <utility>
#include <vector>

#include "pbpt/math/basic/type_alias.hpp"
#include "pbpt/math/spatial/vector.hpp"
#include "rtr/system/physics/collider.hpp"

namespace rtr::system::physics {

enum class RigidBodyType { Static, Dynamic, Kinematic };

struct TranslationState {
    pbpt::math::Vec3 position{0.0f};
    pbpt::math::Vec3 linear_velocity{0.0f};
};

struct ForceAccumulator {
    pbpt::math::Vec3 accumulated_force{0.0f};
};

struct RigidBodyState {
    TranslationState  translation{};
    ForceAccumulator  forces{};
    pbpt::math::Float mass{1.0f};
};

class RigidBody {
private:
    RigidBodyType      m_type{RigidBodyType::Static};
    RigidBodyState     m_state{};
    bool               m_is_awake{false};
    bool               m_use_gravity{true};
    bool               m_half_step_initialized{false};
    pbpt::math::Vec3   m_half_step_linear_velocity{0.0f};
    std::vector<Collider> m_colliders{};

public:
    RigidBody() = default;
    RigidBody(RigidBodyType type, const RigidBodyState& state, const std::vector<Collider>& colliders = {},
              bool is_awake = true, bool use_gravity = true)
        : m_type(type), m_state(state), m_is_awake(is_awake), m_use_gravity(use_gravity), m_colliders(colliders) {}

    void set_awake(bool awake) { m_is_awake = awake; }
    bool is_awake() const { return m_is_awake; }

    void          set_type(RigidBodyType type) { m_type = type; }
    RigidBodyType type() const { return m_type; }

    void set_use_gravity(bool use_gravity) {
        m_use_gravity = use_gravity;
        invalidate_integrator_state();
    }
    bool use_gravity() const { return m_use_gravity; }

    const RigidBodyState& state() const { return m_state; }
    RigidBodyState&       state() { return m_state; }

    void clear_forces() { m_state.forces.accumulated_force = pbpt::math::Vec3(0.0f); }

    void reset_dynamics() {
        m_state.translation.linear_velocity = pbpt::math::Vec3(0.0f);
        clear_forces();
        invalidate_integrator_state();
    }

    void invalidate_integrator_state() {
        m_half_step_linear_velocity = pbpt::math::Vec3(0.0f);
        m_half_step_initialized     = false;
    }

    bool half_step_initialized() const { return m_half_step_initialized; }

    void initialize_half_step_linear_velocity(const pbpt::math::Vec3& linear_velocity) {
        m_half_step_linear_velocity = linear_velocity;
        m_half_step_initialized     = true;
    }

    const pbpt::math::Vec3& half_step_linear_velocity() const { return m_half_step_linear_velocity; }
    pbpt::math::Vec3&       half_step_linear_velocity() { return m_half_step_linear_velocity; }

    template <typename... Args>
    RigidBody& emplace_collider(Args&&... args) {
        m_colliders.emplace_back(std::forward<Args>(args)...);
        return *this;
    }

    RigidBody& add_collider(const Collider& collider) {
        m_colliders.push_back(collider);
        return *this;
    }

    const std::vector<Collider>& colliders() const { return m_colliders; }
    std::vector<Collider>&       colliders() { return m_colliders; }
};

}  // namespace rtr::system::physics
