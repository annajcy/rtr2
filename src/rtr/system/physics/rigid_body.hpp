#pragma once

#include "pbpt/math/basic/type_alias.hpp"
#include "pbpt/math/complex/quaternion.hpp"
#include "pbpt/math/spatial/vector.hpp"
#include "rtr/system/physics/collider.hpp"

namespace rtr::system::physics {

enum class RigidBodyType {
    Static,
    Dynamic,
    Kinematic   
};

struct RigidBodyState {
    pbpt::math::Vec3 position;
    pbpt::math::Quat orientation;
    
    pbpt::math::Vec3 linear_velocity;
    pbpt::math::Vec3 angular_velocity;

    pbpt::math::Vec3 force;
    pbpt::math::Vec3 torque;

    pbpt::math::Float mass;
    pbpt::math::Float inverse_mass;

    pbpt::math::Float friction;
    pbpt::math::Float restitution;
};

class RigidBody {
private:
    RigidBodyType m_type;
    RigidBodyState m_state;
    bool m_is_awake;

    std::vector<Colider> m_colliders;

public:
    RigidBody() : m_type(RigidBodyType::Static), m_state{}, m_is_awake(false) {}
    RigidBody(RigidBodyType type, const RigidBodyState& state, const std::vector<Colider>& colliders = {}, bool is_awake = true) : m_type(type), m_state(state), m_colliders(colliders), m_is_awake(is_awake) {}  

    void set_awake(bool awake) { m_is_awake = awake; }
    bool is_awake() const { return m_is_awake; }

    void set_type(RigidBodyType type) { m_type = type; }
    RigidBodyType type() const { return m_type; }

    const RigidBodyState& state() const { return m_state; }
    RigidBodyState& state() { return m_state; }

    template<typename... Args>
    RigidBody& emplace_collider(Args&&... args) {
        m_colliders.emplace_back(std::forward<Args>(args)...);
        return *this;
    }

    RigidBody& add_collider(const Colider& collider) {
        m_colliders.push_back(collider);
        return *this;
    }

    const std::vector<Colider>& colliders() const { return m_colliders; }
    std::vector<Colider>& colliders() { return m_colliders; }
};


}

