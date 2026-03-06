#pragma once

#include "pbpt/math/basic/type_alias.hpp"
#include "pbpt/math/complex/quaternion.hpp"
#include "pbpt/math/spatial/vector.hpp"
#include "rtr/system/physics/collider.hpp"

namespace rtr::system::physics {

enum class RigidBodyType { Static, Dynamic, Kinematic };

struct TranslationState {
    pbpt::math::Vec3 position;
    pbpt::math::Vec3 linear_velocity;
    pbpt::math::Vec3 force;
};

struct RigidBodyState {
    TranslationState translation_state;
    pbpt::math::Float mass;
};

class RigidBody {
private:
    RigidBodyType  m_type;
    RigidBodyState m_state;
    bool           m_is_awake;

    std::vector<Collider> m_colliders;

public:
    RigidBody() : m_type(RigidBodyType::Static), m_state{}, m_is_awake(false) {}
    RigidBody(RigidBodyType type, const RigidBodyState& state, const std::vector<Collider>& colliders = {},
              bool is_awake = true)
        : m_type(type), m_state(state), m_colliders(colliders), m_is_awake(is_awake) {}

    void set_awake(bool awake) { m_is_awake = awake; }
    bool is_awake() const { return m_is_awake; }

    void          set_type(RigidBodyType type) { m_type = type; }
    RigidBodyType type() const { return m_type; }

    const RigidBodyState& state() const { return m_state; }
    RigidBodyState&       state() { return m_state; }

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
