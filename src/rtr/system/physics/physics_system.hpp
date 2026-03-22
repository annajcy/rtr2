#pragma once

#include "rtr/system/physics/rigid_body/rigid_body_world.hpp"

namespace rtr::system::physics {

class PhysicsSystem {
private:
    RigidBodyWorld m_rigid_body_world{};

public:
    RigidBodyWorld& rigid_body_world() { return m_rigid_body_world; }
    const RigidBodyWorld& rigid_body_world() const { return m_rigid_body_world; }

    void step(float delta_seconds) {
        m_rigid_body_world.step(delta_seconds);
    }
};

}  // namespace rtr::system::physics
