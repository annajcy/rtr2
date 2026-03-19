#pragma once

#include "rtr/system/physics/cloth/cloth_world.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_world.hpp"

namespace rtr::system::physics {

class PhysicsSystem {
private:
    ClothWorld m_cloth_world{};
    RigidBodyWorld m_rigid_body_world{};

public:
    ClothWorld& cloth_world() { return m_cloth_world; }
    const ClothWorld& cloth_world() const { return m_cloth_world; }

    RigidBodyWorld& rigid_body_world() { return m_rigid_body_world; }
    const RigidBodyWorld& rigid_body_world() const { return m_rigid_body_world; }

    void step(float delta_seconds) {
        m_rigid_body_world.step(delta_seconds);
        m_cloth_world.step(delta_seconds);
    }
};

}  // namespace rtr::system::physics
