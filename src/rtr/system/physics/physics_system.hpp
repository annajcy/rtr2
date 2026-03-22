#pragma once

#include "rtr/system/physics/ipc/core/ipc_system.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_world.hpp"

namespace rtr::system::physics {

class PhysicsSystem {
private:
    RigidBodyWorld m_rigid_body_world{};
    ipc::IPCSystem m_ipc_system{ipc::IPCConfig{}};

public:
    RigidBodyWorld& rigid_body_world() { return m_rigid_body_world; }
    const RigidBodyWorld& rigid_body_world() const { return m_rigid_body_world; }
    ipc::IPCSystem& ipc_system() { return m_ipc_system; }
    const ipc::IPCSystem& ipc_system() const { return m_ipc_system; }

    void step(float delta_seconds) {
        m_rigid_body_world.step(delta_seconds);
    }
};

}  // namespace rtr::system::physics
