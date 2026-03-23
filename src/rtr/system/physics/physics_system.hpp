#pragma once

#include "rtr/system/physics/ipc/core/ipc_system.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_system.hpp"

namespace rtr::system::physics {

class PhysicsSystem {
private:
    rb::RigidBodySystem m_rigid_body_system{};
    ipc::IPCSystem m_ipc_system{ipc::IPCConfig{}};

public:
    rb::RigidBodySystem& rigid_body_system() { return m_rigid_body_system; }
    const rb::RigidBodySystem& rigid_body_system() const { return m_rigid_body_system; }
    ipc::IPCSystem& ipc_system() { return m_ipc_system; }
    const ipc::IPCSystem& ipc_system() const { return m_ipc_system; }

    void step(float delta_seconds) {
        m_rigid_body_system.step(delta_seconds);
    }
};

}  // namespace rtr::system::physics
