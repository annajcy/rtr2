#pragma once

#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/physics/rigid_body_scene_sync.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_world.hpp"

namespace rtr::system::physics {

class PhysicsSystem {
private:
    RigidBodyWorld m_rigid_body_world{};

public:
    RigidBodyWorld& rigid_body_world() { return m_rigid_body_world; }
    const RigidBodyWorld& rigid_body_world() const { return m_rigid_body_world; }

    void step(framework::core::Scene& scene, float delta_seconds) {
        framework::integration::physics::sync_scene_to_rigid_body(scene, m_rigid_body_world);
        m_rigid_body_world.step(delta_seconds);
        framework::integration::physics::sync_rigid_body_to_scene(scene, m_rigid_body_world);
    }
};

}  // namespace rtr::system::physics
