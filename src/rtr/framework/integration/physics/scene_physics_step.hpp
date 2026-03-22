#pragma once

#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/physics/rigid_body_scene_sync.hpp"
#include "rtr/system/physics/physics_system.hpp"

namespace rtr::framework::integration::physics {

inline void step_scene_physics(core::Scene& scene,
                               system::physics::PhysicsSystem& physics_system,
                               float delta_seconds) {
    sync_scene_to_rigid_body(scene, physics_system.rigid_body_world());
    physics_system.step(delta_seconds);
    sync_rigid_body_to_scene(scene, physics_system.rigid_body_world());
    physics_system.ipc_system().step();
}

}  // namespace rtr::framework::integration::physics
