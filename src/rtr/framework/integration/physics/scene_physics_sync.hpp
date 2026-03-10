#pragma once

#include "rtr/framework/component/physics/box_collider.hpp"
#include "rtr/framework/component/physics/collider.hpp"
#include "rtr/framework/component/physics/rigid_body.hpp"
#include "rtr/framework/component/physics/sphere_collider.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/system/physics/physics_world.hpp"

namespace rtr::framework::integration::physics {

inline void sync_scene_to_physics(core::Scene& scene, system::physics::PhysicsWorld& physics_world) {
    scene.scene_graph().update_world_transforms();

    const auto active_nodes = scene.scene_graph().active_nodes();
    for (const auto id : active_nodes) {
        auto* game_object = scene.find_game_object(id);
        if (game_object == nullptr) {
            continue;
        }

        auto* rigid_body_component = game_object->get_component<component::RigidBody>();
        if (rigid_body_component != nullptr && rigid_body_component->enabled() &&
            physics_world.has_rigid_body(rigid_body_component->rigid_body_id())) {
            auto& body = physics_world.get_rigid_body(rigid_body_component->rigid_body_id());
            body.state().scale = game_object->node().world_scale();
            if (body.type() != system::physics::RigidBodyType::Dynamic) {
                body.state().translation.position = game_object->node().world_position();
                body.state().rotation.orientation = game_object->node().world_rotation();
            }
        }

        auto* collider_component = game_object->get_component<component::Collider>();
        if (collider_component != nullptr && collider_component->enabled()) {
            collider_component->sync_to_physics();
        }
    }
}

inline void sync_physics_to_scene(core::Scene& scene, system::physics::PhysicsWorld& physics_world) {
    const auto active_nodes = scene.scene_graph().active_nodes();
    for (const auto id : active_nodes) {
        auto* game_object = scene.find_game_object(id);
        if (game_object == nullptr) {
            continue;
        }

        auto* rigid_body_component = game_object->get_component<component::RigidBody>();
        if (rigid_body_component == nullptr || !rigid_body_component->enabled()) {
            continue;
        }
        if (!physics_world.has_rigid_body(rigid_body_component->rigid_body_id())) {
            continue;
        }

        const auto& body = physics_world.get_rigid_body(rigid_body_component->rigid_body_id());
        if (body.type() != system::physics::RigidBodyType::Dynamic) {
            continue;
        }

        const auto& state = body.state();
        scene.scene_graph().node(id).set_local_position(state.translation.position);
        scene.scene_graph().node(id).set_local_rotation(state.rotation.orientation);
        scene.scene_graph().node(id).set_local_scale(state.scale);
    }

    scene.scene_graph().update_world_transforms();
}

}  // namespace rtr::framework::integration::physics
