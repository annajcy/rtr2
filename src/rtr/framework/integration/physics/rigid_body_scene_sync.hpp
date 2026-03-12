#pragma once

#include "rtr/framework/component/physics/rigid_body/box_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/collider.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/component/physics/rigid_body/sphere_collider.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_world.hpp"

namespace rtr::framework::integration::physics {

inline void sync_scene_to_rigid_body(core::Scene& scene, system::physics::RigidBodyWorld& rigid_body_world) {
    scene.scene_graph().update_world_transforms();

    const auto active_nodes = scene.scene_graph().active_nodes();
    for (const auto id : active_nodes) {
        auto* game_object = scene.find_game_object(id);
        if (game_object == nullptr) {
            continue;
        }

        auto* rigid_body_component = game_object->get_component<component::RigidBody>();
        if (rigid_body_component != nullptr && rigid_body_component->enabled() &&
            rigid_body_world.has_rigid_body(rigid_body_component->rigid_body_id())) {
            auto& body = rigid_body_world.get_rigid_body(rigid_body_component->rigid_body_id());
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

inline void sync_rigid_body_to_scene(core::Scene& scene, system::physics::RigidBodyWorld& rigid_body_world) {
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
        if (!rigid_body_world.has_rigid_body(rigid_body_component->rigid_body_id())) {
            continue;
        }

        const auto& body = rigid_body_world.get_rigid_body(rigid_body_component->rigid_body_id());
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
