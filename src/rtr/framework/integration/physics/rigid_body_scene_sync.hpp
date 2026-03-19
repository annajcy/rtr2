#pragma once

#include <utility>

#include "rtr/framework/component/material/mesh_component.hpp"
#include "rtr/framework/component/physics/rigid_body/box_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/collider.hpp"
#include "rtr/framework/component/physics/rigid_body/mesh_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/plane_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/component/physics/rigid_body/sphere_collider.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/system/physics/collision/collider_shape.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_world.hpp"

namespace rtr::framework::integration::physics {

inline void sync_sphere_collider_to_physics(const component::SphereCollider& sphere_collider,
                                            system::physics::RigidBodyWorld& rigid_body_world) {
    if (!sphere_collider.has_collider() || !rigid_body_world.has_collider(sphere_collider.collider_id())) {
        return;
    }

    auto& collider                      = rigid_body_world.get_collider(sphere_collider.collider_id());
    collider.shape                      = system::physics::SphereShape{.radius = sphere_collider.radius()};
    collider.local_transform.position   = sphere_collider.local_position();
    collider.local_transform.rotation   = sphere_collider.local_rotation();
    collider.local_transform.scale      = sphere_collider.local_scale();
}

inline void sync_box_collider_to_physics(const component::BoxCollider& box_collider,
                                         system::physics::RigidBodyWorld& rigid_body_world) {
    if (!box_collider.has_collider() || !rigid_body_world.has_collider(box_collider.collider_id())) {
        return;
    }

    auto& collider                      = rigid_body_world.get_collider(box_collider.collider_id());
    collider.shape                      = system::physics::BoxShape{.half_extents = box_collider.half_extents()};
    collider.local_transform.position   = box_collider.local_position();
    collider.local_transform.rotation   = box_collider.local_rotation();
    collider.local_transform.scale      = box_collider.local_scale();
}

inline void sync_plane_collider_to_physics(const component::PlaneCollider& plane_collider,
                                           system::physics::RigidBodyWorld& rigid_body_world) {
    if (!plane_collider.has_collider() || !rigid_body_world.has_collider(plane_collider.collider_id())) {
        return;
    }

    auto& collider                      = rigid_body_world.get_collider(plane_collider.collider_id());
    collider.shape                      = system::physics::PlaneShape{.normal_local = plane_collider.normal_local()};
    collider.local_transform.position   = plane_collider.local_position();
    collider.local_transform.rotation   = plane_collider.local_rotation();
    collider.local_transform.scale      = pbpt::math::Vec3{1.0f};
}

inline void sync_mesh_collider_to_physics(const component::MeshCollider& mesh_collider,
                                          system::physics::RigidBodyWorld& rigid_body_world) {
    if (!mesh_collider.has_collider() || !rigid_body_world.has_collider(mesh_collider.collider_id())) {
        return;
    }

    const auto* mesh_component = mesh_collider.owner().get_component<component::MeshComponent>();
    if (mesh_component == nullptr || !mesh_component->has_valid_mesh()) {
        return;
    }

    auto& collider                    = rigid_body_world.get_collider(mesh_collider.collider_id());
    collider.shape                    = system::physics::MeshShape{.local_vertices = mesh_component->local_vertices()};
    collider.local_transform.position = mesh_collider.local_position();
    collider.local_transform.rotation = mesh_collider.local_rotation();
    collider.local_transform.scale    = mesh_collider.local_scale();
}

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
        if (collider_component == nullptr || !collider_component->enabled()) {
            continue;
        }

        if (const auto* sphere_collider = dynamic_cast<const component::SphereCollider*>(collider_component);
            sphere_collider != nullptr) {
            sync_sphere_collider_to_physics(*sphere_collider, rigid_body_world);
            continue;
        }
        if (const auto* box_collider = dynamic_cast<const component::BoxCollider*>(collider_component);
            box_collider != nullptr) {
            sync_box_collider_to_physics(*box_collider, rigid_body_world);
            continue;
        }
        if (const auto* plane_collider = dynamic_cast<const component::PlaneCollider*>(collider_component);
            plane_collider != nullptr) {
            sync_plane_collider_to_physics(*plane_collider, rigid_body_world);
            continue;
        }
        if (const auto* mesh_collider = dynamic_cast<const component::MeshCollider*>(collider_component);
            mesh_collider != nullptr) {
            sync_mesh_collider_to_physics(*mesh_collider, rigid_body_world);
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
