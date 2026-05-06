#pragma once

#include <unordered_set>

#include "rtr/framework/component/physics/rigid_body/box_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/collider.hpp"
#include "rtr/framework/component/physics/rigid_body/plane_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/component/physics/rigid_body/sphere_collider.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_system.hpp"

namespace rtr::framework::integration::physics {

inline void apply_pending_rigid_body_command(
    system::physics::rb::RigidBody& body,
    const component::RigidBody::PendingRigidBodyCommand& command) {
    switch (command.type) {
    case component::RigidBody::PendingRigidBodyCommandType::AddForce:
        body.state().forces.accumulated_force += command.value;
        break;
    case component::RigidBody::PendingRigidBodyCommandType::AddTorque:
        body.state().forces.accumulated_torque += command.value;
        break;
    case component::RigidBody::PendingRigidBodyCommandType::AddForceAtPoint: {
        body.state().forces.accumulated_force += command.value;
        const pbpt::math::Vec3 lever_arm = command.point - body.state().translation.position;
        body.state().forces.accumulated_torque += pbpt::math::cross(lever_arm, command.value);
        break;
    }
    case component::RigidBody::PendingRigidBodyCommandType::ClearForces:
        body.clear_forces();
        break;
    case component::RigidBody::PendingRigidBodyCommandType::ResetDynamics:
        body.reset_dynamics();
        break;
    case component::RigidBody::PendingRigidBodyCommandType::ResetTranslationDynamics:
        body.reset_translation_dynamics();
        break;
    case component::RigidBody::PendingRigidBodyCommandType::ResetRotationalDynamics:
        body.reset_rotational_dynamics();
        break;
    }
}

inline system::physics::rb::Collider build_sphere_collider(const component::SphereCollider& sphere_collider) {
    system::physics::rb::Collider collider;
    collider.shape = system::physics::rb::SphereShape{.radius = sphere_collider.radius()};
    collider.local_transform.position = sphere_collider.local_position();
    collider.local_transform.rotation = sphere_collider.local_rotation();
    collider.local_transform.scale = sphere_collider.local_scale();
    return collider;
}

inline system::physics::rb::Collider build_box_collider(const component::BoxCollider& box_collider) {
    system::physics::rb::Collider collider;
    collider.shape = system::physics::rb::BoxShape{.half_extents = box_collider.half_extents()};
    collider.local_transform.position = box_collider.local_position();
    collider.local_transform.rotation = box_collider.local_rotation();
    collider.local_transform.scale = box_collider.local_scale();
    return collider;
}

inline system::physics::rb::Collider build_plane_collider(const component::PlaneCollider& plane_collider) {
    system::physics::rb::Collider collider;
    collider.shape = system::physics::rb::PlaneShape{.normal_local = plane_collider.normal_local()};
    collider.local_transform.position = plane_collider.local_position();
    collider.local_transform.rotation = plane_collider.local_rotation();
    collider.local_transform.scale = pbpt::math::Vec3{1.0f};
    return collider;
}

inline void sync_scene_to_rigid_body(core::Scene& scene, system::physics::rb::RigidBodySystem& rigid_body_system) {
    scene.scene_graph().update_world_transforms();

    std::unordered_set<framework::core::GameObjectId> desired_rigid_body_owners{};
    std::unordered_set<framework::core::GameObjectId> desired_collider_owners{};

    const auto active_nodes = scene.scene_graph().active_nodes();
    for (const auto id : active_nodes) {
        auto* game_object = scene.find_game_object(id);
        if (game_object == nullptr) {
            continue;
        }

        if (auto* rigid_body_component = game_object->get_component<component::RigidBody>();
            rigid_body_component != nullptr && rigid_body_component->enabled() &&
            rigid_body_component->should_exist_in_runtime()) {
            desired_rigid_body_owners.insert(id);

            if (!rigid_body_system.has_rigid_body_for_owner(id)) {
                (void)rigid_body_system.create_or_replace_rigid_body(id, rigid_body_component->make_runtime_body_snapshot());
            }

            auto* body = rigid_body_system.try_get_rigid_body_for_owner(id);
            if (body == nullptr) {
                continue;
            }

            for (const auto& command : rigid_body_component->pending_commands()) {
                apply_pending_rigid_body_command(*body, command);
            }

            if (rigid_body_component->source_dirty()) {
                auto runtime_body = rigid_body_component->make_runtime_body_snapshot();
                runtime_body.state().translation.position = body->state().translation.position;
                runtime_body.state().rotation.orientation = body->state().rotation.orientation;
                runtime_body.state().scale = game_object->node().world_scale();
                runtime_body.state().forces = body->state().forces;
                (void)rigid_body_system.create_or_replace_rigid_body(id, std::move(runtime_body));
                body = rigid_body_system.try_get_rigid_body_for_owner(id);
            }

            if (body != nullptr &&
                (body->type() != system::physics::rb::RigidBodyType::Dynamic || rigid_body_component->transform_dirty())) {
                body->state().translation.position = game_object->node().world_position();
                body->state().rotation.orientation = game_object->node().world_rotation();
                body->state().scale = game_object->node().world_scale();
            }

            rigid_body_component->clear_pending_commands();
            rigid_body_component->clear_source_dirty();
            rigid_body_component->clear_transform_dirty();
            rigid_body_component->clear_lifecycle_dirty();
        }

        auto* collider_component = game_object->get_component<component::Collider>();
        if (collider_component == nullptr || !collider_component->enabled() ||
            !collider_component->should_exist_in_runtime()) {
            continue;
        }
        if (game_object->get_component<component::RigidBody>() == nullptr ||
            !rigid_body_system.has_rigid_body_for_owner(id)) {
            continue;
        }

        desired_collider_owners.insert(id);

        if (const auto* sphere_collider = dynamic_cast<const component::SphereCollider*>(collider_component);
            sphere_collider != nullptr) {
            if (!rigid_body_system.has_collider_for_owner(id) ||
                sphere_collider->shape_dirty() ||
                sphere_collider->transform_dirty() ||
                sphere_collider->lifecycle_dirty()) {
                (void)rigid_body_system.create_or_replace_collider(id, id, build_sphere_collider(*sphere_collider));
            }
            const_cast<component::SphereCollider*>(sphere_collider)->clear_shape_dirty();
            const_cast<component::SphereCollider*>(sphere_collider)->clear_transform_dirty();
            const_cast<component::SphereCollider*>(sphere_collider)->clear_lifecycle_dirty();
            continue;
        }
        if (const auto* box_collider = dynamic_cast<const component::BoxCollider*>(collider_component);
            box_collider != nullptr) {
            if (!rigid_body_system.has_collider_for_owner(id) ||
                box_collider->shape_dirty() ||
                box_collider->transform_dirty() ||
                box_collider->lifecycle_dirty()) {
                (void)rigid_body_system.create_or_replace_collider(id, id, build_box_collider(*box_collider));
            }
            const_cast<component::BoxCollider*>(box_collider)->clear_shape_dirty();
            const_cast<component::BoxCollider*>(box_collider)->clear_transform_dirty();
            const_cast<component::BoxCollider*>(box_collider)->clear_lifecycle_dirty();
            continue;
        }
        if (const auto* plane_collider = dynamic_cast<const component::PlaneCollider*>(collider_component);
            plane_collider != nullptr) {
            if (!rigid_body_system.has_collider_for_owner(id) ||
                plane_collider->shape_dirty() ||
                plane_collider->transform_dirty() ||
                plane_collider->lifecycle_dirty()) {
                (void)rigid_body_system.create_or_replace_collider(id, id, build_plane_collider(*plane_collider));
            }
            const_cast<component::PlaneCollider*>(plane_collider)->clear_shape_dirty();
            const_cast<component::PlaneCollider*>(plane_collider)->clear_transform_dirty();
            const_cast<component::PlaneCollider*>(plane_collider)->clear_lifecycle_dirty();
        }
    }

    for (const auto owner_id : rigid_body_system.collider_owner_ids()) {
        if (!desired_collider_owners.contains(owner_id)) {
            (void)rigid_body_system.remove_collider_for_owner(owner_id);
        }
    }
    for (const auto owner_id : rigid_body_system.rigid_body_owner_ids()) {
        if (!desired_rigid_body_owners.contains(owner_id)) {
            (void)rigid_body_system.remove_rigid_body_for_owner(owner_id);
        }
    }
}

inline void sync_rigid_body_to_scene(core::Scene& scene, system::physics::rb::RigidBodySystem& rigid_body_system) {
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
        if (!rigid_body_component->should_exist_in_runtime() ||
            rigid_body_component->source_dirty() ||
            rigid_body_component->transform_dirty() ||
            rigid_body_component->lifecycle_dirty() ||
            rigid_body_component->has_pending_commands()) {
            continue;
        }
        if (!rigid_body_system.has_rigid_body_for_owner(id) ||
            !rigid_body_system.is_scene_sync_dirty_for_owner(id)) {
            continue;
        }

        const auto* body = rigid_body_system.try_get_rigid_body_for_owner(id);
        if (body == nullptr || body->type() != system::physics::rb::RigidBodyType::Dynamic) {
            continue;
        }

        const auto& state = body->state();
        scene.scene_graph().node(id).set_world_position(state.translation.position);
        scene.scene_graph().node(id).set_world_rotation(state.rotation.orientation);
        scene.scene_graph().node(id).set_world_scale(state.scale);
        rigid_body_component->sync_runtime_state_from(*body);
        rigid_body_system.clear_scene_sync_dirty_for_owner(id);
    }

    scene.scene_graph().update_world_transforms();
}

}  // namespace rtr::framework::integration::physics
