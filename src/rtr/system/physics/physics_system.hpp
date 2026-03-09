#pragma once

#include "rtr/framework/component/physics/box_collider.hpp"
#include "rtr/framework/component/physics/collider.hpp"
#include "rtr/framework/component/physics/rigid_body.hpp"
#include "rtr/framework/component/physics/sphere_collider.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/system/physics/physics_world.hpp"

namespace rtr::system::physics {

class PhysicsSystem {
public:
    void fixed_tick(framework::core::Scene& scene, const framework::core::FixedTickContext& ctx) {
        sync_scene_to_physics(scene);
        m_physics_world.tick(static_cast<float>(ctx.fixed_delta_seconds));
        sync_physics_to_scene(scene);
        scene.scene_graph().update_world_transforms();
    }

    PhysicsWorld&       world() { return m_physics_world; }
    const PhysicsWorld& world() const { return m_physics_world; }

private:
    void sync_scene_to_physics(framework::core::Scene& scene) {
        scene.scene_graph().update_world_transforms();

        const auto active_nodes = scene.scene_graph().active_nodes();
        for (const auto id : active_nodes) {
            auto* game_object = scene.find_game_object(id);
            if (game_object == nullptr) {
                continue;
            }

            auto* rigid_body_component = game_object->get_component<framework::component::RigidBody>();
            if (rigid_body_component != nullptr && rigid_body_component->enabled() &&
                m_physics_world.has_rigid_body(rigid_body_component->rigid_body_id())) {
                auto& body = m_physics_world.get_rigid_body(rigid_body_component->rigid_body_id());
                if (body.type() != RigidBodyType::Dynamic) {
                    body.state().translation.position = game_object->node().world_position();
                    body.state().rotation.orientation = game_object->node().world_rotation();
                    body.invalidate_integrator_state();
                }
            }

            auto* collider_component = game_object->get_component<framework::component::Collider>();
            if (collider_component != nullptr && collider_component->enabled()) {
                collider_component->sync_to_physics();
            }
        }
    }

    void sync_physics_to_scene(framework::core::Scene& scene) {
        const auto active_nodes = scene.scene_graph().active_nodes();
        for (const auto id : active_nodes) {
            auto* game_object = scene.find_game_object(id);
            if (game_object == nullptr) {
                continue;
            }

            auto* rigid_body_component = game_object->get_component<framework::component::RigidBody>();
            if (rigid_body_component == nullptr || !rigid_body_component->enabled()) {
                continue;
            }
            if (!m_physics_world.has_rigid_body(rigid_body_component->rigid_body_id())) {
                continue;
            }

            const auto& body = m_physics_world.get_rigid_body(rigid_body_component->rigid_body_id());
            if (body.type() != RigidBodyType::Dynamic) {
                continue;
            }

            const auto& state = body.state();
            scene.scene_graph().node(id).set_local_position(state.translation.position);
            scene.scene_graph().node(id).set_local_rotation(state.rotation.orientation);
        }
    }

    PhysicsWorld m_physics_world{};
};

}  // namespace rtr::system::physics
