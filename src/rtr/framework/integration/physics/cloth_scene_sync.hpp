#pragma once

#include <vector>

#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/component/physics/cloth/cloth_component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/system/physics/cloth/cloth_world.hpp"
#include "rtr/system/physics/common/normal_recompute.hpp"

namespace rtr::framework::integration::physics {

inline void sync_scene_to_cloth(core::Scene& scene, system::physics::ClothWorld& cloth_world) {
    scene.scene_graph().update_world_transforms();

    const auto active_nodes = scene.scene_graph().active_nodes();
    for (const auto id : active_nodes) {
        auto* game_object = scene.find_game_object(id);
        if (game_object == nullptr) {
            continue;
        }

        auto* cloth_component = game_object->get_component<component::ClothComponent>();
        if (cloth_component == nullptr || !cloth_component->enabled()) {
            continue;
        }
        if (!cloth_component->has_cloth()) {
            continue;
        }
        if (!cloth_world.has_cloth(cloth_component->cloth_id())) {
            continue;
        }

        // Cloth runtime state is owned by ClothWorld after registration. The scene-side
        // pre-pass remains as the explicit scene/physics boundary for future validation
        // and config sync, but currently does not mutate cloth runtime positions.
        (void)cloth_world.get_cloth(cloth_component->cloth_id());
    }
}

inline void sync_cloth_to_scene(core::Scene& scene, system::physics::ClothWorld& cloth_world) {
    const auto active_nodes = scene.scene_graph().active_nodes();
    for (const auto id : active_nodes) {
        auto* game_object = scene.find_game_object(id);
        if (game_object == nullptr) {
            continue;
        }

        auto* cloth_component = game_object->get_component<component::ClothComponent>();
        if (cloth_component == nullptr || !cloth_component->enabled()) {
            continue;
        }
        if (!cloth_component->has_cloth()) {
            continue;
        }
        if (!cloth_world.has_cloth(cloth_component->cloth_id())) {
            continue;
        }

        auto* renderer = game_object->get_component<component::DeformableMeshComponent>();
        if (renderer == nullptr || !renderer->has_valid_mesh()) {
            continue;
        }

        const auto& instance = cloth_world.get_cloth(cloth_component->cloth_id());
        const auto& local_positions = instance.state.positions;

        const auto normals =
            system::physics::recompute_vertex_normals(local_positions, instance.topology.render_triangle_indices);
        renderer->apply_deformed_surface(local_positions, normals);
    }
}

}  // namespace rtr::framework::integration::physics
