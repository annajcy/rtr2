#pragma once

#include <stdexcept>
#include <vector>

#include <pbpt/math/math.h>

#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/component/physics/ipc/ipc_tet_component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/system/physics/ipc/core/ipc_system.hpp"
#include "rtr/system/physics/ipc/model/tet_mesh_convert.hpp"

namespace rtr::framework::integration::physics {

inline void sync_ipc_to_scene(core::Scene& scene,
                              const system::physics::ipc::IPCSystem& ipc_system) {
    const auto active_nodes = scene.scene_graph().active_nodes();
    for (const auto id : active_nodes) {
        auto* game_object = scene.find_game_object(id);
        if (game_object == nullptr) {
            continue;
        }

        auto* ipc_tet = game_object->get_component<component::IPCTetComponent>();
        if (ipc_tet == nullptr || !ipc_tet->enabled()) {
            continue;
        }

        auto* deformable = game_object->get_component<component::DeformableMeshComponent>();
        if (deformable == nullptr || !deformable->enabled()) {
            continue;
        }

        if (ipc_tet->body_index() >= ipc_system.tet_body_count()) {
            throw std::out_of_range("IPCTetComponent body_index is out of range for IPCSystem.");
        }

        const auto& body = ipc_system.tet_body(ipc_tet->body_index());
        const std::size_t vertex_offset = body.info.dof_offset / 3u;
        system::physics::ipc::update_mesh_positions(
            ipc_tet->mesh_cache(),
            ipc_system.state().x,
            ipc_tet->surface_cache(),
            vertex_offset
        );

        std::vector<pbpt::math::Vec3> positions{};
        std::vector<pbpt::math::Vec3> normals{};
        positions.reserve(ipc_tet->mesh_cache().vertices.size());
        normals.reserve(ipc_tet->mesh_cache().vertices.size());
        for (const auto& vertex : ipc_tet->mesh_cache().vertices) {
            positions.push_back(vertex.position);
            normals.push_back(vertex.normal);
        }

        deformable->apply_deformed_surface(positions, normals);
    }
}

}  // namespace rtr::framework::integration::physics
