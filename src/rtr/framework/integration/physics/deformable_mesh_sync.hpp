#pragma once

#include "rtr/framework/component/material/deformable_mesh_renderer.hpp"
#include "rtr/rhi/device.hpp"
#include "rtr/system/physics/common/deformable_mesh_state.hpp"
#include "rtr/system/physics/common/normal_recompute.hpp"

namespace rtr::framework::integration::physics {

inline void sync_deformable_mesh_to_renderer(
    const system::physics::DeformableMeshState& state,
    component::DeformableMeshRenderer& renderer,
    rhi::Device& device
) {
    if (state.positions.empty()) {
        return;
    }

    auto normals = system::physics::recompute_vertex_normals(state.positions, state.indices);

    // DynamicMesh::Vertex layout matches ObjVertex: position, uv, normal
    std::vector<rhi::DynamicMesh::Vertex> update_vertices(state.positions.size());
    for (size_t i = 0; i < state.positions.size(); ++i) {
        update_vertices[i].position = state.positions[i];
        update_vertices[i].normal   = normals[i];
        update_vertices[i].uv       = pbpt::math::Vec2{0.0f};
    }

    renderer.update_positions(device, update_vertices);
}

} // namespace rtr::framework::integration::physics
