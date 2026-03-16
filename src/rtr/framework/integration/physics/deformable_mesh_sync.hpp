#pragma once

#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/system/physics/common/deformable_mesh_state.hpp"
#include "rtr/system/physics/common/normal_recompute.hpp"

namespace rtr::framework::integration::physics {

inline void sync_deformable_mesh_to_renderer(
    const system::physics::DeformableMeshState& state,
    component::DeformableMeshComponent& renderer
) {
    if (state.positions.empty()) {
        return;
    }

    auto normals = system::physics::recompute_vertex_normals(state.positions, state.indices);
    renderer.apply_deformed_surface(state.positions, normals);
}

} // namespace rtr::framework::integration::physics
