#pragma once

#include <cstddef>
#include <stdexcept>
#include <utility>

#include "rtr/framework/component/component.hpp"
#include "rtr/system/physics/ipc/model/tet_mesh_convert.hpp"
#include "rtr/utils/obj_types.hpp"

namespace rtr::framework::component {

class IPCTetComponent final : public Component {
private:
    std::size_t m_body_index{0};
    system::physics::ipc::TetSurfaceResult m_surface_cache{};
    utils::ObjMeshData m_mesh_cache{};

public:
    explicit IPCTetComponent(core::GameObject& owner,
                             std::size_t body_index,
                             system::physics::ipc::TetSurfaceResult surface_cache,
                             utils::ObjMeshData mesh_cache)
        : Component(owner),
          m_body_index(body_index),
          m_surface_cache(std::move(surface_cache)),
          m_mesh_cache(std::move(mesh_cache)) {
        if (m_mesh_cache.vertices.size() != m_surface_cache.surface_vertex_ids.size()) {
            throw std::invalid_argument(
                "IPCTetComponent mesh vertex count must match surface vertex id count."
            );
        }
        if (m_mesh_cache.indices.size() != m_surface_cache.surface_indices.size()) {
            throw std::invalid_argument(
                "IPCTetComponent mesh index count must match surface index count."
            );
        }
    }

    std::size_t body_index() const { return m_body_index; }

    const system::physics::ipc::TetSurfaceResult& surface_cache() const { return m_surface_cache; }
    system::physics::ipc::TetSurfaceResult& surface_cache() { return m_surface_cache; }

    const utils::ObjMeshData& mesh_cache() const { return m_mesh_cache; }
    utils::ObjMeshData& mesh_cache() { return m_mesh_cache; }
};

}  // namespace rtr::framework::component
