#pragma once

#include <cstddef>
#include <stdexcept>
#include <utility>

#include "rtr/framework/component/component.hpp"
#include "rtr/system/physics/ipc/core/ipc_system.hpp"
#include "rtr/system/physics/ipc/energy/material_model/tet_fixed_corotated.hpp"
#include "rtr/system/physics/ipc/model/tet_body.hpp"
#include "rtr/system/physics/ipc/model/tet_mesh_convert.hpp"
#include "rtr/utils/obj_types.hpp"

namespace rtr::framework::component {

class IPCTetComponent final : public Component {
private:
    system::physics::ipc::IPCSystem& m_ipc_world;
    system::physics::ipc::IPCBodyID m_body_id{system::physics::ipc::kInvalidIPCBodyId};
    bool m_registered{false};
    system::physics::ipc::TetBody m_source_body{};
    system::physics::ipc::TetSurfaceResult m_surface_cache{};
    utils::ObjMeshData m_mesh_cache{};

    void rebuild_surface_cache_from_source() {
        m_surface_cache = system::physics::ipc::extract_tet_surface(m_source_body);
        m_mesh_cache = system::physics::ipc::tet_to_mesh(m_source_body.geometry, m_surface_cache);
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

public:
    explicit IPCTetComponent(core::GameObject& owner,
                             system::physics::ipc::IPCSystem& ipc_world,
                             system::physics::ipc::TetBody source_body)
        : Component(owner),
          m_ipc_world(ipc_world),
          m_source_body(std::move(source_body)) {
        rebuild_surface_cache_from_source();
    }

    void on_enable() override {
        if (m_registered) {
            return;
        }
        rebuild_surface_cache_from_source();
        m_body_id = m_ipc_world.create_tet_body(m_source_body);
        m_registered = true;
    }

    void on_disable() override {
        if (!m_registered) {
            return;
        }
        (void)m_ipc_world.remove_tet_body(m_body_id);
        m_body_id = system::physics::ipc::kInvalidIPCBodyId;
        m_registered = false;
    }

    bool has_registered_body() const {
        return m_registered && m_ipc_world.has_tet_body(m_body_id);
    }

    system::physics::ipc::IPCBodyID body_id() const { return m_body_id; }
    const system::physics::ipc::TetBody& source_body() const { return m_source_body; }

    system::physics::ipc::TetBody* runtime_body() {
        if (!has_registered_body()) {
            return nullptr;
        }
        return &m_ipc_world.get_tet_body(m_body_id);
    }

    const system::physics::ipc::TetBody* runtime_body() const {
        if (!has_registered_body()) {
            return nullptr;
        }
        return &m_ipc_world.get_tet_body(m_body_id);
    }

    template <typename TMaterial>
    TMaterial* source_material_if() {
        return std::get_if<TMaterial>(&m_source_body.material);
    }

    template <typename TMaterial>
    const TMaterial* source_material_if() const {
        return std::get_if<TMaterial>(&m_source_body.material);
    }

    bool apply_source_material_to_runtime() {
        if (!has_registered_body()) {
            return false;
        }

        auto& runtime_body_ref = m_ipc_world.get_tet_body(m_body_id);
        runtime_body_ref.material = m_source_body.material;
        m_ipc_world.refresh_tet_body_runtime_data(m_body_id);
        return true;
    }

    const system::physics::ipc::TetSurfaceResult& surface_cache() const { return m_surface_cache; }
    system::physics::ipc::TetSurfaceResult& surface_cache() { return m_surface_cache; }

    const utils::ObjMeshData& mesh_cache() const { return m_mesh_cache; }
    utils::ObjMeshData& mesh_cache() { return m_mesh_cache; }
};

}  // namespace rtr::framework::component
