#pragma once

#include <algorithm>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <utility>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/system/physics/ipc/core/ipc_system.hpp"
#include "rtr/system/physics/ipc/energy/material_model/tet_fixed_corotated.hpp"
#include "rtr/system/physics/ipc/model/tet_body.hpp"
#include "rtr/system/physics/ipc/model/mesh_tet_converter/tet_to_mesh.hpp"
#include "rtr/utils/obj_types.hpp"

namespace rtr::framework::component {

class IPCTetComponent final : public Component {
private:
    system::physics::ipc::TetBody m_source_body{};
    system::physics::ipc::TetSurfaceMapping m_surface_cache{};
    utils::ObjMeshData m_mesh_cache{};
    bool m_source_dirty{false};
    bool m_surface_cache_dirty{false};
    bool m_lifecycle_dirty{false};
    bool m_should_exist_in_runtime{false};

    void rebuild_surface_cache_from_source() {
        m_surface_cache = system::physics::ipc::build_tet_surface_mapping(m_source_body);
        m_mesh_cache = system::physics::ipc::tet_rest_to_surface_mesh(m_source_body.geometry, m_surface_cache);
        if (m_mesh_cache.vertices.size() != m_surface_cache.surface_vertex_ids.size()) {
            throw std::invalid_argument("IPCTetComponent mesh vertex count must match surface vertex id count.");
        }
        if (m_mesh_cache.indices.size() != m_surface_cache.surface_indices.size()) {
            throw std::invalid_argument("IPCTetComponent mesh index count must match surface index count.");
        }
        m_surface_cache_dirty = false;
    }

public:
    explicit IPCTetComponent(core::GameObject& owner,
                             system::physics::ipc::TetBody source_body)
        : Component(owner),
          m_source_body(std::move(source_body)) {
        rebuild_surface_cache_from_source();
    }

    void on_enable() override {
        m_should_exist_in_runtime = true;
        m_lifecycle_dirty = true;
    }

    void on_disable() override {
        m_should_exist_in_runtime = false;
        m_lifecycle_dirty = true;
    }

    void on_destroy() override {
        m_should_exist_in_runtime = false;
        m_lifecycle_dirty = true;
    }

    const system::physics::ipc::TetBody& source_body() const { return m_source_body; }
    bool source_dirty() const { return m_source_dirty; }
    bool surface_cache_dirty() const { return m_surface_cache_dirty; }
    bool lifecycle_dirty() const { return m_lifecycle_dirty; }
    bool should_exist_in_runtime() const { return m_should_exist_in_runtime; }

    void clear_source_dirty() { m_source_dirty = false; }
    void clear_lifecycle_dirty() { m_lifecycle_dirty = false; }

    void set_source_body(system::physics::ipc::TetBody source_body) {
        m_source_body = std::move(source_body);
        m_source_dirty = true;
        m_surface_cache_dirty = true;
    }

    bool uses_fixed_corotated_material() const {
        return std::holds_alternative<system::physics::ipc::FixedCorotatedMaterial>(m_source_body.material);
    }

    std::optional<double> density() const {
        if (const auto* material = std::get_if<system::physics::ipc::FixedCorotatedMaterial>(&m_source_body.material);
            material != nullptr) {
            return material->mass_density;
        }
        return std::nullopt;
    }

    std::optional<double> youngs_modulus() const {
        if (const auto* material = std::get_if<system::physics::ipc::FixedCorotatedMaterial>(&m_source_body.material);
            material != nullptr) {
            return material->youngs_modulus;
        }
        return std::nullopt;
    }

    std::optional<double> poisson_ratio() const {
        if (const auto* material = std::get_if<system::physics::ipc::FixedCorotatedMaterial>(&m_source_body.material);
            material != nullptr) {
            return material->poisson_ratio;
        }
        return std::nullopt;
    }

    bool set_density(double density) {
        auto* material = std::get_if<system::physics::ipc::FixedCorotatedMaterial>(&m_source_body.material);
        if (material == nullptr) {
            return false;
        }
        material->mass_density = std::max(0.001, density);
        m_source_dirty = true;
        return true;
    }

    bool set_youngs_modulus(double youngs_modulus) {
        auto* material = std::get_if<system::physics::ipc::FixedCorotatedMaterial>(&m_source_body.material);
        if (material == nullptr) {
            return false;
        }
        material->youngs_modulus = std::max(1.0, youngs_modulus);
        m_source_dirty = true;
        return true;
    }

    bool set_poisson_ratio(double poisson_ratio) {
        auto* material = std::get_if<system::physics::ipc::FixedCorotatedMaterial>(&m_source_body.material);
        if (material == nullptr) {
            return false;
        }
        material->poisson_ratio = std::clamp(poisson_ratio, -0.999, 0.499);
        m_source_dirty = true;
        return true;
    }

    void mark_surface_cache_dirty() { m_surface_cache_dirty = true; }

    const system::physics::ipc::TetSurfaceMapping& surface_cache() const { return m_surface_cache; }
    system::physics::ipc::TetSurfaceMapping& surface_cache() { return m_surface_cache; }
    void rebuild_surface_cache_if_dirty() {
        if (m_surface_cache_dirty) {
            rebuild_surface_cache_from_source();
        }
    }

    const utils::ObjMeshData& mesh_cache() const { return m_mesh_cache; }
    utils::ObjMeshData& mesh_cache() { return m_mesh_cache; }
};

}  // namespace rtr::framework::component
