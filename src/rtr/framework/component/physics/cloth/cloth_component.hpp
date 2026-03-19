#pragma once

#include <pbpt/math/math.h>

#include <stdexcept>
#include <utility>
#include <vector>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/system/physics/cloth/cloth_mesh_topology_builder.hpp"
#include "rtr/system/physics/cloth/cloth_world.hpp"

namespace rtr::framework::component {

class ClothComponent final : public Component {
private:
    system::physics::ClothWorld& m_cloth_world;
    system::physics::ClothID m_cloth_id{system::physics::kInvalidClothId};
    std::vector<system::physics::VertexID> m_pinned_vertices;
    system::physics::ClothParams m_params{};

    void require_valid_cloth_id() const {
        if (!has_cloth()) {
            throw std::runtime_error("ClothComponent is not registered.");
        }
    }

public:
    explicit ClothComponent(
        core::GameObject& owner,
        system::physics::ClothWorld& cloth_world,
        std::vector<system::physics::VertexID> pinned_vertices = {},
        system::physics::ClothParams params = {}
    )
        : Component(owner),
          m_cloth_world(cloth_world),
          m_pinned_vertices(std::move(pinned_vertices)),
          m_params(params) {}

    void on_enable() override {
        if (has_cloth()) {
            return;
        }

        auto* renderer = owner().get_component<DeformableMeshComponent>();
        if (renderer == nullptr) {
            throw std::runtime_error("ClothComponent requires a DeformableMeshComponent.");
        }
        if (!renderer->has_valid_mesh()) {
            throw std::runtime_error("ClothComponent requires a valid deformable mesh.");
        }

        const auto local_vertices = renderer->local_vertices();
        const auto indices = renderer->indices();
        const auto topology = system::physics::build_cloth_topology(local_vertices, indices);

        auto state = system::physics::ClothState::from_topology(topology, m_params.default_vertex_mass);
        for (const auto pinned_vertex : m_pinned_vertices) {
            if (!topology.is_valid_vertex(pinned_vertex)) {
                throw std::out_of_range("ClothComponent pinned vertex is invalid.");
            }
            state.set_pinned(pinned_vertex, true);
        }

        m_cloth_id = m_cloth_world.create_cloth(system::physics::ClothInstance{
            .topology = topology,
            .state = std::move(state),
            .params = m_params,
        });
    }

    void on_disable() override {
        if (!has_cloth()) {
            return;
        }
        (void)m_cloth_world.remove_cloth(m_cloth_id);
        m_cloth_id = system::physics::kInvalidClothId;
    }

    bool has_cloth() const {
        return m_cloth_id != system::physics::kInvalidClothId && m_cloth_world.has_cloth(m_cloth_id);
    }

    system::physics::ClothID cloth_id() const {
        return m_cloth_id;
    }

    const system::physics::ClothInstance& cloth() const {
        require_valid_cloth_id();
        return m_cloth_world.get_cloth(m_cloth_id);
    }

    system::physics::ClothInstance& cloth() {
        require_valid_cloth_id();
        return m_cloth_world.get_cloth(m_cloth_id);
    }

    const std::vector<system::physics::VertexID>& pinned_vertices() const {
        return m_pinned_vertices;
    }

    const system::physics::ClothParams& params() const {
        return m_params;
    }
};

}  // namespace rtr::framework::component
