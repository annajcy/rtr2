#pragma once

#include <pbpt/math/math.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "rtr/system/physics/cloth/cloth_topology.hpp"

namespace rtr::system::physics {

struct ClothState {
    std::vector<pbpt::math::Vec3> positions;
    std::vector<pbpt::math::Vec3> velocities;
    std::vector<pbpt::math::Float> masses;
    std::vector<std::uint8_t> pinned_mask;

private:
    std::size_t checked_vertex_index(VertexID id) const {
        if (id < 0 || static_cast<std::size_t>(id) >= positions.size()) {
            throw std::out_of_range("VertexID is invalid.");
        }
        return static_cast<std::size_t>(id);
    }

public:
    static ClothState from_topology(const ClothTopology& topology, pbpt::math::Float default_mass = 1.0f) {
        if (!std::isfinite(default_mass) || default_mass <= 0.0f) {
            throw std::invalid_argument("ClothState default mass must be finite and positive.");
        }

        ClothState state{};
        state.positions = topology.rest_positions;
        state.velocities.assign(topology.rest_positions.size(), pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
        state.masses.assign(topology.rest_positions.size(), default_mass);
        state.pinned_mask.assign(topology.rest_positions.size(), 0u);
        return state;
    }

    std::size_t vertex_count() const {
        return positions.size();
    }

    const pbpt::math::Vec3& position(VertexID id) const {
        return positions[checked_vertex_index(id)];
    }

    pbpt::math::Vec3& position(VertexID id) {
        return positions[checked_vertex_index(id)];
    }

    const pbpt::math::Vec3& velocity(VertexID id) const {
        return velocities[checked_vertex_index(id)];
    }

    pbpt::math::Vec3& velocity(VertexID id) {
        return velocities[checked_vertex_index(id)];
    }

    pbpt::math::Float mass(VertexID id) const {
        return masses[checked_vertex_index(id)];
    }

    pbpt::math::Float& mass(VertexID id) {
        return masses[checked_vertex_index(id)];
    }

    bool pinned(VertexID id) const {
        return pinned_mask[checked_vertex_index(id)] != 0u;
    }

    void set_pinned(VertexID id, bool pinned) {
        pinned_mask[checked_vertex_index(id)] = pinned ? 1u : 0u;
    }

    std::array<pbpt::math::Vec3, 2> edge_positions(const ClothTopology& topology, EdgeID id) const {
        const auto [v0, v1] = topology.edge_vertices(id);
        return {position(v0), position(v1)};
    }

    std::array<pbpt::math::Vec3, 2> edge_velocities(const ClothTopology& topology, EdgeID id) const {
        const auto [v0, v1] = topology.edge_vertices(id);
        return {velocity(v0), velocity(v1)};
    }

    std::array<pbpt::math::Vec3, 3> triangle_positions(const ClothTopology& topology, TriangleID id) const {
        const auto [v0, v1, v2] = topology.triangle_vertices(id);
        return {position(v0), position(v1), position(v2)};
    }

    std::array<pbpt::math::Vec3, 3> triangle_velocities(const ClothTopology& topology, TriangleID id) const {
        const auto [v0, v1, v2] = topology.triangle_vertices(id);
        return {velocity(v0), velocity(v1), velocity(v2)};
    }
};

}  // namespace rtr::system::physics
