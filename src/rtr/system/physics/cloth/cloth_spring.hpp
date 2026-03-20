#pragma once

#include <pbpt/math/math.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "rtr/system/physics/cloth/cloth_topology.hpp"

namespace rtr::system::physics {

enum class ClothSpringKind {
    Edge,
    Bend,
};

struct ClothSpring {
    ClothSpringKind kind{ClothSpringKind::Edge};
    std::array<VertexID, 2> vertices{kInvalidVertexID, kInvalidVertexID};
    pbpt::math::Float rest_length{0.0f};
};

struct ClothSpringNetwork {
    std::vector<ClothSpring> springs{};

    std::size_t spring_count() const {
        return springs.size();
    }

    std::size_t spring_count(ClothSpringKind kind) const {
        return static_cast<std::size_t>(std::count_if(
            springs.begin(),
            springs.end(),
            [kind](const ClothSpring& spring) {
                return spring.kind == kind;
            }
        ));
    }
};

namespace detail {

inline VertexID opposite_triangle_vertex(const ClothTopology& topology,
                                         TriangleID triangle_id,
                                         VertexID edge_vertex_a,
                                         VertexID edge_vertex_b) {
    const auto vertices = topology.triangle_vertices(triangle_id);
    for (const auto vertex_id : vertices) {
        if (vertex_id != edge_vertex_a && vertex_id != edge_vertex_b) {
            return vertex_id;
        }
    }
    throw std::invalid_argument("Cloth bend spring construction failed: opposite triangle vertex is missing.");
}

}  // namespace detail

inline ClothSpringNetwork build_cloth_spring_network(const ClothTopology& topology) {
    ClothSpringNetwork network{};
    network.springs.reserve(topology.edges.size() * 2u);

    for (std::size_t edge_index = 0; edge_index < topology.edges.size(); ++edge_index) {
        const auto edge_id = static_cast<EdgeID>(edge_index);
        const auto [vertex_a, vertex_b] = topology.edge_vertices(edge_id);
        network.springs.push_back(ClothSpring{
            .kind = ClothSpringKind::Edge,
            .vertices = {vertex_a, vertex_b},
            .rest_length = topology.edge_rest_length(edge_id),
        });

        if (topology.edge_is_boundary(edge_id)) {
            continue;
        }

        const auto [triangle_a, triangle_b] = topology.edge_adjacent_triangles(edge_id);
        const auto opposite_a = detail::opposite_triangle_vertex(topology, triangle_a, vertex_a, vertex_b);
        const auto opposite_b = detail::opposite_triangle_vertex(topology, triangle_b, vertex_a, vertex_b);
        const auto bend_rest_length = pbpt::math::length(
            topology.rest_position(opposite_b) - topology.rest_position(opposite_a)
        );

        network.springs.push_back(ClothSpring{
            .kind = ClothSpringKind::Bend,
            .vertices = {opposite_a, opposite_b},
            .rest_length = bend_rest_length,
        });
    }

    return network;
}

}  // namespace rtr::system::physics
