#pragma once

#include <pbpt/math/math.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "rtr/system/physics/cloth/cloth_topology.hpp"
#include "rtr/utils/obj_types.hpp"

namespace rtr::system::physics {

struct EdgeTriple {
    VertexID v0{kInvalidVertexID};
    VertexID v1{kInvalidVertexID};
    TriangleID triangle{kInvalidTriangleID};
    int edge_slot{-1};
};

inline ClothTopology build_cloth_topology(
    std::span<const pbpt::math::Vec3> local_positions,
    std::span<const std::uint32_t> triangle_indices
) {
    if (triangle_indices.empty() || triangle_indices.size() % 3 != 0) {
        throw std::invalid_argument("Cloth topology requires a non-empty triangle index list.");
    }
    if (local_positions.empty()) {
        throw std::invalid_argument("Cloth topology requires a non-empty vertex list.");
    }
    if (local_positions.size() > static_cast<std::size_t>(std::numeric_limits<VertexID>::max())) {
        throw std::invalid_argument("Cloth topology vertex count exceeds VertexID range.");
    }

    ClothTopology topology{};
    topology.vertex_count = static_cast<std::int32_t>(local_positions.size());
    topology.rest_positions.reserve(local_positions.size());
    topology.render_triangle_indices.assign(triangle_indices.begin(), triangle_indices.end());
    topology.vertex_edge_adjacency.resize(local_positions.size());
    topology.vertex_triangle_adjacency.resize(local_positions.size());

    topology.rest_positions.assign(local_positions.begin(), local_positions.end());

    const auto triangle_count = triangle_indices.size() / 3u;
    if (triangle_count > static_cast<std::size_t>(std::numeric_limits<TriangleID>::max())) {
        throw std::invalid_argument("Cloth topology triangle count exceeds TriangleID range.");
    }

    topology.triangles.resize(triangle_count);
    std::vector<EdgeTriple> triples;
    triples.reserve(triangle_count * 3u);

    auto push_edge = [&](TriangleID triangle_id, int edge_slot, VertexID a, VertexID b) {
        if (a == b) {
            throw std::invalid_argument("Cloth topology contains a degenerate edge.");
        }
        const auto [v0, v1] = std::minmax(a, b);
        triples.push_back(EdgeTriple{
            .v0 = v0,
            .v1 = v1,
            .triangle = triangle_id,
            .edge_slot = edge_slot,
        });
    };

    for (std::size_t triangle_index = 0; triangle_index < triangle_count; ++triangle_index) {
        const auto i0 = triangle_indices[triangle_index * 3u + 0u];
        const auto i1 = triangle_indices[triangle_index * 3u + 1u];
        const auto i2 = triangle_indices[triangle_index * 3u + 2u];

        if (i0 >= local_positions.size() || i1 >= local_positions.size() || i2 >= local_positions.size()) {
            throw std::invalid_argument("Cloth topology triangle index is out of range.");
        }
        if (i0 == i1 || i1 == i2 || i2 == i0) {
            throw std::invalid_argument("Cloth topology contains a degenerate triangle.");
        }

        const VertexID v0 = static_cast<VertexID>(i0);
        const VertexID v1 = static_cast<VertexID>(i1);
        const VertexID v2 = static_cast<VertexID>(i2);
        const auto triangle_id = static_cast<TriangleID>(triangle_index);

        const auto p0 = topology.rest_positions[static_cast<std::size_t>(v0)];
        const auto p1 = topology.rest_positions[static_cast<std::size_t>(v1)];
        const auto p2 = topology.rest_positions[static_cast<std::size_t>(v2)];
        const auto face_normal = pbpt::math::cross(p1 - p0, p2 - p0);
        if (pbpt::math::dot(face_normal, face_normal) <= 1e-12f) {
            throw std::invalid_argument("Cloth topology contains a degenerate triangle.");
        }

        topology.triangles[triangle_index].vertices = {v0, v1, v2};
        topology.vertex_triangle_adjacency[static_cast<std::size_t>(v0)].push_back(triangle_id);
        topology.vertex_triangle_adjacency[static_cast<std::size_t>(v1)].push_back(triangle_id);
        topology.vertex_triangle_adjacency[static_cast<std::size_t>(v2)].push_back(triangle_id);

        push_edge(triangle_id, 0, v0, v1);
        push_edge(triangle_id, 1, v1, v2);
        push_edge(triangle_id, 2, v2, v0);
    }

    std::sort(triples.begin(), triples.end(), [](const EdgeTriple& lhs, const EdgeTriple& rhs) {
        if (lhs.v0 != rhs.v0) {
            return lhs.v0 < rhs.v0;
        }
        if (lhs.v1 != rhs.v1) {
            return lhs.v1 < rhs.v1;
        }
        return lhs.triangle < rhs.triangle;
    });

    topology.edges.reserve(triples.size());
    topology.edge_rest_lengths.reserve(triples.size());

    for (std::size_t i = 0; i < triples.size();) {
        const VertexID v0 = triples[i].v0;
        const VertexID v1 = triples[i].v1;
        std::vector<EdgeTriple> group;
        while (i < triples.size() && triples[i].v0 == v0 && triples[i].v1 == v1) {
            group.push_back(triples[i]);
            ++i;
        }

        if (group.size() > 2u) {
            throw std::invalid_argument("Cloth topology contains a non-manifold edge.");
        }

        if (topology.edges.size() > static_cast<std::size_t>(std::numeric_limits<EdgeID>::max())) {
            throw std::invalid_argument("Cloth topology edge count exceeds EdgeID range.");
        }
        const auto edge_id = static_cast<EdgeID>(topology.edges.size());

        ClothEdge edge{};
        edge.vertices = {v0, v1};
        edge.adjacent_triangles = {group[0].triangle, kInvalidTriangleID};
        if (group.size() == 2u) {
            edge.adjacent_triangles[1] = group[1].triangle;
        }
        topology.edges.push_back(edge);

        const auto length = pbpt::math::length(
            topology.rest_positions[static_cast<std::size_t>(v1)] -
            topology.rest_positions[static_cast<std::size_t>(v0)]
        );
        topology.edge_rest_lengths.push_back(length);

        topology.vertex_edge_adjacency[static_cast<std::size_t>(v0)].push_back(edge_id);
        topology.vertex_edge_adjacency[static_cast<std::size_t>(v1)].push_back(edge_id);

        for (const auto& triple : group) {
            topology.triangles[static_cast<std::size_t>(triple.triangle)].edges[static_cast<std::size_t>(triple.edge_slot)] = edge_id;
        }
    }

    return topology;
}

inline ClothTopology build_cloth_topology(const utils::ObjMeshData& mesh) {
    std::vector<pbpt::math::Vec3> positions;
    positions.reserve(mesh.vertices.size());
    for (const auto& vertex : mesh.vertices) {
        positions.push_back(vertex.position);
    }
    return build_cloth_topology(positions, mesh.indices);
}

}  // namespace rtr::system::physics
