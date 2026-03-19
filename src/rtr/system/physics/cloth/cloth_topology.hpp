#pragma once

#include <pbpt/math/math.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "rtr/system/physics/cloth/cloth_types.hpp"

namespace rtr::system::physics {

struct ClothTriangle {
    std::array<VertexID, 3> vertices{kInvalidVertexID, kInvalidVertexID, kInvalidVertexID};
    std::array<EdgeID, 3> edges{kInvalidEdgeID, kInvalidEdgeID, kInvalidEdgeID};
};

struct ClothEdge {
    std::array<VertexID, 2> vertices{kInvalidVertexID, kInvalidVertexID};
    std::array<TriangleID, 2> adjacent_triangles{kInvalidTriangleID, kInvalidTriangleID};
};

struct ClothTopology {
    std::int32_t vertex_count{0};

    std::vector<ClothTriangle> triangles;
    std::vector<ClothEdge> edges;

    std::vector<std::uint32_t> render_triangle_indices;

    std::vector<std::vector<EdgeID>> vertex_edge_adjacency;
    std::vector<std::vector<TriangleID>> vertex_triangle_adjacency;

    std::vector<pbpt::math::Vec3> rest_positions;
    std::vector<pbpt::math::Float> edge_rest_lengths;

private:
    template <typename TId>
    static std::size_t checked_index(TId id, std::size_t size, const char* label) {
        if (id < 0 || static_cast<std::size_t>(id) >= size) {
            throw std::out_of_range(std::string(label) + " is invalid.");
        }
        return static_cast<std::size_t>(id);
    }

public:
    bool is_valid_vertex(VertexID id) const {
        return id >= 0 && static_cast<std::size_t>(id) < rest_positions.size();
    }

    bool is_valid_edge(EdgeID id) const {
        return id >= 0 && static_cast<std::size_t>(id) < edges.size();
    }

    bool is_valid_triangle(TriangleID id) const {
        return id >= 0 && static_cast<std::size_t>(id) < triangles.size();
    }

    const ClothEdge& edge(EdgeID id) const {
        return edges[checked_index(id, edges.size(), "EdgeID")];
    }

    const ClothTriangle& triangle(TriangleID id) const {
        return triangles[checked_index(id, triangles.size(), "TriangleID")];
    }

    std::array<VertexID, 2> edge_vertices(EdgeID id) const {
        return edge(id).vertices;
    }

    std::array<VertexID, 3> triangle_vertices(TriangleID id) const {
        return triangle(id).vertices;
    }

    std::array<EdgeID, 3> triangle_edges(TriangleID id) const {
        return triangle(id).edges;
    }

    std::span<const EdgeID> vertex_incident_edges(VertexID id) const {
        return vertex_edge_adjacency[checked_index(id, vertex_edge_adjacency.size(), "VertexID")];
    }

    std::span<const TriangleID> vertex_incident_triangles(VertexID id) const {
        return vertex_triangle_adjacency[checked_index(id, vertex_triangle_adjacency.size(), "VertexID")];
    }

    std::array<TriangleID, 2> edge_adjacent_triangles(EdgeID id) const {
        return edge(id).adjacent_triangles;
    }

    bool edge_is_boundary(EdgeID id) const {
        return edge(id).adjacent_triangles[1] == kInvalidTriangleID;
    }

    const pbpt::math::Vec3& rest_position(VertexID id) const {
        return rest_positions[checked_index(id, rest_positions.size(), "VertexID")];
    }

    pbpt::math::Float edge_rest_length(EdgeID id) const {
        return edge_rest_lengths[checked_index(id, edge_rest_lengths.size(), "EdgeID")];
    }
};

}  // namespace rtr::system::physics
