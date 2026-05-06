#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "rtr/system/physics/ipc/core/ipc_state.hpp"
#include "rtr/system/physics/ipc/core/ipc_system.hpp"
#include "rtr/system/physics/ipc/model/mesh_tet_converter/tet_to_mesh.hpp"

namespace rtr::system::physics::ipc {

enum class CollisionVertexKind {
    Deformable,
    Obstacle,
};

struct CollisionVertex {
    IPCBodyID body_id{0};
    CollisionVertexKind kind{CollisionVertexKind::Deformable};
    std::size_t body_vertex_index{0};
    std::optional<std::size_t> global_vertex_index{};
    Eigen::Vector3d static_position{Eigen::Vector3d::Zero()};
};

struct CollisionTriangle {
    std::array<std::size_t, 3> vertices{};
    IPCBodyID body_id{0};
};

struct CollisionEdge {
    std::array<std::size_t, 2> vertices{};
    IPCBodyID body_id{0};
};

struct CollisionMesh {
    std::vector<CollisionVertex> vertices{};
    std::vector<CollisionTriangle> triangles{};
    std::vector<CollisionEdge> edges{};

    std::vector<std::size_t> deformable_vertex_indices{};
    std::vector<std::size_t> obstacle_vertex_indices{};
    std::vector<std::size_t> deformable_triangle_indices{};
    std::vector<std::size_t> obstacle_triangle_indices{};
    std::vector<std::size_t> deformable_edge_indices{};
    std::vector<std::size_t> obstacle_edge_indices{};

    std::size_t vertex_count() const { return vertices.size(); }
    std::size_t triangle_count() const { return triangles.size(); }
    std::size_t edge_count() const { return edges.size(); }
};

namespace detail::collision_mesh {

inline bool is_deformable_vertex(const CollisionVertex& vertex) {
    return vertex.kind == CollisionVertexKind::Deformable;
}

inline bool is_obstacle_vertex(const CollisionVertex& vertex) {
    return vertex.kind == CollisionVertexKind::Obstacle;
}

inline bool triangle_is_deformable(const CollisionMesh& mesh, std::size_t triangle_idx) {
    const auto& triangle = mesh.triangles.at(triangle_idx);
    return is_deformable_vertex(mesh.vertices.at(triangle.vertices[0]));
}

inline bool edge_is_deformable(const CollisionMesh& mesh, std::size_t edge_idx) {
    const auto& edge = mesh.edges.at(edge_idx);
    return is_deformable_vertex(mesh.vertices.at(edge.vertices[0]));
}

inline void append_triangle_edges(std::vector<std::tuple<IPCBodyID, std::size_t, std::size_t>>& keys,
                                  IPCBodyID body_id,
                                  const std::array<std::size_t, 3>& triangle) {
    auto push_edge = [&](std::size_t a, std::size_t b) {
        if (a > b) {
            std::swap(a, b);
        }
        keys.emplace_back(body_id, a, b);
    };

    push_edge(triangle[0], triangle[1]);
    push_edge(triangle[1], triangle[2]);
    push_edge(triangle[0], triangle[2]);
}

inline void classify_primitives(CollisionMesh& mesh) {
    mesh.deformable_vertex_indices.clear();
    mesh.obstacle_vertex_indices.clear();
    mesh.deformable_triangle_indices.clear();
    mesh.obstacle_triangle_indices.clear();
    mesh.deformable_edge_indices.clear();
    mesh.obstacle_edge_indices.clear();

    for (std::size_t vertex_idx = 0; vertex_idx < mesh.vertices.size(); ++vertex_idx) {
        if (is_deformable_vertex(mesh.vertices[vertex_idx])) {
            mesh.deformable_vertex_indices.push_back(vertex_idx);
        } else {
            mesh.obstacle_vertex_indices.push_back(vertex_idx);
        }
    }

    for (std::size_t triangle_idx = 0; triangle_idx < mesh.triangles.size(); ++triangle_idx) {
        if (triangle_is_deformable(mesh, triangle_idx)) {
            mesh.deformable_triangle_indices.push_back(triangle_idx);
        } else {
            mesh.obstacle_triangle_indices.push_back(triangle_idx);
        }
    }

    for (std::size_t edge_idx = 0; edge_idx < mesh.edges.size(); ++edge_idx) {
        if (edge_is_deformable(mesh, edge_idx)) {
            mesh.deformable_edge_indices.push_back(edge_idx);
        } else {
            mesh.obstacle_edge_indices.push_back(edge_idx);
        }
    }
}

}  // namespace detail::collision_mesh

inline Eigen::Vector3d read_collision_vertex_position(const CollisionMesh& mesh,
                                                      const IPCState& state,
                                                      std::size_t surface_vertex_idx) {
    const auto& vertex = mesh.vertices.at(surface_vertex_idx);
    if (vertex.kind == CollisionVertexKind::Deformable) {
        if (!vertex.global_vertex_index.has_value()) {
            throw std::logic_error("Collision deformable vertex is missing a global vertex index.");
        }
        if (*vertex.global_vertex_index >= state.vertex_count()) {
            throw std::out_of_range("Collision deformable vertex global index out of range.");
        }
        return state.position(*vertex.global_vertex_index);
    }
    return vertex.static_position;
}

inline CollisionMesh build_collision_mesh(const IPCSystem& system) {
    CollisionMesh mesh{};

    for (const IPCBodyID body_id : system.tet_body_ids()) {
        const TetBody& body = system.get_tet_body(body_id);
        if (!body.info.enabled) {
            continue;
        }

        const TetSurfaceMapping surface = build_tet_surface_mapping(body);
        std::unordered_map<std::size_t, std::size_t> local_to_surface{};
        local_to_surface.reserve(surface.surface_vertex_ids.size());

        for (const uint32_t local_vertex_id_u32 : surface.surface_vertex_ids) {
            const std::size_t local_vertex_id = static_cast<std::size_t>(local_vertex_id_u32);
            const std::size_t surface_vertex_idx = mesh.vertices.size();
            local_to_surface.emplace(local_vertex_id, surface_vertex_idx);

            mesh.vertices.push_back(CollisionVertex{
                .body_id = body_id,
                .kind = CollisionVertexKind::Deformable,
                .body_vertex_index = local_vertex_id,
                .global_vertex_index = body.info.dof_offset / 3u + local_vertex_id,
                .static_position = Eigen::Vector3d::Zero(),
            });
        }

        if ((surface.surface_indices.size() % 3u) != 0u) {
            throw std::logic_error("TetSurfaceMapping surface indices must encode triangles.");
        }

        for (std::size_t i = 0; i < surface.surface_indices.size(); i += 3u) {
            const std::size_t a = local_to_surface.at(static_cast<std::size_t>(surface.surface_indices[i + 0u]));
            const std::size_t b = local_to_surface.at(static_cast<std::size_t>(surface.surface_indices[i + 1u]));
            const std::size_t c = local_to_surface.at(static_cast<std::size_t>(surface.surface_indices[i + 2u]));
            mesh.triangles.push_back(CollisionTriangle{
                .vertices = {a, b, c},
                .body_id = body_id,
            });
        }
    }

    for (const IPCBodyID body_id : system.obstacle_body_ids()) {
        const ObstacleBody& body = system.get_obstacle_body(body_id);
        if (!body.info.enabled) {
            continue;
        }

        const std::size_t vertex_offset = mesh.vertices.size();
        for (std::size_t local_vertex_id = 0; local_vertex_id < body.positions.size(); ++local_vertex_id) {
            mesh.vertices.push_back(CollisionVertex{
                .body_id = body_id,
                .kind = CollisionVertexKind::Obstacle,
                .body_vertex_index = local_vertex_id,
                .global_vertex_index = std::nullopt,
                .static_position = body.positions[local_vertex_id],
            });
        }

        for (const auto& triangle : body.triangles) {
            mesh.triangles.push_back(CollisionTriangle{
                .vertices = {
                    vertex_offset + triangle[0],
                    vertex_offset + triangle[1],
                    vertex_offset + triangle[2],
                },
                .body_id = body_id,
            });
        }
    }

    std::vector<std::tuple<IPCBodyID, std::size_t, std::size_t>> edge_keys{};
    edge_keys.reserve(mesh.triangles.size() * 3u);
    for (const auto& triangle : mesh.triangles) {
        detail::collision_mesh::append_triangle_edges(edge_keys, triangle.body_id, triangle.vertices);
    }

    std::sort(edge_keys.begin(), edge_keys.end());
    edge_keys.erase(std::unique(edge_keys.begin(), edge_keys.end()), edge_keys.end());

    mesh.edges.reserve(edge_keys.size());
    for (const auto& [body_id, a, b] : edge_keys) {
        mesh.edges.push_back(CollisionEdge{
            .vertices = {a, b},
            .body_id = body_id,
        });
    }

    detail::collision_mesh::classify_primitives(mesh);
    return mesh;
}

}  // namespace rtr::system::physics::ipc
