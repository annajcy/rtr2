#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "rtr/system/physics/ipc/contact/collision_mesh.hpp"

namespace rtr::system::physics::ipc {

struct PTCandidate {
    std::size_t point_vertex_idx{0};
    std::size_t triangle_idx{0};
};

struct EECandidate {
    std::size_t edge_a_idx{0};
    std::size_t edge_b_idx{0};
};

struct CollisionCandidates {
    std::vector<PTCandidate> pt_candidates{};
    std::vector<EECandidate> ee_candidates{};
};

struct BroadPhaseConfig {
    bool enable_aabb_prefilter{false};
    double aabb_padding{0.0};
};

namespace detail::collision_candidates {

struct AABB {
    Eigen::Vector3d min{Eigen::Vector3d::Zero()};
    Eigen::Vector3d max{Eigen::Vector3d::Zero()};
};

inline AABB make_padded_aabb(const std::array<Eigen::Vector3d, 3>& points, double padding) {
    AABB box{.min = points[0], .max = points[0]};
    for (std::size_t i = 1; i < points.size(); ++i) {
        box.min = box.min.cwiseMin(points[i]);
        box.max = box.max.cwiseMax(points[i]);
    }
    box.min.array() -= padding;
    box.max.array() += padding;
    return box;
}

inline AABB make_padded_point_aabb(const Eigen::Vector3d& point, double padding) {
    return AABB{
        .min = point.array() - padding,
        .max = point.array() + padding,
    };
}

inline bool aabb_overlaps(const AABB& a, const AABB& b) {
    return (a.min.array() <= b.max.array()).all() && (b.min.array() <= a.max.array()).all();
}

inline bool is_obstacle_triangle(const CollisionMesh& mesh, std::size_t triangle_idx) {
    return !detail::collision_mesh::triangle_is_deformable(mesh, triangle_idx);
}

inline bool is_obstacle_edge(const CollisionMesh& mesh, std::size_t edge_idx) {
    return !detail::collision_mesh::edge_is_deformable(mesh, edge_idx);
}

inline bool should_keep_pt_candidate(const CollisionMesh& mesh,
                                     std::size_t point_vertex_idx,
                                     std::size_t triangle_idx) {
    const auto& point = mesh.vertices.at(point_vertex_idx);
    const auto& triangle = mesh.triangles.at(triangle_idx);

    if (point_vertex_idx == triangle.vertices[0] || point_vertex_idx == triangle.vertices[1] ||
        point_vertex_idx == triangle.vertices[2]) {
        return false;
    }
    if (point.body_id == triangle.body_id) {
        return false;
    }

    const bool point_is_obstacle = detail::collision_mesh::is_obstacle_vertex(point);
    const bool triangle_is_obstacle = is_obstacle_triangle(mesh, triangle_idx);
    if (point_is_obstacle && triangle_is_obstacle) {
        return false;
    }
    if (!point_is_obstacle && !triangle_is_obstacle) {
        return false;
    }
    return true;
}

inline bool should_keep_ee_candidate(const CollisionMesh& mesh,
                                     std::size_t edge_a_idx,
                                     std::size_t edge_b_idx) {
    const auto& edge_a = mesh.edges.at(edge_a_idx);
    const auto& edge_b = mesh.edges.at(edge_b_idx);

    if (edge_a.body_id == edge_b.body_id) {
        return false;
    }
    if (edge_a.vertices[0] == edge_b.vertices[0] || edge_a.vertices[0] == edge_b.vertices[1] ||
        edge_a.vertices[1] == edge_b.vertices[0] || edge_a.vertices[1] == edge_b.vertices[1]) {
        return false;
    }

    const bool edge_a_is_obstacle = is_obstacle_edge(mesh, edge_a_idx);
    const bool edge_b_is_obstacle = is_obstacle_edge(mesh, edge_b_idx);
    if (edge_a_is_obstacle == edge_b_is_obstacle) {
        return false;
    }
    return true;
}

inline bool passes_pt_aabb_filter(const CollisionMesh& mesh,
                                  const IPCState& state,
                                  std::size_t point_vertex_idx,
                                  std::size_t triangle_idx,
                                  double padding) {
    const Eigen::Vector3d point = read_collision_vertex_position(mesh, state, point_vertex_idx);
    const auto& triangle = mesh.triangles.at(triangle_idx);
    const AABB point_box = make_padded_point_aabb(point, padding);
    const AABB triangle_box = make_padded_aabb(
        {
            read_collision_vertex_position(mesh, state, triangle.vertices[0]),
            read_collision_vertex_position(mesh, state, triangle.vertices[1]),
            read_collision_vertex_position(mesh, state, triangle.vertices[2]),
        },
        padding
    );
    return aabb_overlaps(point_box, triangle_box);
}

inline bool passes_ee_aabb_filter(const CollisionMesh& mesh,
                                  const IPCState& state,
                                  std::size_t edge_a_idx,
                                  std::size_t edge_b_idx,
                                  double padding) {
    const auto& edge_a = mesh.edges.at(edge_a_idx);
    const auto& edge_b = mesh.edges.at(edge_b_idx);
    const AABB edge_a_box = make_padded_aabb(
        {
            read_collision_vertex_position(mesh, state, edge_a.vertices[0]),
            read_collision_vertex_position(mesh, state, edge_a.vertices[1]),
            read_collision_vertex_position(mesh, state, edge_a.vertices[1]),
        },
        padding
    );
    const AABB edge_b_box = make_padded_aabb(
        {
            read_collision_vertex_position(mesh, state, edge_b.vertices[0]),
            read_collision_vertex_position(mesh, state, edge_b.vertices[1]),
            read_collision_vertex_position(mesh, state, edge_b.vertices[1]),
        },
        padding
    );
    return aabb_overlaps(edge_a_box, edge_b_box);
}

}  // namespace detail::collision_candidates

inline CollisionCandidates build_collision_candidates(const CollisionMesh& mesh,
                                                      const IPCState& state,
                                                      const BroadPhaseConfig& config = {}) {
    CollisionCandidates candidates{};

    candidates.pt_candidates.reserve(
        mesh.deformable_vertex_indices.size() * mesh.obstacle_triangle_indices.size() +
        mesh.obstacle_vertex_indices.size() * mesh.deformable_triangle_indices.size()
    );
    candidates.ee_candidates.reserve(mesh.deformable_edge_indices.size() * mesh.obstacle_edge_indices.size());

    const auto append_pt_candidates =
        [&](const std::vector<std::size_t>& point_indices, const std::vector<std::size_t>& triangle_indices) {
            for (const std::size_t point_vertex_idx : point_indices) {
                for (const std::size_t triangle_idx : triangle_indices) {
                    if (!detail::collision_candidates::should_keep_pt_candidate(
                            mesh,
                            point_vertex_idx,
                            triangle_idx
                        )) {
                        continue;
                    }
                    if (config.enable_aabb_prefilter &&
                        !detail::collision_candidates::passes_pt_aabb_filter(
                            mesh,
                            state,
                            point_vertex_idx,
                            triangle_idx,
                            config.aabb_padding
                        )) {
                        continue;
                    }
                    candidates.pt_candidates.push_back(PTCandidate{
                        .point_vertex_idx = point_vertex_idx,
                        .triangle_idx = triangle_idx,
                    });
                }
            }
        };

    append_pt_candidates(mesh.deformable_vertex_indices, mesh.obstacle_triangle_indices);
    append_pt_candidates(mesh.obstacle_vertex_indices, mesh.deformable_triangle_indices);

    for (const std::size_t edge_a_idx : mesh.deformable_edge_indices) {
        for (const std::size_t edge_b_idx : mesh.obstacle_edge_indices) {
            if (!detail::collision_candidates::should_keep_ee_candidate(mesh, edge_a_idx, edge_b_idx)) {
                continue;
            }
            if (config.enable_aabb_prefilter &&
                !detail::collision_candidates::passes_ee_aabb_filter(
                    mesh,
                    state,
                    edge_a_idx,
                    edge_b_idx,
                    config.aabb_padding
                )) {
                continue;
            }
            candidates.ee_candidates.push_back(EECandidate{
                .edge_a_idx = edge_a_idx,
                .edge_b_idx = edge_b_idx,
            });
        }
    }

    return candidates;
}

}  // namespace rtr::system::physics::ipc
