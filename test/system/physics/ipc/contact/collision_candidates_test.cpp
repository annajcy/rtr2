#include <gtest/gtest.h>

#include <Eigen/Core>

#include "rtr/system/physics/ipc/contact/collision_candidates.hpp"

namespace rtr::system::physics::ipc {
namespace {

TetBody make_single_tet_body() {
    TetBody body{};
    body.geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
    };
    body.geometry.tets = {{{0, 1, 2, 3}}};
    body.material = FixedCorotatedMaterial{
        .mass_density = 2.0,
        .youngs_modulus = 100.0,
        .poisson_ratio = 0.3,
    };
    return body;
}

ObstacleBody make_single_triangle_obstacle(double y = -0.5) {
    ObstacleBody body{};
    body.positions = {
        Eigen::Vector3d(-1.0, y, -1.0),
        Eigen::Vector3d(2.0, y, -1.0),
        Eigen::Vector3d(-1.0, y, 2.0),
    };
    body.triangles = {{{0, 1, 2}}};
    return body;
}

CollisionMesh make_manual_filter_test_mesh() {
    CollisionMesh mesh{};
    mesh.vertices = {
        CollisionVertex{.body_id = 10u, .kind = CollisionVertexKind::Deformable, .body_vertex_index = 0u,
                        .global_vertex_index = 0u},
        CollisionVertex{.body_id = 10u, .kind = CollisionVertexKind::Deformable, .body_vertex_index = 1u,
                        .global_vertex_index = 1u},
        CollisionVertex{.body_id = 10u, .kind = CollisionVertexKind::Deformable, .body_vertex_index = 2u,
                        .global_vertex_index = 2u},
        CollisionVertex{.body_id = 20u, .kind = CollisionVertexKind::Obstacle, .body_vertex_index = 0u,
                        .global_vertex_index = std::nullopt, .static_position = Eigen::Vector3d(2.0, 0.0, 0.0)},
        CollisionVertex{.body_id = 20u, .kind = CollisionVertexKind::Obstacle, .body_vertex_index = 1u,
                        .global_vertex_index = std::nullopt, .static_position = Eigen::Vector3d(2.0, 1.0, 0.0)},
        CollisionVertex{.body_id = 20u, .kind = CollisionVertexKind::Obstacle, .body_vertex_index = 2u,
                        .global_vertex_index = std::nullopt, .static_position = Eigen::Vector3d(2.0, 0.0, 1.0)},
    };
    mesh.triangles = {
        CollisionTriangle{.vertices = {0u, 1u, 2u}, .body_id = 10u},
        CollisionTriangle{.vertices = {3u, 4u, 5u}, .body_id = 20u},
    };
    mesh.edges = {
        CollisionEdge{.vertices = {0u, 1u}, .body_id = 10u},
        CollisionEdge{.vertices = {1u, 2u}, .body_id = 10u},
        CollisionEdge{.vertices = {1u, 3u}, .body_id = 20u},
        CollisionEdge{.vertices = {4u, 5u}, .body_id = 20u},
    };

    // Deliberately inject mixed groups to verify filtering instead of loop structure.
    mesh.deformable_vertex_indices = {0u};
    mesh.obstacle_vertex_indices = {3u};
    mesh.obstacle_triangle_indices = {0u, 1u};
    mesh.deformable_triangle_indices = {0u, 1u};
    mesh.deformable_edge_indices = {0u, 1u, 3u};
    mesh.obstacle_edge_indices = {1u, 2u, 3u};
    return mesh;
}

TEST(CollisionCandidatesTest, BuildsDeterministicDay2CandidateSets) {
    IPCSystem system{};
    system.create_tet_body(make_single_tet_body());
    system.create_obstacle_body(make_single_triangle_obstacle());
    system.initialize();

    const CollisionMesh mesh = build_collision_mesh(system);
    const CollisionCandidates candidates = build_collision_candidates(mesh, system.state());

    ASSERT_EQ(mesh.obstacle_triangle_indices.size(), 1u);
    const std::size_t obstacle_triangle_idx = mesh.obstacle_triangle_indices.front();

    EXPECT_EQ(candidates.pt_candidates.size(), 16u);
    for (std::size_t i = 0; i < 4u; ++i) {
        EXPECT_EQ(candidates.pt_candidates[i].point_vertex_idx, i);
        EXPECT_EQ(candidates.pt_candidates[i].triangle_idx, obstacle_triangle_idx);
    }

    EXPECT_EQ(candidates.ee_candidates.size(), 18u);
    EXPECT_EQ(candidates.ee_candidates.front().edge_a_idx, mesh.deformable_edge_indices.front());
    EXPECT_EQ(candidates.ee_candidates.front().edge_b_idx, mesh.obstacle_edge_indices.front());
    EXPECT_EQ(candidates.ee_candidates.back().edge_a_idx, mesh.deformable_edge_indices.back());
    EXPECT_EQ(candidates.ee_candidates.back().edge_b_idx, mesh.obstacle_edge_indices.back());
}

TEST(CollisionCandidatesTest, FilteringRemovesSameBodySharedEndpointAndSameSidePairs) {
    IPCState state{};
    state.resize(3u);
    state.position(0u) = Eigen::Vector3d(0.0, 0.0, 0.0);
    state.position(1u) = Eigen::Vector3d(1.0, 0.0, 0.0);
    state.position(2u) = Eigen::Vector3d(0.0, 1.0, 0.0);

    const CollisionMesh mesh = make_manual_filter_test_mesh();
    const CollisionCandidates candidates = build_collision_candidates(mesh, state);

    ASSERT_EQ(candidates.pt_candidates.size(), 2u);
    EXPECT_EQ(candidates.pt_candidates[0].point_vertex_idx, 0u);
    EXPECT_EQ(candidates.pt_candidates[0].triangle_idx, 1u);
    EXPECT_EQ(candidates.pt_candidates[1].point_vertex_idx, 3u);
    EXPECT_EQ(candidates.pt_candidates[1].triangle_idx, 0u);

    ASSERT_EQ(candidates.ee_candidates.size(), 3u);
    EXPECT_EQ(candidates.ee_candidates[0].edge_a_idx, 0u);
    EXPECT_EQ(candidates.ee_candidates[0].edge_b_idx, 3u);
    EXPECT_EQ(candidates.ee_candidates[1].edge_a_idx, 1u);
    EXPECT_EQ(candidates.ee_candidates[1].edge_b_idx, 3u);
    EXPECT_EQ(candidates.ee_candidates[2].edge_a_idx, 3u);
    EXPECT_EQ(candidates.ee_candidates[2].edge_b_idx, 1u);
}

TEST(CollisionCandidatesTest, OptionalAabbPrefilterCanRejectSeparatedPairs) {
    IPCSystem system{};
    system.create_tet_body(make_single_tet_body());
    system.create_obstacle_body(make_single_triangle_obstacle(-10.0));
    system.initialize();

    const CollisionMesh mesh = build_collision_mesh(system);
    const CollisionCandidates candidates = build_collision_candidates(
        mesh,
        system.state(),
        BroadPhaseConfig{.enable_aabb_prefilter = true, .aabb_padding = 0.0}
    );

    EXPECT_TRUE(candidates.pt_candidates.empty());
    EXPECT_TRUE(candidates.ee_candidates.empty());
}

}  // namespace
}  // namespace rtr::system::physics::ipc
