#include <gtest/gtest.h>

#include <Eigen/Core>

#include "rtr/system/physics/ipc/contact/collision_mesh.hpp"

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

ObstacleBody make_single_triangle_obstacle() {
    ObstacleBody body{};
    body.positions = {
        Eigen::Vector3d(-1.0, -0.5, -1.0),
        Eigen::Vector3d(2.0, -0.5, -1.0),
        Eigen::Vector3d(-1.0, -0.5, 2.0),
    };
    body.triangles = {{{0, 1, 2}}};
    return body;
}

TEST(CollisionMeshTest, BuildsUnifiedSurfaceForTetAndObstacleBodies) {
    IPCSystem system{};
    const IPCBodyID tet_body_id = system.create_tet_body(make_single_tet_body());
    const IPCBodyID obstacle_body_id = system.create_obstacle_body(make_single_triangle_obstacle());
    system.initialize();

    const CollisionMesh mesh = build_collision_mesh(system);

    EXPECT_EQ(system.obstacle_body_count(), 1u);
    EXPECT_EQ(system.get_obstacle_body(obstacle_body_id).edge_count(), 3u);

    EXPECT_EQ(mesh.vertex_count(), 7u);
    EXPECT_EQ(mesh.triangle_count(), 5u);
    EXPECT_EQ(mesh.edge_count(), 9u);

    EXPECT_EQ(mesh.deformable_vertex_indices.size(), 4u);
    EXPECT_EQ(mesh.obstacle_vertex_indices.size(), 3u);
    EXPECT_EQ(mesh.deformable_triangle_indices.size(), 4u);
    EXPECT_EQ(mesh.obstacle_triangle_indices.size(), 1u);
    EXPECT_EQ(mesh.deformable_edge_indices.size(), 6u);
    EXPECT_EQ(mesh.obstacle_edge_indices.size(), 3u);

    for (std::size_t i = 0; i < 4u; ++i) {
        ASSERT_TRUE(mesh.vertices[i].global_vertex_index.has_value());
        EXPECT_EQ(mesh.vertices[i].body_id, tet_body_id);
        EXPECT_EQ(*mesh.vertices[i].global_vertex_index, i);
        EXPECT_EQ(mesh.vertices[i].kind, CollisionVertexKind::Deformable);
    }

    for (std::size_t i = 0; i < 3u; ++i) {
        const auto& vertex = mesh.vertices[4u + i];
        EXPECT_EQ(vertex.body_id, obstacle_body_id);
        EXPECT_EQ(vertex.kind, CollisionVertexKind::Obstacle);
        EXPECT_FALSE(vertex.global_vertex_index.has_value());
        EXPECT_TRUE(vertex.static_position.isApprox(system.get_obstacle_body(obstacle_body_id).positions[i], 0.0));
    }

    EXPECT_TRUE(read_collision_vertex_position(mesh, system.state(), 0u).isApprox(
        system.state().position(0u),
        0.0
    ));
    EXPECT_TRUE(read_collision_vertex_position(mesh, system.state(), 4u).isApprox(
        system.get_obstacle_body(obstacle_body_id).positions[0],
        0.0
    ));
}

}  // namespace
}  // namespace rtr::system::physics::ipc
