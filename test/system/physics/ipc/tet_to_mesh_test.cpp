#include <algorithm>
#include <array>
#include <vector>

#include <gtest/gtest.h>

#include "rtr/system/physics/ipc/model/mesh_tet_converter/tet_to_mesh.hpp"

namespace rtr::system::physics::ipc::test {

TEST(TetToMeshTest, BuildTetSurfaceMappingSingleTet) {
    TetGeometry geometry{};
    geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
    };
    geometry.tets = {{{0, 1, 2, 3}}};

    const TetSurfaceMapping surface = build_tet_surface_mapping(geometry);

    EXPECT_EQ(surface.surface_indices.size(), 12u);
    EXPECT_EQ(surface.surface_vertex_ids.size(), 4u);
    EXPECT_EQ(surface.surface_vertex_ids.front(), 0u);
    EXPECT_EQ(surface.surface_vertex_ids.back(), 3u);
}

TEST(TetToMeshTest, BuildTetSurfaceMappingSharedFace) {
    TetGeometry geometry{};
    geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(0.0, 0.0, -1.0),
    };
    geometry.tets = {{{0, 1, 2, 3}, {0, 2, 1, 4}}};

    const TetSurfaceMapping surface = build_tet_surface_mapping(geometry);

    EXPECT_EQ(surface.surface_indices.size(), 18u);
    EXPECT_EQ(surface.surface_vertex_ids.size(), 5u);
}

TEST(TetToMeshTest, TetDofsToSurfaceMesh) {
    TetGeometry geometry{};
    geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
    };
    geometry.tets = {{{0, 1, 2, 3}}};
    const TetSurfaceMapping surface = build_tet_surface_mapping(geometry);

    Eigen::VectorXd positions(12);
    positions << 0.0, 0.0, 0.0,
                 1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0;

    const utils::ObjMeshData mesh = tet_dofs_to_surface_mesh(positions, surface);

    ASSERT_EQ(mesh.vertices.size(), 4u);
    ASSERT_EQ(mesh.indices.size(), 12u);
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        EXPECT_NEAR(mesh.vertices[i].position.x(), static_cast<float>(positions[3 * i + 0]), 1e-6f);
        EXPECT_NEAR(mesh.vertices[i].position.y(), static_cast<float>(positions[3 * i + 1]), 1e-6f);
        EXPECT_NEAR(mesh.vertices[i].position.z(), static_cast<float>(positions[3 * i + 2]), 1e-6f);
        EXPECT_GT(pbpt::math::length(mesh.vertices[i].normal), 0.0f);
    }
}

TEST(TetToMeshTest, TetRestToSurfaceMesh) {
    TetGeometry geometry{};
    geometry.rest_positions = {
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d(1.5, 0.0, 0.0),
        Eigen::Vector3d(0.5, 1.0, 0.0),
        Eigen::Vector3d(0.5, 0.0, 1.0),
    };
    geometry.tets = {{{0, 1, 2, 3}}};
    const TetSurfaceMapping surface = build_tet_surface_mapping(geometry);

    const utils::ObjMeshData mesh = tet_rest_to_surface_mesh(geometry, surface);

    ASSERT_EQ(mesh.vertices.size(), surface.surface_vertex_ids.size());
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        const uint32_t original_vertex = surface.surface_vertex_ids[i];
        EXPECT_NEAR(mesh.vertices[i].position.x(), static_cast<float>(geometry.rest_positions[original_vertex].x()), 1e-6f);
        EXPECT_NEAR(mesh.vertices[i].position.y(), static_cast<float>(geometry.rest_positions[original_vertex].y()), 1e-6f);
        EXPECT_NEAR(mesh.vertices[i].position.z(), static_cast<float>(geometry.rest_positions[original_vertex].z()), 1e-6f);
    }
}

TEST(TetToMeshTest, TetDofsToSurfaceMeshVertexRemapping) {
    TetGeometry geometry{};
    geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(0.0, 0.0, -1.0),
    };
    geometry.tets = {{{0, 1, 2, 3}, {0, 2, 1, 4}}};
    const TetSurfaceMapping surface = build_tet_surface_mapping(geometry);

    Eigen::VectorXd positions(15);
    positions << 0.0, 0.0, 0.0,
                 1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0,
                 0.0, 0.0, -1.0;

    const utils::ObjMeshData mesh = tet_dofs_to_surface_mesh(positions, surface);

    ASSERT_EQ(mesh.vertices.size(), surface.surface_vertex_ids.size());
    for (uint32_t index : mesh.indices) {
        EXPECT_LT(index, mesh.vertices.size());
    }
}

TEST(TetToMeshTest, UpdateSurfaceMeshFromTetDofs) {
    TetGeometry geometry{};
    geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
    };
    geometry.tets = {{{0, 1, 2, 3}}};
    const TetSurfaceMapping surface = build_tet_surface_mapping(geometry);

    Eigen::VectorXd positions(12);
    positions << 0.0, 0.0, 0.0,
                 1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0;
    utils::ObjMeshData mesh = tet_dofs_to_surface_mesh(positions, surface);

    positions << 0.5, 0.0, 0.0,
                 1.5, 0.0, 0.0,
                 0.5, 1.0, 0.0,
                 0.5, 0.0, 1.0;
    update_surface_mesh_from_tet_dofs(mesh, positions, surface);

    ASSERT_EQ(mesh.vertices.size(), 4u);
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        EXPECT_NEAR(mesh.vertices[i].position.x(), static_cast<float>(positions[3 * i + 0]), 1e-6f);
        EXPECT_NEAR(mesh.vertices[i].position.y(), static_cast<float>(positions[3 * i + 1]), 1e-6f);
        EXPECT_NEAR(mesh.vertices[i].position.z(), static_cast<float>(positions[3 * i + 2]), 1e-6f);
        EXPECT_GT(pbpt::math::length(mesh.vertices[i].normal), 0.0f);
    }
}

TEST(TetToMeshTest, UpdateSurfaceMeshFromTetDofsSupportsVertexOffset) {
    TetGeometry geometry{};
    geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
    };
    geometry.tets = {{{0, 1, 2, 3}}};
    const TetSurfaceMapping surface = build_tet_surface_mapping(geometry);

    Eigen::VectorXd positions(24);
    positions <<
        -2.0, 0.0, 0.0,
        -1.0, 0.0, 0.0,
        -2.0, 1.0, 0.0,
        -2.0, 0.0, 1.0,
         0.5, 0.0, 0.0,
         1.5, 0.0, 0.0,
         0.5, 1.0, 0.0,
         0.5, 0.0, 1.0;
    utils::ObjMeshData mesh = tet_dofs_to_surface_mesh(positions, surface, 4u);

    positions <<
        -2.0, 0.0, 0.0,
        -1.0, 0.0, 0.0,
        -2.0, 1.0, 0.0,
        -2.0, 0.0, 1.0,
         1.0, 0.5, 0.0,
         2.0, 0.5, 0.0,
         1.0, 1.5, 0.0,
         1.0, 0.5, 1.0;
    update_surface_mesh_from_tet_dofs(mesh, positions, surface, 4u);

    ASSERT_EQ(mesh.vertices.size(), 4u);
    EXPECT_NEAR(mesh.vertices[0].position.x(), 1.0f, 1e-6f);
    EXPECT_NEAR(mesh.vertices[0].position.y(), 0.5f, 1e-6f);
    EXPECT_NEAR(mesh.vertices[1].position.x(), 2.0f, 1e-6f);
    EXPECT_NEAR(mesh.vertices[2].position.y(), 1.5f, 1e-6f);
}

}  // namespace rtr::system::physics::ipc::test
