#include <algorithm>
#include <array>
#include <vector>

#include <gtest/gtest.h>

#include "rtr/system/physics/ipc/model/tet_mesh_convert.hpp"

namespace rtr::system::physics::ipc::test {

TEST(TetMeshConvertTest, ExtractSurfaceSingleTet) {
    TetGeometry geometry{};
    geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
    };
    geometry.tets = {{{0, 1, 2, 3}}};

    const TetSurfaceResult surface = extract_tet_surface(geometry);

    EXPECT_EQ(surface.surface_indices.size(), 12u);
    EXPECT_EQ(surface.surface_vertex_ids.size(), 4u);
    EXPECT_EQ(surface.surface_vertex_ids.front(), 0u);
    EXPECT_EQ(surface.surface_vertex_ids.back(), 3u);
}

TEST(TetMeshConvertTest, ExtractSurfaceSharedFace) {
    TetGeometry geometry{};
    geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(0.0, 0.0, -1.0),
    };
    geometry.tets = {{{0, 1, 2, 3}, {0, 2, 1, 4}}};

    const TetSurfaceResult surface = extract_tet_surface(geometry);

    EXPECT_EQ(surface.surface_indices.size(), 18u);
    EXPECT_EQ(surface.surface_vertex_ids.size(), 5u);
}

TEST(TetMeshConvertTest, TetToMeshSingleTetFromVectorXd) {
    TetGeometry geometry{};
    geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
    };
    geometry.tets = {{{0, 1, 2, 3}}};
    const TetSurfaceResult surface = extract_tet_surface(geometry);

    Eigen::VectorXd positions(12);
    positions << 0.0, 0.0, 0.0,
                 1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0;

    const utils::ObjMeshData mesh = tet_to_mesh(positions, surface);

    ASSERT_EQ(mesh.vertices.size(), 4u);
    ASSERT_EQ(mesh.indices.size(), 12u);
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        EXPECT_NEAR(mesh.vertices[i].position.x(), static_cast<float>(positions[3 * i + 0]), 1e-6f);
        EXPECT_NEAR(mesh.vertices[i].position.y(), static_cast<float>(positions[3 * i + 1]), 1e-6f);
        EXPECT_NEAR(mesh.vertices[i].position.z(), static_cast<float>(positions[3 * i + 2]), 1e-6f);
        EXPECT_GT(pbpt::math::length(mesh.vertices[i].normal), 0.0f);
    }
}

TEST(TetMeshConvertTest, TetToMeshRestGeometry) {
    TetGeometry geometry{};
    geometry.rest_positions = {
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d(1.5, 0.0, 0.0),
        Eigen::Vector3d(0.5, 1.0, 0.0),
        Eigen::Vector3d(0.5, 0.0, 1.0),
    };
    geometry.tets = {{{0, 1, 2, 3}}};
    const TetSurfaceResult surface = extract_tet_surface(geometry);

    const utils::ObjMeshData mesh = tet_to_mesh(geometry, surface);

    ASSERT_EQ(mesh.vertices.size(), surface.surface_vertex_ids.size());
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        const uint32_t original_vertex = surface.surface_vertex_ids[i];
        EXPECT_NEAR(mesh.vertices[i].position.x(), static_cast<float>(geometry.rest_positions[original_vertex].x()), 1e-6f);
        EXPECT_NEAR(mesh.vertices[i].position.y(), static_cast<float>(geometry.rest_positions[original_vertex].y()), 1e-6f);
        EXPECT_NEAR(mesh.vertices[i].position.z(), static_cast<float>(geometry.rest_positions[original_vertex].z()), 1e-6f);
    }
}

TEST(TetMeshConvertTest, TetToMeshVertexRemapping) {
    TetGeometry geometry{};
    geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(0.0, 0.0, -1.0),
    };
    geometry.tets = {{{0, 1, 2, 3}, {0, 2, 1, 4}}};
    const TetSurfaceResult surface = extract_tet_surface(geometry);

    Eigen::VectorXd positions(15);
    positions << 0.0, 0.0, 0.0,
                 1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0,
                 0.0, 0.0, -1.0;

    const utils::ObjMeshData mesh = tet_to_mesh(positions, surface);

    ASSERT_EQ(mesh.vertices.size(), surface.surface_vertex_ids.size());
    for (uint32_t index : mesh.indices) {
        EXPECT_LT(index, mesh.vertices.size());
    }
}

TEST(TetMeshConvertTest, UpdateMeshPositions) {
    TetGeometry geometry{};
    geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
    };
    geometry.tets = {{{0, 1, 2, 3}}};
    const TetSurfaceResult surface = extract_tet_surface(geometry);

    Eigen::VectorXd positions(12);
    positions << 0.0, 0.0, 0.0,
                 1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0;
    utils::ObjMeshData mesh = tet_to_mesh(positions, surface);

    positions << 0.5, 0.0, 0.0,
                 1.5, 0.0, 0.0,
                 0.5, 1.0, 0.0,
                 0.5, 0.0, 1.0;
    update_mesh_positions(mesh, positions, surface);

    ASSERT_EQ(mesh.vertices.size(), 4u);
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        EXPECT_NEAR(mesh.vertices[i].position.x(), static_cast<float>(positions[3 * i + 0]), 1e-6f);
        EXPECT_NEAR(mesh.vertices[i].position.y(), static_cast<float>(positions[3 * i + 1]), 1e-6f);
        EXPECT_NEAR(mesh.vertices[i].position.z(), static_cast<float>(positions[3 * i + 2]), 1e-6f);
        EXPECT_GT(pbpt::math::length(mesh.vertices[i].normal), 0.0f);
    }
}

TEST(TetMeshConvertTest, MeshPositionsToEigen) {
    utils::ObjMeshData mesh{};
    mesh.vertices = {
        {.position = pbpt::math::Vec3(1.0f, 2.0f, 3.0f)},
        {.position = pbpt::math::Vec3(4.0f, 5.0f, 6.0f)},
        {.position = pbpt::math::Vec3(7.0f, 8.0f, 9.0f)},
    };
    mesh.indices = {0u, 1u, 2u};

    const std::vector<Eigen::Vector3d> positions = mesh_positions_to_eigen(mesh);

    ASSERT_EQ(positions.size(), mesh.vertices.size());
    EXPECT_DOUBLE_EQ(positions[0].x(), 1.0);
    EXPECT_DOUBLE_EQ(positions[1].y(), 5.0);
    EXPECT_DOUBLE_EQ(positions[2].z(), 9.0);
}

TEST(TetMeshConvertTest, MeshTriangles) {
    utils::ObjMeshData mesh{};
    mesh.vertices = {
        {.position = pbpt::math::Vec3(0.0f, 0.0f, 0.0f)},
        {.position = pbpt::math::Vec3(1.0f, 0.0f, 0.0f)},
        {.position = pbpt::math::Vec3(0.0f, 1.0f, 0.0f)},
        {.position = pbpt::math::Vec3(0.0f, 0.0f, 1.0f)},
    };
    mesh.indices = {0u, 1u, 2u, 0u, 2u, 3u};

    const auto triangles = mesh_triangles(mesh);

    ASSERT_EQ(triangles.size(), 2u);
    EXPECT_EQ(triangles[0], (std::array<uint32_t, 3>{0u, 1u, 2u}));
    EXPECT_EQ(triangles[1], (std::array<uint32_t, 3>{0u, 2u, 3u}));
}

TEST(TetMeshConvertTest, MeshTrianglesRejectsMalformedInput) {
    utils::ObjMeshData malformed{};
    malformed.vertices = {
        {.position = pbpt::math::Vec3(0.0f, 0.0f, 0.0f)},
        {.position = pbpt::math::Vec3(1.0f, 0.0f, 0.0f)},
        {.position = pbpt::math::Vec3(0.0f, 1.0f, 0.0f)},
    };
    malformed.indices = {0u, 1u, 2u, 1u};
    EXPECT_THROW(mesh_triangles(malformed), std::invalid_argument);

    utils::ObjMeshData out_of_range{};
    out_of_range.vertices = malformed.vertices;
    out_of_range.indices = {0u, 1u, 3u};
    EXPECT_THROW(mesh_triangles(out_of_range), std::out_of_range);
}

}  // namespace rtr::system::physics::ipc::test
