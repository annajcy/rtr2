#include <algorithm>
#include <array>
#include <stdexcept>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "rtr/system/physics/ipc/core/ipc_system.hpp"
#include "rtr/system/physics/ipc/model/mesh_tet_converter/mesh_to_tet.hpp"

namespace rtr::system::physics::ipc::test {

namespace {

utils::ObjMeshData make_closed_tetra_surface_mesh() {
    utils::ObjMeshData mesh{};
    mesh.vertices = {
        {.position = pbpt::math::Vec3(0.0f, 0.0f, 0.0f)},
        {.position = pbpt::math::Vec3(1.0f, 0.0f, 0.0f)},
        {.position = pbpt::math::Vec3(0.0f, 1.0f, 0.0f)},
        {.position = pbpt::math::Vec3(0.0f, 0.0f, 1.0f)},
    };
    mesh.indices = {
        0u, 2u, 1u,
        0u, 1u, 3u,
        1u, 2u, 3u,
        2u, 0u, 3u,
    };
    return mesh;
}

double tet_signed_determinant(const TetGeometry& geometry, const std::array<std::size_t, 4>& tet) {
    const Eigen::Vector3d& x0 = geometry.rest_positions[tet[0]];
    Eigen::Matrix3d dm{};
    dm.col(0) = geometry.rest_positions[tet[1]] - x0;
    dm.col(1) = geometry.rest_positions[tet[2]] - x0;
    dm.col(2) = geometry.rest_positions[tet[3]] - x0;
    return dm.determinant();
}

}  // namespace

TEST(MeshToTetTest, FtetwildAvailabilityMatchesBuildFlag) {
#ifdef RTR_HAS_FTETWILD
    EXPECT_TRUE(ftetwild_available());
#else
    EXPECT_FALSE(ftetwild_available());
#endif
}

TEST(MeshToTetTest, ReportsUnavailableWhenFeatureDisabled) {
    if (ftetwild_available()) {
        GTEST_SKIP() << "fTetWild is enabled in this build.";
    }

    const TetMeshingResult result = tetrahedralize_obj_mesh(make_closed_tetra_surface_mesh());
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

TEST(MeshToTetTest, ObjMeshToEigenPositions) {
    utils::ObjMeshData mesh{};
    mesh.vertices = {
        {.position = pbpt::math::Vec3(1.0f, 2.0f, 3.0f)},
        {.position = pbpt::math::Vec3(4.0f, 5.0f, 6.0f)},
        {.position = pbpt::math::Vec3(7.0f, 8.0f, 9.0f)},
    };
    mesh.indices = {0u, 1u, 2u};

    const std::vector<Eigen::Vector3d> positions = obj_mesh_to_eigen_positions(mesh);

    ASSERT_EQ(positions.size(), mesh.vertices.size());
    EXPECT_DOUBLE_EQ(positions[0].x(), 1.0);
    EXPECT_DOUBLE_EQ(positions[1].y(), 5.0);
    EXPECT_DOUBLE_EQ(positions[2].z(), 9.0);
}

TEST(MeshToTetTest, ObjMeshToTriangleIndices) {
    utils::ObjMeshData mesh{};
    mesh.vertices = {
        {.position = pbpt::math::Vec3(0.0f, 0.0f, 0.0f)},
        {.position = pbpt::math::Vec3(1.0f, 0.0f, 0.0f)},
        {.position = pbpt::math::Vec3(0.0f, 1.0f, 0.0f)},
        {.position = pbpt::math::Vec3(0.0f, 0.0f, 1.0f)},
    };
    mesh.indices = {0u, 1u, 2u, 0u, 2u, 3u};

    const auto triangles = obj_mesh_to_triangle_indices(mesh);

    ASSERT_EQ(triangles.size(), 2u);
    EXPECT_EQ(triangles[0], (std::array<uint32_t, 3>{0u, 1u, 2u}));
    EXPECT_EQ(triangles[1], (std::array<uint32_t, 3>{0u, 2u, 3u}));
}

TEST(MeshToTetTest, TetrahedralizeClosedTetraSurfaceMesh) {
    if (!ftetwild_available()) {
        GTEST_SKIP() << "fTetWild is not available in this build.";
    }

    const TetMeshingResult result = tetrahedralize_obj_mesh(make_closed_tetra_surface_mesh());

    ASSERT_TRUE(result.success) << result.error_message;
    ASSERT_GT(result.geometry.vertex_count(), 0u);
    ASSERT_GT(result.geometry.tet_count(), 0u);
    for (const auto& tet : result.geometry.tets) {
        for (const std::size_t vertex_index : tet) {
            EXPECT_LT(vertex_index, result.geometry.vertex_count());
        }
        EXPECT_GT(tet_signed_determinant(result.geometry, tet), 0.0);
    }
}

TEST(MeshToTetTest, ObjMeshToTetBodySupportsFixedVerticesAndIPCSystem) {
    if (!ftetwild_available()) {
        GTEST_SKIP() << "fTetWild is not available in this build.";
    }

    TetBody body = obj_mesh_to_tet_body(make_closed_tetra_surface_mesh());

    ASSERT_GT(body.vertex_count(), 0u);
    body.fixed_vertices.assign(body.vertex_count(), false);
    const double max_y = std::max_element(
        body.geometry.rest_positions.begin(),
        body.geometry.rest_positions.end(),
        [](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) { return lhs.y() < rhs.y(); }
    )->y();
    bool fixed_any_vertex = false;
    for (std::size_t vertex_index = 0; vertex_index < body.vertex_count(); ++vertex_index) {
        if (body.geometry.rest_positions[vertex_index].y() >= max_y - 1e-9) {
            body.fixed_vertices[vertex_index] = true;
            fixed_any_vertex = true;
        }
    }
    ASSERT_TRUE(fixed_any_vertex);

    EXPECT_NO_THROW(body.precompute());

    IPCSystem system{};
    const IPCBodyID body_id = system.create_tet_body(body);
    EXPECT_NO_THROW(system.initialize());
    EXPECT_TRUE(system.initialized());
    EXPECT_TRUE(system.has_tet_body(body_id));
    EXPECT_NO_THROW(system.step(0.01));
}

TEST(MeshToTetTest, RejectsEmptyMesh) {
    if (!ftetwild_available()) {
        GTEST_SKIP() << "fTetWild is not available in this build.";
    }

    const TetMeshingResult result = tetrahedralize_obj_mesh(utils::ObjMeshData{});
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

TEST(MeshToTetTest, RejectsMalformedTriangleIndices) {
    if (!ftetwild_available()) {
        GTEST_SKIP() << "fTetWild is not available in this build.";
    }

    utils::ObjMeshData mesh = make_closed_tetra_surface_mesh();
    mesh.indices.push_back(0u);

    const TetMeshingResult result = tetrahedralize_obj_mesh(mesh);
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

TEST(MeshToTetTest, RejectsOutOfRangeIndices) {
    if (!ftetwild_available()) {
        GTEST_SKIP() << "fTetWild is not available in this build.";
    }

    utils::ObjMeshData mesh = make_closed_tetra_surface_mesh();
    mesh.indices[0] = 99u;

    const TetMeshingResult result = tetrahedralize_obj_mesh(mesh);
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

}  // namespace rtr::system::physics::ipc::test
