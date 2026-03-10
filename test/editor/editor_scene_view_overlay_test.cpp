#include <algorithm>
#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "rtr/editor/panel/scene_view_panel.hpp"

namespace rtr::editor::test {

TEST(EditorSceneViewOverlayTest, MeshEdgesDeduplicateSharedTriangleEdge) {
    utils::ObjMeshData mesh{};
    mesh.vertices.resize(4u);
    mesh.indices = {0u, 1u, 2u, 0u, 2u, 3u};

    const auto edges = scene_view_detail::build_unique_mesh_edges(mesh);

    ASSERT_EQ(edges.size(), 5u);
    EXPECT_EQ(edges[0], (scene_view_detail::MeshEdge{.a = 0u, .b = 1u}));
    EXPECT_EQ(edges[1], (scene_view_detail::MeshEdge{.a = 0u, .b = 2u}));
    EXPECT_EQ(edges[2], (scene_view_detail::MeshEdge{.a = 0u, .b = 3u}));
    EXPECT_EQ(edges[3], (scene_view_detail::MeshEdge{.a = 1u, .b = 2u}));
    EXPECT_EQ(edges[4], (scene_view_detail::MeshEdge{.a = 2u, .b = 3u}));
}

TEST(EditorSceneViewOverlayTest, ProjectWorldPointMapsClipCenterToViewportCenter) {
    const auto screen = scene_view_detail::project_world_to_viewport(
        pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        pbpt::math::Mat4::identity(),
        pbpt::math::Mat4::identity(),
        EditorViewportRect{.x = 10.0f, .y = 20.0f, .width = 200.0f, .height = 100.0f}
    );

    ASSERT_TRUE(screen.has_value());
    EXPECT_FLOAT_EQ(screen->x, 110.0f);
    EXPECT_FLOAT_EQ(screen->y, 70.0f);
}

TEST(EditorSceneViewOverlayTest, ProjectWorldPointRejectsNegativeClipW) {
    auto proj = pbpt::math::Mat4::identity();
    proj[3][3] = -1.0f;

    const auto screen = scene_view_detail::project_world_to_viewport(
        pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        pbpt::math::Mat4::identity(),
        proj,
        EditorViewportRect{.x = 0.0f, .y = 0.0f, .width = 100.0f, .height = 100.0f}
    );

    EXPECT_FALSE(screen.has_value());
}

TEST(EditorSceneViewOverlayTest, ProjectWorldPointRejectsNonFiniteClipCoordinates) {
    auto proj = pbpt::math::Mat4::identity();
    proj[0][0] = std::numeric_limits<float>::quiet_NaN();

    const auto screen = scene_view_detail::project_world_to_viewport(
        pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        pbpt::math::Mat4::identity(),
        proj,
        EditorViewportRect{.x = 0.0f, .y = 0.0f, .width = 100.0f, .height = 100.0f}
    );

    EXPECT_FALSE(screen.has_value());
}

TEST(EditorSceneViewOverlayTest, ImGuizmoProjectionMatrixRemovesVulkanYFlip) {
    const pbpt::math::Mat4 vulkan_projection =
        pbpt::math::scale(pbpt::math::Vec3{1.0f, -1.0f, 1.0f}) * pbpt::math::Mat4::identity();

    const auto imguizmo_projection = scene_view_detail::imguizmo_projection_matrix(vulkan_projection);

    EXPECT_EQ(imguizmo_projection, pbpt::math::Mat4::identity());
}

TEST(EditorSceneViewOverlayTest, SphereAndBoxWorldSizeHelpersMatchColliderFormula) {
    const float sphere_radius = scene_view_detail::sphere_world_radius(
        0.5f,
        pbpt::math::Vec3{2.0f, 1.0f, 3.0f},
        pbpt::math::Vec3{0.5f, 4.0f, 1.0f}
    );
    const auto box_half_extents = scene_view_detail::box_world_half_extents(
        pbpt::math::Vec3{1.0f, 2.0f, 3.0f},
        pbpt::math::Vec3{2.0f, 3.0f, 4.0f},
        pbpt::math::Vec3{0.5f, 1.5f, 2.0f}
    );

    EXPECT_FLOAT_EQ(sphere_radius, 2.0f);
    EXPECT_FLOAT_EQ(box_half_extents.x(), 1.0f);
    EXPECT_FLOAT_EQ(box_half_extents.y(), 9.0f);
    EXPECT_FLOAT_EQ(box_half_extents.z(), 24.0f);
}

TEST(EditorSceneViewOverlayTest, SphereRingPointsStayOnRequestedRadius) {
    const auto points = scene_view_detail::build_sphere_ring_points(
        pbpt::math::Vec3{1.0f, 2.0f, 3.0f},
        2.0f,
        pbpt::math::Vec3{1.0f, 0.0f, 0.0f},
        pbpt::math::Vec3{0.0f, 1.0f, 0.0f},
        16u
    );

    ASSERT_EQ(points.size(), 16u);
    for (const auto& point : points) {
        const auto offset = point - pbpt::math::Vec3{1.0f, 2.0f, 3.0f};
        EXPECT_NEAR(offset.length(), 2.0f, 1e-4f);
        EXPECT_NEAR(point.z(), 3.0f, 1e-4f);
    }
}

TEST(EditorSceneViewOverlayTest, BoxCornersUseCenterRotationAndHalfExtents) {
    const auto corners = scene_view_detail::build_box_corners(
        pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        pbpt::math::Quat::identity(),
        pbpt::math::Vec3{1.0f, 2.0f, 3.0f}
    );

    EXPECT_EQ(corners[0], (pbpt::math::Vec3{-1.0f, -2.0f, -3.0f}));
    EXPECT_EQ(corners[6], (pbpt::math::Vec3{1.0f, 2.0f, 3.0f}));
}

}  // namespace rtr::editor::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
