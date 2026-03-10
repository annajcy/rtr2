#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/framework/core/scene.hpp"

namespace rtr::framework::core::test {

static void expect_vec3_near(const pbpt::math::Vec3& lhs, const pbpt::math::Vec3& rhs, float eps = 1e-5f) {
    EXPECT_NEAR(lhs.x(), rhs.x(), eps);
    EXPECT_NEAR(lhs.y(), rhs.y(), eps);
    EXPECT_NEAR(lhs.z(), rhs.z(), eps);
}

static void expect_mat4_near(const pbpt::math::Mat4& lhs, const pbpt::math::Mat4& rhs, float eps = 1e-5f) {
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            EXPECT_NEAR(lhs[c][r], rhs[c][r], eps);
        }
    }
}

TEST(FrameworkCameraTest, PerspectiveProjectionMatchesPbptProjectionHelper) {
    Scene scene(1);
    auto& go     = scene.create_game_object("camera");
    auto& camera = go.add_component<component::PerspectiveCamera>();

    camera.fov_degrees()  = 60.0f;
    camera.aspect_ratio() = 2.0f;
    camera.near_bound()   = 0.2f;
    camera.far_bound()    = 200.0f;

    const pbpt::math::Mat4 expected =
        pbpt::math::scale(pbpt::math::Vec3{1.0f, -1.0f, 1.0f}) *
        pbpt::math::perspective(pbpt::math::radians(60.0f), 2.0f, -0.2f, -200.0f);
    expect_mat4_near(camera.projection_matrix(), expected);
}

TEST(FrameworkCameraTest, OrthographicProjectionMatchesPbptProjectionHelper) {
    Scene scene(1);
    auto& go     = scene.create_game_object("camera");
    auto& camera = go.add_component<component::OrthographicCamera>();

    camera.left_bound()   = -10.0f;
    camera.right_bound()  = 10.0f;
    camera.bottom_bound() = -4.0f;
    camera.top_bound()    = 4.0f;
    camera.near_bound()   = 20.0f;
    camera.far_bound()    = 30.0f;

    const pbpt::math::Mat4 expected =
        pbpt::math::scale(pbpt::math::Vec3{1.0f, -1.0f, 1.0f}) *
        pbpt::math::orthographic(-10.0f, 10.0f, -4.0f, 4.0f, -20.0f, -30.0f);
    expect_mat4_near(camera.projection_matrix(), expected);
}

TEST(FrameworkCameraTest, ViewMatrixUsesNodeWorldTransform) {
    Scene scene(1);
    auto& go     = scene.create_game_object("camera");
    auto& camera = go.add_component<component::PerspectiveCamera>();

    go.node().set_local_position({1.0f, 2.0f, 3.0f});
    scene.scene_graph().update_world_transforms();

    const pbpt::math::Mat4 expected = pbpt::math::look_at(
        pbpt::math::Vec3(1.0f, 2.0f, 3.0f), pbpt::math::Vec3(1.0f, 2.0f, 2.0f), pbpt::math::Vec3(0.0f, 1.0f, 0.0f));
    expect_mat4_near(camera.view_matrix(), expected);
}

TEST(FrameworkCameraTest, PerspectiveProjectionKeepsForwardPointInsideDepthRange) {
    Scene scene(1);
    auto& go     = scene.create_game_object("camera");
    auto& camera = go.add_component<component::PerspectiveCamera>();

    camera.fov_degrees()  = 45.0f;
    camera.aspect_ratio() = 16.0f / 9.0f;
    camera.near_bound()   = 0.1f;
    camera.far_bound()    = 100.0f;

    go.node().set_world_position({0.0f, 1.0f, 6.0f});
    camera.camera_look_at_point_world({0.0f, 0.0f, 0.0f});
    scene.scene_graph().update_world_transforms();

    const pbpt::math::Vec4 clip = camera.projection_matrix() * (camera.view_matrix() * pbpt::math::Vec4{0.0f, 0.0f, 0.0f, 1.0f});
    const float            ndc_z = clip.z() / clip.w();

    EXPECT_GT(clip.w(), 0.0f);
    EXPECT_GE(ndc_z, 0.0f);
    EXPECT_LE(ndc_z, 1.0f);
}

TEST(FrameworkCameraTest, LookAtDirectionLocalAndWorldDifferWithParentRotation) {
    Scene scene(1);
    auto& parent    = scene.create_game_object("parent");
    auto& camera_go = scene.create_game_object("camera");
    auto& camera    = camera_go.add_component<component::PerspectiveCamera>();
    ASSERT_TRUE(scene.scene_graph().set_parent(camera_go.id(), parent.id(), false));

    scene.scene_graph()
        .node(parent.id())
        .set_local_rotation(pbpt::math::Quat::from_axis_angle(pbpt::math::radians(90.0f), pbpt::math::Vec3(0.0f, 1.0f, 0.0f)));
    scene.scene_graph().node(camera_go.id()).set_local_rotation(pbpt::math::Quat::identity());

    camera.camera_look_at_direction_local({0.0f, 0.0f, -1.0f});
    scene.scene_graph().update_world_transforms();
    const pbpt::math::Vec3 front_after_local = camera.camera_world_front();

    scene.scene_graph().node(camera_go.id()).set_local_rotation(pbpt::math::Quat::identity());
    camera.camera_look_at_direction_world({0.0f, 0.0f, -1.0f});
    scene.scene_graph().update_world_transforms();
    const pbpt::math::Vec3 front_after_world = camera.camera_world_front();

    expect_vec3_near(front_after_local, {-1.0f, 0.0f, 0.0f});
    expect_vec3_near(front_after_world, {0.0f, 0.0f, -1.0f});
}

TEST(FrameworkCameraTest, PerspectiveAdjustZoomMovesAlongFront) {
    Scene scene(1);
    auto& go     = scene.create_game_object("camera");
    auto& camera = go.add_component<component::PerspectiveCamera>();

    go.node().set_world_position({0.0f, 0.0f, 0.0f});
    scene.scene_graph().update_world_transforms();
    camera.adjust_zoom(1.25f);

    const auto pos = go.node().world_position();
    EXPECT_NEAR(pos.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(pos.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(pos.z(), -1.25f, 1e-5f);
}

TEST(FrameworkCameraTest, OrthographicAdjustZoomExpandsBoundsAroundCenter) {
    Scene scene(1);
    auto& go              = scene.create_game_object("camera");
    auto& camera          = go.add_component<component::OrthographicCamera>();
    camera.left_bound()   = -2.0f;
    camera.right_bound()  = 2.0f;
    camera.bottom_bound() = -1.0f;
    camera.top_bound()    = 1.0f;

    camera.adjust_zoom(0.5f);
    EXPECT_NEAR(camera.left_bound(), -2.5f, 1e-5f);
    EXPECT_NEAR(camera.right_bound(), 2.5f, 1e-5f);
    EXPECT_NEAR(camera.bottom_bound(), -1.5f, 1e-5f);
    EXPECT_NEAR(camera.top_bound(), 1.5f, 1e-5f);
}

TEST(FrameworkCameraTest, CameraDefaultsToInactive) {
    Scene scene(1);
    auto& go     = scene.create_game_object("camera");
    auto& camera = go.add_component<component::PerspectiveCamera>();
    EXPECT_FALSE(camera.active());
}

TEST(FrameworkCameraTest, AddingSecondCameraComponentOnSameGameObjectThrows) {
    Scene scene(1);
    auto& go = scene.create_game_object("camera");
    (void)go.add_component<component::PerspectiveCamera>();
    EXPECT_THROW((void)go.add_component<component::OrthographicCamera>(), std::runtime_error);
}

}  // namespace rtr::framework::core::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
