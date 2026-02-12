#include "gtest/gtest.h"

#include <stdexcept>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "framework/core/camera.hpp"
#include "framework/core/scene.hpp"

namespace rtr::framework::core::test {

static void expect_vec3_near(const glm::vec3& lhs, const glm::vec3& rhs, float eps = 1e-5f) {
    EXPECT_NEAR(lhs.x, rhs.x, eps);
    EXPECT_NEAR(lhs.y, rhs.y, eps);
    EXPECT_NEAR(lhs.z, rhs.z, eps);
}

static void expect_mat4_near(const glm::mat4& lhs, const glm::mat4& rhs, float eps = 1e-5f) {
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            EXPECT_NEAR(lhs[c][r], rhs[c][r], eps);
        }
    }
}

TEST(FrameworkCameraTest, CameraManagerSupportsMultipleCamerasAndActiveSelection) {
    Scene scene(1, "scene");
    auto& go_a = scene.create_game_object("camera_a");
    auto& go_b = scene.create_game_object("camera_b");

    auto& camera_a = scene.camera_manager().create_perspective_camera(go_a.id());
    auto& camera_b = scene.camera_manager().create_orthographic_camera(go_b.id());

    EXPECT_EQ(scene.camera_manager().camera_count(), 2u);
    ASSERT_NE(scene.active_camera(), nullptr);
    EXPECT_EQ(scene.active_camera(), &camera_a);
    EXPECT_EQ(scene.camera_manager().active_camera_owner_id(), go_a.id());
    EXPECT_EQ(scene.camera_manager().camera(go_b.id()), &camera_b);

    EXPECT_TRUE(scene.set_active_camera(go_b.id()));
    EXPECT_EQ(scene.active_camera(), &camera_b);
    EXPECT_EQ(scene.camera_manager().active_camera_owner_id(), go_b.id());
}

TEST(FrameworkCameraTest, CameraManagerRejectsDuplicateCameraOwner) {
    Scene scene(1, "scene");
    auto& go = scene.create_game_object("camera_go");

    (void)scene.camera_manager().create_perspective_camera(go.id());
    EXPECT_THROW(
        (void)scene.camera_manager().create_orthographic_camera(go.id()),
        std::runtime_error
    );
}

TEST(FrameworkCameraTest, SetActiveCameraFailsWhenOwnerHasNoCamera) {
    Scene scene(1, "scene");
    auto& go = scene.create_game_object("go");

    EXPECT_FALSE(scene.set_active_camera(go.id()));
}

TEST(FrameworkCameraTest, CreateCameraThrowsForInvalidOwner) {
    Scene scene(1, "scene");

    EXPECT_THROW(
        (void)scene.camera_manager().create_perspective_camera(core::kInvalidGameObjectId),
        std::runtime_error
    );
    EXPECT_THROW(
        (void)scene.camera_manager().create_orthographic_camera(9999),
        std::runtime_error
    );
}

TEST(FrameworkCameraTest, ActiveCameraRotatesToNextWhenDestroyed) {
    Scene scene(1, "scene");
    auto& go_a = scene.create_game_object("camera_a");
    auto& go_b = scene.create_game_object("camera_b");
    auto& go_c = scene.create_game_object("camera_c");

    auto& camera_a = scene.camera_manager().create_perspective_camera(go_a.id());
    auto& camera_b = scene.camera_manager().create_perspective_camera(go_b.id());
    auto& camera_c = scene.camera_manager().create_perspective_camera(go_c.id());
    (void)camera_a;
    (void)camera_b;
    (void)camera_c;

    ASSERT_TRUE(scene.set_active_camera(go_b.id()));
    EXPECT_EQ(scene.camera_manager().active_camera_owner_id(), go_b.id());

    EXPECT_TRUE(scene.camera_manager().destroy_camera(go_b.id()));
    EXPECT_EQ(scene.camera_manager().active_camera_owner_id(), go_c.id());

    EXPECT_TRUE(scene.camera_manager().destroy_camera(go_c.id()));
    EXPECT_EQ(scene.camera_manager().active_camera_owner_id(), go_a.id());

    EXPECT_TRUE(scene.camera_manager().destroy_camera(go_a.id()));
    EXPECT_EQ(scene.active_camera(), nullptr);
    EXPECT_EQ(scene.camera_manager().active_camera_owner_id(), core::kInvalidGameObjectId);
}

TEST(FrameworkCameraTest, DestroySubtreeRemovesBoundCameras) {
    Scene scene(1, "scene");
    auto& parent = scene.create_game_object("parent");
    auto& child = scene.create_game_object("child");
    auto& other = scene.create_game_object("other");
    ASSERT_TRUE(scene.scene_graph().set_parent(child.id(), parent.id(), false));

    (void)scene.camera_manager().create_perspective_camera(parent.id());
    (void)scene.camera_manager().create_orthographic_camera(child.id());
    (void)scene.camera_manager().create_perspective_camera(other.id());
    ASSERT_EQ(scene.camera_manager().camera_count(), 3u);

    EXPECT_TRUE(scene.destroy_game_object(parent.id()));
    EXPECT_EQ(scene.camera_manager().camera_count(), 1u);
    EXPECT_TRUE(scene.camera_manager().has_camera(other.id()));
    EXPECT_FALSE(scene.camera_manager().has_camera(parent.id()));
    EXPECT_FALSE(scene.camera_manager().has_camera(child.id()));
}

TEST(FrameworkCameraTest, CameraFrontMatchesGameObjectWorldFront) {
    Scene scene(1, "scene");
    auto& parent = scene.create_game_object("parent");
    auto& camera_go = scene.create_game_object("camera_go");
    auto& camera = scene.camera_manager().create_perspective_camera(camera_go.id());

    ASSERT_TRUE(scene.scene_graph().set_parent(camera_go.id(), parent.id(), false));
    scene.scene_graph().node(parent.id()).set_local_rotation(
        glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f))
    );
    scene.scene_graph().update_world_transforms();

    expect_vec3_near(camera.front(), scene.scene_graph().node(camera_go.id()).world_front());
}

TEST(FrameworkCameraTest, CameraViewMatrixUsesNodeWorldTransform) {
    Scene scene(1, "scene");
    auto& camera_go = scene.create_game_object("camera_go");
    auto& camera = scene.camera_manager().create_perspective_camera(camera_go.id());

    scene.scene_graph().node(camera_go.id()).set_local_position({1.0f, 2.0f, 3.0f});
    scene.scene_graph().update_world_transforms();

    const glm::mat4 expected = glm::lookAt(
        glm::vec3(1.0f, 2.0f, 3.0f),
        glm::vec3(1.0f, 2.0f, 4.0f),
        glm::vec3(0.0f, 1.0f, 0.0f)
    );
    expect_mat4_near(camera.view_matrix(), expected);
}

TEST(FrameworkCameraTest, PerspectiveAndOrthographicProjectionMatricesMatchGLM) {
    PerspectiveCamera perspective;
    perspective.fov_degrees() = 60.0f;
    perspective.aspect_ratio() = 2.0f;
    perspective.near_bound() = 0.2f;
    perspective.far_bound() = 200.0f;
    expect_mat4_near(
        perspective.projection_matrix(),
        glm::perspective(glm::radians(60.0f), 2.0f, 0.2f, 200.0f)
    );

    OrthographicCamera orthographic;
    orthographic.left_bound() = -10.0f;
    orthographic.right_bound() = 10.0f;
    orthographic.bottom_bound() = -4.0f;
    orthographic.top_bound() = 4.0f;
    orthographic.near_bound() = -20.0f;
    orthographic.far_bound() = 30.0f;
    expect_mat4_near(
        orthographic.projection_matrix(),
        glm::ortho(-10.0f, 10.0f, -4.0f, 4.0f, -20.0f, 30.0f)
    );
}

TEST(FrameworkCameraTest, ActiveCameraIsNullWhenNoCameraExists) {
    Scene scene(1, "scene");
    EXPECT_EQ(scene.active_camera(), nullptr);
    EXPECT_EQ(scene.camera_manager().camera_count(), 0u);
}

} // namespace rtr::framework::core::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
