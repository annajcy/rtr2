#include <stdexcept>

#include "gtest/gtest.h"

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/trigonometric.hpp>

#include "rtr/framework/component/camera_control/trackball_camera_controller.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/system/input/input_state.hpp"
#include "rtr/system/input/input_types.hpp"

namespace rtr::framework::component::test {

static void expect_vec3_near(const glm::vec3& lhs, const glm::vec3& rhs, float eps = 1e-4f) {
    EXPECT_NEAR(lhs.x, rhs.x, eps);
    EXPECT_NEAR(lhs.y, rhs.y, eps);
    EXPECT_NEAR(lhs.z, rhs.z, eps);
}

TEST(FrameworkTrackballCameraControllerTest, LeftDragOrbitsAroundTargetAndPreservesRadius) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("camera_go");
    (void)scene.camera_manager().create_perspective_camera(go.id());
    go.node().set_world_position({0.0f, 0.0f, -10.0f});

    system::input::InputState input{};
    auto& controller = go.add_component<TrackBallCameraController>(&input, &scene.camera_manager());
    scene.scene_graph().update_world_transforms();

    const glm::vec3 before = go.node().world_position();
    const float before_radius = glm::length(before - controller.target());

    input.update_mouse_button(
        system::input::MouseButton::LEFT,
        system::input::KeyAction::PRESS,
        system::input::KeyMod::NONE
    );
    input.update_mouse_position(120.0, 40.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});

    const glm::vec3 after = go.node().world_position();
    const float after_radius = glm::length(after - controller.target());

    EXPECT_GT(glm::length(after - before), 1e-4f);
    EXPECT_NEAR(before_radius, after_radius, 1e-3f);
}

TEST(FrameworkTrackballCameraControllerTest, MiddleDragPansCameraAndTargetTogether) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("camera_go");
    (void)scene.camera_manager().create_perspective_camera(go.id());
    go.node().set_world_position({0.0f, 0.0f, -10.0f});

    system::input::InputState input{};
    auto& controller = go.add_component<TrackBallCameraController>(&input, &scene.camera_manager());
    scene.scene_graph().update_world_transforms();

    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});
    const glm::vec3 before_pos = go.node().world_position();
    const glm::vec3 before_target = controller.target();
    const glm::vec3 before_offset = before_target - before_pos;

    input.reset_deltas();
    input.update_mouse_button(
        system::input::MouseButton::MIDDLE,
        system::input::KeyAction::PRESS,
        system::input::KeyMod::NONE
    );
    input.update_mouse_position(80.0, -30.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 1});

    const glm::vec3 after_pos = go.node().world_position();
    const glm::vec3 after_target = controller.target();
    const glm::vec3 after_offset = after_target - after_pos;

    EXPECT_GT(glm::length(after_target - before_target), 1e-4f);
    expect_vec3_near(after_offset, before_offset, 1e-3f);
}

TEST(FrameworkTrackballCameraControllerTest, ScrollCallsAdjustZoomPerspective) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("camera_go");
    (void)scene.camera_manager().create_perspective_camera(go.id());
    go.node().set_world_position({0.0f, 0.0f, -5.0f});

    system::input::InputState input{};
    (void)go.add_component<TrackBallCameraController>(&input, &scene.camera_manager());
    scene.scene_graph().update_world_transforms();

    const glm::vec3 before = go.node().world_position();
    input.update_mouse_scroll(0.0, 1.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});
    const glm::vec3 after = go.node().world_position();

    EXPECT_NEAR(after.z - before.z, -0.35f, 1e-4f);
}

TEST(FrameworkTrackballCameraControllerTest, OnlyActiveCameraResponds) {
    core::Scene scene(1, "scene");
    auto& go_a = scene.create_game_object("camera_a");
    auto& go_b = scene.create_game_object("camera_b");
    (void)scene.camera_manager().create_perspective_camera(go_a.id());
    (void)scene.camera_manager().create_perspective_camera(go_b.id());
    go_a.node().set_world_position({0.0f, 0.0f, -10.0f});
    go_b.node().set_world_position({2.0f, 0.0f, -10.0f});

    system::input::InputState input{};
    (void)go_a.add_component<TrackBallCameraController>(&input, &scene.camera_manager());
    (void)go_b.add_component<TrackBallCameraController>(&input, &scene.camera_manager());
    scene.scene_graph().update_world_transforms();

    const glm::vec3 a_before = go_a.node().world_position();
    const glm::vec3 b_before = go_b.node().world_position();

    input.update_mouse_button(
        system::input::MouseButton::LEFT,
        system::input::KeyAction::PRESS,
        system::input::KeyMod::NONE
    );
    input.update_mouse_position(40.0, 10.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});

    const glm::vec3 a_after_first = go_a.node().world_position();
    const glm::vec3 b_after_first = go_b.node().world_position();
    EXPECT_GT(glm::length(a_after_first - a_before), 1e-4f);
    EXPECT_LE(glm::length(b_after_first - b_before), 1e-4f);

    ASSERT_TRUE(scene.set_active_camera(go_b.id()));
    input.reset_deltas();
    input.update_mouse_position(90.0, 20.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 1});

    const glm::vec3 a_after_second = go_a.node().world_position();
    const glm::vec3 b_after_second = go_b.node().world_position();
    EXPECT_LE(glm::length(a_after_second - a_after_first), 1e-4f);
    EXPECT_GT(glm::length(b_after_second - b_after_first), 1e-4f);
}

TEST(FrameworkTrackballCameraControllerTest, ThrowsWhenOwnerHasNoCamera) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("go");
    system::input::InputState input{};

    EXPECT_THROW(
        (void)go.add_component<TrackBallCameraController>(&input, &scene.camera_manager()),
        std::runtime_error
    );
}

TEST(FrameworkTrackballCameraControllerTest, PitchIsClamped) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("camera_go");
    (void)scene.camera_manager().create_perspective_camera(go.id());
    go.node().set_world_position({0.0f, 0.0f, -10.0f});

    system::input::InputState input{};
    auto& controller = go.add_component<TrackBallCameraController>(&input, &scene.camera_manager());
    scene.scene_graph().update_world_transforms();

    input.update_mouse_button(
        system::input::MouseButton::LEFT,
        system::input::KeyAction::PRESS,
        system::input::KeyMod::NONE
    );
    input.update_mouse_position(0.0, 8000.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});

    const glm::vec3 offset = go.node().world_position() - controller.target();
    const float radius = glm::max(glm::length(offset), 1e-5f);
    const float pitch_deg = glm::degrees(glm::asin(glm::clamp(offset.y / radius, -1.0f, 1.0f)));
    EXPECT_LE(pitch_deg, 89.0f + 1e-3f);
    EXPECT_GE(pitch_deg, -89.0f - 1e-3f);
}

TEST(FrameworkTrackballCameraControllerTest, LeftHasPriorityOverMiddle) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("camera_go");
    (void)scene.camera_manager().create_perspective_camera(go.id());
    go.node().set_world_position({0.0f, 0.0f, -10.0f});

    system::input::InputState input{};
    auto& controller = go.add_component<TrackBallCameraController>(&input, &scene.camera_manager());
    scene.scene_graph().update_world_transforms();
    const glm::vec3 before_pos = go.node().world_position();

    input.update_mouse_button(
        system::input::MouseButton::LEFT,
        system::input::KeyAction::PRESS,
        system::input::KeyMod::NONE
    );
    input.update_mouse_button(
        system::input::MouseButton::MIDDLE,
        system::input::KeyAction::PRESS,
        system::input::KeyMod::NONE
    );
    input.update_mouse_position(50.0, 25.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});

    EXPECT_GT(glm::length(go.node().world_position() - before_pos), 1e-4f);
    expect_vec3_near(controller.target(), glm::vec3(0.0f), 1e-5f);
}

TEST(FrameworkTrackballCameraControllerTest, CustomTargetOrbitWorks) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("camera_go");
    (void)scene.camera_manager().create_perspective_camera(go.id());
    go.node().set_world_position({5.0f, 0.0f, -10.0f});

    system::input::InputState input{};
    auto& controller = go.add_component<TrackBallCameraController>(&input, &scene.camera_manager());
    controller.set_target({5.0f, 0.0f, 0.0f});
    scene.scene_graph().update_world_transforms();

    const glm::vec3 target = controller.target();
    const glm::vec3 before = go.node().world_position();
    const float before_radius = glm::length(before - target);

    input.update_mouse_button(
        system::input::MouseButton::LEFT,
        system::input::KeyAction::PRESS,
        system::input::KeyMod::NONE
    );
    input.update_mouse_position(80.0, -20.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});

    const glm::vec3 after = go.node().world_position();
    const float after_radius = glm::length(after - target);
    EXPECT_GT(glm::length(after - before), 1e-4f);
    EXPECT_NEAR(before_radius, after_radius, 1e-3f);
}

TEST(FrameworkTrackballCameraControllerTest, InitializesLookingAtTargetBeforeMouseInput) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("camera_go");
    (void)scene.camera_manager().create_perspective_camera(go.id());
    go.node().set_world_position({3.0f, 2.0f, -6.0f});

    system::input::InputState input{};
    auto& controller = go.add_component<TrackBallCameraController>(&input, &scene.camera_manager());
    controller.set_target({0.0f, 0.0f, 0.0f});
    scene.scene_graph().update_world_transforms();

    // No mouse input; first tick should still align camera front to target.
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});

    const glm::vec3 pos = go.node().world_position();
    const glm::vec3 expect_front = glm::normalize(controller.target() - pos);
    const glm::vec3 actual_front = glm::normalize(go.node().world_back());
    EXPECT_GT(glm::dot(expect_front, actual_front), 0.999f);
}

} // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
