#include <pbpt/math/math.h>

#include <stdexcept>

#include "gtest/gtest.h"

#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/framework/component/camera_control/trackball_camera_controller.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/system/input/input_state.hpp"
#include "rtr/system/input/input_types.hpp"

namespace rtr::framework::component::test {

static void expect_vec3_near(const pbpt::math::vec3& lhs, const pbpt::math::vec3& rhs, float eps = 1e-4f) {
    EXPECT_NEAR(lhs.x(), rhs.x(), eps);
    EXPECT_NEAR(lhs.y(), rhs.y(), eps);
    EXPECT_NEAR(lhs.z(), rhs.z(), eps);
}

TEST(FrameworkTrackballCameraControllerTest, LeftDragOrbitsAroundTargetAndPreservesRadius) {
    core::Scene scene(1);
    auto&       go     = scene.create_game_object("camera_go");
    auto&       camera = go.add_component<PerspectiveCamera>();
    camera.set_active(true);
    go.node().set_world_position({0.0f, 0.0f, -10.0f});

    system::input::InputState input{};
    auto&                     controller = go.add_component<TrackBallCameraController>(input);

    const pbpt::math::vec3 before        = go.node().world_position();
    const float            before_radius = pbpt::math::length(before - controller.target());

    input.update_mouse_button(system::input::MouseButton::LEFT, system::input::KeyAction::PRESS,
                              system::input::KeyMod::NONE);
    input.update_mouse_position(120.0, 40.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});

    const pbpt::math::vec3 after        = go.node().world_position();
    const float            after_radius = pbpt::math::length(after - controller.target());

    EXPECT_GT(pbpt::math::length(after - before), 1e-4f);
    EXPECT_NEAR(before_radius, after_radius, 1e-3f);
}

TEST(FrameworkTrackballCameraControllerTest, MiddleDragPansCameraAndTargetTogether) {
    core::Scene scene(1);
    auto&       go     = scene.create_game_object("camera_go");
    auto&       camera = go.add_component<PerspectiveCamera>();
    camera.set_active(true);
    go.node().set_world_position({0.0f, 0.0f, -10.0f});

    system::input::InputState input{};
    auto&                     controller = go.add_component<TrackBallCameraController>(input);

    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});
    const pbpt::math::vec3 before_pos    = go.node().world_position();
    const pbpt::math::vec3 before_target = controller.target();
    const pbpt::math::vec3 before_offset = before_target - before_pos;

    input.reset_deltas();
    input.update_mouse_button(system::input::MouseButton::MIDDLE, system::input::KeyAction::PRESS,
                              system::input::KeyMod::NONE);
    input.update_mouse_position(80.0, -30.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 1});

    const pbpt::math::vec3 after_pos    = go.node().world_position();
    const pbpt::math::vec3 after_target = controller.target();
    const pbpt::math::vec3 after_offset = after_target - after_pos;

    EXPECT_GT(pbpt::math::length(after_target - before_target), 1e-4f);
    expect_vec3_near(after_offset, before_offset, 1e-3f);
}

TEST(FrameworkTrackballCameraControllerTest, ScrollCallsAdjustZoomPerspective) {
    core::Scene scene(1);
    auto&       go     = scene.create_game_object("camera_go");
    auto&       camera = go.add_component<PerspectiveCamera>();
    camera.set_active(true);
    go.node().set_world_position({0.0f, 0.0f, -5.0f});

    system::input::InputState input{};
    (void)go.add_component<TrackBallCameraController>(input);

    const pbpt::math::vec3 before = go.node().world_position();
    input.update_mouse_scroll(0.0, 1.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});
    const pbpt::math::vec3 after = go.node().world_position();

    EXPECT_NEAR(after.z() - before.z(), 0.35f, 1e-4f);
}

TEST(FrameworkTrackballCameraControllerTest, OnlyActiveCameraResponds) {
    core::Scene scene(1);
    auto&       go_a     = scene.create_game_object("camera_a");
    auto&       go_b     = scene.create_game_object("camera_b");
    auto&       camera_a = go_a.add_component<PerspectiveCamera>();
    auto&       camera_b = go_b.add_component<PerspectiveCamera>();
    camera_a.set_active(true);
    camera_b.set_active(false);
    go_a.node().set_world_position({0.0f, 0.0f, -10.0f});
    go_b.node().set_world_position({2.0f, 0.0f, -10.0f});

    system::input::InputState input{};
    (void)go_a.add_component<TrackBallCameraController>(input);
    (void)go_b.add_component<TrackBallCameraController>(input);

    const pbpt::math::vec3 a_before = go_a.node().world_position();
    const pbpt::math::vec3 b_before = go_b.node().world_position();

    input.update_mouse_button(system::input::MouseButton::LEFT, system::input::KeyAction::PRESS,
                              system::input::KeyMod::NONE);
    input.update_mouse_position(40.0, 10.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});

    const pbpt::math::vec3 a_after_first = go_a.node().world_position();
    const pbpt::math::vec3 b_after_first = go_b.node().world_position();
    EXPECT_GT(pbpt::math::length(a_after_first - a_before), 1e-4f);
    EXPECT_LE(pbpt::math::length(b_after_first - b_before), 1e-4f);

    camera_a.set_active(false);
    camera_b.set_active(true);
    input.reset_deltas();
    input.update_mouse_position(90.0, 20.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 1});

    const pbpt::math::vec3 a_after_second = go_a.node().world_position();
    const pbpt::math::vec3 b_after_second = go_b.node().world_position();
    EXPECT_LE(pbpt::math::length(a_after_second - a_after_first), 1e-4f);
    EXPECT_GT(pbpt::math::length(b_after_second - b_after_first), 1e-4f);
}

TEST(FrameworkTrackballCameraControllerTest, ThrowsWhenOwnerHasNoCameraComponent) {
    core::Scene               scene(1);
    auto&                     go = scene.create_game_object("go");
    system::input::InputState input{};

    EXPECT_THROW((void)go.add_component<TrackBallCameraController>(input), std::runtime_error);
}

TEST(FrameworkTrackballCameraControllerTest, PitchIsClamped) {
    core::Scene scene(1);
    auto&       go     = scene.create_game_object("camera_go");
    auto&       camera = go.add_component<PerspectiveCamera>();
    camera.set_active(true);
    go.node().set_world_position({0.0f, 0.0f, -10.0f});

    system::input::InputState input{};
    auto&                     controller = go.add_component<TrackBallCameraController>(input);

    input.update_mouse_button(system::input::MouseButton::LEFT, system::input::KeyAction::PRESS,
                              system::input::KeyMod::NONE);
    input.update_mouse_position(0.0, 8000.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});

    const pbpt::math::vec3 offset = go.node().world_position() - controller.target();
    const float            radius = pbpt::math::max(pbpt::math::length(offset), 1e-5f);
    const float pitch_deg = pbpt::math::degrees(std::asin(pbpt::math::clamp(offset.y() / radius, -1.0f, 1.0f)));
    EXPECT_LE(pitch_deg, 89.0f + 1e-3f);
    EXPECT_GE(pitch_deg, -89.0f - 1e-3f);
}

TEST(FrameworkTrackballCameraControllerTest, LeftHasPriorityOverMiddle) {
    core::Scene scene(1);
    auto&       go     = scene.create_game_object("camera_go");
    auto&       camera = go.add_component<PerspectiveCamera>();
    camera.set_active(true);
    go.node().set_world_position({0.0f, 0.0f, -10.0f});

    system::input::InputState input{};
    auto&                     controller = go.add_component<TrackBallCameraController>(input);
    const pbpt::math::vec3    before_pos = go.node().world_position();

    input.update_mouse_button(system::input::MouseButton::LEFT, system::input::KeyAction::PRESS,
                              system::input::KeyMod::NONE);
    input.update_mouse_button(system::input::MouseButton::MIDDLE, system::input::KeyAction::PRESS,
                              system::input::KeyMod::NONE);
    input.update_mouse_position(50.0, 25.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});

    EXPECT_GT(pbpt::math::length(go.node().world_position() - before_pos), 1e-4f);
    expect_vec3_near(controller.target(), pbpt::math::vec3(0.0f), 1e-5f);
}

TEST(FrameworkTrackballCameraControllerTest, CustomTargetOrbitWorks) {
    core::Scene scene(1);
    auto&       go     = scene.create_game_object("camera_go");
    auto&       camera = go.add_component<PerspectiveCamera>();
    camera.set_active(true);
    go.node().set_world_position({5.0f, 0.0f, -10.0f});

    system::input::InputState input{};
    auto&                     controller = go.add_component<TrackBallCameraController>(input);
    controller.set_target({5.0f, 0.0f, 0.0f});

    const pbpt::math::vec3 target        = controller.target();
    const pbpt::math::vec3 before        = go.node().world_position();
    const float            before_radius = pbpt::math::length(before - target);

    input.update_mouse_button(system::input::MouseButton::LEFT, system::input::KeyAction::PRESS,
                              system::input::KeyMod::NONE);
    input.update_mouse_position(80.0, -20.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});

    const pbpt::math::vec3 after        = go.node().world_position();
    const float            after_radius = pbpt::math::length(after - target);
    EXPECT_GT(pbpt::math::length(after - before), 1e-4f);
    EXPECT_NEAR(before_radius, after_radius, 1e-3f);
}

TEST(FrameworkTrackballCameraControllerTest, InitializesLookingAtTargetBeforeMouseInput) {
    core::Scene scene(1);
    auto&       go     = scene.create_game_object("camera_go");
    auto&       camera = go.add_component<PerspectiveCamera>();
    camera.set_active(true);
    go.node().set_world_position({3.0f, 2.0f, -6.0f});

    system::input::InputState input{};
    auto&                     controller = go.add_component<TrackBallCameraController>(input);
    controller.set_target({0.0f, 0.0f, 0.0f});

    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});

    const pbpt::math::vec3 pos          = go.node().world_position();
    const pbpt::math::vec3 expect_front = pbpt::math::normalize(controller.target() - pos);
    const pbpt::math::vec3 actual_front = pbpt::math::normalize(go.node().world_back());
    EXPECT_GT(pbpt::math::dot(expect_front, actual_front), 0.999f);
}

}  // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
