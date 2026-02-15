#include <pbpt/math/math.h>
#include <stdexcept>

#include "gtest/gtest.h"


#include "rtr/framework/component/component.hpp"
#include "rtr/framework/component/camera_control/free_look_camera_controller.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/scene_graph.hpp"
#include "rtr/system/input/input_state.hpp"
#include "rtr/system/input/input_types.hpp"

namespace rtr::framework::component::test {

static void expect_vec3_near(const pbpt::math::vec3& lhs, const pbpt::math::vec3& rhs, float eps = 1e-4f) {
    EXPECT_NEAR(lhs.x(), rhs.x(), eps);
    EXPECT_NEAR(lhs.y(), rhs.y(), eps);
    EXPECT_NEAR(lhs.z(), rhs.z(), eps);
}

TEST(FrameworkFreeLookCameraControllerTest, MovesWithWASDQE) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("camera_go");
    (void)scene.camera_manager().create_perspective_camera(go.id());

    system::input::InputState input{};
    (void)go.add_component<FreeLookCameraController>(&input, &scene.camera_manager());

    core::FrameTickContext ctx{.delta_seconds = 1.0, .unscaled_delta_seconds = 1.0, .frame_index = 0};

    input.update_key(system::input::KeyCode::W, system::input::KeyAction::PRESS, system::input::KeyMod::NONE);
    scene.tick(ctx);
    input.update_key(system::input::KeyCode::W, system::input::KeyAction::RELEASE, system::input::KeyMod::NONE);

    input.update_key(system::input::KeyCode::D, system::input::KeyAction::PRESS, system::input::KeyMod::NONE);
    scene.tick(ctx);
    input.update_key(system::input::KeyCode::D, system::input::KeyAction::RELEASE, system::input::KeyMod::NONE);

    input.update_key(system::input::KeyCode::E, system::input::KeyAction::PRESS, system::input::KeyMod::NONE);
    scene.tick(ctx);
    input.update_key(system::input::KeyCode::E, system::input::KeyAction::RELEASE, system::input::KeyMod::NONE);

    input.update_key(system::input::KeyCode::S, system::input::KeyAction::PRESS, system::input::KeyMod::NONE);
    scene.tick(ctx);
    input.update_key(system::input::KeyCode::S, system::input::KeyAction::RELEASE, system::input::KeyMod::NONE);

    input.update_key(system::input::KeyCode::A, system::input::KeyAction::PRESS, system::input::KeyMod::NONE);
    scene.tick(ctx);
    input.update_key(system::input::KeyCode::A, system::input::KeyAction::RELEASE, system::input::KeyMod::NONE);

    input.update_key(system::input::KeyCode::Q, system::input::KeyAction::PRESS, system::input::KeyMod::NONE);
    scene.tick(ctx);
    input.update_key(system::input::KeyCode::Q, system::input::KeyAction::RELEASE, system::input::KeyMod::NONE);

    expect_vec3_near(scene.scene_graph().node(go.id()).world_position(), pbpt::math::vec3(0.0f));
}

TEST(FrameworkFreeLookCameraControllerTest, ShiftAppliesSprintMultiplier) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("camera_go");
    (void)scene.camera_manager().create_perspective_camera(go.id());

    system::input::InputState input{};
    auto& controller = go.add_component<FreeLookCameraController>(&input, &scene.camera_manager());
    (void)controller;

    core::FrameTickContext ctx{.delta_seconds = 1.0, .unscaled_delta_seconds = 1.0, .frame_index = 0};

    input.update_key(system::input::KeyCode::W, system::input::KeyAction::PRESS, system::input::KeyMod::NONE);
    scene.tick(ctx);
    const float normal_distance = scene.scene_graph().node(go.id()).world_position().z();

    auto node = scene.scene_graph().node(go.id());
    node.set_world_position(pbpt::math::vec3(0.0f));
    scene.scene_graph().update_world_transforms();

    input.update_key(system::input::KeyCode::LEFT_SHIFT, system::input::KeyAction::PRESS, system::input::KeyMod::SHIFT);
    scene.tick(ctx);
    const float sprint_distance = scene.scene_graph().node(go.id()).world_position().z();

    EXPECT_NEAR(normal_distance, -5.0f, 1e-4f);
    EXPECT_NEAR(sprint_distance, -15.0f, 1e-4f);
    EXPECT_NEAR(sprint_distance / normal_distance, 3.0f, 1e-4f);
}

TEST(FrameworkFreeLookCameraControllerTest, RightMouseRequiredForLook) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("camera_go");
    (void)scene.camera_manager().create_perspective_camera(go.id());

    system::input::InputState input{};
    (void)go.add_component<FreeLookCameraController>(&input, &scene.camera_manager());

    core::FrameTickContext ctx{.delta_seconds = 1.0, .unscaled_delta_seconds = 1.0, .frame_index = 0};
    const pbpt::math::vec3 before_front = scene.scene_graph().node(go.id()).world_front();

    input.update_mouse_position(30.0, 0.0);
    scene.tick(ctx);
    const pbpt::math::vec3 without_right_front = scene.scene_graph().node(go.id()).world_front();
    expect_vec3_near(without_right_front, before_front);

    input.reset_deltas();
    input.update_mouse_button(
        system::input::MouseButton::RIGHT,
        system::input::KeyAction::PRESS,
        system::input::KeyMod::NONE
    );
    input.update_mouse_position(60.0, 0.0);
    scene.tick(ctx);
    const pbpt::math::vec3 with_right_front = scene.scene_graph().node(go.id()).world_front();

    EXPECT_GT(pbpt::math::length(with_right_front - before_front), 1e-4f);
}

TEST(FrameworkFreeLookCameraControllerTest, PitchIsClamped) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("camera_go");
    (void)scene.camera_manager().create_perspective_camera(go.id());

    system::input::InputState input{};
    (void)go.add_component<FreeLookCameraController>(&input, &scene.camera_manager());

    core::FrameTickContext ctx{.delta_seconds = 1.0, .unscaled_delta_seconds = 1.0, .frame_index = 0};

    input.update_mouse_button(
        system::input::MouseButton::RIGHT,
        system::input::KeyAction::PRESS,
        system::input::KeyMod::NONE
    );
    input.update_mouse_position(0.0, -2000.0);
    scene.tick(ctx);

    const auto* camera = dynamic_cast<const core::PerspectiveCamera*>(scene.active_camera());
    ASSERT_NE(camera, nullptr);
    const pbpt::math::vec3 front = camera->front();
    const float pitch_deg = pbpt::math::degrees(std::asin(pbpt::math::clamp(front.y(), -1.0f, 1.0f)));
    EXPECT_LE(pitch_deg, 89.0f + 1e-3f);
    EXPECT_GE(pitch_deg, -89.0f - 1e-3f);
}

TEST(FrameworkFreeLookCameraControllerTest, ScrollCallsAdjustZoom) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("camera_go");
    (void)scene.camera_manager().create_perspective_camera(go.id());

    system::input::InputState input{};
    (void)go.add_component<FreeLookCameraController>(&input, &scene.camera_manager());

    core::FrameTickContext ctx{.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0};
    input.update_mouse_scroll(0.0, 1.0);
    scene.tick(ctx);

    const pbpt::math::vec3 pos = scene.scene_graph().node(go.id()).world_position();
    EXPECT_NEAR(pos.z(), -0.8f, 1e-4f);
}

TEST(FrameworkFreeLookCameraControllerTest, OnlyActiveCameraResponds) {
    core::Scene scene(1, "scene");
    auto& go_a = scene.create_game_object("camera_a");
    auto& go_b = scene.create_game_object("camera_b");
    (void)scene.camera_manager().create_perspective_camera(go_a.id());
    (void)scene.camera_manager().create_perspective_camera(go_b.id());

    system::input::InputState input{};
    (void)go_a.add_component<FreeLookCameraController>(&input, &scene.camera_manager());
    (void)go_b.add_component<FreeLookCameraController>(&input, &scene.camera_manager());

    core::FrameTickContext ctx{.delta_seconds = 1.0, .unscaled_delta_seconds = 1.0, .frame_index = 0};
    input.update_key(system::input::KeyCode::W, system::input::KeyAction::PRESS, system::input::KeyMod::NONE);

    scene.tick(ctx);
    EXPECT_NEAR(scene.scene_graph().node(go_a.id()).world_position().z(), -5.0f, 1e-4f);
    EXPECT_NEAR(scene.scene_graph().node(go_b.id()).world_position().z(), 0.0f, 1e-4f);

    ASSERT_TRUE(scene.set_active_camera(go_b.id()));
    scene.tick(ctx);
    EXPECT_NEAR(scene.scene_graph().node(go_a.id()).world_position().z(), -5.0f, 1e-4f);
    EXPECT_NEAR(scene.scene_graph().node(go_b.id()).world_position().z(), -5.0f, 1e-4f);
}

TEST(FrameworkFreeLookCameraControllerTest, ThrowsWhenOwnerHasNoCamera) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("go");

    system::input::InputState input{};
    EXPECT_THROW(
        (void)go.add_component<FreeLookCameraController>(&input, &scene.camera_manager()),
        std::runtime_error
    );
}

class MoveOnUpdateComponent final : public Component {
public:
    void on_update(const core::FrameTickContext& /*ctx*/) override {
        owner()->node().set_local_position({1.0f, 2.0f, 3.0f});
    }
};

TEST(FrameworkFreeLookCameraControllerTest, SceneTickRefreshesWorldTransformAfterComponentUpdate) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("go");
    (void)go.add_component<MoveOnUpdateComponent>();

    core::FrameTickContext ctx{.delta_seconds = 1.0, .unscaled_delta_seconds = 1.0, .frame_index = 0};
    scene.tick(ctx);

    const pbpt::math::vec3 world_pos = scene.scene_graph().node(go.id()).world_position();
    expect_vec3_near(world_pos, {1.0f, 2.0f, 3.0f});
}

} // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
