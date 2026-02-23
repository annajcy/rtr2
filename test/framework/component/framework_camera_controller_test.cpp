#include <stdexcept>

#include "gtest/gtest.h"

#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/framework/component/camera_control/camera_controller.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/system/input/input_state.hpp"

namespace rtr::framework::component::test {

class ProbeCameraController final : public CameraController {
public:
    int update_calls{0};

    ProbeCameraController(core::GameObject& owner, const system::input::InputState& input_state)
        : CameraController(owner, input_state) {}

protected:
    void on_update_active_camera(const core::FrameTickContext&, Camera&) override { ++update_calls; }
};

TEST(FrameworkCameraControllerTest, ThrowsWhenOwnerHasNoCameraComponent) {
    core::Scene               scene(1, "scene");
    auto&                     go = scene.create_game_object("go");
    system::input::InputState input{};
    EXPECT_THROW((void)go.add_component<ProbeCameraController>(input), std::runtime_error);
}

TEST(FrameworkCameraControllerTest, InactiveCameraDoesNotRunDerivedUpdate) {
    core::Scene scene(1, "scene");
    auto&       go = scene.create_game_object("camera_go");
    (void)go.add_component<PerspectiveCamera>();

    system::input::InputState input{};
    auto&                     controller = go.add_component<ProbeCameraController>(input);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});
    EXPECT_EQ(controller.update_calls, 0);
}

TEST(FrameworkCameraControllerTest, ActiveCameraRunsDerivedUpdate) {
    core::Scene scene(1, "scene");
    auto&       go     = scene.create_game_object("camera_go");
    auto&       camera = go.add_component<PerspectiveCamera>();
    camera.set_active(true);

    system::input::InputState input{};
    auto&                     controller = go.add_component<ProbeCameraController>(input);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});
    EXPECT_EQ(controller.update_calls, 1);
}

TEST(FrameworkCameraControllerTest, ConstructorInjectedInputKeepsControllerWorking) {
    core::Scene scene(1, "scene");
    auto&       go     = scene.create_game_object("camera_go");
    auto&       camera = go.add_component<PerspectiveCamera>();
    camera.set_active(true);

    system::input::InputState input{};
    auto&                     controller = go.add_component<ProbeCameraController>(input);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 0});
    EXPECT_EQ(controller.update_calls, 1);
}

}  // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
