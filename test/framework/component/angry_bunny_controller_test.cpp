#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "../../../examples/games103_lab/lab1_angry_bunny/angry_bunny_controller.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/system/input/input_state.hpp"
#include "rtr/system/input/input_types.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_world.hpp"

namespace rtr::examples::games103_lab::lab1_angry_bunny::test {

TEST(AngryBunnyControllerTest, WaitingStateKeepsBunnySpinningWithoutGravity) {
    system::physics::RigidBodyWorld physics_world;
    framework::core::Scene        scene(1);
    system::input::InputState     input_state;

    auto& bunny = scene.create_game_object("bunny");
    bunny.node().set_local_position(pbpt::math::Vec3{0.0f, 0.6f, 0.0f});
    auto& rigid_body = bunny.add_component<framework::component::RigidBody>(physics_world);
    auto& controller = bunny.add_component<AngryBunnyController>(input_state);

    scene.fixed_tick(framework::core::FixedTickContext{.fixed_delta_seconds = 1.0 / 60.0, .fixed_tick_index = 0});

    EXPECT_FALSE(controller.launched());
    EXPECT_FALSE(rigid_body.use_gravity());
    EXPECT_NEAR(rigid_body.linear_velocity().length(), 0.0f, 1e-5f);
    EXPECT_NEAR(rigid_body.angular_velocity().y(), controller.initial_angular_velocity().y(), 1e-5f);
}

TEST(AngryBunnyControllerTest, LaunchAndResetEdgesUpdateStateOnce) {
    system::physics::RigidBodyWorld physics_world;
    framework::core::Scene        scene(1);
    system::input::InputState     input_state;

    auto& bunny = scene.create_game_object("bunny");
    bunny.node().set_local_position(pbpt::math::Vec3{0.0f, 0.6f, 0.0f});
    auto& rigid_body = bunny.add_component<framework::component::RigidBody>(physics_world);
    auto& controller = bunny.add_component<AngryBunnyController>(input_state);

    input_state.update_key(system::input::KeyCode::L, system::input::KeyAction::PRESS, system::input::KeyMod::NONE);
    scene.fixed_tick(framework::core::FixedTickContext{.fixed_delta_seconds = 1.0 / 60.0, .fixed_tick_index = 0});

    EXPECT_TRUE(controller.launched());
    EXPECT_TRUE(rigid_body.use_gravity());
    EXPECT_NEAR(rigid_body.linear_velocity().x(), controller.launch_linear_velocity().x(), 1e-5f);

    input_state.update_key(system::input::KeyCode::L, system::input::KeyAction::REPEAT, system::input::KeyMod::NONE);
    scene.fixed_tick(framework::core::FixedTickContext{.fixed_delta_seconds = 1.0 / 60.0, .fixed_tick_index = 1});
    EXPECT_NEAR(rigid_body.linear_velocity().x(), controller.launch_linear_velocity().x(), 1e-5f);

    input_state.update_key(system::input::KeyCode::L, system::input::KeyAction::RELEASE, system::input::KeyMod::NONE);
    input_state.update_key(system::input::KeyCode::R, system::input::KeyAction::PRESS, system::input::KeyMod::NONE);
    scene.fixed_tick(framework::core::FixedTickContext{.fixed_delta_seconds = 1.0 / 60.0, .fixed_tick_index = 2});

    EXPECT_FALSE(controller.launched());
    EXPECT_FALSE(rigid_body.use_gravity());
    EXPECT_NEAR(rigid_body.position().x(), controller.reset_position().x(), 1e-5f);
    EXPECT_NEAR(rigid_body.position().y(), controller.reset_position().y(), 1e-5f);
    EXPECT_NEAR(rigid_body.linear_velocity().length(), 0.0f, 1e-5f);
    EXPECT_NEAR(rigid_body.angular_velocity().y(), controller.initial_angular_velocity().y(), 1e-5f);
}

}  // namespace rtr::examples::games103_lab::lab1_angry_bunny::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
