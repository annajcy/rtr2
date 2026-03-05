#include <pbpt/math/math.h>

#include <stdexcept>

#include "gtest/gtest.h"

#include "rtr/framework/component/physics/ping_pong_component.hpp"
#include "rtr/framework/component/physics/rigid_body_component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/system/physics/physics_system.hpp"

namespace rtr::framework::component::test {

TEST(FrameworkPingPongComponentTest, FlipsVelocityAtBothBounds) {
    system::physics::PhysicsSystem physics_system;
    core::Scene scene(1);
    auto& go = scene.create_game_object("moving");
    go.node().set_local_position({0.0f, 0.0f, 0.0f});

    auto& rigid_body = go.add_component<RigidBodyComponent>(physics_system.world(), pbpt::math::Vec3{1.0f, 0.0f, 0.0f});
    auto& ping_pong = go.add_component<PingPongComponent>();
    ping_pong.set_bounds(-1.0f, 1.0f);
    ping_pong.set_speed(1.0f);
    ping_pong.set_start_positive(true);

    rigid_body.set_position(pbpt::math::Vec3{1.0f, 0.0f, 0.0f});
    rigid_body.set_linear_velocity(pbpt::math::Vec3{1.0f, 0.0f, 0.0f});
    scene.fixed_tick(core::FixedTickContext{.fixed_delta_seconds = 0.1, .fixed_tick_index = 0});
    EXPECT_LT(rigid_body.linear_velocity().x(), 0.0f);

    rigid_body.set_position(pbpt::math::Vec3{-1.0f, 0.0f, 0.0f});
    rigid_body.set_linear_velocity(pbpt::math::Vec3{-1.0f, 0.0f, 0.0f});
    scene.fixed_tick(core::FixedTickContext{.fixed_delta_seconds = 0.1, .fixed_tick_index = 1});
    EXPECT_GT(rigid_body.linear_velocity().x(), 0.0f);
}

TEST(FrameworkPingPongComponentTest, ThrowsWhenRigidBodyIsMissing) {
    core::Scene scene(1);
    auto& go = scene.create_game_object("moving");
    EXPECT_THROW((void)go.add_component<PingPongComponent>(), std::runtime_error);
}

TEST(FrameworkPingPongComponentTest, UpdatedParametersAffectNextFixedTick) {
    system::physics::PhysicsSystem physics_system;
    core::Scene scene(1);
    auto& go = scene.create_game_object("moving");
    go.node().set_local_position({0.0f, 0.0f, 0.0f});

    auto& rigid_body = go.add_component<RigidBodyComponent>(physics_system.world(), pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    auto& ping_pong = go.add_component<PingPongComponent>();

    ping_pong.set_bounds(-0.2f, 0.2f);
    ping_pong.set_speed(3.0f);
    ping_pong.set_start_positive(true);

    rigid_body.set_position(pbpt::math::Vec3{0.2f, 0.0f, 0.0f});
    rigid_body.set_linear_velocity(pbpt::math::Vec3{0.5f, 0.0f, 0.0f});
    scene.fixed_tick(core::FixedTickContext{.fixed_delta_seconds = 0.1, .fixed_tick_index = 0});

    EXPECT_NEAR(rigid_body.position().x(), 0.2f, 1e-5f);
    EXPECT_NEAR(rigid_body.linear_velocity().x(), -3.0f, 1e-5f);
}

}  // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
