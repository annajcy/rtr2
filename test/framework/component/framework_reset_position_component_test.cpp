#include <pbpt/math/math.h>

#include <stdexcept>

#include "gtest/gtest.h"

#include "rtr/framework/component/physics/rigid_body/reset_position.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_world.hpp"

namespace rtr::framework::component::test {

TEST(FrameworkResetPositionComponentTest, ThrowsWhenRigidBodyIsMissing) {
    core::Scene scene(1);
    auto&       go = scene.create_game_object("falling");
    EXPECT_THROW((void)go.add_component<ResetPosition>(), std::runtime_error);
}

TEST(FrameworkResetPositionComponentTest, ResetsPositionWhenThresholdIsReached) {
    system::physics::RigidBodyWorld physics_world;
    core::Scene                   scene(1);
    auto&                         go = scene.create_game_object("falling");
    go.node().set_local_position({0.0f, 2.0f, 0.0f});

    auto& rigid_body     = go.add_component<RigidBody>(physics_world);
    auto& reset_position = go.add_component<ResetPosition>();
    reset_position.set_threshold_y(-1.0f);
    reset_position.set_reset_position(pbpt::math::Vec3{0.5f, 3.0f, -0.5f});

    rigid_body.set_position(pbpt::math::Vec3{0.0f, -1.5f, 0.0f});
    scene.fixed_tick(core::FixedTickContext{.fixed_delta_seconds = 0.1, .fixed_tick_index = 0});

    const auto position = rigid_body.position();
    EXPECT_NEAR(position.x(), 0.5f, 1e-5f);
    EXPECT_NEAR(position.y(), 3.0f, 1e-5f);
    EXPECT_NEAR(position.z(), -0.5f, 1e-5f);
}

TEST(FrameworkResetPositionComponentTest, ResetClearsTranslationDynamicsButPreservesSpin) {
    system::physics::RigidBodyWorld physics_world;
    core::Scene                   scene(1);
    auto&                         go = scene.create_game_object("falling");
    go.node().set_local_position({0.0f, 2.0f, 0.0f});

    auto& rigid_body     = go.add_component<RigidBody>(physics_world);
    auto& reset_position = go.add_component<ResetPosition>();
    reset_position.set_threshold_y(-1.0f);
    reset_position.set_reset_position(pbpt::math::Vec3{0.0f, 2.0f, 0.0f});
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[1][1] = 1.0f;
    rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

    auto& physics_body = physics_world.get_rigid_body(rigid_body.rigid_body_id());
    physics_body.state().translation.linear_velocity = pbpt::math::Vec3{0.0f, -3.0f, 0.0f};
    physics_body.state().rotation.angular_velocity   = pbpt::math::Vec3{0.0f, 2.0f, 0.0f};
    physics_body.state().forces.accumulated_force    = pbpt::math::Vec3{1.0f, -4.0f, 0.0f};
    physics_body.state().forces.accumulated_torque   = pbpt::math::Vec3{0.0f, 3.0f, 0.0f};

    rigid_body.set_position(pbpt::math::Vec3{0.0f, -1.5f, 0.0f});
    scene.fixed_tick(core::FixedTickContext{.fixed_delta_seconds = 0.1, .fixed_tick_index = 0});

    const auto velocity = rigid_body.linear_velocity();
    EXPECT_NEAR(velocity.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(velocity.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(velocity.z(), 0.0f, 1e-5f);
    const auto angular_velocity = rigid_body.angular_velocity();
    EXPECT_NEAR(angular_velocity.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(angular_velocity.y(), 2.0f, 1e-5f);
    EXPECT_NEAR(angular_velocity.z(), 0.0f, 1e-5f);
    EXPECT_NEAR(physics_body.state().forces.accumulated_force.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(physics_body.state().forces.accumulated_force.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(physics_body.state().forces.accumulated_force.z(), 0.0f, 1e-5f);
    EXPECT_NEAR(physics_body.state().forces.accumulated_torque.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(physics_body.state().forces.accumulated_torque.y(), 3.0f, 1e-5f);
    EXPECT_NEAR(physics_body.state().forces.accumulated_torque.z(), 0.0f, 1e-5f);
}

TEST(FrameworkResetPositionComponentTest, UpdatedParametersAffectNextFixedTick) {
    system::physics::RigidBodyWorld physics_world;
    core::Scene                   scene(1);
    auto&                         go = scene.create_game_object("falling");
    go.node().set_local_position({0.0f, 2.0f, 0.0f});

    auto& rigid_body     = go.add_component<RigidBody>(physics_world);
    auto& reset_position = go.add_component<ResetPosition>();
    reset_position.set_threshold_y(-0.2f);
    reset_position.set_reset_position(pbpt::math::Vec3{1.0f, 4.0f, 0.0f});

    rigid_body.set_position(pbpt::math::Vec3{0.0f, -0.2f, 0.0f});
    scene.fixed_tick(core::FixedTickContext{.fixed_delta_seconds = 0.1, .fixed_tick_index = 0});

    const auto position = rigid_body.position();
    EXPECT_NEAR(position.x(), 1.0f, 1e-5f);
    EXPECT_NEAR(position.y(), 4.0f, 1e-5f);
    EXPECT_NEAR(position.z(), 0.0f, 1e-5f);
}

}  // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
