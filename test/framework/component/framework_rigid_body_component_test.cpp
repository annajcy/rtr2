#include <limits>
#include <stdexcept>

#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/framework/component/physics/rigid_body.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/system/physics/physics_system.hpp"

namespace rtr::framework::component::test {

TEST(FrameworkRigidBodyComponentTest, ConstructorWritesMaterialParametersToPhysicsBody) {
    system::physics::PhysicsSystem physics_system;
    core::Scene                    scene(1);
    auto&                          go = scene.create_game_object("body");

    auto& rigid_body = go.add_component<RigidBody>(physics_system.world(),
                                                   1.0f,
                                                   system::physics::RigidBodyType::Dynamic,
                                                   true,
                                                   pbpt::math::Mat3::zeros(),
                                                   0.25f,
                                                   0.36f);

    const auto& physics_body = physics_system.world().get_rigid_body(rigid_body.rigid_body_id());
    EXPECT_FLOAT_EQ(physics_body.restitution(), 0.25f);
    EXPECT_FLOAT_EQ(physics_body.friction(), 0.36f);
    EXPECT_FLOAT_EQ(rigid_body.restitution(), 0.25f);
    EXPECT_FLOAT_EQ(rigid_body.friction(), 0.36f);
}

TEST(FrameworkRigidBodyComponentTest, SetterSyncsMaterialParametersToPhysicsBody) {
    system::physics::PhysicsSystem physics_system;
    core::Scene                    scene(1);
    auto&                          go = scene.create_game_object("body");

    auto& rigid_body = go.add_component<RigidBody>(physics_system.world());
    rigid_body.set_restitution(0.6f);
    rigid_body.set_friction(0.49f);

    const auto& physics_body = physics_system.world().get_rigid_body(rigid_body.rigid_body_id());
    EXPECT_FLOAT_EQ(physics_body.restitution(), 0.6f);
    EXPECT_FLOAT_EQ(physics_body.friction(), 0.49f);
}

TEST(FrameworkRigidBodyComponentTest, InvalidRestitutionThrows) {
    system::physics::PhysicsSystem physics_system;
    core::Scene                    scene(1);
    auto&                          go = scene.create_game_object("body");

    EXPECT_THROW((void)go.add_component<RigidBody>(physics_system.world(),
                                                   1.0f,
                                                   system::physics::RigidBodyType::Dynamic,
                                                   true,
                                                   pbpt::math::Mat3::zeros(),
                                                   -0.1f,
                                                   0.0f),
                 std::invalid_argument);
    EXPECT_THROW((void)go.add_component<RigidBody>(physics_system.world(),
                                                   1.0f,
                                                   system::physics::RigidBodyType::Dynamic,
                                                   true,
                                                   pbpt::math::Mat3::zeros(),
                                                   1.1f,
                                                   0.0f),
                 std::invalid_argument);

    auto& valid_go = scene.create_game_object("valid_body");
    auto& rigid_body = valid_go.add_component<RigidBody>(physics_system.world());
    EXPECT_THROW(rigid_body.set_restitution(-0.1f), std::invalid_argument);
    EXPECT_THROW(rigid_body.set_restitution(std::numeric_limits<float>::infinity()), std::invalid_argument);
}

TEST(FrameworkRigidBodyComponentTest, InvalidFrictionThrows) {
    system::physics::PhysicsSystem physics_system;
    core::Scene                    scene(1);
    auto&                          go = scene.create_game_object("body");

    EXPECT_THROW((void)go.add_component<RigidBody>(physics_system.world(),
                                                   1.0f,
                                                   system::physics::RigidBodyType::Dynamic,
                                                   true,
                                                   pbpt::math::Mat3::zeros(),
                                                   0.0f,
                                                   -0.1f),
                 std::invalid_argument);

    auto& valid_go = scene.create_game_object("valid_body");
    auto& rigid_body = valid_go.add_component<RigidBody>(physics_system.world());
    EXPECT_THROW(rigid_body.set_friction(-0.1f), std::invalid_argument);
    EXPECT_THROW(rigid_body.set_friction(std::numeric_limits<float>::infinity()), std::invalid_argument);
}

}  // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
