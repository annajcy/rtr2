#include <limits>
#include <stdexcept>

#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_system.hpp"

namespace rtr::framework::component::test {

TEST(FrameworkRigidBodyComponentTest, ConstructorWritesParametersToSourceAndRuntimeBody) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                           scene(1);
    auto&                                 go = scene.create_game_object("body");
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[0][0] = 2.0f;
    inverse_inertia_tensor_ref[1][1] = 3.0f;
    inverse_inertia_tensor_ref[2][2] = 4.0f;

    auto& rigid_body = go.add_component<RigidBody>(physics_world,
                                                   2.5f,
                                                   system::physics::rb::RigidBodyType::Kinematic,
                                                   false,
                                                   inverse_inertia_tensor_ref,
                                                   0.25f,
                                                   0.36f,
                                                   0.99f,
                                                   0.98f);

    const auto& source_body  = rigid_body.source_body();
    const auto& physics_body = physics_world.get_rigid_body(rigid_body.rigid_body_id());
    EXPECT_EQ(source_body.type(), system::physics::rb::RigidBodyType::Kinematic);
    EXPECT_FALSE(source_body.use_gravity());
    EXPECT_FLOAT_EQ(source_body.state().mass, 2.5f);
    EXPECT_FLOAT_EQ(source_body.restitution(), 0.25f);
    EXPECT_FLOAT_EQ(source_body.friction(), 0.36f);
    EXPECT_FLOAT_EQ(source_body.linear_decay(), 0.99f);
    EXPECT_FLOAT_EQ(source_body.angular_decay(), 0.98f);
    EXPECT_EQ(source_body.inverse_inertia_tensor_ref(), inverse_inertia_tensor_ref);

    EXPECT_EQ(physics_body.type(), source_body.type());
    EXPECT_EQ(physics_body.use_gravity(), source_body.use_gravity());
    EXPECT_FLOAT_EQ(physics_body.state().mass, source_body.state().mass);
    EXPECT_FLOAT_EQ(physics_body.restitution(), 0.25f);
    EXPECT_FLOAT_EQ(physics_body.friction(), 0.36f);
    EXPECT_FLOAT_EQ(physics_body.linear_decay(), 0.99f);
    EXPECT_FLOAT_EQ(physics_body.angular_decay(), 0.98f);
}

TEST(FrameworkRigidBodyComponentTest, SetterSyncsAuthoringParametersToSourceAndRuntimeBody) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                           scene(1);
    auto&                                 go = scene.create_game_object("body");
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[0][0] = 1.0f;
    inverse_inertia_tensor_ref[1][1] = 2.0f;
    inverse_inertia_tensor_ref[2][2] = 3.0f;

    auto& rigid_body = go.add_component<RigidBody>(physics_world);
    rigid_body.set_type(system::physics::rb::RigidBodyType::Static);
    rigid_body.set_mass(2.5f);
    rigid_body.set_use_gravity(false);
    rigid_body.set_restitution(0.6f);
    rigid_body.set_friction(0.49f);
    rigid_body.set_linear_decay(0.97f);
    rigid_body.set_angular_decay(0.96f);
    rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

    const auto& source_body  = rigid_body.source_body();
    const auto& physics_body = physics_world.get_rigid_body(rigid_body.rigid_body_id());
    EXPECT_EQ(source_body.type(), system::physics::rb::RigidBodyType::Static);
    EXPECT_FALSE(source_body.use_gravity());
    EXPECT_FLOAT_EQ(source_body.state().mass, 2.5f);
    EXPECT_FLOAT_EQ(source_body.restitution(), 0.6f);
    EXPECT_FLOAT_EQ(source_body.friction(), 0.49f);
    EXPECT_FLOAT_EQ(source_body.linear_decay(), 0.97f);
    EXPECT_FLOAT_EQ(source_body.angular_decay(), 0.96f);
    EXPECT_EQ(source_body.inverse_inertia_tensor_ref(), inverse_inertia_tensor_ref);

    EXPECT_EQ(physics_body.type(), source_body.type());
    EXPECT_EQ(physics_body.use_gravity(), source_body.use_gravity());
    EXPECT_FLOAT_EQ(physics_body.state().mass, source_body.state().mass);
    EXPECT_FLOAT_EQ(physics_body.restitution(), source_body.restitution());
    EXPECT_FLOAT_EQ(physics_body.friction(), source_body.friction());
    EXPECT_FLOAT_EQ(physics_body.linear_decay(), 0.97f);
    EXPECT_FLOAT_EQ(physics_body.angular_decay(), 0.96f);
    EXPECT_EQ(physics_body.inverse_inertia_tensor_ref(), source_body.inverse_inertia_tensor_ref());
}

TEST(FrameworkRigidBodyComponentTest, InvalidMassThrows) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                   scene(1);
    auto&                         go = scene.create_game_object("body");

    EXPECT_THROW((void)go.add_component<RigidBody>(physics_world, 0.0f), std::invalid_argument);

    auto& valid_go = scene.create_game_object("valid_body");
    auto& rigid_body = valid_go.add_component<RigidBody>(physics_world);
    EXPECT_THROW(rigid_body.set_mass(0.0f), std::invalid_argument);
    EXPECT_THROW(rigid_body.set_mass(std::numeric_limits<float>::infinity()), std::invalid_argument);
}

TEST(FrameworkRigidBodyComponentTest, InvalidRestitutionThrows) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                   scene(1);
    auto&                         go = scene.create_game_object("body");

    EXPECT_THROW((void)go.add_component<RigidBody>(physics_world,
                                                   1.0f,
                                                   system::physics::rb::RigidBodyType::Dynamic,
                                                   true,
                                                   pbpt::math::Mat3::zeros(),
                                                   -0.1f,
                                                   0.0f),
                 std::invalid_argument);
    EXPECT_THROW((void)go.add_component<RigidBody>(physics_world,
                                                   1.0f,
                                                   system::physics::rb::RigidBodyType::Dynamic,
                                                   true,
                                                   pbpt::math::Mat3::zeros(),
                                                   1.1f,
                                                   0.0f),
                 std::invalid_argument);

    auto& valid_go = scene.create_game_object("valid_body");
    auto& rigid_body = valid_go.add_component<RigidBody>(physics_world);
    EXPECT_THROW(rigid_body.set_restitution(-0.1f), std::invalid_argument);
    EXPECT_THROW(rigid_body.set_restitution(std::numeric_limits<float>::infinity()), std::invalid_argument);
}

TEST(FrameworkRigidBodyComponentTest, InvalidFrictionThrows) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                   scene(1);
    auto&                         go = scene.create_game_object("body");

    EXPECT_THROW((void)go.add_component<RigidBody>(physics_world,
                                                   1.0f,
                                                   system::physics::rb::RigidBodyType::Dynamic,
                                                   true,
                                                   pbpt::math::Mat3::zeros(),
                                                   0.0f,
                                                   -0.1f),
                 std::invalid_argument);

    auto& valid_go = scene.create_game_object("valid_body");
    auto& rigid_body = valid_go.add_component<RigidBody>(physics_world);
    EXPECT_THROW(rigid_body.set_friction(-0.1f), std::invalid_argument);
    EXPECT_THROW(rigid_body.set_friction(std::numeric_limits<float>::infinity()), std::invalid_argument);
}

TEST(FrameworkRigidBodyComponentTest, InvalidDecayThrows) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                   scene(1);
    auto&                         go = scene.create_game_object("body");

    EXPECT_THROW((void)go.add_component<RigidBody>(physics_world,
                                                   1.0f,
                                                   system::physics::rb::RigidBodyType::Dynamic,
                                                   true,
                                                   pbpt::math::Mat3::zeros(),
                                                   0.0f,
                                                   0.0f,
                                                   1.1f,
                                                   1.0f),
                 std::invalid_argument);

    auto& valid_go = scene.create_game_object("valid_body");
    auto& rigid_body = valid_go.add_component<RigidBody>(physics_world);
    EXPECT_THROW(rigid_body.set_linear_decay(-0.1f), std::invalid_argument);
    EXPECT_THROW(rigid_body.set_angular_decay(std::numeric_limits<float>::infinity()), std::invalid_argument);
}

TEST(FrameworkRigidBodyComponentTest, VelocitySettersPersistInSourceAndPropagateOnEnable) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                         scene(1);
    auto&                               go = scene.create_game_object("body");

    auto& rigid_body = go.add_component<RigidBody>(physics_world);
    go.set_component_enabled<RigidBody>(false);

    const auto linear_velocity  = pbpt::math::Vec3{1.0f, 2.0f, 3.0f};
    const auto angular_velocity = pbpt::math::Vec3{4.0f, 5.0f, 6.0f};
    rigid_body.set_linear_velocity(linear_velocity);
    rigid_body.set_angular_velocity(angular_velocity);

    EXPECT_EQ(rigid_body.source_body().state().translation.linear_velocity, linear_velocity);
    EXPECT_EQ(rigid_body.source_body().state().rotation.angular_velocity, angular_velocity);
    EXPECT_EQ(rigid_body.runtime_body(), nullptr);

    go.set_component_enabled<RigidBody>(true);
    ASSERT_NE(rigid_body.runtime_body(), nullptr);
    EXPECT_EQ(rigid_body.runtime_body()->state().translation.linear_velocity, linear_velocity);
    EXPECT_EQ(rigid_body.runtime_body()->state().rotation.angular_velocity, angular_velocity);
}

TEST(FrameworkRigidBodyComponentTest, TransformSettersSyncSceneNodeAndRuntimeBody) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                         scene(1);
    auto&                               parent = scene.create_game_object("parent");
    auto&                               child  = scene.create_game_object("child");
    ASSERT_TRUE(scene.scene_graph().set_parent(child.id(), parent.id()));
    parent.node().set_local_position(pbpt::math::Vec3{10.0f, 0.0f, 0.0f});

    auto& rigid_body = child.add_component<RigidBody>(physics_world);

    const auto target_position = pbpt::math::Vec3{1.0f, 2.0f, 3.0f};
    const auto target_rotation = pbpt::math::normalize(pbpt::math::Quat{0.9f, 0.1f, 0.2f, 0.3f});
    rigid_body.set_position(target_position);
    rigid_body.set_orientation(target_rotation);

    EXPECT_EQ(child.node().world_position(), target_position);
    EXPECT_EQ(child.node().world_rotation(), target_rotation);
    ASSERT_NE(rigid_body.runtime_body(), nullptr);
    EXPECT_EQ(rigid_body.runtime_body()->state().translation.position, target_position);
    EXPECT_EQ(rigid_body.runtime_body()->state().rotation.orientation, target_rotation);
}

TEST(FrameworkRigidBodyComponentTest, DisableEnablePreservesSourceAndRebuildsRuntimeFromCurrentNodePose) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                         scene(1);
    auto&                               go = scene.create_game_object("body");

    auto& rigid_body = go.add_component<RigidBody>(physics_world);
    rigid_body.set_mass(2.5f);
    rigid_body.set_use_gravity(false);
    rigid_body.set_linear_velocity(pbpt::math::Vec3{1.0f, 2.0f, 3.0f});
    rigid_body.set_angular_velocity(pbpt::math::Vec3{0.5f, 1.0f, 1.5f});

    const auto first_body_id = rigid_body.rigid_body_id();
    ASSERT_NE(rigid_body.runtime_body(), nullptr);
    rigid_body.runtime_body()->state().translation.position = pbpt::math::Vec3{100.0f, 100.0f, 100.0f};
    rigid_body.runtime_body()->state().rotation.orientation =
        pbpt::math::normalize(pbpt::math::Quat{0.8f, 0.2f, 0.1f, 0.0f});

    go.set_component_enabled<RigidBody>(false);
    EXPECT_FALSE(rigid_body.has_rigid_body());
    EXPECT_FALSE(physics_world.has_rigid_body(first_body_id));

    const auto target_position = pbpt::math::Vec3{7.0f, 8.0f, 9.0f};
    const auto target_rotation = pbpt::math::normalize(pbpt::math::Quat{0.7f, 0.1f, 0.2f, 0.3f});
    go.node().set_world_position(target_position);
    go.node().set_world_rotation(target_rotation);

    go.set_component_enabled<RigidBody>(true);
    ASSERT_TRUE(rigid_body.has_rigid_body());
    ASSERT_NE(rigid_body.runtime_body(), nullptr);
    EXPECT_NE(rigid_body.rigid_body_id(), first_body_id);
    EXPECT_FLOAT_EQ(rigid_body.source_body().state().mass, 2.5f);
    EXPECT_FALSE(rigid_body.source_body().use_gravity());
    EXPECT_EQ(rigid_body.source_body().state().translation.linear_velocity, pbpt::math::Vec3(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(rigid_body.source_body().state().rotation.angular_velocity, pbpt::math::Vec3(0.5f, 1.0f, 1.5f));
    EXPECT_EQ(rigid_body.runtime_body()->state().translation.position, target_position);
    EXPECT_EQ(rigid_body.runtime_body()->state().rotation.orientation, target_rotation);
    EXPECT_EQ(rigid_body.runtime_body()->state().translation.linear_velocity, pbpt::math::Vec3(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(rigid_body.runtime_body()->state().rotation.angular_velocity, pbpt::math::Vec3(0.5f, 1.0f, 1.5f));
}

}  // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
