#include <limits>
#include <stdexcept>

#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/physics/rigid_body_scene_sync.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_system.hpp"

namespace rtr::framework::component::test {

namespace {

system::physics::rb::RigidBody make_source_body(
    pbpt::math::Float mass = 1.0f,
    system::physics::rb::RigidBodyType type = system::physics::rb::RigidBodyType::Dynamic,
    bool use_gravity = true,
    const pbpt::math::Mat3& inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros(),
    pbpt::math::Float restitution = 0.0f,
    pbpt::math::Float friction = 0.0f,
    pbpt::math::Float linear_decay = 1.0f,
    pbpt::math::Float angular_decay = 1.0f) {
    system::physics::rb::RigidBody source_body{};
    source_body.state().mass = mass;
    source_body.set_type(type);
    source_body.set_use_gravity(use_gravity);
    source_body.set_restitution(restitution);
    source_body.set_friction(friction);
    source_body.set_linear_decay(linear_decay);
    source_body.set_angular_decay(angular_decay);
    source_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);
    return source_body;
}

const system::physics::rb::RigidBody* runtime_body_for(const system::physics::rb::RigidBodySystem& physics_world,
                                                       const core::GameObject& go) {
    return physics_world.try_get_rigid_body_for_owner(go.id());
}

system::physics::rb::RigidBody* runtime_body_for(system::physics::rb::RigidBodySystem& physics_world,
                                                 const core::GameObject& go) {
    return physics_world.try_get_rigid_body_for_owner(go.id());
}

system::physics::rb::RigidBodyID runtime_body_id_for(const system::physics::rb::RigidBodySystem& physics_world,
                                                     const core::GameObject& go) {
    const auto body_id = physics_world.try_get_rigid_body_id_for_owner(go.id());
    EXPECT_TRUE(body_id.has_value());
    return *body_id;
}

}  // namespace

TEST(FrameworkRigidBodyComponentTest, ConstructorWritesParametersToSourceAndRuntimeBody) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                           scene(1);
    auto&                                 go = scene.create_game_object("body");
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[0][0] = 2.0f;
    inverse_inertia_tensor_ref[1][1] = 3.0f;
    inverse_inertia_tensor_ref[2][2] = 4.0f;

    auto& rigid_body = go.add_component<RigidBody>(make_source_body(
        2.5f,
        system::physics::rb::RigidBodyType::Kinematic,
        false,
        inverse_inertia_tensor_ref,
        0.25f,
        0.36f,
        0.99f,
        0.98f));
    integration::physics::sync_scene_to_rigid_body(scene, physics_world);

    const auto& source_body  = rigid_body.source_body();
    const auto* physics_body = runtime_body_for(physics_world, go);
    ASSERT_NE(physics_body, nullptr);
    EXPECT_EQ(source_body.type(), system::physics::rb::RigidBodyType::Kinematic);
    EXPECT_FALSE(source_body.use_gravity());
    EXPECT_FLOAT_EQ(source_body.state().mass, 2.5f);
    EXPECT_FLOAT_EQ(source_body.restitution(), 0.25f);
    EXPECT_FLOAT_EQ(source_body.friction(), 0.36f);
    EXPECT_FLOAT_EQ(source_body.linear_decay(), 0.99f);
    EXPECT_FLOAT_EQ(source_body.angular_decay(), 0.98f);
    EXPECT_EQ(source_body.inverse_inertia_tensor_ref(), inverse_inertia_tensor_ref);

    EXPECT_EQ(physics_body->type(), source_body.type());
    EXPECT_EQ(physics_body->use_gravity(), source_body.use_gravity());
    EXPECT_FLOAT_EQ(physics_body->state().mass, source_body.state().mass);
    EXPECT_FLOAT_EQ(physics_body->restitution(), 0.25f);
    EXPECT_FLOAT_EQ(physics_body->friction(), 0.36f);
    EXPECT_FLOAT_EQ(physics_body->linear_decay(), 0.99f);
    EXPECT_FLOAT_EQ(physics_body->angular_decay(), 0.98f);
}

TEST(FrameworkRigidBodyComponentTest, SetterSyncsAuthoringParametersToSourceAndRuntimeBody) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                           scene(1);
    auto&                                 go = scene.create_game_object("body");
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[0][0] = 1.0f;
    inverse_inertia_tensor_ref[1][1] = 2.0f;
    inverse_inertia_tensor_ref[2][2] = 3.0f;

    auto& rigid_body = go.add_component<RigidBody>();
    rigid_body.set_type(system::physics::rb::RigidBodyType::Static);
    rigid_body.set_mass(2.5f);
    rigid_body.set_use_gravity(false);
    rigid_body.set_restitution(0.6f);
    rigid_body.set_friction(0.49f);
    rigid_body.set_linear_decay(0.97f);
    rigid_body.set_angular_decay(0.96f);
    rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

    const auto& source_body  = rigid_body.source_body();
    EXPECT_EQ(source_body.type(), system::physics::rb::RigidBodyType::Static);
    EXPECT_FALSE(source_body.use_gravity());
    EXPECT_FLOAT_EQ(source_body.state().mass, 2.5f);
    EXPECT_FLOAT_EQ(source_body.restitution(), 0.6f);
    EXPECT_FLOAT_EQ(source_body.friction(), 0.49f);
    EXPECT_FLOAT_EQ(source_body.linear_decay(), 0.97f);
    EXPECT_FLOAT_EQ(source_body.angular_decay(), 0.96f);
    EXPECT_EQ(source_body.inverse_inertia_tensor_ref(), inverse_inertia_tensor_ref);

    EXPECT_TRUE(rigid_body.source_dirty());

    integration::physics::sync_scene_to_rigid_body(scene, physics_world);
    const auto* physics_body = runtime_body_for(physics_world, go);
    ASSERT_NE(physics_body, nullptr);
    EXPECT_EQ(physics_body->type(), source_body.type());
    EXPECT_FLOAT_EQ(physics_body->state().mass, source_body.state().mass);

    const auto* synced_body = runtime_body_for(physics_world, go);
    ASSERT_NE(synced_body, nullptr);
    EXPECT_FALSE(rigid_body.source_dirty());
    EXPECT_EQ(synced_body->type(), source_body.type());
    EXPECT_EQ(synced_body->use_gravity(), source_body.use_gravity());
    EXPECT_FLOAT_EQ(synced_body->state().mass, source_body.state().mass);
    EXPECT_FLOAT_EQ(synced_body->restitution(), source_body.restitution());
    EXPECT_FLOAT_EQ(synced_body->friction(), source_body.friction());
    EXPECT_FLOAT_EQ(synced_body->linear_decay(), 0.97f);
    EXPECT_FLOAT_EQ(synced_body->angular_decay(), 0.96f);
    EXPECT_EQ(synced_body->inverse_inertia_tensor_ref(), source_body.inverse_inertia_tensor_ref());
}

TEST(FrameworkRigidBodyComponentTest, InvalidMassThrows) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                   scene(1);
    auto&                         go = scene.create_game_object("body");

    EXPECT_THROW((void)go.add_component<RigidBody>(make_source_body(0.0f)), std::invalid_argument);

    auto& valid_go = scene.create_game_object("valid_body");
    auto& rigid_body = valid_go.add_component<RigidBody>();
    EXPECT_THROW(rigid_body.set_mass(0.0f), std::invalid_argument);
    EXPECT_THROW(rigid_body.set_mass(std::numeric_limits<float>::infinity()), std::invalid_argument);
}

TEST(FrameworkRigidBodyComponentTest, InvalidRestitutionThrows) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                   scene(1);
    auto&                         go = scene.create_game_object("body");

    EXPECT_THROW((void)go.add_component<RigidBody>(make_source_body(
                                                   1.0f,
                                                   system::physics::rb::RigidBodyType::Dynamic,
                                                   true,
                                                   pbpt::math::Mat3::zeros(),
                                                   -0.1f,
                                                   0.0f)),
                 std::invalid_argument);
    EXPECT_THROW((void)go.add_component<RigidBody>(make_source_body(
                                                   1.0f,
                                                   system::physics::rb::RigidBodyType::Dynamic,
                                                   true,
                                                   pbpt::math::Mat3::zeros(),
                                                   1.1f,
                                                   0.0f)),
                 std::invalid_argument);

    auto& valid_go = scene.create_game_object("valid_body");
    auto& rigid_body = valid_go.add_component<RigidBody>();
    EXPECT_THROW(rigid_body.set_restitution(-0.1f), std::invalid_argument);
    EXPECT_THROW(rigid_body.set_restitution(std::numeric_limits<float>::infinity()), std::invalid_argument);
}

TEST(FrameworkRigidBodyComponentTest, InvalidFrictionThrows) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                   scene(1);
    auto&                         go = scene.create_game_object("body");

    EXPECT_THROW((void)go.add_component<RigidBody>(make_source_body(
                                                   1.0f,
                                                   system::physics::rb::RigidBodyType::Dynamic,
                                                   true,
                                                   pbpt::math::Mat3::zeros(),
                                                   0.0f,
                                                   -0.1f)),
                 std::invalid_argument);

    auto& valid_go = scene.create_game_object("valid_body");
    auto& rigid_body = valid_go.add_component<RigidBody>();
    EXPECT_THROW(rigid_body.set_friction(-0.1f), std::invalid_argument);
    EXPECT_THROW(rigid_body.set_friction(std::numeric_limits<float>::infinity()), std::invalid_argument);
}

TEST(FrameworkRigidBodyComponentTest, InvalidDecayThrows) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                   scene(1);
    auto&                         go = scene.create_game_object("body");

    EXPECT_THROW((void)go.add_component<RigidBody>(make_source_body(
                                                   1.0f,
                                                   system::physics::rb::RigidBodyType::Dynamic,
                                                   true,
                                                   pbpt::math::Mat3::zeros(),
                                                   0.0f,
                                                   0.0f,
                                                   1.1f,
                                                   1.0f)),
                 std::invalid_argument);

    auto& valid_go = scene.create_game_object("valid_body");
    auto& rigid_body = valid_go.add_component<RigidBody>();
    EXPECT_THROW(rigid_body.set_linear_decay(-0.1f), std::invalid_argument);
    EXPECT_THROW(rigid_body.set_angular_decay(std::numeric_limits<float>::infinity()), std::invalid_argument);
}

TEST(FrameworkRigidBodyComponentTest, VelocitySettersPersistInSourceAndPropagateOnEnable) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                         scene(1);
    auto&                               go = scene.create_game_object("body");

    auto& rigid_body = go.add_component<RigidBody>();
    go.set_component_enabled<RigidBody>(false);

    const auto linear_velocity  = pbpt::math::Vec3{1.0f, 2.0f, 3.0f};
    const auto angular_velocity = pbpt::math::Vec3{4.0f, 5.0f, 6.0f};
    rigid_body.set_linear_velocity(linear_velocity);
    rigid_body.set_angular_velocity(angular_velocity);

    EXPECT_EQ(rigid_body.source_body().state().translation.linear_velocity, linear_velocity);
    EXPECT_EQ(rigid_body.source_body().state().rotation.angular_velocity, angular_velocity);
    EXPECT_EQ(runtime_body_for(physics_world, go), nullptr);

    go.set_component_enabled<RigidBody>(true);
    integration::physics::sync_scene_to_rigid_body(scene, physics_world);
    const auto* runtime_body = runtime_body_for(physics_world, go);
    ASSERT_NE(runtime_body, nullptr);
    EXPECT_EQ(runtime_body->state().translation.linear_velocity, linear_velocity);
    EXPECT_EQ(runtime_body->state().rotation.angular_velocity, angular_velocity);
}

TEST(FrameworkRigidBodyComponentTest, TransformSettersSyncSceneNodeAndRuntimeBody) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                         scene(1);
    auto&                               parent = scene.create_game_object("parent");
    auto&                               child  = scene.create_game_object("child");
    ASSERT_TRUE(scene.scene_graph().set_parent(child.id(), parent.id()));
    parent.node().set_local_position(pbpt::math::Vec3{10.0f, 0.0f, 0.0f});

    auto& rigid_body = child.add_component<RigidBody>();

    const auto target_position = pbpt::math::Vec3{1.0f, 2.0f, 3.0f};
    const auto target_rotation = pbpt::math::normalize(pbpt::math::Quat{0.9f, 0.1f, 0.2f, 0.3f});
    rigid_body.set_position(target_position);
    rigid_body.set_orientation(target_rotation);

    EXPECT_EQ(child.node().world_position(), target_position);
    EXPECT_EQ(child.node().world_rotation(), target_rotation);
    EXPECT_TRUE(rigid_body.transform_dirty());
    integration::physics::sync_scene_to_rigid_body(scene, physics_world);
    const auto* runtime_body = runtime_body_for(physics_world, child);
    ASSERT_NE(runtime_body, nullptr);
    EXPECT_EQ(runtime_body->state().translation.position, target_position);

    integration::physics::sync_scene_to_rigid_body(scene, physics_world);

    runtime_body = runtime_body_for(physics_world, child);
    ASSERT_NE(runtime_body, nullptr);
    EXPECT_FALSE(rigid_body.transform_dirty());
    EXPECT_EQ(runtime_body->state().translation.position, target_position);
    EXPECT_EQ(runtime_body->state().rotation.orientation, target_rotation);
}

TEST(FrameworkRigidBodyComponentTest, DisableEnablePreservesSourceAndRebuildsRuntimeFromCurrentNodePose) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                         scene(1);
    auto&                               go = scene.create_game_object("body");

    auto& rigid_body = go.add_component<RigidBody>();
    rigid_body.set_mass(2.5f);
    rigid_body.set_use_gravity(false);
    rigid_body.set_linear_velocity(pbpt::math::Vec3{1.0f, 2.0f, 3.0f});
    rigid_body.set_angular_velocity(pbpt::math::Vec3{0.5f, 1.0f, 1.5f});
    integration::physics::sync_scene_to_rigid_body(scene, physics_world);

    const auto first_body_id = runtime_body_id_for(physics_world, go);
    auto* first_runtime_body = runtime_body_for(physics_world, go);
    ASSERT_NE(first_runtime_body, nullptr);
    first_runtime_body->state().translation.position = pbpt::math::Vec3{100.0f, 100.0f, 100.0f};
    first_runtime_body->state().rotation.orientation =
        pbpt::math::normalize(pbpt::math::Quat{0.8f, 0.2f, 0.1f, 0.0f});

    go.set_component_enabled<RigidBody>(false);
    integration::physics::sync_scene_to_rigid_body(scene, physics_world);
    EXPECT_EQ(runtime_body_for(physics_world, go), nullptr);
    EXPECT_FALSE(physics_world.has_rigid_body(first_body_id));

    const auto target_position = pbpt::math::Vec3{7.0f, 8.0f, 9.0f};
    const auto target_rotation = pbpt::math::normalize(pbpt::math::Quat{0.7f, 0.1f, 0.2f, 0.3f});
    go.node().set_world_position(target_position);
    go.node().set_world_rotation(target_rotation);

    go.set_component_enabled<RigidBody>(true);
    integration::physics::sync_scene_to_rigid_body(scene, physics_world);
    auto* runtime_body = runtime_body_for(physics_world, go);
    ASSERT_NE(runtime_body, nullptr);
    EXPECT_NE(runtime_body_id_for(physics_world, go), first_body_id);
    EXPECT_FLOAT_EQ(rigid_body.source_body().state().mass, 2.5f);
    EXPECT_FALSE(rigid_body.source_body().use_gravity());
    EXPECT_EQ(rigid_body.source_body().state().translation.linear_velocity, pbpt::math::Vec3(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(rigid_body.source_body().state().rotation.angular_velocity, pbpt::math::Vec3(0.5f, 1.0f, 1.5f));
    EXPECT_EQ(runtime_body->state().translation.position, target_position);
    EXPECT_EQ(runtime_body->state().rotation.orientation, target_rotation);
    EXPECT_EQ(runtime_body->state().translation.linear_velocity, pbpt::math::Vec3(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(runtime_body->state().rotation.angular_velocity, pbpt::math::Vec3(0.5f, 1.0f, 1.5f));
}

}  // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
