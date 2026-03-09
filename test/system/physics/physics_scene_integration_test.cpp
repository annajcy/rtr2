#include <cmath>

#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/framework/component/physics/box_collider.hpp"
#include "rtr/framework/component/physics/rigid_body.hpp"
#include "rtr/framework/component/physics/sphere_collider.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/system/physics/collision.hpp"
#include "rtr/system/physics/physics_system.hpp"

namespace rtr::system::physics::test {

TEST(PhysicsSceneIntegrationTest, FixedTickAppliesGravityAndSyncsBackToSceneGraph) {
    PhysicsSystem          physics_system;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    moving.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    auto& rigid_body = moving.add_component<framework::component::RigidBody>(physics_system.world());

    for (std::uint64_t i = 0; i < 10; ++i) {
        physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
                                             .fixed_tick_index    = i,
                                         });
    }

    const auto pos = scene.scene_graph().node(moving.id()).local_position();
    EXPECT_NEAR(pos.x(), 0.0f, 1e-4f);
    EXPECT_NEAR(pos.y(), -4.905f, 1e-4f);
    EXPECT_NEAR(pos.z(), 0.0f, 1e-4f);
    EXPECT_NEAR(rigid_body.linear_velocity().y(), -9.81f, 1e-4f);
}

TEST(PhysicsSceneIntegrationTest, DisableGravityStopsAutomaticFalling) {
    PhysicsSystem          physics_system;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    moving.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    auto& rigid_body = moving.add_component<framework::component::RigidBody>(physics_system.world());
    rigid_body.set_use_gravity(false);

    for (std::uint64_t i = 0; i < 10; ++i) {
        physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
                                             .fixed_tick_index    = i,
                                         });
    }

    const auto pos = scene.scene_graph().node(moving.id()).local_position();
    EXPECT_NEAR(pos.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(rigid_body.linear_velocity().y(), 0.0f, 1e-5f);
}

TEST(PhysicsSceneIntegrationTest, UpwardForceCanCancelGravity) {
    PhysicsSystem          physics_system;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    moving.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    auto& rigid_body = moving.add_component<framework::component::RigidBody>(physics_system.world());

    for (std::uint64_t i = 0; i < 10; ++i) {
        rigid_body.add_force(pbpt::math::Vec3{0.0f, 9.81f * rigid_body.mass(), 0.0f});
        physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
                                             .fixed_tick_index    = i,
                                         });
    }

    const auto pos = scene.scene_graph().node(moving.id()).local_position();
    EXPECT_NEAR(pos.y(), 0.0f, 1e-4f);
    EXPECT_NEAR(rigid_body.linear_velocity().y(), 0.0f, 1e-4f);
}

TEST(PhysicsSceneIntegrationTest, FixedTickClearsAccumulatedForce) {
    PhysicsSystem          physics_system;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    auto& rigid_body = moving.add_component<framework::component::RigidBody>(physics_system.world());
    rigid_body.add_force(pbpt::math::Vec3{1.0f, 2.0f, 3.0f});

    physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 0,
                                     });

    const auto& state = physics_system.world().get_rigid_body(rigid_body.rigid_body_id()).state();
    EXPECT_NEAR(state.forces.accumulated_force.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(state.forces.accumulated_force.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(state.forces.accumulated_force.z(), 0.0f, 1e-5f);
}

TEST(PhysicsSceneIntegrationTest, TorqueUpdatesAngularVelocityAndSyncsRotation) {
    PhysicsSystem          physics_system;
    framework::core::Scene scene(1);

    auto& spinning = scene.create_game_object("spinning");
    auto& rigid_body = spinning.add_component<framework::component::RigidBody>(physics_system.world());
    rigid_body.set_use_gravity(false);
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[1][1] = 1.0f;
    rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

    for (std::uint64_t i = 0; i < 10; ++i) {
        rigid_body.add_torque(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});
        physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
                                             .fixed_tick_index    = i,
                                         });
    }

    EXPECT_GT(rigid_body.angular_velocity().y(), 0.9f);
    const auto rotation = scene.scene_graph().node(spinning.id()).local_rotation();
    EXPECT_GT(std::abs(rotation.y()), 1e-3f);
}

TEST(PhysicsSceneIntegrationTest, ZeroInverseInertiaPreventsRotation) {
    PhysicsSystem          physics_system;
    framework::core::Scene scene(1);

    auto& spinning = scene.create_game_object("spinning");
    auto& rigid_body = spinning.add_component<framework::component::RigidBody>(physics_system.world());
    rigid_body.set_use_gravity(false);

    for (std::uint64_t i = 0; i < 10; ++i) {
        rigid_body.add_torque(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});
        physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
                                             .fixed_tick_index    = i,
                                         });
    }

    EXPECT_NEAR(rigid_body.angular_velocity().y(), 0.0f, 1e-5f);
    const auto rotation = scene.scene_graph().node(spinning.id()).local_rotation();
    EXPECT_NEAR(rotation.w(), 1.0f, 1e-5f);
    EXPECT_NEAR(rotation.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(rotation.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(rotation.z(), 0.0f, 1e-5f);
}

TEST(PhysicsSceneIntegrationTest, ForceAtCenterProducesNoTorque) {
    PhysicsSystem          physics_system;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    auto& rigid_body = moving.add_component<framework::component::RigidBody>(physics_system.world());
    rigid_body.set_use_gravity(false);
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[1][1] = 1.0f;
    rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

    rigid_body.add_force_at_point(pbpt::math::Vec3{1.0f, 0.0f, 0.0f}, rigid_body.position());
    physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 0,
                                     });

    EXPECT_GT(rigid_body.linear_velocity().x(), 0.0f);
    EXPECT_NEAR(rigid_body.angular_velocity().y(), 0.0f, 1e-5f);
}

TEST(PhysicsSceneIntegrationTest, OffCenterForceProducesTranslationAndRotation) {
    PhysicsSystem          physics_system;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    auto& rigid_body = moving.add_component<framework::component::RigidBody>(physics_system.world());
    rigid_body.set_use_gravity(false);
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[1][1] = 1.0f;
    rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

    rigid_body.add_force_at_point(pbpt::math::Vec3{1.0f, 0.0f, 0.0f}, pbpt::math::Vec3{0.0f, 0.0f, 1.0f});
    physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 0,
                                     });

    EXPECT_GT(rigid_body.linear_velocity().x(), 0.0f);
    EXPECT_GT(rigid_body.angular_velocity().y(), 0.0f);
}

TEST(PhysicsSceneIntegrationTest, ResetDynamicsAndPositionInvalidateLeapfrogStateSafely) {
    PhysicsSystem          physics_system;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    moving.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    auto& rigid_body = moving.add_component<framework::component::RigidBody>(physics_system.world());
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[1][1] = 1.0f;
    rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

    for (std::uint64_t i = 0; i < 5; ++i) {
        rigid_body.add_torque(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});
        physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
                                             .fixed_tick_index    = i,
                                         });
    }

    rigid_body.set_position(pbpt::math::Vec3{0.0f, 2.0f, 0.0f});
    rigid_body.reset_dynamics();
    rigid_body.set_use_gravity(false);

    physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 5,
                                     });

    const auto pos = scene.scene_graph().node(moving.id()).local_position();
    EXPECT_TRUE(std::isfinite(pos.y()));
    EXPECT_NEAR(pos.y(), 2.0f, 1e-5f);
    EXPECT_NEAR(rigid_body.linear_velocity().y(), 0.0f, 1e-5f);
    EXPECT_NEAR(rigid_body.angular_velocity().y(), 0.0f, 1e-5f);
}

TEST(PhysicsSceneIntegrationTest, SetOrientationNormalizesAndSyncsBackToSceneGraph) {
    PhysicsSystem          physics_system;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    auto& rigid_body = moving.add_component<framework::component::RigidBody>(physics_system.world());
    rigid_body.set_use_gravity(false);

    const pbpt::math::Quat raw_orientation =
        pbpt::math::angle_axis(pbpt::math::radians(90.0f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}) * 2.0f;
    rigid_body.set_orientation(raw_orientation);
    physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 0,
                                     });

    const auto orientation = rigid_body.orientation();
    EXPECT_NEAR(orientation.length(), 1.0f, 1e-5f);
    const auto rotation = scene.scene_graph().node(moving.id()).local_rotation();
    EXPECT_NEAR(rotation.length(), 1.0f, 1e-5f);
}

TEST(PhysicsSceneIntegrationTest, DestroyRemovesRigidBodyFromPhysicsWorld) {
    PhysicsSystem          physics_system;
    framework::core::Scene scene(1);

    auto& moving               = scene.create_game_object("moving");
    auto& rigid_body_component = moving.add_component<framework::component::RigidBody>(physics_system.world());
    (void)moving.add_component<framework::component::SphereCollider>(physics_system.world(), 0.5f);
    const auto rigid_body_id = rigid_body_component.rigid_body_id();
    const auto collider_ids  = physics_system.world().colliders_for_body(rigid_body_id);
    ASSERT_EQ(collider_ids.size(), 1u);

    EXPECT_TRUE(physics_system.world().has_rigid_body(rigid_body_id));
    EXPECT_TRUE(physics_system.world().has_collider(collider_ids.front()));
    EXPECT_TRUE(scene.destroy_game_object(moving.id()));
    EXPECT_FALSE(physics_system.world().has_rigid_body(rigid_body_id));
    EXPECT_FALSE(physics_system.world().has_collider(collider_ids.front()));
}

TEST(PhysicsSceneIntegrationTest, StaticBodyTransformSyncsColliderOnNextFixedTick) {
    PhysicsSystem          physics_system;
    framework::core::Scene scene(1);

    auto& wall = scene.create_game_object("wall");
    wall.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    auto& wall_body = wall.add_component<framework::component::RigidBody>(physics_system.world());
    wall_body.set_type(RigidBodyType::Static);
    (void)wall.add_component<framework::component::BoxCollider>(
        physics_system.world(), pbpt::math::Vec3{0.5f, 0.5f, 0.5f});

    physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 0,
                                     });

    wall.node().set_local_position(pbpt::math::Vec3{2.0f, 0.0f, 0.0f});
    physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 1,
                                     });

    const auto wall_colliders = physics_system.world().colliders_for_body(wall_body.rigid_body_id());
    ASSERT_EQ(wall_colliders.size(), 1u);
    const auto& collider = physics_system.world().get_collider(wall_colliders.front());
    EXPECT_NEAR(collider.world_position.x(), 2.0f, 1e-5f);
    EXPECT_NEAR(collider.world_position.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(collider.world_position.z(), 0.0f, 1e-5f);
}

TEST(PhysicsSceneIntegrationTest, DynamicSpheresCollideAndStayStoppedOnNextTick) {
    PhysicsSystem          physics_system;
    framework::core::Scene scene(1);

    auto& left = scene.create_game_object("left");
    left.node().set_local_position(pbpt::math::Vec3{-1.0f, 0.0f, 0.0f});
    auto& left_body   = left.add_component<framework::component::RigidBody>(physics_system.world());
    (void)left.add_component<framework::component::SphereCollider>(physics_system.world(), 0.5f);
    left_body.set_use_gravity(false);

    auto& right = scene.create_game_object("right");
    right.node().set_local_position(pbpt::math::Vec3{1.0f, 0.0f, 0.0f});
    auto& right_body   = right.add_component<framework::component::RigidBody>(physics_system.world());
    (void)right.add_component<framework::component::SphereCollider>(physics_system.world(), 0.5f);
    right_body.set_use_gravity(false);

    auto& left_physics_body  = physics_system.world().get_rigid_body(left_body.rigid_body_id());
    auto& right_physics_body = physics_system.world().get_rigid_body(right_body.rigid_body_id());
    left_physics_body.state().translation.linear_velocity  = pbpt::math::Vec3{1.0f, 0.0f, 0.0f};
    right_physics_body.state().translation.linear_velocity = pbpt::math::Vec3{-1.0f, 0.0f, 0.0f};

    physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.5,
                                         .fixed_tick_index    = 0,
                                     });

    EXPECT_NEAR(left_body.linear_velocity().x(), 0.0f, 1e-4f);
    EXPECT_NEAR(right_body.linear_velocity().x(), 0.0f, 1e-4f);

    physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.5,
                                         .fixed_tick_index    = 1,
                                     });

    EXPECT_NEAR(left_body.linear_velocity().x(), 0.0f, 1e-4f);
    EXPECT_NEAR(right_body.linear_velocity().x(), 0.0f, 1e-4f);

    const auto left_colliders  = physics_system.world().colliders_for_body(left_body.rigid_body_id());
    const auto right_colliders = physics_system.world().colliders_for_body(right_body.rigid_body_id());
    ASSERT_EQ(left_colliders.size(), 1u);
    ASSERT_EQ(right_colliders.size(), 1u);
    const auto sphere_contact = collide_sphere_sphere(left_colliders.front(),
                                                      physics_system.world().get_collider(left_colliders.front()),
                                                      right_colliders.front(),
                                                      physics_system.world().get_collider(right_colliders.front()));
    ASSERT_TRUE(sphere_contact.has_value());
    EXPECT_LE(sphere_contact->penetration, 0.05f);
}

TEST(PhysicsSceneIntegrationTest, FallingSphereStaysNearStaticRotatedBoxSurface) {
    PhysicsSystem          physics_system;
    framework::core::Scene scene(1);

    auto& floor = scene.create_game_object("floor");
    floor.node().set_local_position(pbpt::math::Vec3{0.0f, -0.5f, 0.0f});
    floor.node().set_local_rotation(
        pbpt::math::angle_axis(pbpt::math::radians(15.0f), pbpt::math::Vec3{0.0f, 0.0f, 1.0f}));
    auto& floor_body = floor.add_component<framework::component::RigidBody>(physics_system.world());
    floor_body.set_type(RigidBodyType::Static);
    (void)floor.add_component<framework::component::BoxCollider>(
        physics_system.world(), pbpt::math::Vec3{2.5f, 0.25f, 2.5f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(physics_system.world());
    (void)sphere.add_component<framework::component::SphereCollider>(physics_system.world(), 0.5f);

    for (std::uint64_t i = 0; i < 90; ++i) {
        physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 1.0 / 60.0,
                                             .fixed_tick_index    = i,
                                         });
    }

    const auto sphere_colliders = physics_system.world().colliders_for_body(sphere_body.rigid_body_id());
    const auto floor_colliders  = physics_system.world().colliders_for_body(floor_body.rigid_body_id());
    ASSERT_EQ(sphere_colliders.size(), 1u);
    ASSERT_EQ(floor_colliders.size(), 1u);
    const auto contact = collide_sphere_box(sphere_colliders.front(),
                                            physics_system.world().get_collider(sphere_colliders.front()),
                                            floor_colliders.front(),
                                            physics_system.world().get_collider(floor_colliders.front()));
    ASSERT_TRUE(contact.has_value());
    EXPECT_LE(contact->penetration, 0.05f);
    EXPECT_GT(sphere_body.position().y(), -1.0f);
}

TEST(PhysicsSceneIntegrationTest, SphereAgainstStaticBoxClearsOnlyNormalVelocity) {
    PhysicsSystem          physics_system;
    framework::core::Scene scene(1);

    auto& wall = scene.create_game_object("wall");
    wall.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    auto& wall_body = wall.add_component<framework::component::RigidBody>(physics_system.world());
    wall_body.set_type(RigidBodyType::Static);
    (void)wall.add_component<framework::component::BoxCollider>(
        physics_system.world(), pbpt::math::Vec3{0.2f, 2.0f, 2.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{-1.0f, 0.0f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(physics_system.world());
    (void)sphere.add_component<framework::component::SphereCollider>(physics_system.world(), 0.5f);
    sphere_body.set_use_gravity(false);

    auto& physics_body = physics_system.world().get_rigid_body(sphere_body.rigid_body_id());
    physics_body.state().translation.linear_velocity = pbpt::math::Vec3{2.0f, 1.0f, 0.0f};

    physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.25,
                                         .fixed_tick_index    = 0,
                                     });

    EXPECT_NEAR(sphere_body.linear_velocity().x(), 0.0f, 1e-4f);
    EXPECT_NEAR(sphere_body.linear_velocity().y(), 1.0f, 1e-4f);

    const auto sphere_colliders = physics_system.world().colliders_for_body(sphere_body.rigid_body_id());
    const auto wall_colliders   = physics_system.world().colliders_for_body(wall_body.rigid_body_id());
    ASSERT_EQ(sphere_colliders.size(), 1u);
    ASSERT_EQ(wall_colliders.size(), 1u);
    const auto contact = collide_sphere_box(sphere_colliders.front(),
                                            physics_system.world().get_collider(sphere_colliders.front()),
                                            wall_colliders.front(),
                                            physics_system.world().get_collider(wall_colliders.front()));
    ASSERT_TRUE(contact.has_value());
    EXPECT_LE(contact->penetration, 0.05f);
}

}  // namespace rtr::system::physics::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
