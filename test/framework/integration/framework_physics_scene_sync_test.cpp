#include <cmath>

#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/framework/component/physics/box_collider.hpp"
#include "rtr/framework/component/physics/rigid_body.hpp"
#include "rtr/framework/component/physics/sphere_collider.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/framework/integration/physics/scene_physics_sync.hpp"
#include "rtr/system/physics/collision.hpp"
#include "rtr/system/physics/physics_world.hpp"

namespace rtr::framework::integration::physics::test {

using system::physics::ContactPairTrait;
using system::physics::RigidBodyType;
using system::physics::WorldBox;
using system::physics::WorldSphere;

namespace {

class PhysicsStepper {
public:
    system::physics::PhysicsWorld& world() { return m_world; }
    const system::physics::PhysicsWorld& world() const { return m_world; }

    void fixed_tick(framework::core::Scene& scene, const framework::core::FixedTickContext& ctx) {
        sync_scene_to_physics(scene, m_world);
        m_world.tick(static_cast<float>(ctx.fixed_delta_seconds));
        sync_physics_to_scene(scene, m_world);
    }

private:
    system::physics::PhysicsWorld m_world{};
};

pbpt::math::Mat3 diagonal_inverse_inertia(const pbpt::math::Float x,
                                          const pbpt::math::Float y,
                                          const pbpt::math::Float z) {
    pbpt::math::Mat3 result = pbpt::math::Mat3::zeros();
    result[0][0]            = x;
    result[1][1]            = y;
    result[2][2]            = z;
    return result;
}

}  // namespace

TEST(PhysicsSceneIntegrationTest, FixedTickAppliesGravityAndSyncsBackToSceneGraph) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    moving.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    auto& rigid_body = moving.add_component<framework::component::RigidBody>(physics.world());

    for (std::uint64_t i = 0; i < 10; ++i) {
        physics.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
                                             .fixed_tick_index    = i,
                                         });
    }

    const auto pos = scene.scene_graph().node(moving.id()).local_position();
    const auto expected_y = -9.81f * 0.1f * 0.1f * (10.0f * 11.0f * 0.5f);
    EXPECT_NEAR(pos.x(), 0.0f, 1e-4f);
    EXPECT_NEAR(pos.y(), expected_y, 1e-4f);
    EXPECT_NEAR(pos.z(), 0.0f, 1e-4f);
    EXPECT_NEAR(rigid_body.linear_velocity().y(), -9.81f, 1e-4f);
}

TEST(PhysicsSceneIntegrationTest, DisableGravityStopsAutomaticFalling) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    moving.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    auto& rigid_body = moving.add_component<framework::component::RigidBody>(physics.world());
    rigid_body.set_use_gravity(false);

    for (std::uint64_t i = 0; i < 10; ++i) {
        physics.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
                                             .fixed_tick_index    = i,
                                         });
    }

    const auto pos = scene.scene_graph().node(moving.id()).local_position();
    EXPECT_NEAR(pos.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(rigid_body.linear_velocity().y(), 0.0f, 1e-5f);
}

TEST(PhysicsSceneIntegrationTest, UpwardForceCanCancelGravity) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    moving.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    auto& rigid_body = moving.add_component<framework::component::RigidBody>(physics.world());

    for (std::uint64_t i = 0; i < 10; ++i) {
        rigid_body.add_force(pbpt::math::Vec3{0.0f, 9.81f * rigid_body.mass(), 0.0f});
        physics.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
                                             .fixed_tick_index    = i,
                                         });
    }

    const auto pos = scene.scene_graph().node(moving.id()).local_position();
    EXPECT_NEAR(pos.y(), 0.0f, 1e-4f);
    EXPECT_NEAR(rigid_body.linear_velocity().y(), 0.0f, 1e-4f);
}

TEST(PhysicsSceneIntegrationTest, FixedTickClearsAccumulatedForce) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    auto& rigid_body = moving.add_component<framework::component::RigidBody>(physics.world());
    rigid_body.add_force(pbpt::math::Vec3{1.0f, 2.0f, 3.0f});

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 0,
                                     });

    const auto& state = physics.world().get_rigid_body(rigid_body.rigid_body_id()).state();
    EXPECT_NEAR(state.forces.accumulated_force.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(state.forces.accumulated_force.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(state.forces.accumulated_force.z(), 0.0f, 1e-5f);
}

TEST(PhysicsSceneIntegrationTest, TorqueUpdatesAngularVelocityAndSyncsRotation) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& spinning = scene.create_game_object("spinning");
    auto& rigid_body = spinning.add_component<framework::component::RigidBody>(physics.world());
    rigid_body.set_use_gravity(false);
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[1][1] = 1.0f;
    rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

    for (std::uint64_t i = 0; i < 10; ++i) {
        rigid_body.add_torque(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});
        physics.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
                                             .fixed_tick_index    = i,
                                         });
    }

    EXPECT_GT(rigid_body.angular_velocity().y(), 0.9f);
    const auto rotation = scene.scene_graph().node(spinning.id()).local_rotation();
    EXPECT_GT(std::abs(rotation.y()), 1e-3f);
}

TEST(PhysicsSceneIntegrationTest, ZeroInverseInertiaPreventsRotation) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& spinning = scene.create_game_object("spinning");
    auto& rigid_body = spinning.add_component<framework::component::RigidBody>(physics.world());
    rigid_body.set_use_gravity(false);

    for (std::uint64_t i = 0; i < 10; ++i) {
        rigid_body.add_torque(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});
        physics.fixed_tick(scene, framework::core::FixedTickContext{
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
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    auto& rigid_body = moving.add_component<framework::component::RigidBody>(physics.world());
    rigid_body.set_use_gravity(false);
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[1][1] = 1.0f;
    rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

    rigid_body.add_force_at_point(pbpt::math::Vec3{1.0f, 0.0f, 0.0f}, rigid_body.position());
    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 0,
                                     });

    EXPECT_GT(rigid_body.linear_velocity().x(), 0.0f);
    EXPECT_NEAR(rigid_body.angular_velocity().y(), 0.0f, 1e-5f);
}

TEST(PhysicsSceneIntegrationTest, OffCenterForceProducesTranslationAndRotation) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    auto& rigid_body = moving.add_component<framework::component::RigidBody>(physics.world());
    rigid_body.set_use_gravity(false);
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[1][1] = 1.0f;
    rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

    rigid_body.add_force_at_point(pbpt::math::Vec3{1.0f, 0.0f, 0.0f}, pbpt::math::Vec3{0.0f, 0.0f, 1.0f});
    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 0,
                                     });

    EXPECT_GT(rigid_body.linear_velocity().x(), 0.0f);
    EXPECT_GT(rigid_body.angular_velocity().y(), 0.0f);
}

TEST(PhysicsSceneIntegrationTest, ResetDynamicsAndPositionInvalidateLeapfrogStateSafely) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    moving.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    auto& rigid_body = moving.add_component<framework::component::RigidBody>(physics.world());
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[1][1] = 1.0f;
    rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

    for (std::uint64_t i = 0; i < 5; ++i) {
        rigid_body.add_torque(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});
        physics.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
                                             .fixed_tick_index    = i,
                                         });
    }

    rigid_body.set_position(pbpt::math::Vec3{0.0f, 2.0f, 0.0f});
    rigid_body.reset_dynamics();
    rigid_body.set_use_gravity(false);

    physics.fixed_tick(scene, framework::core::FixedTickContext{
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
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    auto& rigid_body = moving.add_component<framework::component::RigidBody>(physics.world());
    rigid_body.set_use_gravity(false);

    const pbpt::math::Quat raw_orientation =
        pbpt::math::angle_axis(pbpt::math::radians(90.0f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}) * 2.0f;
    rigid_body.set_orientation(raw_orientation);
    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 0,
                                     });

    const auto orientation = rigid_body.orientation();
    EXPECT_NEAR(orientation.length(), 1.0f, 1e-5f);
    const auto rotation = scene.scene_graph().node(moving.id()).local_rotation();
    EXPECT_NEAR(rotation.length(), 1.0f, 1e-5f);
}

TEST(PhysicsSceneIntegrationTest, DestroyRemovesRigidBodyFromPhysicsWorld) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& moving               = scene.create_game_object("moving");
    auto& rigid_body_component = moving.add_component<framework::component::RigidBody>(physics.world());
    (void)moving.add_component<framework::component::SphereCollider>(physics.world(), 0.5f);
    const auto rigid_body_id = rigid_body_component.rigid_body_id();
    const auto collider_ids  = physics.world().colliders_for_body(rigid_body_id);
    ASSERT_EQ(collider_ids.size(), 1u);

    EXPECT_TRUE(physics.world().has_rigid_body(rigid_body_id));
    EXPECT_TRUE(physics.world().has_collider(collider_ids.front()));
    EXPECT_TRUE(scene.destroy_game_object(moving.id()));
    EXPECT_FALSE(physics.world().has_rigid_body(rigid_body_id));
    EXPECT_FALSE(physics.world().has_collider(collider_ids.front()));
}

TEST(PhysicsSceneIntegrationTest, StaticBodyTransformSyncsColliderOnNextFixedTick) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& wall = scene.create_game_object("wall");
    wall.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    auto& wall_body = wall.add_component<framework::component::RigidBody>(physics.world());
    wall_body.set_type(RigidBodyType::Static);
    (void)wall.add_component<framework::component::BoxCollider>(
        physics.world(), pbpt::math::Vec3{0.5f, 0.5f, 0.5f});

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 0,
                                     });

    wall.node().set_local_position(pbpt::math::Vec3{2.0f, 0.0f, 0.0f});
    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 1,
                                     });

    const auto wall_colliders = physics.world().colliders_for_body(wall_body.rigid_body_id());
    ASSERT_EQ(wall_colliders.size(), 1u);
    const auto world_collider = physics.world().get_world_collider(wall_colliders.front());
    const auto* world_box = std::get_if<WorldBox>(&world_collider);
    ASSERT_NE(world_box, nullptr);
    EXPECT_NEAR(world_box->center.x(), 2.0f, 1e-5f);
    EXPECT_NEAR(world_box->center.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(world_box->center.z(), 0.0f, 1e-5f);
}

TEST(PhysicsSceneIntegrationTest, DynamicSpheresCollideAndStayStoppedOnNextTick) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& left = scene.create_game_object("left");
    left.node().set_local_position(pbpt::math::Vec3{-1.0f, 0.0f, 0.0f});
    auto& left_body   = left.add_component<framework::component::RigidBody>(physics.world());
    (void)left.add_component<framework::component::SphereCollider>(physics.world(), 0.5f);
    left_body.set_use_gravity(false);

    auto& right = scene.create_game_object("right");
    right.node().set_local_position(pbpt::math::Vec3{1.0f, 0.0f, 0.0f});
    auto& right_body   = right.add_component<framework::component::RigidBody>(physics.world());
    (void)right.add_component<framework::component::SphereCollider>(physics.world(), 0.5f);
    right_body.set_use_gravity(false);

    auto& left_physics_body  = physics.world().get_rigid_body(left_body.rigid_body_id());
    auto& right_physics_body = physics.world().get_rigid_body(right_body.rigid_body_id());
    left_physics_body.state().translation.linear_velocity  = pbpt::math::Vec3{1.0f, 0.0f, 0.0f};
    right_physics_body.state().translation.linear_velocity = pbpt::math::Vec3{-1.0f, 0.0f, 0.0f};

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.6,
                                         .fixed_tick_index    = 0,
                                     });

    EXPECT_NEAR(left_body.linear_velocity().x(), 0.0f, 1e-4f);
    EXPECT_NEAR(right_body.linear_velocity().x(), 0.0f, 1e-4f);

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.5,
                                         .fixed_tick_index    = 1,
                                     });

    EXPECT_NEAR(left_body.linear_velocity().x(), 0.0f, 1e-4f);
    EXPECT_NEAR(right_body.linear_velocity().x(), 0.0f, 1e-4f);

    const auto left_colliders  = physics.world().colliders_for_body(left_body.rigid_body_id());
    const auto right_colliders = physics.world().colliders_for_body(right_body.rigid_body_id());
    ASSERT_EQ(left_colliders.size(), 1u);
    ASSERT_EQ(right_colliders.size(), 1u);
    const auto left_world_collider = physics.world().get_world_collider(left_colliders.front());
    const auto right_world_collider = physics.world().get_world_collider(right_colliders.front());
    const auto* left_sphere = std::get_if<WorldSphere>(&left_world_collider);
    const auto* right_sphere = std::get_if<WorldSphere>(&right_world_collider);
    ASSERT_NE(left_sphere, nullptr);
    ASSERT_NE(right_sphere, nullptr);
    const auto center_distance = (right_sphere->center - left_sphere->center).length();
    EXPECT_GE(center_distance, left_sphere->radius + right_sphere->radius - 1e-4f);
}

TEST(PhysicsSceneIntegrationTest, FallingSphereStaysNearStaticRotatedBoxSurface) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& floor = scene.create_game_object("floor");
    floor.node().set_local_position(pbpt::math::Vec3{0.0f, -0.5f, 0.0f});
    floor.node().set_local_rotation(
        pbpt::math::angle_axis(pbpt::math::radians(15.0f), pbpt::math::Vec3{0.0f, 0.0f, 1.0f}));
    auto& floor_body = floor.add_component<framework::component::RigidBody>(physics.world());
    floor_body.set_type(RigidBodyType::Static);
    (void)floor.add_component<framework::component::BoxCollider>(
        physics.world(), pbpt::math::Vec3{2.5f, 0.25f, 2.5f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(physics.world());
    (void)sphere.add_component<framework::component::SphereCollider>(physics.world(), 0.5f);

    for (std::uint64_t i = 0; i < 90; ++i) {
        physics.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 1.0 / 60.0,
                                             .fixed_tick_index    = i,
                                         });
    }

    const auto sphere_colliders = physics.world().colliders_for_body(sphere_body.rigid_body_id());
    const auto floor_colliders  = physics.world().colliders_for_body(floor_body.rigid_body_id());
    ASSERT_EQ(sphere_colliders.size(), 1u);
    ASSERT_EQ(floor_colliders.size(), 1u);
    const auto sphere_world_collider = physics.world().get_world_collider(sphere_colliders.front());
    const auto floor_world_collider = physics.world().get_world_collider(floor_colliders.front());
    const auto* world_sphere = std::get_if<WorldSphere>(&sphere_world_collider);
    const auto* world_box = std::get_if<WorldBox>(&floor_world_collider);
    ASSERT_NE(world_sphere, nullptr);
    ASSERT_NE(world_box, nullptr);
    const auto contact = ContactPairTrait<WorldSphere, WorldBox>::generate(*world_sphere, *world_box);
    EXPECT_FALSE(contact.is_valid() && contact.penetration > 0.05f);
    EXPECT_GT(sphere_body.position().y(), -1.0f);
}

TEST(PhysicsSceneIntegrationTest, SphereAgainstStaticBoxClearsOnlyNormalVelocity) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& wall = scene.create_game_object("wall");
    wall.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    auto& wall_body = wall.add_component<framework::component::RigidBody>(physics.world());
    wall_body.set_type(RigidBodyType::Static);
    (void)wall.add_component<framework::component::BoxCollider>(
        physics.world(), pbpt::math::Vec3{0.2f, 2.0f, 2.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{-1.0f, 0.0f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(physics.world());
    (void)sphere.add_component<framework::component::SphereCollider>(physics.world(), 0.5f);
    sphere_body.set_use_gravity(false);

    auto& physics_body = physics.world().get_rigid_body(sphere_body.rigid_body_id());
    physics_body.state().translation.linear_velocity = pbpt::math::Vec3{2.0f, 1.0f, 0.0f};

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.25,
                                         .fixed_tick_index    = 0,
                                     });

    EXPECT_NEAR(sphere_body.linear_velocity().x(), 0.0f, 1e-4f);
    EXPECT_NEAR(sphere_body.linear_velocity().y(), 1.0f, 1e-4f);
    EXPECT_NEAR(sphere_body.angular_velocity().x(), 0.0f, 1e-5f);
    EXPECT_NEAR(sphere_body.angular_velocity().y(), 0.0f, 1e-5f);
    EXPECT_NEAR(sphere_body.angular_velocity().z(), 0.0f, 1e-5f);

    const auto sphere_colliders = physics.world().colliders_for_body(sphere_body.rigid_body_id());
    const auto wall_colliders   = physics.world().colliders_for_body(wall_body.rigid_body_id());
    ASSERT_EQ(sphere_colliders.size(), 1u);
    ASSERT_EQ(wall_colliders.size(), 1u);
    const auto sphere_world_collider = physics.world().get_world_collider(sphere_colliders.front());
    const auto wall_world_collider = physics.world().get_world_collider(wall_colliders.front());
    const auto* world_sphere = std::get_if<WorldSphere>(&sphere_world_collider);
    const auto* world_box = std::get_if<WorldBox>(&wall_world_collider);
    ASSERT_NE(world_sphere, nullptr);
    ASSERT_NE(world_box, nullptr);
    const auto contact = ContactPairTrait<WorldSphere, WorldBox>::generate(*world_sphere, *world_box);
    EXPECT_FALSE(contact.is_valid() && contact.penetration > 0.05f);
}

TEST(PhysicsSceneIntegrationTest, SphereAgainstStaticFloorFrictionProducesAngularVelocity) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& floor = scene.create_game_object("floor");
    floor.node().set_local_position(pbpt::math::Vec3{0.0f, -0.5f, 0.0f});
    (void)floor.add_component<framework::component::RigidBody>(
        physics.world(), 1.0f, RigidBodyType::Static, false, pbpt::math::Mat3::zeros(), 0.0f, 0.36f);
    (void)floor.add_component<framework::component::BoxCollider>(
        physics.world(), pbpt::math::Vec3{2.0f, 0.25f, 2.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{0.0f, 0.2f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(
        physics.world(), 1.0f, RigidBodyType::Dynamic, false, diagonal_inverse_inertia(0.0f, 0.0f, 1.0f), 0.0f, 0.25f);
    (void)sphere.add_component<framework::component::SphereCollider>(physics.world(), 0.5f);

    auto& physics_body = physics.world().get_rigid_body(sphere_body.rigid_body_id());
    physics_body.state().translation.linear_velocity = pbpt::math::Vec3{2.0f, -1.0f, 0.0f};

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 0,
                                     });

    EXPECT_NEAR(sphere_body.linear_velocity().y(), 0.0f, 1e-4f);
    EXPECT_LT(sphere_body.linear_velocity().x(), 2.0f);
    EXPECT_GT(std::abs(sphere_body.angular_velocity().z()), 1e-4f);
}

TEST(PhysicsSceneIntegrationTest, SphereAgainstStaticBoxUsesMaxRestitutionForBounce) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& wall = scene.create_game_object("wall");
    auto& wall_body = wall.add_component<framework::component::RigidBody>(
        physics.world(), 1.0f, RigidBodyType::Static, false, pbpt::math::Mat3::zeros(), 0.2f, 0.0f);
    (void)wall.add_component<framework::component::BoxCollider>(
        physics.world(), pbpt::math::Vec3{0.2f, 2.0f, 2.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{-1.0f, 0.0f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(
        physics.world(), 1.0f, RigidBodyType::Dynamic, false, pbpt::math::Mat3::zeros(), 0.8f, 0.0f);
    (void)sphere.add_component<framework::component::SphereCollider>(physics.world(), 0.5f);

    auto& physics_body = physics.world().get_rigid_body(sphere_body.rigid_body_id());
    physics_body.state().translation.linear_velocity = pbpt::math::Vec3{2.0f, 0.0f, 0.0f};

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.25,
                                         .fixed_tick_index    = 0,
                                     });

    EXPECT_NEAR(sphere_body.linear_velocity().x(), -1.6f, 1e-4f);
    EXPECT_NEAR(sphere_body.linear_velocity().y(), 0.0f, 1e-4f);
    EXPECT_NEAR(wall_body.restitution(), 0.2f, 1e-5f);
}

TEST(PhysicsSceneIntegrationTest, SphereAgainstStaticFloorFrictionReducesTangentialVelocity) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& floor = scene.create_game_object("floor");
    floor.node().set_local_position(pbpt::math::Vec3{0.0f, -0.5f, 0.0f});
    (void)floor.add_component<framework::component::RigidBody>(
        physics.world(), 1.0f, RigidBodyType::Static, false, pbpt::math::Mat3::zeros(), 0.0f, 0.36f);
    (void)floor.add_component<framework::component::BoxCollider>(
        physics.world(), pbpt::math::Vec3{2.0f, 0.25f, 2.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{0.0f, 0.2f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(
        physics.world(), 1.0f, RigidBodyType::Dynamic, false, pbpt::math::Mat3::zeros(), 0.0f, 0.25f);
    (void)sphere.add_component<framework::component::SphereCollider>(physics.world(), 0.5f);

    auto& physics_body = physics.world().get_rigid_body(sphere_body.rigid_body_id());
    physics_body.state().translation.linear_velocity = pbpt::math::Vec3{2.0f, -1.0f, 0.0f};

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 0,
                                     });

    EXPECT_NEAR(sphere_body.linear_velocity().x(), 1.7f, 1e-4f);
    EXPECT_NEAR(sphere_body.linear_velocity().y(), 0.0f, 1e-4f);
}

TEST(PhysicsSceneIntegrationTest, ZeroInverseInertiaKeepsCollisionFromAddingAngularVelocity) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& floor = scene.create_game_object("floor");
    floor.node().set_local_position(pbpt::math::Vec3{0.0f, -0.5f, 0.0f});
    (void)floor.add_component<framework::component::RigidBody>(
        physics.world(), 1.0f, RigidBodyType::Static, false, pbpt::math::Mat3::zeros(), 0.0f, 0.36f);
    (void)floor.add_component<framework::component::BoxCollider>(
        physics.world(), pbpt::math::Vec3{2.0f, 0.25f, 2.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{0.0f, 0.2f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(
        physics.world(), 1.0f, RigidBodyType::Dynamic, false, pbpt::math::Mat3::zeros(), 0.0f, 0.25f);
    (void)sphere.add_component<framework::component::SphereCollider>(physics.world(), 0.5f);

    auto& physics_body = physics.world().get_rigid_body(sphere_body.rigid_body_id());
    physics_body.state().translation.linear_velocity = pbpt::math::Vec3{2.0f, -1.0f, 0.0f};

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 0,
                                     });

    EXPECT_NEAR(sphere_body.angular_velocity().x(), 0.0f, 1e-5f);
    EXPECT_NEAR(sphere_body.angular_velocity().y(), 0.0f, 1e-5f);
    EXPECT_NEAR(sphere_body.angular_velocity().z(), 0.0f, 1e-5f);
    EXPECT_LT(sphere_body.linear_velocity().x(), 2.0f);
}

TEST(PhysicsSceneIntegrationTest, ZeroFrictionPreservesTangentialVelocity) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& floor = scene.create_game_object("floor");
    floor.node().set_local_position(pbpt::math::Vec3{0.0f, -0.5f, 0.0f});
    (void)floor.add_component<framework::component::RigidBody>(
        physics.world(), 1.0f, RigidBodyType::Static, false, pbpt::math::Mat3::zeros(), 0.0f, 0.0f);
    (void)floor.add_component<framework::component::BoxCollider>(
        physics.world(), pbpt::math::Vec3{2.0f, 0.25f, 2.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{0.0f, 0.2f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(
        physics.world(), 1.0f, RigidBodyType::Dynamic, false, pbpt::math::Mat3::zeros(), 0.0f, 0.0f);
    (void)sphere.add_component<framework::component::SphereCollider>(physics.world(), 0.5f);

    auto& physics_body = physics.world().get_rigid_body(sphere_body.rigid_body_id());
    physics_body.state().translation.linear_velocity = pbpt::math::Vec3{2.0f, -1.0f, 0.0f};

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                         .fixed_tick_index    = 0,
                                     });

    EXPECT_NEAR(sphere_body.linear_velocity().x(), 2.0f, 1e-4f);
    EXPECT_NEAR(sphere_body.linear_velocity().y(), 0.0f, 1e-4f);
}

TEST(PhysicsSceneIntegrationTest, DynamicBodiesOffCenterCollisionProducesFiniteAngularVelocity) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& box = scene.create_game_object("box");
    auto& box_body = box.add_component<framework::component::RigidBody>(
        physics.world(), 1.0f, RigidBodyType::Dynamic, false, diagonal_inverse_inertia(0.0f, 0.0f, 1.0f), 0.0f, 0.36f);
    (void)box.add_component<framework::component::BoxCollider>(
        physics.world(), pbpt::math::Vec3{0.2f, 2.0f, 2.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{-1.0f, 0.6f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(
        physics.world(), 1.0f, RigidBodyType::Dynamic, false, diagonal_inverse_inertia(0.0f, 0.0f, 1.0f), 0.0f, 0.25f);
    (void)sphere.add_component<framework::component::SphereCollider>(physics.world(), 0.5f);

    auto& sphere_physics_body = physics.world().get_rigid_body(sphere_body.rigid_body_id());
    sphere_physics_body.state().translation.linear_velocity = pbpt::math::Vec3{2.0f, -1.0f, 0.0f};

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.25,
                                         .fixed_tick_index    = 0,
                                     });

    EXPECT_TRUE(std::isfinite(sphere_body.linear_velocity().x()));
    EXPECT_TRUE(std::isfinite(sphere_body.linear_velocity().y()));
    EXPECT_TRUE(std::isfinite(box_body.linear_velocity().x()));
    EXPECT_TRUE(std::isfinite(box_body.linear_velocity().y()));
    EXPECT_TRUE(std::isfinite(sphere_body.angular_velocity().z()));
    EXPECT_TRUE(std::isfinite(box_body.angular_velocity().z()));
    EXPECT_GT(std::abs(sphere_body.angular_velocity().z()), 1e-4f);
    EXPECT_GT(std::abs(box_body.angular_velocity().z()), 1e-4f);
}

}  // namespace rtr::framework::integration::physics::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
