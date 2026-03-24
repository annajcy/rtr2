#include <cmath>

#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/framework/component/physics/rigid_body/box_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/plane_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/component/physics/rigid_body/sphere_collider.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/framework/integration/physics/rigid_body_scene_sync.hpp"
#include "rtr/system/physics/rigid_body/collision/box_box.hpp"
#include "rtr/system/physics/rigid_body/collision/box_plane.hpp"
#include "rtr/system/physics/rigid_body/collision/contact.hpp"
#include "rtr/system/physics/rigid_body/collision/sphere_box.hpp"
#include "rtr/system/physics/rigid_body/collision/sphere_plane.hpp"
#include "rtr/system/physics/rigid_body/collision/sphere_sphere.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_system.hpp"

namespace rtr::framework::integration::physics::test {

using system::physics::rb::ContactPairTrait;
using system::physics::rb::RigidBodyType;
using system::physics::rb::WorldBox;
using system::physics::rb::WorldSphere;

namespace {

class PhysicsStepper {
public:
    system::physics::rb::RigidBodySystem& world() { return m_world; }
    const system::physics::rb::RigidBodySystem& world() const { return m_world; }

    void fixed_tick(framework::core::Scene& scene, const framework::core::FixedTickContext& ctx) {
        sync_scene_to_rigid_body(scene, m_world);
        m_world.step(static_cast<float>(ctx.fixed_delta_seconds));
        sync_rigid_body_to_scene(scene, m_world);
    }

private:
    system::physics::rb::RigidBodySystem m_world{};
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

system::physics::rb::RigidBodyID body_id_for(const PhysicsStepper& physics,
                                             const framework::core::GameObject& game_object) {
    const auto body_id = physics.world().try_get_rigid_body_id_for_owner(game_object.id());
    EXPECT_TRUE(body_id.has_value());
    return *body_id;
}

system::physics::rb::RigidBody& body_for(PhysicsStepper& physics,
                                         const framework::core::GameObject& game_object) {
    auto* body = physics.world().try_get_rigid_body_for_owner(game_object.id());
    EXPECT_NE(body, nullptr);
    return *body;
}

const system::physics::rb::RigidBody& body_for(const PhysicsStepper& physics,
                                               const framework::core::GameObject& game_object) {
    const auto* body = physics.world().try_get_rigid_body_for_owner(game_object.id());
    EXPECT_NE(body, nullptr);
    return *body;
}

std::vector<system::physics::rb::ColliderID> colliders_for(const PhysicsStepper& physics,
                                                           const framework::core::GameObject& game_object) {
    return physics.world().colliders_for_body(body_id_for(physics, game_object));
}

}  // namespace

TEST(PhysicsSceneIntegrationTest, FixedTickAppliesGravityAndSyncsBackToSceneGraph) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    moving.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    auto& rigid_body = moving.add_component<framework::component::RigidBody>();

    for (std::uint64_t i = 0; i < 10; ++i) {
        physics.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
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
    auto& rigid_body = moving.add_component<framework::component::RigidBody>();
    rigid_body.set_use_gravity(false);

    for (std::uint64_t i = 0; i < 10; ++i) {
        physics.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
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
    auto& rigid_body = moving.add_component<framework::component::RigidBody>();

    for (std::uint64_t i = 0; i < 10; ++i) {
        rigid_body.add_force(pbpt::math::Vec3{0.0f, 9.81f * rigid_body.mass(), 0.0f});
        physics.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
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
    auto& rigid_body = moving.add_component<framework::component::RigidBody>();
    rigid_body.add_force(pbpt::math::Vec3{1.0f, 2.0f, 3.0f});

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                     });

    const auto& state = body_for(physics, moving).state();
    EXPECT_NEAR(state.forces.accumulated_force.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(state.forces.accumulated_force.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(state.forces.accumulated_force.z(), 0.0f, 1e-5f);
}

TEST(PhysicsSceneIntegrationTest, TorqueUpdatesAngularVelocityAndSyncsRotation) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& spinning = scene.create_game_object("spinning");
    auto& rigid_body = spinning.add_component<framework::component::RigidBody>();
    rigid_body.set_use_gravity(false);
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[1][1] = 1.0f;
    rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

    for (std::uint64_t i = 0; i < 10; ++i) {
        rigid_body.add_torque(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});
        physics.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
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
    auto& rigid_body = spinning.add_component<framework::component::RigidBody>();
    rigid_body.set_use_gravity(false);

    for (std::uint64_t i = 0; i < 10; ++i) {
        rigid_body.add_torque(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});
        physics.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
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
    auto& rigid_body = moving.add_component<framework::component::RigidBody>();
    rigid_body.set_use_gravity(false);
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[1][1] = 1.0f;
    rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

    rigid_body.add_force_at_point(pbpt::math::Vec3{1.0f, 0.0f, 0.0f}, rigid_body.position());
    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                     });

    EXPECT_GT(rigid_body.linear_velocity().x(), 0.0f);
    EXPECT_NEAR(rigid_body.angular_velocity().y(), 0.0f, 1e-5f);
}

TEST(PhysicsSceneIntegrationTest, OffCenterForceProducesTranslationAndRotation) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    auto& rigid_body = moving.add_component<framework::component::RigidBody>();
    rigid_body.set_use_gravity(false);
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[1][1] = 1.0f;
    rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

    rigid_body.add_force_at_point(pbpt::math::Vec3{1.0f, 0.0f, 0.0f}, pbpt::math::Vec3{0.0f, 0.0f, 1.0f});
    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                     });

    EXPECT_GT(rigid_body.linear_velocity().x(), 0.0f);
    EXPECT_GT(rigid_body.angular_velocity().y(), 0.0f);
}

TEST(PhysicsSceneIntegrationTest, ResetDynamicsAndPositionInvalidateLeapfrogStateSafely) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    moving.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    auto& rigid_body = moving.add_component<framework::component::RigidBody>();
    pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
    inverse_inertia_tensor_ref[1][1] = 1.0f;
    rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

    for (std::uint64_t i = 0; i < 5; ++i) {
        rigid_body.add_torque(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});
        physics.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 0.1,
                                         });
    }

    rigid_body.set_position(pbpt::math::Vec3{0.0f, 2.0f, 0.0f});
    rigid_body.reset_dynamics();
    rigid_body.set_use_gravity(false);

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
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
    auto& rigid_body = moving.add_component<framework::component::RigidBody>();
    rigid_body.set_use_gravity(false);

    const pbpt::math::Quat raw_orientation =
        pbpt::math::angle_axis(pbpt::math::radians(90.0f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}) * 2.0f;
    rigid_body.set_orientation(raw_orientation);
    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                     });

    const auto orientation = rigid_body.orientation();
    EXPECT_NEAR(orientation.length(), 1.0f, 1e-5f);
    const auto rotation = scene.scene_graph().node(moving.id()).local_rotation();
    EXPECT_NEAR(rotation.length(), 1.0f, 1e-5f);
}

TEST(PhysicsSceneIntegrationTest, DestroyRemovesRigidBodyFromRigidBodySystem) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& moving               = scene.create_game_object("moving");
    auto& rigid_body_component = moving.add_component<framework::component::RigidBody>();
    (void)moving.add_component<framework::component::SphereCollider>(0.5f);
    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                     });
    const auto rigid_body_id = body_id_for(physics, moving);
    const auto collider_ids  = physics.world().colliders_for_body(rigid_body_id);
    ASSERT_EQ(collider_ids.size(), 1u);

    EXPECT_TRUE(physics.world().has_rigid_body(rigid_body_id));
    EXPECT_TRUE(physics.world().has_collider(collider_ids.front()));
    EXPECT_TRUE(scene.destroy_game_object(moving.id()));
    sync_scene_to_rigid_body(scene, physics.world());
    EXPECT_FALSE(physics.world().has_rigid_body(rigid_body_id));
    EXPECT_FALSE(physics.world().has_collider(collider_ids.front()));
}

TEST(PhysicsSceneIntegrationTest, StaticBodyTransformSyncsColliderOnNextFixedTick) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& wall = scene.create_game_object("wall");
    wall.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    auto& wall_body = wall.add_component<framework::component::RigidBody>();
    wall_body.set_type(RigidBodyType::Static);
    (void)wall.add_component<framework::component::BoxCollider>(pbpt::math::Vec3{0.5f, 0.5f, 0.5f});

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                     });

    wall.node().set_local_position(pbpt::math::Vec3{2.0f, 0.0f, 0.0f});
    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                     });

    const auto wall_colliders = colliders_for(physics, wall);
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
    auto& left_body   = left.add_component<framework::component::RigidBody>();
    (void)left.add_component<framework::component::SphereCollider>(0.5f);
    left_body.set_use_gravity(false);

    auto& right = scene.create_game_object("right");
    right.node().set_local_position(pbpt::math::Vec3{1.0f, 0.0f, 0.0f});
    auto& right_body   = right.add_component<framework::component::RigidBody>();
    (void)right.add_component<framework::component::SphereCollider>(0.5f);
    right_body.set_use_gravity(false);

    left_body.set_linear_velocity(pbpt::math::Vec3{1.0f, 0.0f, 0.0f});
    right_body.set_linear_velocity(pbpt::math::Vec3{-1.0f, 0.0f, 0.0f});

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.6,
                                     });

    EXPECT_NEAR(left_body.linear_velocity().x(), 0.0f, 1e-4f);
    EXPECT_NEAR(right_body.linear_velocity().x(), 0.0f, 1e-4f);

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.5,
                                     });

    EXPECT_NEAR(left_body.linear_velocity().x(), 0.0f, 1e-4f);
    EXPECT_NEAR(right_body.linear_velocity().x(), 0.0f, 1e-4f);

    const auto left_colliders  = colliders_for(physics, left);
    const auto right_colliders = colliders_for(physics, right);
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
    auto& floor_body = floor.add_component<framework::component::RigidBody>();
    floor_body.set_type(RigidBodyType::Static);
    (void)floor.add_component<framework::component::BoxCollider>(pbpt::math::Vec3{2.5f, 0.25f, 2.5f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>();
    (void)sphere.add_component<framework::component::SphereCollider>(0.5f);

    for (std::uint64_t i = 0; i < 90; ++i) {
        physics.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 1.0 / 60.0,
                                         });
    }

    const auto sphere_colliders = colliders_for(physics, sphere);
    const auto floor_colliders  = colliders_for(physics, floor);
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
    auto& wall_body = wall.add_component<framework::component::RigidBody>();
    wall_body.set_type(RigidBodyType::Static);
    (void)wall.add_component<framework::component::BoxCollider>(pbpt::math::Vec3{0.2f, 2.0f, 2.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{-1.0f, 0.0f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>();
    (void)sphere.add_component<framework::component::SphereCollider>(0.5f);
    sphere_body.set_use_gravity(false);
    sphere_body.set_linear_velocity(pbpt::math::Vec3{2.0f, 1.0f, 0.0f});

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.25,
                                     });

    EXPECT_NEAR(sphere_body.linear_velocity().x(), 0.0f, 1e-4f);
    EXPECT_NEAR(sphere_body.linear_velocity().y(), 1.0f, 1e-4f);
    EXPECT_NEAR(sphere_body.angular_velocity().x(), 0.0f, 1e-5f);
    EXPECT_NEAR(sphere_body.angular_velocity().y(), 0.0f, 1e-5f);
    EXPECT_NEAR(sphere_body.angular_velocity().z(), 0.0f, 1e-5f);

    const auto sphere_colliders = colliders_for(physics, sphere);
    const auto wall_colliders   = colliders_for(physics, wall);
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
        make_source_body(1.0f, RigidBodyType::Static, false, pbpt::math::Mat3::zeros(), 0.0f, 0.36f));
    (void)floor.add_component<framework::component::BoxCollider>(pbpt::math::Vec3{2.0f, 0.25f, 2.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{0.0f, 0.2f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(make_source_body(
        1.0f,
        RigidBodyType::Dynamic,
        false,
        diagonal_inverse_inertia(0.0f, 0.0f, 1.0f),
        0.0f,
        0.25f
    ));
    (void)sphere.add_component<framework::component::SphereCollider>(0.5f);

    sphere_body.set_linear_velocity(pbpt::math::Vec3{2.0f, -1.0f, 0.0f});

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
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
        make_source_body(1.0f, RigidBodyType::Static, false, pbpt::math::Mat3::zeros(), 0.2f, 0.0f));
    (void)wall.add_component<framework::component::BoxCollider>(pbpt::math::Vec3{0.2f, 2.0f, 2.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{-1.0f, 0.0f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(
        make_source_body(1.0f, RigidBodyType::Dynamic, false, pbpt::math::Mat3::zeros(), 0.8f, 0.0f));
    (void)sphere.add_component<framework::component::SphereCollider>(0.5f);

    sphere_body.set_linear_velocity(pbpt::math::Vec3{2.0f, 0.0f, 0.0f});

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.25,
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
        make_source_body(1.0f, RigidBodyType::Static, false, pbpt::math::Mat3::zeros(), 0.0f, 0.36f));
    (void)floor.add_component<framework::component::BoxCollider>(pbpt::math::Vec3{2.0f, 0.25f, 2.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{0.0f, 0.2f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(
        make_source_body(1.0f, RigidBodyType::Dynamic, false, pbpt::math::Mat3::zeros(), 0.0f, 0.25f));
    (void)sphere.add_component<framework::component::SphereCollider>(0.5f);

    sphere_body.set_linear_velocity(pbpt::math::Vec3{2.0f, -1.0f, 0.0f});

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
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
        make_source_body(1.0f, RigidBodyType::Static, false, pbpt::math::Mat3::zeros(), 0.0f, 0.36f));
    (void)floor.add_component<framework::component::BoxCollider>(pbpt::math::Vec3{2.0f, 0.25f, 2.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{0.0f, 0.2f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(
        make_source_body(1.0f, RigidBodyType::Dynamic, false, pbpt::math::Mat3::zeros(), 0.0f, 0.25f));
    (void)sphere.add_component<framework::component::SphereCollider>(0.5f);

    sphere_body.set_linear_velocity(pbpt::math::Vec3{2.0f, -1.0f, 0.0f});

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
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
        make_source_body(1.0f, RigidBodyType::Static, false, pbpt::math::Mat3::zeros(), 0.0f, 0.0f));
    (void)floor.add_component<framework::component::BoxCollider>(pbpt::math::Vec3{2.0f, 0.25f, 2.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{0.0f, 0.2f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(
        make_source_body(1.0f, RigidBodyType::Dynamic, false, pbpt::math::Mat3::zeros(), 0.0f, 0.0f));
    (void)sphere.add_component<framework::component::SphereCollider>(0.5f);

    sphere_body.set_linear_velocity(pbpt::math::Vec3{2.0f, -1.0f, 0.0f});

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                     });

    EXPECT_NEAR(sphere_body.linear_velocity().x(), 2.0f, 1e-4f);
    EXPECT_NEAR(sphere_body.linear_velocity().y(), 0.0f, 1e-4f);
}

TEST(PhysicsSceneIntegrationTest, DynamicBodiesOffCenterCollisionProducesFiniteAngularVelocity) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& box = scene.create_game_object("box");
    auto& box_body = box.add_component<framework::component::RigidBody>(make_source_body(
        1.0f,
        RigidBodyType::Dynamic,
        false,
        diagonal_inverse_inertia(0.0f, 0.0f, 1.0f),
        0.0f,
        0.36f
    ));
    (void)box.add_component<framework::component::BoxCollider>(pbpt::math::Vec3{0.2f, 2.0f, 2.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{-1.0f, 0.6f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(make_source_body(
        1.0f,
        RigidBodyType::Dynamic,
        false,
        diagonal_inverse_inertia(0.0f, 0.0f, 1.0f),
        0.0f,
        0.25f
    ));
    (void)sphere.add_component<framework::component::SphereCollider>(0.5f);

    sphere_body.set_linear_velocity(pbpt::math::Vec3{2.0f, -1.0f, 0.0f});

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.25,
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

TEST(PhysicsSceneIntegrationTest, SolverIterationsReducePenetrationAcrossStackedContacts) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);
    physics.world().set_velocity_iterations(8);
    physics.world().set_position_iterations(3);

    auto& floor = scene.create_game_object("floor");
    (void)floor.add_component<framework::component::RigidBody>(
        make_source_body(1.0f, RigidBodyType::Static, false, pbpt::math::Mat3::zeros(), 0.0f, 0.0f));
    (void)floor.add_component<framework::component::PlaneCollider>(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});

    auto& lower = scene.create_game_object("lower");
    lower.node().set_local_position(pbpt::math::Vec3{0.0f, 0.45f, 0.0f});
    auto& lower_body = lower.add_component<framework::component::RigidBody>(
        make_source_body(1.0f, RigidBodyType::Dynamic, true, pbpt::math::Mat3::zeros(), 0.0f, 0.0f));
    (void)lower.add_component<framework::component::SphereCollider>(0.5f);

    auto& upper = scene.create_game_object("upper");
    upper.node().set_local_position(pbpt::math::Vec3{0.0f, 1.30f, 0.0f});
    auto& upper_body = upper.add_component<framework::component::RigidBody>(
        make_source_body(1.0f, RigidBodyType::Dynamic, true, pbpt::math::Mat3::zeros(), 0.0f, 0.0f));
    (void)upper.add_component<framework::component::SphereCollider>(0.5f);

    for (std::uint64_t i = 0; i < 180; ++i) {
        physics.fixed_tick(scene, framework::core::FixedTickContext{
                                             .fixed_delta_seconds = 1.0 / 120.0,
                                         });
    }

    EXPECT_TRUE(std::isfinite(lower_body.position().y()));
    EXPECT_TRUE(std::isfinite(upper_body.position().y()));
    EXPECT_TRUE(std::isfinite(lower_body.linear_velocity().y()));
    EXPECT_TRUE(std::isfinite(upper_body.linear_velocity().y()));

    const auto lower_colliders = colliders_for(physics, lower);
    const auto upper_colliders = colliders_for(physics, upper);
    ASSERT_EQ(lower_colliders.size(), 1u);
    ASSERT_EQ(upper_colliders.size(), 1u);

    const auto lower_world_collider = physics.world().get_world_collider(lower_colliders.front());
    const auto upper_world_collider = physics.world().get_world_collider(upper_colliders.front());
    const auto* lower_sphere = std::get_if<WorldSphere>(&lower_world_collider);
    const auto* upper_sphere = std::get_if<WorldSphere>(&upper_world_collider);
    ASSERT_NE(lower_sphere, nullptr);
    ASSERT_NE(upper_sphere, nullptr);

    const auto floor_contact = ContactPairTrait<WorldSphere, system::physics::rb::WorldPlane>::generate(
        *lower_sphere,
        system::physics::rb::WorldPlane{
            .point = pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
            .normal = pbpt::math::Vec3{0.0f, 1.0f, 0.0f},
        });
    const auto stacked_contact = ContactPairTrait<WorldSphere, WorldSphere>::generate(*lower_sphere, *upper_sphere);

    EXPECT_FALSE(floor_contact.is_valid() && floor_contact.penetration > 0.02f);
    EXPECT_FALSE(stacked_contact.is_valid() && stacked_contact.penetration > 0.02f);
    EXPECT_LT(std::abs(lower_body.linear_velocity().y()), 0.2f);
    EXPECT_LT(std::abs(upper_body.linear_velocity().y()), 0.2f);
}

TEST(PhysicsSceneIntegrationTest, AccumulatedFrictionDoesNotReverseTangentialVelocityAcrossVelocityIterations) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);
    physics.world().set_velocity_iterations(12);
    physics.world().set_position_iterations(3);

    auto& floor = scene.create_game_object("floor");
    floor.node().set_local_position(pbpt::math::Vec3{0.0f, -0.5f, 0.0f});
    (void)floor.add_component<framework::component::RigidBody>(
        make_source_body(1.0f, RigidBodyType::Static, false, pbpt::math::Mat3::zeros(), 0.0f, 1.0f));
    (void)floor.add_component<framework::component::BoxCollider>(pbpt::math::Vec3{2.0f, 0.25f, 2.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{0.0f, 0.2f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(
        make_source_body(1.0f, RigidBodyType::Dynamic, false, pbpt::math::Mat3::zeros(), 0.0f, 1.0f));
    (void)sphere.add_component<framework::component::SphereCollider>(0.5f);

    sphere_body.set_linear_velocity(pbpt::math::Vec3{0.4f, -1.0f, 0.0f});

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                     });

    EXPECT_GE(sphere_body.linear_velocity().x(), -1e-4f);
    EXPECT_LE(sphere_body.linear_velocity().x(), 0.4f);
    EXPECT_NEAR(sphere_body.linear_velocity().y(), 0.0f, 1e-4f);
}

TEST(PhysicsSceneIntegrationTest, DynamicSphereCollidesWithStaticPlaneAndCorrectsNormalVelocity) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& floor = scene.create_game_object("floor");
    (void)floor.add_component<framework::component::RigidBody>(
        make_source_body(1.0f, RigidBodyType::Static, false, pbpt::math::Mat3::zeros(), 0.0f, 0.36f));
    (void)floor.add_component<framework::component::PlaneCollider>(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});

    auto& sphere = scene.create_game_object("sphere");
    sphere.node().set_local_position(pbpt::math::Vec3{0.0f, 0.2f, 0.0f});
    auto& sphere_body = sphere.add_component<framework::component::RigidBody>(
        make_source_body(1.0f, RigidBodyType::Dynamic, false, pbpt::math::Mat3::zeros(), 0.0f, 0.25f));
    (void)sphere.add_component<framework::component::SphereCollider>(0.5f);
    sphere_body.set_linear_velocity(pbpt::math::Vec3{2.0f, -1.0f, 0.0f});

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                     });

    EXPECT_TRUE(std::isfinite(sphere_body.position().x()));
    EXPECT_TRUE(std::isfinite(sphere_body.position().y()));
    EXPECT_TRUE(std::isfinite(sphere_body.position().z()));
    EXPECT_TRUE(std::isfinite(sphere_body.linear_velocity().x()));
    EXPECT_TRUE(std::isfinite(sphere_body.linear_velocity().y()));
    EXPECT_GT(sphere_body.linear_velocity().y(), -0.25f);

    const auto sphere_colliders = colliders_for(physics, sphere);
    ASSERT_EQ(sphere_colliders.size(), 1u);
    const auto sphere_world_collider = physics.world().get_world_collider(sphere_colliders.front());
    const auto* world_sphere = std::get_if<WorldSphere>(&sphere_world_collider);
    ASSERT_NE(world_sphere, nullptr);
    const auto contact = ContactPairTrait<WorldSphere, system::physics::rb::WorldPlane>::generate(
        *world_sphere,
        system::physics::rb::WorldPlane{
            .point = pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
            .normal = pbpt::math::Vec3{0.0f, 1.0f, 0.0f},
        });
    EXPECT_FALSE(contact.is_valid() && contact.penetration > 0.05f);
}

TEST(PhysicsSceneIntegrationTest, DynamicBoxCollidesWithStaticPlaneAndGeneratesAngularVelocity) {
    PhysicsStepper         physics;
    framework::core::Scene scene(1);

    auto& floor = scene.create_game_object("floor");
    (void)floor.add_component<framework::component::RigidBody>(
        make_source_body(1.0f, RigidBodyType::Static, false, pbpt::math::Mat3::zeros(), 0.0f, 0.36f));
    (void)floor.add_component<framework::component::PlaneCollider>(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});

    auto& box = scene.create_game_object("box");
    box.node().set_local_position(pbpt::math::Vec3{0.5f, 0.1f, 0.0f});
    auto& box_body = box.add_component<framework::component::RigidBody>(make_source_body(
        1.0f,
        RigidBodyType::Dynamic,
        false,
        diagonal_inverse_inertia(0.0f, 0.0f, 1.0f),
        0.0f,
        0.25f
    ));
    (void)box.add_component<framework::component::BoxCollider>(pbpt::math::Vec3{0.4f, 0.25f, 0.4f});
    box_body.set_linear_velocity(pbpt::math::Vec3{2.0f, -1.0f, 0.0f});

    physics.fixed_tick(scene, framework::core::FixedTickContext{
                                         .fixed_delta_seconds = 0.1,
                                     });

    EXPECT_TRUE(std::isfinite(box_body.position().x()));
    EXPECT_TRUE(std::isfinite(box_body.position().y()));
    EXPECT_TRUE(std::isfinite(box_body.linear_velocity().x()));
    EXPECT_TRUE(std::isfinite(box_body.linear_velocity().y()));
    EXPECT_TRUE(std::isfinite(box_body.angular_velocity().z()));
    EXPECT_GT(box_body.linear_velocity().y(), -0.25f);
    EXPECT_GT(std::abs(box_body.angular_velocity().z()), 1e-4f);

    const auto box_colliders = colliders_for(physics, box);
    ASSERT_EQ(box_colliders.size(), 1u);
    const auto box_world_collider = physics.world().get_world_collider(box_colliders.front());
    const auto* world_box = std::get_if<WorldBox>(&box_world_collider);
    ASSERT_NE(world_box, nullptr);
    const auto contact = ContactPairTrait<WorldBox, system::physics::rb::WorldPlane>::generate(
        *world_box,
        system::physics::rb::WorldPlane{
            .point = pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
            .normal = pbpt::math::Vec3{0.0f, 1.0f, 0.0f},
        });
    EXPECT_FALSE(contact.is_valid() && contact.penetration > 0.05f);
}

}  // namespace rtr::framework::integration::physics::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
