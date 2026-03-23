#include <stdexcept>

#include "gtest/gtest.h"

#include "rtr/framework/component/physics/rigid_body/box_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/plane_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/component/physics/rigid_body/sphere_collider.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/physics/rigid_body_scene_sync.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_system.hpp"

namespace rtr::framework::component::test {

namespace {

system::physics::rb::RigidBodyID body_id_for(const system::physics::rb::RigidBodySystem& physics_world,
                                             const core::GameObject& go) {
    const auto body_id = physics_world.try_get_rigid_body_id_for_owner(go.id());
    EXPECT_TRUE(body_id.has_value());
    return *body_id;
}

std::vector<system::physics::rb::ColliderID> colliders_for(const system::physics::rb::RigidBodySystem& physics_world,
                                                           const core::GameObject& go) {
    return physics_world.colliders_for_body(body_id_for(physics_world, go));
}

}  // namespace

TEST(FrameworkColliderComponentTest, SphereColliderThrowsWhenRigidBodyIsMissing) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("sphere");
    EXPECT_THROW((void)go.add_component<SphereCollider>(0.5f), std::runtime_error);
}

TEST(FrameworkColliderComponentTest, BoxColliderThrowsWhenRigidBodyIsMissing) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("box");
    EXPECT_THROW((void)go.add_component<BoxCollider>(pbpt::math::Vec3{1.0f, 1.0f, 1.0f}), std::runtime_error);
}

TEST(FrameworkColliderComponentTest, PlaneColliderThrowsWhenRigidBodyIsMissing) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("plane");
    EXPECT_THROW((void)go.add_component<PlaneCollider>(), std::runtime_error);
}

TEST(FrameworkColliderComponentTest, StaticRigidBodyCanOwnBoxColliderAndDestroyRemovesCollider) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("wall");
    auto& rigid_body = go.add_component<RigidBody>();
    rigid_body.set_type(system::physics::rb::RigidBodyType::Static);
    (void)go.add_component<BoxCollider>(pbpt::math::Vec3{1.0f, 1.0f, 1.0f});
    integration::physics::sync_scene_to_rigid_body(scene, physics_world);

    const auto collider_ids = colliders_for(physics_world, go);
    ASSERT_EQ(collider_ids.size(), 1u);
    EXPECT_TRUE(physics_world.has_collider(collider_ids.front()));

    EXPECT_TRUE(scene.destroy_game_object(go.id()));
    integration::physics::sync_scene_to_rigid_body(scene, physics_world);
    EXPECT_FALSE(physics_world.has_rigid_body_for_owner(go.id()));
    EXPECT_FALSE(physics_world.has_collider(collider_ids.front()));
}

TEST(FrameworkColliderComponentTest, SphereColliderSettersDeferPhysicsSyncUntilScenePrePass) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("sphere");
    auto& rigid_body = go.add_component<RigidBody>();
    auto& sphere = go.add_component<SphereCollider>(0.5f);
    integration::physics::sync_scene_to_rigid_body(scene, physics_world);

    sphere.set_local_position(pbpt::math::Vec3{1.0f, 2.0f, 3.0f});
    sphere.set_local_rotation(
        pbpt::math::angle_axis(pbpt::math::radians(30.0f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}));
    sphere.set_local_scale(pbpt::math::Vec3{2.0f, 3.0f, 4.0f});

    const auto collider_ids = colliders_for(physics_world, go);
    ASSERT_EQ(collider_ids.size(), 1u);
    const auto& stale_collider = physics_world.get_collider(collider_ids.front());

    EXPECT_NEAR(stale_collider.local_transform.position.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(stale_collider.local_transform.position.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(stale_collider.local_transform.position.z(), 0.0f, 1e-5f);
    EXPECT_NEAR(stale_collider.local_transform.scale.x(), 1.0f, 1e-5f);
    EXPECT_NEAR(stale_collider.local_transform.scale.y(), 1.0f, 1e-5f);
    EXPECT_NEAR(stale_collider.local_transform.scale.z(), 1.0f, 1e-5f);

    integration::physics::sync_scene_to_rigid_body(scene, physics_world);

    const auto& collider = physics_world.get_collider(collider_ids.front());
    EXPECT_NEAR(collider.local_transform.position.x(), 1.0f, 1e-5f);
    EXPECT_NEAR(collider.local_transform.position.y(), 2.0f, 1e-5f);
    EXPECT_NEAR(collider.local_transform.position.z(), 3.0f, 1e-5f);
    EXPECT_NEAR(collider.local_transform.scale.x(), 2.0f, 1e-5f);
    EXPECT_NEAR(collider.local_transform.scale.y(), 3.0f, 1e-5f);
    EXPECT_NEAR(collider.local_transform.scale.z(), 4.0f, 1e-5f);
    EXPECT_NEAR(collider.local_transform.rotation.w(), sphere.local_rotation().w(), 1e-5f);
    EXPECT_NEAR(collider.local_transform.rotation.x(), sphere.local_rotation().x(), 1e-5f);
    EXPECT_NEAR(collider.local_transform.rotation.y(), sphere.local_rotation().y(), 1e-5f);
    EXPECT_NEAR(collider.local_transform.rotation.z(), sphere.local_rotation().z(), 1e-5f);
}

TEST(FrameworkColliderComponentTest, BoxColliderSettersDeferPhysicsSyncUntilScenePrePass) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("box");
    auto& rigid_body = go.add_component<RigidBody>();
    auto& box = go.add_component<BoxCollider>(pbpt::math::Vec3{0.5f, 0.75f, 1.0f});
    integration::physics::sync_scene_to_rigid_body(scene, physics_world);

    box.set_local_position(pbpt::math::Vec3{-1.0f, 0.5f, 2.0f});
    box.set_local_rotation(
        pbpt::math::angle_axis(pbpt::math::radians(45.0f), pbpt::math::Vec3{1.0f, 0.0f, 0.0f}));
    box.set_local_scale(pbpt::math::Vec3{1.5f, 2.0f, 2.5f});

    const auto collider_ids = colliders_for(physics_world, go);
    ASSERT_EQ(collider_ids.size(), 1u);
    const auto& stale_collider = physics_world.get_collider(collider_ids.front());

    EXPECT_NEAR(stale_collider.local_transform.position.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(stale_collider.local_transform.position.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(stale_collider.local_transform.position.z(), 0.0f, 1e-5f);
    EXPECT_NEAR(stale_collider.local_transform.scale.x(), 1.0f, 1e-5f);
    EXPECT_NEAR(stale_collider.local_transform.scale.y(), 1.0f, 1e-5f);
    EXPECT_NEAR(stale_collider.local_transform.scale.z(), 1.0f, 1e-5f);

    integration::physics::sync_scene_to_rigid_body(scene, physics_world);

    const auto& collider = physics_world.get_collider(collider_ids.front());
    EXPECT_NEAR(collider.local_transform.position.x(), -1.0f, 1e-5f);
    EXPECT_NEAR(collider.local_transform.position.y(), 0.5f, 1e-5f);
    EXPECT_NEAR(collider.local_transform.position.z(), 2.0f, 1e-5f);
    EXPECT_NEAR(collider.local_transform.scale.x(), 1.5f, 1e-5f);
    EXPECT_NEAR(collider.local_transform.scale.y(), 2.0f, 1e-5f);
    EXPECT_NEAR(collider.local_transform.scale.z(), 2.5f, 1e-5f);
    EXPECT_NEAR(collider.local_transform.rotation.w(), box.local_rotation().w(), 1e-5f);
    EXPECT_NEAR(collider.local_transform.rotation.x(), box.local_rotation().x(), 1e-5f);
    EXPECT_NEAR(collider.local_transform.rotation.y(), box.local_rotation().y(), 1e-5f);
    EXPECT_NEAR(collider.local_transform.rotation.z(), box.local_rotation().z(), 1e-5f);
}

TEST(FrameworkColliderComponentTest, PlaneColliderSyncsWorldNormalWithNodeRotation) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("plane");
    go.node().set_local_rotation(
        pbpt::math::angle_axis(pbpt::math::radians(90.0f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}));
    auto& rigid_body = go.add_component<RigidBody>();
    rigid_body.set_type(system::physics::rb::RigidBodyType::Static);
    (void)go.add_component<PlaneCollider>(pbpt::math::Vec3{0.0f, 0.0f, 1.0f});

    integration::physics::sync_scene_to_rigid_body(scene, physics_world);

    const auto collider_ids = colliders_for(physics_world, go);
    ASSERT_EQ(collider_ids.size(), 1u);
    const auto world_collider = physics_world.get_world_collider(collider_ids.front());
    const auto* world_plane = std::get_if<system::physics::rb::WorldPlane>(&world_collider);
    ASSERT_NE(world_plane, nullptr);
    EXPECT_NEAR(world_plane->normal.x(), 1.0f, 1e-5f);
    EXPECT_NEAR(world_plane->normal.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(world_plane->normal.z(), 0.0f, 1e-5f);
}

TEST(FrameworkColliderComponentTest, SphereColliderRadiusChangeAppliesOnNextScenePrePass) {
    system::physics::rb::RigidBodySystem physics_world;
    core::Scene scene(1);

    auto& go = scene.create_game_object("sphere");
    auto& rigid_body = go.add_component<RigidBody>();
    auto& sphere = go.add_component<SphereCollider>(0.5f);
    integration::physics::sync_scene_to_rigid_body(scene, physics_world);

    const auto collider_ids = colliders_for(physics_world, go);
    ASSERT_EQ(collider_ids.size(), 1u);

    sphere.set_radius(1.25f);

    const auto* stale_shape = std::get_if<system::physics::rb::SphereShape>(
        &physics_world.get_collider(collider_ids.front()).shape
    );
    ASSERT_NE(stale_shape, nullptr);
    EXPECT_NEAR(stale_shape->radius, 0.5f, 1e-5f);

    integration::physics::sync_scene_to_rigid_body(scene, physics_world);

    const auto* synced_shape = std::get_if<system::physics::rb::SphereShape>(
        &physics_world.get_collider(collider_ids.front()).shape
    );
    ASSERT_NE(synced_shape, nullptr);
    EXPECT_NEAR(synced_shape->radius, 1.25f, 1e-5f);
}

}  // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
