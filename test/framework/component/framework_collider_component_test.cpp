#include <stdexcept>

#include "gtest/gtest.h"

#include "rtr/framework/component/physics/box_collider.hpp"
#include "rtr/framework/component/physics/rigid_body.hpp"
#include "rtr/framework/component/physics/sphere_collider.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/system/physics/physics_world.hpp"

namespace rtr::framework::component::test {

TEST(FrameworkColliderComponentTest, SphereColliderThrowsWhenRigidBodyIsMissing) {
    system::physics::PhysicsWorld physics_world;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("sphere");
    EXPECT_THROW((void)go.add_component<SphereCollider>(physics_world, 0.5f), std::runtime_error);
}

TEST(FrameworkColliderComponentTest, BoxColliderThrowsWhenRigidBodyIsMissing) {
    system::physics::PhysicsWorld physics_world;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("box");
    EXPECT_THROW((void)go.add_component<BoxCollider>(physics_world, pbpt::math::Vec3{1.0f, 1.0f, 1.0f}),
                 std::runtime_error);
}

TEST(FrameworkColliderComponentTest, StaticRigidBodyCanOwnBoxColliderAndDestroyRemovesCollider) {
    system::physics::PhysicsWorld physics_world;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("wall");
    auto& rigid_body = go.add_component<RigidBody>(physics_world);
    rigid_body.set_type(system::physics::RigidBodyType::Static);
    (void)go.add_component<BoxCollider>(physics_world, pbpt::math::Vec3{1.0f, 1.0f, 1.0f});

    const auto collider_ids = physics_world.colliders_for_body(rigid_body.rigid_body_id());
    ASSERT_EQ(collider_ids.size(), 1u);
    EXPECT_TRUE(physics_world.has_collider(collider_ids.front()));

    EXPECT_TRUE(scene.destroy_game_object(go.id()));
    EXPECT_FALSE(physics_world.has_rigid_body(rigid_body.rigid_body_id()));
    EXPECT_FALSE(physics_world.has_collider(collider_ids.front()));
}

TEST(FrameworkColliderComponentTest, SphereColliderSyncsFullLocalTransformToPhysicsCollider) {
    system::physics::PhysicsWorld physics_world;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("sphere");
    auto& rigid_body = go.add_component<RigidBody>(physics_world);
    auto& sphere = go.add_component<SphereCollider>(physics_world, 0.5f);

    sphere.set_local_position(pbpt::math::Vec3{1.0f, 2.0f, 3.0f});
    sphere.set_local_rotation(
        pbpt::math::angle_axis(pbpt::math::radians(30.0f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}));
    sphere.set_local_scale(pbpt::math::Vec3{2.0f, 3.0f, 4.0f});

    const auto collider_ids = physics_world.colliders_for_body(rigid_body.rigid_body_id());
    ASSERT_EQ(collider_ids.size(), 1u);
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

TEST(FrameworkColliderComponentTest, BoxColliderSyncsFullLocalTransformToPhysicsCollider) {
    system::physics::PhysicsWorld physics_world;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("box");
    auto& rigid_body = go.add_component<RigidBody>(physics_world);
    auto& box = go.add_component<BoxCollider>(physics_world, pbpt::math::Vec3{0.5f, 0.75f, 1.0f});

    box.set_local_position(pbpt::math::Vec3{-1.0f, 0.5f, 2.0f});
    box.set_local_rotation(
        pbpt::math::angle_axis(pbpt::math::radians(45.0f), pbpt::math::Vec3{1.0f, 0.0f, 0.0f}));
    box.set_local_scale(pbpt::math::Vec3{1.5f, 2.0f, 2.5f});

    const auto collider_ids = physics_world.colliders_for_body(rigid_body.rigid_body_id());
    ASSERT_EQ(collider_ids.size(), 1u);
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

}  // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
