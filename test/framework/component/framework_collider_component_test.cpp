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

}  // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
