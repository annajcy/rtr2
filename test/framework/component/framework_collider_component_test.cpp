#include <stdexcept>

#include "gtest/gtest.h"

#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/physics/rigid_body/box_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/mesh_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/plane_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/component/physics/rigid_body/sphere_collider.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_world.hpp"

namespace rtr::framework::component::test {

namespace {

utils::ObjMeshData make_triangle_mesh() {
    utils::ObjMeshData mesh{};
    mesh.vertices = {
        {.position = pbpt::math::Vec3{0.2f, -0.2f, 0.0f}},
        {.position = pbpt::math::Vec3{0.4f, -0.2f, 0.0f}},
        {.position = pbpt::math::Vec3{0.6f, 0.2f, 0.0f}},
    };
    mesh.indices = {0, 1, 2};
    return mesh;
}

}  // namespace

TEST(FrameworkColliderComponentTest, SphereColliderThrowsWhenRigidBodyIsMissing) {
    system::physics::RigidBodyWorld physics_world;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("sphere");
    EXPECT_THROW((void)go.add_component<SphereCollider>(physics_world, 0.5f), std::runtime_error);
}

TEST(FrameworkColliderComponentTest, BoxColliderThrowsWhenRigidBodyIsMissing) {
    system::physics::RigidBodyWorld physics_world;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("box");
    EXPECT_THROW((void)go.add_component<BoxCollider>(physics_world, pbpt::math::Vec3{1.0f, 1.0f, 1.0f}),
                 std::runtime_error);
}

TEST(FrameworkColliderComponentTest, PlaneColliderThrowsWhenRigidBodyIsMissing) {
    system::physics::RigidBodyWorld physics_world;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("plane");
    EXPECT_THROW((void)go.add_component<PlaneCollider>(physics_world), std::runtime_error);
}

TEST(FrameworkColliderComponentTest, MeshColliderThrowsWhenMeshRendererIsMissing) {
    system::physics::RigidBodyWorld physics_world;
    resource::ResourceManager     resources;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("mesh");
    (void)go.add_component<RigidBody>(physics_world);
    EXPECT_THROW((void)go.add_component<MeshCollider>(physics_world), std::runtime_error);
}

TEST(FrameworkColliderComponentTest, StaticRigidBodyCanOwnBoxColliderAndDestroyRemovesCollider) {
    system::physics::RigidBodyWorld physics_world;
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
    system::physics::RigidBodyWorld physics_world;
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
    system::physics::RigidBodyWorld physics_world;
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

TEST(FrameworkColliderComponentTest, PlaneColliderSyncsWorldNormalWithNodeRotation) {
    system::physics::RigidBodyWorld physics_world;
    core::Scene                   scene(1);

    auto& go = scene.create_game_object("plane");
    go.node().set_local_rotation(
        pbpt::math::angle_axis(pbpt::math::radians(90.0f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}));
    auto& rigid_body = go.add_component<RigidBody>(physics_world);
    rigid_body.set_type(system::physics::RigidBodyType::Static);
    (void)go.add_component<PlaneCollider>(physics_world, pbpt::math::Vec3{0.0f, 0.0f, 1.0f});

    const auto collider_ids = physics_world.colliders_for_body(rigid_body.rigid_body_id());
    ASSERT_EQ(collider_ids.size(), 1u);
    const auto world_collider = physics_world.get_world_collider(collider_ids.front());
    const auto* world_plane = std::get_if<system::physics::WorldPlane>(&world_collider);
    ASSERT_NE(world_plane, nullptr);
    EXPECT_NEAR(world_plane->normal.x(), 1.0f, 1e-5f);
    EXPECT_NEAR(world_plane->normal.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(world_plane->normal.z(), 0.0f, 1e-5f);
}

TEST(FrameworkColliderComponentTest, MeshColliderReadsLocalVerticesFromMeshRenderer) {
    system::physics::RigidBodyWorld physics_world;
    resource::ResourceManager     resources;
    core::Scene                   scene(1);
    const auto mesh_handle = resources.create<resource::MeshResourceKind>(make_triangle_mesh());

    auto& go = scene.create_game_object("mesh");
    auto& rigid_body = go.add_component<RigidBody>(physics_world);
    auto& mesh_renderer = go.add_component<MeshRenderer>(resources, mesh_handle);
    auto& mesh = go.add_component<MeshCollider>(physics_world);

    const auto local_vertices = mesh_renderer.local_vertices();
    ASSERT_EQ(local_vertices.size(), 3u);
    const auto collider_ids = physics_world.colliders_for_body(rigid_body.rigid_body_id());
    ASSERT_EQ(collider_ids.size(), 1u);
    const auto& collider = physics_world.get_collider(collider_ids.front());
    const auto* mesh_shape = std::get_if<system::physics::MeshShape>(&collider.shape);
    ASSERT_NE(mesh_shape, nullptr);
    ASSERT_EQ(mesh_shape->local_vertices.size(), 3u);
    EXPECT_NEAR(mesh_shape->local_vertices[0].x(), 0.2f, 1e-5f);
}

}  // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
