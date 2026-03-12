#include "gtest/gtest.h"

#include "rtr/editor/core/scene_picking.hpp"
#include "rtr/framework/component/physics/rigid_body/box_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/component/physics/rigid_body/sphere_collider.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_world.hpp"

namespace rtr::editor::test {

TEST(EditorScenePickingTest, SpherePickUsesColliderLocalScale) {
    resource::ResourceManager      resources;
    system::physics::RigidBodyWorld  physics_world;
    framework::core::Scene         scene(1);

    auto& go = scene.create_game_object("sphere");
    (void)go.add_component<framework::component::RigidBody>(physics_world);
    auto& sphere = go.add_component<framework::component::SphereCollider>(physics_world, 0.5f);
    sphere.set_local_scale(pbpt::math::Vec3{4.0f, 1.0f, 1.0f});

    scene.scene_graph().update_world_transforms();

    const auto result = pick_scene_game_object(
        resources,
        scene,
        ScenePickRay{
            .origin = pbpt::math::Pt3{1.5f, 0.0f, -5.0f},
            .direction = pbpt::math::Vec3{0.0f, 0.0f, 1.0f},
        }
    );

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->game_object_id, go.id());
    EXPECT_EQ(result->source, ScenePickHitSource::SphereCollider);
}

TEST(EditorScenePickingTest, BoxPickUsesColliderLocalScale) {
    resource::ResourceManager      resources;
    system::physics::RigidBodyWorld  physics_world;
    framework::core::Scene         scene(1);

    auto& go = scene.create_game_object("box");
    (void)go.add_component<framework::component::RigidBody>(physics_world);
    auto& box = go.add_component<framework::component::BoxCollider>(
        physics_world, pbpt::math::Vec3{0.5f, 0.5f, 0.5f});
    box.set_local_scale(pbpt::math::Vec3{3.0f, 1.0f, 1.0f});

    scene.scene_graph().update_world_transforms();

    const auto result = pick_scene_game_object(
        resources,
        scene,
        ScenePickRay{
            .origin = pbpt::math::Pt3{1.2f, 0.0f, -5.0f},
            .direction = pbpt::math::Vec3{0.0f, 0.0f, 1.0f},
        }
    );

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->game_object_id, go.id());
    EXPECT_EQ(result->source, ScenePickHitSource::BoxCollider);
}

}  // namespace rtr::editor::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
