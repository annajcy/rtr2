#include <stdexcept>

#include "gtest/gtest.h"

#include "framework/framework.hpp"

namespace rtr::framework::core::test {

class DummyComponentA final : public component::Component {};
class DummyComponentB final : public component::Component {};

TEST(FrameworkWorldSceneTest, GameObjectEnforcesUniqueComponentType) {
    Scene scene(1, "scene");
    auto& go = scene.create_game_object("player");

    auto& comp_a = go.add_component<DummyComponentA>();
    (void)comp_a;
    EXPECT_TRUE(go.has_component<DummyComponentA>());
    EXPECT_FALSE(go.has_component<DummyComponentB>());
    EXPECT_EQ(go.component_count(), 1u);

    EXPECT_THROW((void)go.add_component<DummyComponentA>(), std::runtime_error);
    EXPECT_EQ(go.component_count(), 1u);

    auto& comp_b = go.add_component<DummyComponentB>();
    (void)comp_b;
    EXPECT_TRUE(go.has_component<DummyComponentB>());
    EXPECT_EQ(go.component_count(), 2u);
}

TEST(FrameworkWorldSceneTest, SceneGameObjectHandleIsInvalidAfterDestroy) {
    Scene scene(1, "scene");
    auto& go_a = scene.create_game_object("a");
    auto& go_b = scene.create_game_object("b");
    const GameObjectId id_a = go_a.id();
    const GameObjectId id_b = go_b.id();

    EXPECT_TRUE(scene.has_game_object(id_a));
    EXPECT_TRUE(scene.has_game_object(id_b));
    EXPECT_EQ(scene.game_object_count(), 2u);

    EXPECT_TRUE(scene.destroy_game_object(id_a));
    EXPECT_FALSE(scene.has_game_object(id_a));
    EXPECT_EQ(scene.find_game_object(id_a), nullptr);
    EXPECT_EQ(scene.game_object_count(), 1u);

    EXPECT_FALSE(scene.destroy_game_object(id_a));
    EXPECT_TRUE(scene.has_game_object(id_b));

    auto& go_c = scene.create_game_object("c");
    EXPECT_NE(go_c.id(), id_a);
    EXPECT_TRUE(scene.has_game_object(go_c.id()));
}

TEST(FrameworkWorldSceneTest, WorldSceneHandleIsInvalidAfterDestroy) {
    World world;
    auto& scene_a = world.create_scene("a");
    auto& scene_b = world.create_scene("b");
    const SceneId id_a = scene_a.id();
    const SceneId id_b = scene_b.id();

    EXPECT_EQ(world.scene_count(), 2u);
    EXPECT_EQ(world.active_scene_id(), id_a);
    EXPECT_TRUE(world.set_active_scene(id_b));
    EXPECT_EQ(world.active_scene_id(), id_b);

    EXPECT_TRUE(world.destroy_scene(id_b));
    EXPECT_FALSE(world.has_scene(id_b));
    EXPECT_EQ(world.find_scene(id_b), nullptr);
    EXPECT_EQ(world.scene_count(), 1u);

    // Active scene falls back to the first remaining scene.
    ASSERT_NE(world.active_scene(), nullptr);
    EXPECT_EQ(world.active_scene()->id(), id_a);

    EXPECT_FALSE(world.destroy_scene(id_b));
    EXPECT_TRUE(world.destroy_scene(id_a));
    EXPECT_EQ(world.scene_count(), 0u);
    EXPECT_EQ(world.active_scene_id(), kInvalidSceneId);
    EXPECT_EQ(world.active_scene(), nullptr);
}

} // namespace rtr::framework::core::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
