#include <stdexcept>

#include "gtest/gtest.h"

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/scene_graph.hpp"
#include "rtr/framework/core/world.hpp"
#include "rtr/resource/resource_manager.hpp"

namespace rtr::framework::core::test {

class DummyComponentA final : public component::Component {
public:
    explicit DummyComponentA(core::GameObject& owner)
        : Component(owner) {}
};

class DummyComponentB final : public component::Component {
public:
    explicit DummyComponentB(core::GameObject& owner)
        : Component(owner) {}
};

TEST(FrameworkWorldSceneTest, GameObjectEnforcesUniqueComponentType) {
    Scene scene(1);
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

TEST(FrameworkWorldSceneTest, GameObjectComponentOrThrowProvidesStrongDependencyAccess) {
    Scene scene(1);
    auto& go = scene.create_game_object("player");

    auto& comp_a = go.add_component<DummyComponentA>();
    auto& required = go.component_or_throw<DummyComponentA>();
    EXPECT_EQ(&required, &comp_a);
    EXPECT_TRUE(go.has_component<DummyComponentA>());
    EXPECT_NE(go.get_component<DummyComponentA>(), nullptr);

    EXPECT_THROW((void)go.component_or_throw<DummyComponentB>(), std::runtime_error);
    EXPECT_FALSE(go.has_component<DummyComponentB>());
    EXPECT_EQ(go.get_component<DummyComponentB>(), nullptr);

    const Scene& const_scene = scene;
    const auto* const_go = const_scene.find_game_object(go.id());
    ASSERT_NE(const_go, nullptr);
    const auto& const_required = const_go->component_or_throw<DummyComponentA>();
    EXPECT_EQ(&const_required, &comp_a);
    EXPECT_THROW((void)const_go->component_or_throw<DummyComponentB>(), std::runtime_error);
}

TEST(FrameworkWorldSceneTest, SceneGameObjectHandleIsInvalidAfterDestroy) {
    Scene scene(1);
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

TEST(FrameworkWorldSceneTest, SceneCreateGameObjectGeneratesUniqueNamesAndSupportsNameLookup) {
    Scene scene(1);
    auto& a = scene.create_game_object("Camera");
    auto& b = scene.create_game_object("Camera");
    auto& c = scene.create_game_object("Camera");
    auto& d = scene.create_game_object("");

    EXPECT_EQ(scene.game_object_name(a.id()).value_or(""), "Camera");
    EXPECT_EQ(scene.game_object_name(b.id()).value_or(""), "Camera_1");
    EXPECT_EQ(scene.game_object_name(c.id()).value_or(""), "Camera_2");
    EXPECT_EQ(scene.game_object_name(d.id()).value_or(""), "GameObject");

    ASSERT_NE(scene.find_game_object("Camera"), nullptr);
    EXPECT_EQ(scene.find_game_object("Camera")->id(), a.id());
    ASSERT_NE(scene.find_game_object("Camera_1"), nullptr);
    EXPECT_EQ(scene.find_game_object("Camera_1")->id(), b.id());
    ASSERT_NE(scene.find_game_object("Camera_2"), nullptr);
    EXPECT_EQ(scene.find_game_object("Camera_2")->id(), c.id());
    ASSERT_NE(scene.find_game_object("GameObject"), nullptr);
    EXPECT_EQ(scene.find_game_object("GameObject")->id(), d.id());

    const Scene& const_scene = scene;
    ASSERT_NE(const_scene.find_game_object("Camera_1"), nullptr);
    EXPECT_EQ(const_scene.find_game_object("Camera_1")->id(), b.id());
    EXPECT_TRUE(scene.has_game_object("Camera_2"));
    EXPECT_FALSE(scene.has_game_object("Camera_99"));
}

TEST(FrameworkWorldSceneTest, SceneRenameGameObjectMaintainsUniqueNames) {
    Scene scene(1);
    auto& light = scene.create_game_object("Light");
    auto& fill  = scene.create_game_object("Light");

    EXPECT_EQ(scene.game_object_name(light.id()).value_or(""), "Light");
    EXPECT_EQ(scene.game_object_name(fill.id()).value_or(""), "Light_1");

    EXPECT_TRUE(scene.rename_game_object(light.id(), "Light_1"));
    EXPECT_EQ(scene.game_object_name(light.id()).value_or(""), "Light_1_1");
    EXPECT_TRUE(scene.has_game_object("Light_1_1"));
    EXPECT_EQ(scene.find_game_object("Light_1_1")->id(), light.id());

    EXPECT_TRUE(scene.rename_game_object(fill.id(), "Light_1"));
    EXPECT_EQ(scene.game_object_name(fill.id()).value_or(""), "Light_1");

    EXPECT_TRUE(scene.rename_game_object(fill.id(), ""));
    EXPECT_EQ(scene.game_object_name(fill.id()).value_or(""), "GameObject");
    EXPECT_TRUE(scene.has_game_object("GameObject"));

    EXPECT_FALSE(scene.rename_game_object(999999, "ghost"));
}

TEST(FrameworkWorldSceneTest, SceneDestroyGameObjectClearsIdAndNameIndexesForSubtree) {
    Scene scene(1);
    auto& root       = scene.create_game_object("root");
    auto& child      = scene.create_game_object("child");
    auto& grandchild = scene.create_game_object("grandchild");
    auto& survivor   = scene.create_game_object("survivor");

    ASSERT_TRUE(scene.scene_graph().set_parent(child.id(), root.id()));
    ASSERT_TRUE(scene.scene_graph().set_parent(grandchild.id(), child.id()));

    EXPECT_TRUE(scene.destroy_game_object(root.id()));
    EXPECT_FALSE(scene.has_game_object(root.id()));
    EXPECT_FALSE(scene.has_game_object(child.id()));
    EXPECT_FALSE(scene.has_game_object(grandchild.id()));
    EXPECT_FALSE(scene.game_object_name(root.id()).has_value());
    EXPECT_FALSE(scene.game_object_name(child.id()).has_value());
    EXPECT_FALSE(scene.game_object_name(grandchild.id()).has_value());
    EXPECT_FALSE(scene.has_game_object("root"));
    EXPECT_FALSE(scene.has_game_object("child"));
    EXPECT_FALSE(scene.has_game_object("grandchild"));

    EXPECT_TRUE(scene.has_game_object(survivor.id()));
    EXPECT_EQ(scene.game_object_name(survivor.id()).value_or(""), "survivor");
    EXPECT_TRUE(scene.has_game_object("survivor"));
    ASSERT_NE(scene.find_game_object("survivor"), nullptr);
    EXPECT_EQ(scene.find_game_object("survivor")->id(), survivor.id());
}

TEST(FrameworkWorldSceneTest, WorldSceneNamesAreUniqueAndSearchable) {
    resource::ResourceManager resources;
    World world(resources);
    auto& scene_a = world.create_scene("Scene");
    auto& scene_b = world.create_scene("Scene");
    auto& scene_c = world.create_scene("");

    const SceneId id_a = scene_a.id();
    const SceneId id_b = scene_b.id();
    const SceneId id_c = scene_c.id();

    EXPECT_EQ(world.scene_name(id_a).value_or(""), "Scene");
    EXPECT_EQ(world.scene_name(id_b).value_or(""), "Scene_1");
    EXPECT_EQ(world.scene_name(id_c).value_or(""), "Scene_2");
    EXPECT_TRUE(world.has_scene("Scene_1"));
    ASSERT_NE(world.find_scene("Scene_1"), nullptr);
    EXPECT_EQ(world.find_scene("Scene_1")->id(), id_b);

    EXPECT_TRUE(world.rename_scene(id_a, "Scene_1"));
    EXPECT_EQ(world.scene_name(id_a).value_or(""), "Scene_1_1");
    EXPECT_TRUE(world.has_scene("Scene_1_1"));

    EXPECT_TRUE(world.rename_scene(id_b, "renamed"));
    EXPECT_EQ(world.scene_name(id_b).value_or(""), "renamed");
    EXPECT_TRUE(world.set_active_scene("renamed"));
    EXPECT_EQ(world.active_scene_id(), id_b);
}

TEST(FrameworkWorldSceneTest, WorldDestroySceneRejectsActiveSceneAndMaintainsIndexes) {
    resource::ResourceManager resources;
    World world(resources);
    auto& scene_a = world.create_scene("a");
    auto& scene_b = world.create_scene("b");
    const SceneId id_a = scene_a.id();
    const SceneId id_b = scene_b.id();

    EXPECT_EQ(world.scene_count(), 2u);
    EXPECT_EQ(world.active_scene_id(), id_a);
    EXPECT_TRUE(world.set_active_scene(id_b));
    EXPECT_EQ(world.active_scene_id(), id_b);

    EXPECT_FALSE(world.destroy_scene(id_b));
    EXPECT_TRUE(world.has_scene(id_b));
    EXPECT_EQ(world.scene_count(), 2u);

    EXPECT_TRUE(world.destroy_scene(id_a));
    EXPECT_FALSE(world.has_scene(id_a));
    EXPECT_FALSE(world.has_scene("a"));
    EXPECT_FALSE(world.scene_name(id_a).has_value());
    EXPECT_EQ(world.scene_count(), 1u);

    ASSERT_NE(world.active_scene(), nullptr);
    EXPECT_EQ(world.active_scene()->id(), id_b);

    EXPECT_FALSE(world.destroy_scene(id_b));
    EXPECT_TRUE(world.has_scene(id_b));

    EXPECT_TRUE(world.set_active_scene("b"));
    EXPECT_EQ(world.active_scene_id(), id_b);

    EXPECT_FALSE(world.destroy_scene(999999));
}

TEST(FrameworkWorldSceneTest, WorldSetActiveSceneByNameMatchesIdPath) {
    resource::ResourceManager resources;
    World world(resources);
    auto& scene_a = world.create_scene("main");
    auto& scene_b = world.create_scene("secondary");

    EXPECT_TRUE(world.set_active_scene(scene_b.id()));
    EXPECT_EQ(world.active_scene_id(), scene_b.id());
    EXPECT_TRUE(world.set_active_scene("main"));
    EXPECT_EQ(world.active_scene_id(), scene_a.id());
    EXPECT_FALSE(world.set_active_scene("missing"));

    EXPECT_NE(world.find_scene(scene_b.id()), nullptr);
}

} // namespace rtr::framework::core::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
