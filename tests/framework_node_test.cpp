#include <glm/glm.hpp>

#include "gtest/gtest.h"

#include "framework/framework.hpp"

namespace rtr::framework::core::test {

static glm::vec3 translation_of(const glm::mat4& m) {
    return {m[3][0], m[3][1], m[3][2]};
}

TEST(FrameworkNodeTest, ParentChildWorldTransformPropagation) {
    Scene scene(1, "scene");
    auto& parent = scene.create_game_object("parent");
    auto& child = scene.create_game_object("child");

    auto& parent_node = parent.add_component<component::NodeComponent>();
    auto& child_node = child.add_component<component::NodeComponent>();

    parent_node.set_local_position({1.0f, 2.0f, 3.0f});
    child_node.set_local_position({0.0f, 1.0f, 0.0f});

    ASSERT_TRUE(scene.set_parent(child.id(), parent.id()));
    scene.update_world_transforms();

    EXPECT_EQ(translation_of(parent_node.world_matrix()), glm::vec3(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(translation_of(child_node.world_matrix()), glm::vec3(1.0f, 3.0f, 3.0f));
}

TEST(FrameworkNodeTest, DirtyPropagatesFromParentToChild) {
    Scene scene(1, "scene");
    auto& parent = scene.create_game_object("parent");
    auto& child = scene.create_game_object("child");

    auto& parent_node = parent.add_component<component::NodeComponent>();
    auto& child_node = child.add_component<component::NodeComponent>();

    parent_node.set_local_position({1.0f, 0.0f, 0.0f});
    child_node.set_local_position({2.0f, 0.0f, 0.0f});
    ASSERT_TRUE(scene.set_parent(child.id(), parent.id()));
    scene.update_world_transforms();
    EXPECT_EQ(translation_of(child_node.world_matrix()), glm::vec3(3.0f, 0.0f, 0.0f));

    parent_node.set_local_position({10.0f, 0.0f, 0.0f});
    scene.update_world_transforms();
    EXPECT_EQ(translation_of(child_node.world_matrix()), glm::vec3(12.0f, 0.0f, 0.0f));
}

TEST(FrameworkNodeTest, DisabledGameObjectSkipsTransformUpdateUntilReenabled) {
    Scene scene(1, "scene");
    auto& parent = scene.create_game_object("parent");
    auto& child = scene.create_game_object("child");

    auto& parent_node = parent.add_component<component::NodeComponent>();
    auto& child_node = child.add_component<component::NodeComponent>();

    parent_node.set_local_position({2.0f, 0.0f, 0.0f});
    child_node.set_local_position({1.0f, 0.0f, 0.0f});
    ASSERT_TRUE(scene.set_parent(child.id(), parent.id()));

    child.set_enabled(false);
    scene.update_world_transforms();
    EXPECT_EQ(translation_of(child_node.world_matrix()), glm::vec3(0.0f, 0.0f, 0.0f));

    child.set_enabled(true);
    scene.update_world_transforms();
    EXPECT_EQ(translation_of(child_node.world_matrix()), glm::vec3(3.0f, 0.0f, 0.0f));
}

TEST(FrameworkNodeTest, RejectsCycleInParenting) {
    Scene scene(1, "scene");
    auto& a = scene.create_game_object("a");
    auto& b = scene.create_game_object("b");
    auto& c = scene.create_game_object("c");

    (void)a.add_component<component::NodeComponent>();
    (void)b.add_component<component::NodeComponent>();
    (void)c.add_component<component::NodeComponent>();

    ASSERT_TRUE(scene.set_parent(b.id(), a.id()));
    ASSERT_TRUE(scene.set_parent(c.id(), b.id()));

    EXPECT_FALSE(scene.set_parent(a.id(), c.id()));
}

} // namespace rtr::framework::core::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
