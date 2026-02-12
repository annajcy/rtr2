#include <algorithm>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "framework/component/component.hpp"
#include "framework/core/scene.hpp"
#include "framework/core/scene_graph.hpp"

namespace rtr::framework::core::test {

class ThrowOnDestroyComponent final : public component::Component {
public:
    void on_destroy() override {
        throw std::runtime_error("destroy failed");
    }
};

static void expect_vec3_near(const glm::vec3& lhs, const glm::vec3& rhs, float eps = 1e-5f) {
    EXPECT_NEAR(lhs.x, rhs.x, eps);
    EXPECT_NEAR(lhs.y, rhs.y, eps);
    EXPECT_NEAR(lhs.z, rhs.z, eps);
}

TEST(SceneGraphTest, CreateGameObjectRegistersNode) {
    Scene scene(1, "scene");
    auto& go = scene.create_game_object("go");

    EXPECT_TRUE(scene.scene_graph().has_node(go.id()));
    EXPECT_EQ(scene.scene_graph().node(go.id()).parent_id(), SceneGraph::kVirtualRootId);
}

TEST(SceneGraphTest, ChildOrderingFollowsInsertionOrder) {
    Scene scene(1, "scene");
    auto& parent = scene.create_game_object("parent");
    auto& c1 = scene.create_game_object("c1");
    auto& c2 = scene.create_game_object("c2");
    auto& c3 = scene.create_game_object("c3");

    ASSERT_TRUE(scene.scene_graph().set_parent(c1.id(), parent.id()));
    ASSERT_TRUE(scene.scene_graph().set_parent(c2.id(), parent.id()));
    ASSERT_TRUE(scene.scene_graph().set_parent(c3.id(), parent.id()));

    const auto& children = scene.scene_graph().node(parent.id()).children();
    std::vector<GameObjectId> expected{c1.id(), c2.id(), c3.id()};
    EXPECT_EQ(children, expected);
}

TEST(SceneGraphTest, SetParentWorldStaysKeepsWorldPosition) {
    Scene scene(1, "scene");
    auto& parent = scene.create_game_object("parent");
    auto& child = scene.create_game_object("child");

    scene.scene_graph().node(parent.id()).set_local_position({10.0f, 0.0f, 0.0f});
    scene.scene_graph().node(child.id()).set_local_position({5.0f, 0.0f, 0.0f});
    scene.scene_graph().update_world_transforms();
    const glm::vec3 before = scene.scene_graph().node(child.id()).world_position();

    ASSERT_TRUE(scene.scene_graph().set_parent(child.id(), parent.id()));
    scene.scene_graph().update_world_transforms();
    const glm::vec3 after = scene.scene_graph().node(child.id()).world_position();

    expect_vec3_near(before, after);
}

TEST(SceneGraphTest, RejectsCycleInHierarchy) {
    Scene scene(1, "scene");
    auto& a = scene.create_game_object("a");
    auto& b = scene.create_game_object("b");
    auto& c = scene.create_game_object("c");

    ASSERT_TRUE(scene.scene_graph().set_parent(b.id(), a.id()));
    ASSERT_TRUE(scene.scene_graph().set_parent(c.id(), b.id()));
    EXPECT_FALSE(scene.scene_graph().set_parent(a.id(), c.id()));
}

TEST(SceneGraphTest, ParentInactiveMakesChildInactive) {
    Scene scene(1, "scene");
    auto& parent = scene.create_game_object("parent");
    auto& child = scene.create_game_object("child");
    ASSERT_TRUE(scene.scene_graph().set_parent(child.id(), parent.id()));

    parent.set_enabled(false);
    scene.tick(FrameTickContext{.delta_seconds = 0.016, .unscaled_delta_seconds = 0.016, .frame_index = 0});

    const auto active = scene.scene_graph().active_nodes();
    EXPECT_FALSE(std::find(active.begin(), active.end(), parent.id()) != active.end());
    EXPECT_FALSE(std::find(active.begin(), active.end(), child.id()) != active.end());
}

TEST(SceneGraphTest, SetEnabledRecursivelyAffectsSubtree) {
    Scene scene(1, "scene");
    auto& parent = scene.create_game_object("parent");
    auto& child = scene.create_game_object("child");
    auto& grandchild = scene.create_game_object("grandchild");
    ASSERT_TRUE(scene.scene_graph().set_parent(child.id(), parent.id()));
    ASSERT_TRUE(scene.scene_graph().set_parent(grandchild.id(), child.id()));

    scene.scene_graph().set_enabled(parent.id(), false);
    EXPECT_FALSE(scene.scene_graph().node(parent.id()).is_enabled());
    EXPECT_FALSE(scene.scene_graph().node(child.id()).is_enabled());
    EXPECT_FALSE(scene.scene_graph().node(grandchild.id()).is_enabled());

    scene.scene_graph().set_enabled(parent.id(), true);
    EXPECT_TRUE(scene.scene_graph().node(parent.id()).is_enabled());
    EXPECT_TRUE(scene.scene_graph().node(child.id()).is_enabled());
    EXPECT_TRUE(scene.scene_graph().node(grandchild.id()).is_enabled());
}

TEST(SceneGraphTest, DirtyFlagPropagatesToSubtree) {
    Scene scene(1, "scene");
    auto& parent = scene.create_game_object("parent");
    auto& child = scene.create_game_object("child");
    auto& grandchild = scene.create_game_object("grandchild");
    ASSERT_TRUE(scene.scene_graph().set_parent(child.id(), parent.id()));
    ASSERT_TRUE(scene.scene_graph().set_parent(grandchild.id(), child.id()));

    scene.scene_graph().update_world_transforms();
    EXPECT_FALSE(scene.scene_graph().node(parent.id()).dirty());
    EXPECT_FALSE(scene.scene_graph().node(child.id()).dirty());
    EXPECT_FALSE(scene.scene_graph().node(grandchild.id()).dirty());

    scene.scene_graph().node(child.id()).set_local_position({1.0f, 2.0f, 3.0f});
    EXPECT_FALSE(scene.scene_graph().node(parent.id()).dirty());
    EXPECT_TRUE(scene.scene_graph().node(child.id()).dirty());
    EXPECT_TRUE(scene.scene_graph().node(grandchild.id()).dirty());
}

TEST(SceneGraphTest, DestroyGameObjectCascadesSubtreeDeletion) {
    Scene scene(1, "scene");
    auto& parent = scene.create_game_object("parent");
    auto& child = scene.create_game_object("child");
    auto& grandchild = scene.create_game_object("grandchild");
    auto& other = scene.create_game_object("other");
    const GameObjectId parent_id = parent.id();
    const GameObjectId child_id = child.id();
    const GameObjectId grandchild_id = grandchild.id();
    const GameObjectId other_id = other.id();

    ASSERT_TRUE(scene.scene_graph().set_parent(child_id, parent_id));
    ASSERT_TRUE(scene.scene_graph().set_parent(grandchild_id, child_id));

    EXPECT_TRUE(scene.destroy_game_object(parent_id));
    EXPECT_FALSE(scene.has_game_object(parent_id));
    EXPECT_FALSE(scene.has_game_object(child_id));
    EXPECT_FALSE(scene.has_game_object(grandchild_id));
    EXPECT_TRUE(scene.has_game_object(other_id));

    EXPECT_FALSE(scene.scene_graph().has_node(parent_id));
    EXPECT_FALSE(scene.scene_graph().has_node(child_id));
    EXPECT_FALSE(scene.scene_graph().has_node(grandchild_id));
    EXPECT_TRUE(scene.scene_graph().has_node(other_id));
}

TEST(SceneGraphTest, DestroyPropagatesComponentDestroyException) {
    Scene scene(1, "scene");
    auto& parent = scene.create_game_object("parent");
    auto& child = scene.create_game_object("child");
    const GameObjectId parent_id = parent.id();
    const GameObjectId child_id = child.id();
    ASSERT_TRUE(scene.scene_graph().set_parent(child_id, parent_id));
    (void)child.add_component<ThrowOnDestroyComponent>();

    EXPECT_THROW(scene.destroy_game_object(parent_id), std::runtime_error);
}

TEST(SceneGraphTest, SnapshotRoundTripPreservesHierarchyAndLocalTransform) {
    SceneGraph graph;
    ASSERT_TRUE(graph.register_node(1));
    ASSERT_TRUE(graph.register_node(2));
    ASSERT_TRUE(graph.register_node(3));
    ASSERT_TRUE(graph.set_parent(2, 1, false));
    ASSERT_TRUE(graph.set_parent(3, 1, false));
    graph.node(1).set_local_position({10.0f, 0.0f, 0.0f});
    graph.node(2).set_local_position({1.0f, 0.0f, 0.0f});
    graph.node(3).set_local_position({2.0f, 0.0f, 0.0f});
    graph.set_enabled(2, false);
    graph.update_world_transforms();

    const SceneGraph::Snapshot snapshot = graph.to_snapshot();

    auto restored_opt = SceneGraph::from_snapshot(snapshot);
    ASSERT_TRUE(restored_opt.has_value());
    SceneGraph& restored = *restored_opt;
    EXPECT_TRUE(restored.has_node(1));
    EXPECT_TRUE(restored.has_node(2));
    EXPECT_TRUE(restored.has_node(3));
    EXPECT_EQ(restored.node(2).parent_id(), 1u);
    EXPECT_EQ(restored.node(3).parent_id(), 1u);
    expect_vec3_near(restored.node(1).local_position(), {10.0f, 0.0f, 0.0f});
    expect_vec3_near(restored.node(2).local_position(), {1.0f, 0.0f, 0.0f});
    expect_vec3_near(restored.node(3).local_position(), {2.0f, 0.0f, 0.0f});
    EXPECT_FALSE(restored.node(2).is_enabled());
}

} // namespace rtr::framework::core::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
