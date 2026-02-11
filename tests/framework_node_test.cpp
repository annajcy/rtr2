#include <glm/glm.hpp>

#include "gtest/gtest.h"

#include "framework/framework.hpp"

namespace rtr::framework::component::test {

TEST(FrameworkNodeTest, ParentChildWorldTransformPropagation) {
    auto parent = Node::create();
    auto child = Node::create();

    parent->set_position({1.0f, 2.0f, 3.0f});
    child->set_position({0.0f, 1.0f, 0.0f});

    parent->add_child(child);

    EXPECT_EQ(parent->world_position(), glm::vec3(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(child->world_position(), glm::vec3(1.0f, 3.0f, 3.0f));
}

TEST(FrameworkNodeTest, DirtyPropagatesFromParentToChild) {
    auto parent = Node::create();
    auto child = Node::create();

    parent->set_position({1.0f, 0.0f, 0.0f});
    child->set_position({2.0f, 0.0f, 0.0f});
    parent->add_child(child);

    EXPECT_EQ(child->world_position(), glm::vec3(3.0f, 0.0f, 0.0f));
    EXPECT_FALSE(parent->is_dirty());
    EXPECT_FALSE(child->is_dirty());

    parent->set_position({10.0f, 0.0f, 0.0f});
    EXPECT_TRUE(parent->is_dirty());
    EXPECT_TRUE(child->is_dirty());

    EXPECT_EQ(child->world_position(), glm::vec3(12.0f, 0.0f, 0.0f));
}

TEST(FrameworkNodeTest, AddChildWithWorldPositionStaysKeepsWorldTransform) {
    auto parent = Node::create();
    auto child = Node::create();

    parent->set_position({10.0f, 0.0f, 0.0f});
    child->set_position({5.0f, 0.0f, 0.0f});
    const glm::vec3 before = child->world_position();

    parent->add_child(child, true);

    EXPECT_EQ(child->world_position(), before);
    EXPECT_EQ(child->position(), glm::vec3(-5.0f, 0.0f, 0.0f));
}

TEST(FrameworkNodeTest, RejectsCycleInParenting) {
    auto a = Node::create();
    auto b = Node::create();
    auto c = Node::create();

    a->add_child(b);
    b->add_child(c);

    EXPECT_THROW(c->add_child(a), std::invalid_argument);
}

TEST(FrameworkNodeTest, NodeComponentOwnsNodeInstanceOnAwake) {
    rtr::framework::core::GameObject go(1, "go");
    auto& node_component = go.add_component<NodeComponent>();

    ASSERT_NE(node_component.node(), nullptr);
    EXPECT_EQ(node_component.node()->children().size(), 0u);
}

} // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
