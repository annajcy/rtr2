#include "gtest/gtest.h"

#include "rtr/editor/editor_context.hpp"
#include "rtr/framework/core/world.hpp"

namespace rtr::editor::test {

TEST(EditorSelectionTest, KeepsValidSelection) {
    framework::core::World world;
    auto& scene = world.create_scene("main");
    auto& go = scene.create_game_object("node");

    EditorContext ctx;
    ctx.bind_runtime(&world, nullptr, nullptr, nullptr);
    ctx.set_selection(scene.id(), go.id());
    ctx.validate_selection();

    EXPECT_TRUE(ctx.selection().has_game_object());
    EXPECT_EQ(ctx.selection().scene_id, scene.id());
    EXPECT_EQ(ctx.selection().game_object_id, go.id());
}

TEST(EditorSelectionTest, ClearsSelectionWhenGameObjectDestroyed) {
    framework::core::World world;
    auto& scene = world.create_scene("main");
    auto& go = scene.create_game_object("node");

    EditorContext ctx;
    ctx.bind_runtime(&world, nullptr, nullptr, nullptr);
    ctx.set_selection(scene.id(), go.id());

    ASSERT_TRUE(scene.destroy_game_object(go.id()));
    ctx.validate_selection();

    EXPECT_FALSE(ctx.selection().has_game_object());
}

TEST(EditorSelectionTest, ClearsSelectionWhenSceneDestroyed) {
    framework::core::World world;
    auto& scene = world.create_scene("main");
    auto& go = scene.create_game_object("node");

    EditorContext ctx;
    ctx.bind_runtime(&world, nullptr, nullptr, nullptr);
    ctx.set_selection(scene.id(), go.id());

    ASSERT_TRUE(world.destroy_scene(scene.id()));
    ctx.validate_selection();

    EXPECT_FALSE(ctx.selection().has_game_object());
}

} // namespace rtr::editor::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

