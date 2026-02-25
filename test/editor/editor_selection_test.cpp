#include <cstdlib>
#include <string>

#include "gtest/gtest.h"

#include "rtr/app/app_runtime.hpp"
#include "rtr/editor/core/editor_context.hpp"

namespace rtr::editor::test {

namespace {

bool gpu_tests_enabled() {
    const char* value = std::getenv("RTR_RUN_GPU_TESTS");
    return value != nullptr && std::string(value) == "1";
}

void require_gpu_tests_enabled() {
    if (!gpu_tests_enabled()) {
        GTEST_SKIP() << "Set RTR_RUN_GPU_TESTS=1 to run integration GPU tests.";
    }
}

} // namespace

TEST(EditorSelectionTest, KeepsValidSelection) {
    require_gpu_tests_enabled();

    app::AppRuntime runtime(app::AppRuntimeConfig{
        .window_width = 320,
        .window_height = 240,
        .window_title = "editor_selection_test",
        .auto_init_logging = false,
    });
    auto& world = runtime.world();
    auto& scene = world.create_scene("main");
    auto& keep_alive_scene = world.create_scene("keep_alive");
    auto& go = scene.create_game_object("node");
    (void)keep_alive_scene;

    EditorContext ctx(world, runtime.resource_manager(), runtime.renderer(), runtime.input_system());
    ctx.set_selection(scene.id(), go.id());
    ctx.validate_selection();

    EXPECT_TRUE(ctx.selection().has_game_object());
    EXPECT_EQ(ctx.selection().scene_id, scene.id());
    EXPECT_EQ(ctx.selection().game_object_id, go.id());
}

TEST(EditorSelectionTest, ClearsSelectionWhenGameObjectDestroyed) {
    require_gpu_tests_enabled();

    app::AppRuntime runtime(app::AppRuntimeConfig{
        .window_width = 320,
        .window_height = 240,
        .window_title = "editor_selection_test",
        .auto_init_logging = false,
    });
    auto& world = runtime.world();
    auto& scene = world.create_scene("main");
    auto& keep_alive_scene = world.create_scene("keep_alive");
    auto& go = scene.create_game_object("node");
    (void)keep_alive_scene;

    EditorContext ctx(world, runtime.resource_manager(), runtime.renderer(), runtime.input_system());
    ctx.set_selection(scene.id(), go.id());

    ASSERT_TRUE(scene.destroy_game_object(go.id()));
    ctx.validate_selection();

    EXPECT_FALSE(ctx.selection().has_game_object());
}

TEST(EditorSelectionTest, ClearsSelectionWhenSceneDestroyed) {
    require_gpu_tests_enabled();

    app::AppRuntime runtime(app::AppRuntimeConfig{
        .window_width = 320,
        .window_height = 240,
        .window_title = "editor_selection_test",
        .auto_init_logging = false,
    });
    auto& world = runtime.world();
    auto& scene = world.create_scene("main");
    auto& keep_alive_scene = world.create_scene("keep_alive");
    auto& go = scene.create_game_object("node");
    (void)keep_alive_scene;

    EditorContext ctx(world, runtime.resource_manager(), runtime.renderer(), runtime.input_system());
    ctx.set_selection(scene.id(), go.id());

    ASSERT_TRUE(world.set_active_scene("keep_alive"));
    ASSERT_TRUE(world.destroy_scene(scene.id()));
    ctx.validate_selection();

    EXPECT_FALSE(ctx.selection().has_game_object());
}

} // namespace rtr::editor::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
