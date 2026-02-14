#include <cstdint>

#include "gtest/gtest.h"

#include "rtr/system/input/input_system.hpp"

namespace rtr::system::input::test {

TEST(InputTypesTest, MapsFromGlfwValuesAndUnknown) {
    EXPECT_EQ(from_glfw_key(GLFW_KEY_A), KeyCode::A);
    EXPECT_EQ(from_glfw_key(GLFW_KEY_F25), KeyCode::F25);
    EXPECT_EQ(from_glfw_key(999999), KeyCode::UNKNOWN);

    EXPECT_EQ(from_glfw_action(GLFW_PRESS), KeyAction::PRESS);
    EXPECT_EQ(from_glfw_action(GLFW_RELEASE), KeyAction::RELEASE);
    EXPECT_EQ(from_glfw_action(GLFW_REPEAT), KeyAction::REPEAT);
    EXPECT_EQ(from_glfw_action(12345), KeyAction::UNKNOWN);

    EXPECT_EQ(from_glfw_button(GLFW_MOUSE_BUTTON_LEFT), MouseButton::BUTTON_1);
    EXPECT_EQ(from_glfw_button(GLFW_MOUSE_BUTTON_8), MouseButton::BUTTON_8);
    EXPECT_EQ(from_glfw_button(12345), MouseButton::UNKNOWN);
}

TEST(InputTypesTest, ModBitmaskHelpersWorkForCombinations) {
    const KeyMod combined = KeyMod::SHIFT | KeyMod::CONTROL;
    EXPECT_TRUE(has_mod(combined, KeyMod::SHIFT));
    EXPECT_TRUE(has_mod(combined, KeyMod::CONTROL));
    EXPECT_FALSE(has_mod(combined, KeyMod::ALT));

    const KeyMod mapped = from_glfw_mods(GLFW_MOD_SHIFT | GLFW_MOD_CONTROL);
    EXPECT_TRUE(has_mod(mapped, KeyMod::SHIFT | KeyMod::CONTROL));
    EXPECT_FALSE(has_mod(mapped, KeyMod::ALT));
}

TEST(InputSystemTest, KeyAndMouseStateTransitions) {
    utils::Event<int, int, int> key_source;
    utils::Event<int, int, int> mouse_button_source;
    utils::Event<double, double> mouse_move_source;
    utils::Event<double, double> mouse_scroll_source;

    InputSystem input(InputSystem::RawEventSource{
        .key_event = &key_source,
        .mouse_button_event = &mouse_button_source,
        .mouse_move_event = &mouse_move_source,
        .mouse_scroll_event = &mouse_scroll_source,
    });

    EXPECT_EQ(input.state().key_action(KeyCode::W), KeyAction::RELEASE);
    EXPECT_FALSE(input.state().key_down(KeyCode::W));

    key_source.execute(GLFW_KEY_W, GLFW_PRESS, GLFW_MOD_SHIFT);
    EXPECT_EQ(input.state().key_action(KeyCode::W), KeyAction::PRESS);
    EXPECT_TRUE(input.state().key_down(KeyCode::W));
    EXPECT_TRUE(input.state().mod_down(KeyMod::SHIFT));

    key_source.execute(GLFW_KEY_W, GLFW_REPEAT, GLFW_MOD_SHIFT);
    EXPECT_EQ(input.state().key_action(KeyCode::W), KeyAction::REPEAT);
    EXPECT_TRUE(input.state().key_down(KeyCode::W));

    key_source.execute(GLFW_KEY_W, GLFW_RELEASE, 0);
    EXPECT_EQ(input.state().key_action(KeyCode::W), KeyAction::RELEASE);
    EXPECT_FALSE(input.state().key_down(KeyCode::W));

    mouse_button_source.execute(GLFW_MOUSE_BUTTON_RIGHT, GLFW_PRESS, GLFW_MOD_CONTROL);
    EXPECT_TRUE(input.state().mouse_button_down(MouseButton::RIGHT));
    EXPECT_TRUE(input.state().mod_down(KeyMod::CONTROL));

    mouse_button_source.execute(GLFW_MOUSE_BUTTON_RIGHT, GLFW_RELEASE, 0);
    EXPECT_FALSE(input.state().mouse_button_down(MouseButton::RIGHT));
}

TEST(InputSystemTest, MouseDeltaAndScrollAccumulateThenResetAtEndFrame) {
    utils::Event<int, int, int> key_source;
    utils::Event<int, int, int> mouse_button_source;
    utils::Event<double, double> mouse_move_source;
    utils::Event<double, double> mouse_scroll_source;

    InputSystem input(InputSystem::RawEventSource{
        .key_event = &key_source,
        .mouse_button_event = &mouse_button_source,
        .mouse_move_event = &mouse_move_source,
        .mouse_scroll_event = &mouse_scroll_source,
    });

    mouse_move_source.execute(10.0, 5.0);
    mouse_move_source.execute(15.0, 13.0);
    EXPECT_DOUBLE_EQ(input.state().mouse_x(), 15.0);
    EXPECT_DOUBLE_EQ(input.state().mouse_y(), 13.0);
    EXPECT_DOUBLE_EQ(input.state().mouse_dx(), 15.0);
    EXPECT_DOUBLE_EQ(input.state().mouse_dy(), 13.0);

    mouse_scroll_source.execute(0.5, 1.0);
    mouse_scroll_source.execute(-0.25, 2.5);
    EXPECT_DOUBLE_EQ(input.state().mouse_scroll_dx(), 0.25);
    EXPECT_DOUBLE_EQ(input.state().mouse_scroll_dy(), 3.5);

    input.end_frame();
    EXPECT_DOUBLE_EQ(input.state().mouse_dx(), 0.0);
    EXPECT_DOUBLE_EQ(input.state().mouse_dy(), 0.0);
    EXPECT_DOUBLE_EQ(input.state().mouse_scroll_dx(), 0.0);
    EXPECT_DOUBLE_EQ(input.state().mouse_scroll_dy(), 0.0);
}

TEST(InputSystemTest, InterceptCaptureBlocksStateMutationAndDispatch) {
    utils::Event<int, int, int> key_source;
    utils::Event<int, int, int> mouse_button_source;
    utils::Event<double, double> mouse_move_source;
    utils::Event<double, double> mouse_scroll_source;

    InputSystem input(InputSystem::RawEventSource{
        .key_event = &key_source,
        .mouse_button_event = &mouse_button_source,
        .mouse_move_event = &mouse_move_source,
        .mouse_scroll_event = &mouse_scroll_source,
    });

    int key_dispatch_count = 0;
    int mouse_dispatch_count = 0;
    input.on_key().add([&key_dispatch_count](KeyCode, KeyAction, KeyMod) {
        ++key_dispatch_count;
    });
    input.on_mouse_move().add([&mouse_dispatch_count](double, double) {
        ++mouse_dispatch_count;
    });

    input.set_is_intercept_capture([](bool) { return true; });

    key_source.execute(GLFW_KEY_A, GLFW_PRESS, GLFW_MOD_SHIFT);
    mouse_move_source.execute(30.0, 40.0);

    EXPECT_FALSE(input.state().key_down(KeyCode::A));
    EXPECT_DOUBLE_EQ(input.state().mouse_x(), 0.0);
    EXPECT_EQ(key_dispatch_count, 0);
    EXPECT_EQ(mouse_dispatch_count, 0);
}

TEST(InputSystemTest, DetachesFromRawSourcesOnDestroy) {
    utils::Event<int, int, int> key_source;
    utils::Event<int, int, int> mouse_button_source;
    utils::Event<double, double> mouse_move_source;
    utils::Event<double, double> mouse_scroll_source;

    EXPECT_EQ(key_source.size(), 0u);
    EXPECT_EQ(mouse_button_source.size(), 0u);
    EXPECT_EQ(mouse_move_source.size(), 0u);
    EXPECT_EQ(mouse_scroll_source.size(), 0u);

    {
        InputSystem input(InputSystem::RawEventSource{
            .key_event = &key_source,
            .mouse_button_event = &mouse_button_source,
            .mouse_move_event = &mouse_move_source,
            .mouse_scroll_event = &mouse_scroll_source,
        });

        EXPECT_EQ(key_source.size(), 1u);
        EXPECT_EQ(mouse_button_source.size(), 1u);
        EXPECT_EQ(mouse_move_source.size(), 1u);
        EXPECT_EQ(mouse_scroll_source.size(), 1u);
    }

    EXPECT_EQ(key_source.size(), 0u);
    EXPECT_EQ(mouse_button_source.size(), 0u);
    EXPECT_EQ(mouse_move_source.size(), 0u);
    EXPECT_EQ(mouse_scroll_source.size(), 0u);
}

} // namespace rtr::system::input::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
