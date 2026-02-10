#pragma once

#include <cstdint>
#include <type_traits>

#ifndef GLFW_INCLUDE_VULKAN
#define GLFW_INCLUDE_VULKAN
#endif
#include "GLFW/glfw3.h"

namespace rtr::input {

enum class KeyCode : int {
    UNKNOWN = GLFW_KEY_UNKNOWN,
    SPACE = GLFW_KEY_SPACE,
    APOSTROPHE = GLFW_KEY_APOSTROPHE,
    COMMA = GLFW_KEY_COMMA,
    MINUS = GLFW_KEY_MINUS,
    PERIOD = GLFW_KEY_PERIOD,
    SLASH = GLFW_KEY_SLASH,
    NUM_0 = GLFW_KEY_0,
    NUM_1 = GLFW_KEY_1,
    NUM_2 = GLFW_KEY_2,
    NUM_3 = GLFW_KEY_3,
    NUM_4 = GLFW_KEY_4,
    NUM_5 = GLFW_KEY_5,
    NUM_6 = GLFW_KEY_6,
    NUM_7 = GLFW_KEY_7,
    NUM_8 = GLFW_KEY_8,
    NUM_9 = GLFW_KEY_9,
    SEMICOLON = GLFW_KEY_SEMICOLON,
    EQUAL = GLFW_KEY_EQUAL,
    A = GLFW_KEY_A,
    B = GLFW_KEY_B,
    C = GLFW_KEY_C,
    D = GLFW_KEY_D,
    E = GLFW_KEY_E,
    F = GLFW_KEY_F,
    G = GLFW_KEY_G,
    H = GLFW_KEY_H,
    I = GLFW_KEY_I,
    J = GLFW_KEY_J,
    K = GLFW_KEY_K,
    L = GLFW_KEY_L,
    M = GLFW_KEY_M,
    N = GLFW_KEY_N,
    O = GLFW_KEY_O,
    P = GLFW_KEY_P,
    Q = GLFW_KEY_Q,
    R = GLFW_KEY_R,
    S = GLFW_KEY_S,
    T = GLFW_KEY_T,
    U = GLFW_KEY_U,
    V = GLFW_KEY_V,
    W = GLFW_KEY_W,
    X = GLFW_KEY_X,
    Y = GLFW_KEY_Y,
    Z = GLFW_KEY_Z,
    LEFT_BRACKET = GLFW_KEY_LEFT_BRACKET,
    BACKSLASH = GLFW_KEY_BACKSLASH,
    RIGHT_BRACKET = GLFW_KEY_RIGHT_BRACKET,
    GRAVE_ACCENT = GLFW_KEY_GRAVE_ACCENT,
    WORLD_1 = GLFW_KEY_WORLD_1,
    WORLD_2 = GLFW_KEY_WORLD_2,
    ESCAPE = GLFW_KEY_ESCAPE,
    ENTER = GLFW_KEY_ENTER,
    TAB = GLFW_KEY_TAB,
    BACKSPACE = GLFW_KEY_BACKSPACE,
    INSERT = GLFW_KEY_INSERT,
    DELETE_KEY = GLFW_KEY_DELETE,
    RIGHT = GLFW_KEY_RIGHT,
    LEFT = GLFW_KEY_LEFT,
    DOWN = GLFW_KEY_DOWN,
    UP = GLFW_KEY_UP,
    PAGE_UP = GLFW_KEY_PAGE_UP,
    PAGE_DOWN = GLFW_KEY_PAGE_DOWN,
    HOME = GLFW_KEY_HOME,
    END = GLFW_KEY_END,
    CAPS_LOCK = GLFW_KEY_CAPS_LOCK,
    SCROLL_LOCK = GLFW_KEY_SCROLL_LOCK,
    NUM_LOCK = GLFW_KEY_NUM_LOCK,
    PRINT_SCREEN = GLFW_KEY_PRINT_SCREEN,
    PAUSE = GLFW_KEY_PAUSE,
    F1 = GLFW_KEY_F1,
    F2 = GLFW_KEY_F2,
    F3 = GLFW_KEY_F3,
    F4 = GLFW_KEY_F4,
    F5 = GLFW_KEY_F5,
    F6 = GLFW_KEY_F6,
    F7 = GLFW_KEY_F7,
    F8 = GLFW_KEY_F8,
    F9 = GLFW_KEY_F9,
    F10 = GLFW_KEY_F10,
    F11 = GLFW_KEY_F11,
    F12 = GLFW_KEY_F12,
    F13 = GLFW_KEY_F13,
    F14 = GLFW_KEY_F14,
    F15 = GLFW_KEY_F15,
    F16 = GLFW_KEY_F16,
    F17 = GLFW_KEY_F17,
    F18 = GLFW_KEY_F18,
    F19 = GLFW_KEY_F19,
    F20 = GLFW_KEY_F20,
    F21 = GLFW_KEY_F21,
    F22 = GLFW_KEY_F22,
    F23 = GLFW_KEY_F23,
    F24 = GLFW_KEY_F24,
    F25 = GLFW_KEY_F25,
    KP_0 = GLFW_KEY_KP_0,
    KP_1 = GLFW_KEY_KP_1,
    KP_2 = GLFW_KEY_KP_2,
    KP_3 = GLFW_KEY_KP_3,
    KP_4 = GLFW_KEY_KP_4,
    KP_5 = GLFW_KEY_KP_5,
    KP_6 = GLFW_KEY_KP_6,
    KP_7 = GLFW_KEY_KP_7,
    KP_8 = GLFW_KEY_KP_8,
    KP_9 = GLFW_KEY_KP_9,
    KP_DECIMAL = GLFW_KEY_KP_DECIMAL,
    KP_DIVIDE = GLFW_KEY_KP_DIVIDE,
    KP_MULTIPLY = GLFW_KEY_KP_MULTIPLY,
    KP_SUBTRACT = GLFW_KEY_KP_SUBTRACT,
    KP_ADD = GLFW_KEY_KP_ADD,
    KP_ENTER = GLFW_KEY_KP_ENTER,
    KP_EQUAL = GLFW_KEY_KP_EQUAL,
    LEFT_SHIFT = GLFW_KEY_LEFT_SHIFT,
    LEFT_CONTROL = GLFW_KEY_LEFT_CONTROL,
    LEFT_ALT = GLFW_KEY_LEFT_ALT,
    LEFT_SUPER = GLFW_KEY_LEFT_SUPER,
    RIGHT_SHIFT = GLFW_KEY_RIGHT_SHIFT,
    RIGHT_CONTROL = GLFW_KEY_RIGHT_CONTROL,
    RIGHT_ALT = GLFW_KEY_RIGHT_ALT,
    RIGHT_SUPER = GLFW_KEY_RIGHT_SUPER,
    MENU = GLFW_KEY_MENU,
};

enum class MouseButton : int {
    UNKNOWN = -1,
    BUTTON_1 = GLFW_MOUSE_BUTTON_1,
    BUTTON_2 = GLFW_MOUSE_BUTTON_2,
    BUTTON_3 = GLFW_MOUSE_BUTTON_3,
    BUTTON_4 = GLFW_MOUSE_BUTTON_4,
    BUTTON_5 = GLFW_MOUSE_BUTTON_5,
    BUTTON_6 = GLFW_MOUSE_BUTTON_6,
    BUTTON_7 = GLFW_MOUSE_BUTTON_7,
    BUTTON_8 = GLFW_MOUSE_BUTTON_8,
    LEFT = GLFW_MOUSE_BUTTON_LEFT,
    RIGHT = GLFW_MOUSE_BUTTON_RIGHT,
    MIDDLE = GLFW_MOUSE_BUTTON_MIDDLE,
};

enum class KeyAction : int {
    RELEASE = GLFW_RELEASE,
    PRESS = GLFW_PRESS,
    REPEAT = GLFW_REPEAT,
    UNKNOWN = -1,
};

enum class KeyMod : uint32_t {
    NONE = 0,
    SHIFT = GLFW_MOD_SHIFT,
    CONTROL = GLFW_MOD_CONTROL,
    ALT = GLFW_MOD_ALT,
    SUPER = GLFW_MOD_SUPER,
#ifdef GLFW_MOD_CAPS_LOCK
    CAPS_LOCK = GLFW_MOD_CAPS_LOCK,
#endif
#ifdef GLFW_MOD_NUM_LOCK
    NUM_LOCK = GLFW_MOD_NUM_LOCK,
#endif
};

constexpr KeyMod operator|(KeyMod lhs, KeyMod rhs) {
    return static_cast<KeyMod>(
        static_cast<uint32_t>(lhs) | static_cast<uint32_t>(rhs)
    );
}

constexpr KeyMod operator&(KeyMod lhs, KeyMod rhs) {
    return static_cast<KeyMod>(
        static_cast<uint32_t>(lhs) & static_cast<uint32_t>(rhs)
    );
}

inline KeyMod& operator|=(KeyMod& lhs, KeyMod rhs) {
    lhs = lhs | rhs;
    return lhs;
}

constexpr bool has_mod(KeyMod value, KeyMod mask) {
    return (static_cast<uint32_t>(value) & static_cast<uint32_t>(mask)) == static_cast<uint32_t>(mask);
}

constexpr KeyCode from_glfw_key(int key) {
    switch (key) {
        case GLFW_KEY_UNKNOWN: return KeyCode::UNKNOWN;
        case GLFW_KEY_SPACE: return KeyCode::SPACE;
        case GLFW_KEY_APOSTROPHE: return KeyCode::APOSTROPHE;
        case GLFW_KEY_COMMA: return KeyCode::COMMA;
        case GLFW_KEY_MINUS: return KeyCode::MINUS;
        case GLFW_KEY_PERIOD: return KeyCode::PERIOD;
        case GLFW_KEY_SLASH: return KeyCode::SLASH;
        case GLFW_KEY_0: return KeyCode::NUM_0;
        case GLFW_KEY_1: return KeyCode::NUM_1;
        case GLFW_KEY_2: return KeyCode::NUM_2;
        case GLFW_KEY_3: return KeyCode::NUM_3;
        case GLFW_KEY_4: return KeyCode::NUM_4;
        case GLFW_KEY_5: return KeyCode::NUM_5;
        case GLFW_KEY_6: return KeyCode::NUM_6;
        case GLFW_KEY_7: return KeyCode::NUM_7;
        case GLFW_KEY_8: return KeyCode::NUM_8;
        case GLFW_KEY_9: return KeyCode::NUM_9;
        case GLFW_KEY_SEMICOLON: return KeyCode::SEMICOLON;
        case GLFW_KEY_EQUAL: return KeyCode::EQUAL;
        case GLFW_KEY_A: return KeyCode::A;
        case GLFW_KEY_B: return KeyCode::B;
        case GLFW_KEY_C: return KeyCode::C;
        case GLFW_KEY_D: return KeyCode::D;
        case GLFW_KEY_E: return KeyCode::E;
        case GLFW_KEY_F: return KeyCode::F;
        case GLFW_KEY_G: return KeyCode::G;
        case GLFW_KEY_H: return KeyCode::H;
        case GLFW_KEY_I: return KeyCode::I;
        case GLFW_KEY_J: return KeyCode::J;
        case GLFW_KEY_K: return KeyCode::K;
        case GLFW_KEY_L: return KeyCode::L;
        case GLFW_KEY_M: return KeyCode::M;
        case GLFW_KEY_N: return KeyCode::N;
        case GLFW_KEY_O: return KeyCode::O;
        case GLFW_KEY_P: return KeyCode::P;
        case GLFW_KEY_Q: return KeyCode::Q;
        case GLFW_KEY_R: return KeyCode::R;
        case GLFW_KEY_S: return KeyCode::S;
        case GLFW_KEY_T: return KeyCode::T;
        case GLFW_KEY_U: return KeyCode::U;
        case GLFW_KEY_V: return KeyCode::V;
        case GLFW_KEY_W: return KeyCode::W;
        case GLFW_KEY_X: return KeyCode::X;
        case GLFW_KEY_Y: return KeyCode::Y;
        case GLFW_KEY_Z: return KeyCode::Z;
        case GLFW_KEY_LEFT_BRACKET: return KeyCode::LEFT_BRACKET;
        case GLFW_KEY_BACKSLASH: return KeyCode::BACKSLASH;
        case GLFW_KEY_RIGHT_BRACKET: return KeyCode::RIGHT_BRACKET;
        case GLFW_KEY_GRAVE_ACCENT: return KeyCode::GRAVE_ACCENT;
        case GLFW_KEY_WORLD_1: return KeyCode::WORLD_1;
        case GLFW_KEY_WORLD_2: return KeyCode::WORLD_2;
        case GLFW_KEY_ESCAPE: return KeyCode::ESCAPE;
        case GLFW_KEY_ENTER: return KeyCode::ENTER;
        case GLFW_KEY_TAB: return KeyCode::TAB;
        case GLFW_KEY_BACKSPACE: return KeyCode::BACKSPACE;
        case GLFW_KEY_INSERT: return KeyCode::INSERT;
        case GLFW_KEY_DELETE: return KeyCode::DELETE_KEY;
        case GLFW_KEY_RIGHT: return KeyCode::RIGHT;
        case GLFW_KEY_LEFT: return KeyCode::LEFT;
        case GLFW_KEY_DOWN: return KeyCode::DOWN;
        case GLFW_KEY_UP: return KeyCode::UP;
        case GLFW_KEY_PAGE_UP: return KeyCode::PAGE_UP;
        case GLFW_KEY_PAGE_DOWN: return KeyCode::PAGE_DOWN;
        case GLFW_KEY_HOME: return KeyCode::HOME;
        case GLFW_KEY_END: return KeyCode::END;
        case GLFW_KEY_CAPS_LOCK: return KeyCode::CAPS_LOCK;
        case GLFW_KEY_SCROLL_LOCK: return KeyCode::SCROLL_LOCK;
        case GLFW_KEY_NUM_LOCK: return KeyCode::NUM_LOCK;
        case GLFW_KEY_PRINT_SCREEN: return KeyCode::PRINT_SCREEN;
        case GLFW_KEY_PAUSE: return KeyCode::PAUSE;
        case GLFW_KEY_F1: return KeyCode::F1;
        case GLFW_KEY_F2: return KeyCode::F2;
        case GLFW_KEY_F3: return KeyCode::F3;
        case GLFW_KEY_F4: return KeyCode::F4;
        case GLFW_KEY_F5: return KeyCode::F5;
        case GLFW_KEY_F6: return KeyCode::F6;
        case GLFW_KEY_F7: return KeyCode::F7;
        case GLFW_KEY_F8: return KeyCode::F8;
        case GLFW_KEY_F9: return KeyCode::F9;
        case GLFW_KEY_F10: return KeyCode::F10;
        case GLFW_KEY_F11: return KeyCode::F11;
        case GLFW_KEY_F12: return KeyCode::F12;
        case GLFW_KEY_F13: return KeyCode::F13;
        case GLFW_KEY_F14: return KeyCode::F14;
        case GLFW_KEY_F15: return KeyCode::F15;
        case GLFW_KEY_F16: return KeyCode::F16;
        case GLFW_KEY_F17: return KeyCode::F17;
        case GLFW_KEY_F18: return KeyCode::F18;
        case GLFW_KEY_F19: return KeyCode::F19;
        case GLFW_KEY_F20: return KeyCode::F20;
        case GLFW_KEY_F21: return KeyCode::F21;
        case GLFW_KEY_F22: return KeyCode::F22;
        case GLFW_KEY_F23: return KeyCode::F23;
        case GLFW_KEY_F24: return KeyCode::F24;
        case GLFW_KEY_F25: return KeyCode::F25;
        case GLFW_KEY_KP_0: return KeyCode::KP_0;
        case GLFW_KEY_KP_1: return KeyCode::KP_1;
        case GLFW_KEY_KP_2: return KeyCode::KP_2;
        case GLFW_KEY_KP_3: return KeyCode::KP_3;
        case GLFW_KEY_KP_4: return KeyCode::KP_4;
        case GLFW_KEY_KP_5: return KeyCode::KP_5;
        case GLFW_KEY_KP_6: return KeyCode::KP_6;
        case GLFW_KEY_KP_7: return KeyCode::KP_7;
        case GLFW_KEY_KP_8: return KeyCode::KP_8;
        case GLFW_KEY_KP_9: return KeyCode::KP_9;
        case GLFW_KEY_KP_DECIMAL: return KeyCode::KP_DECIMAL;
        case GLFW_KEY_KP_DIVIDE: return KeyCode::KP_DIVIDE;
        case GLFW_KEY_KP_MULTIPLY: return KeyCode::KP_MULTIPLY;
        case GLFW_KEY_KP_SUBTRACT: return KeyCode::KP_SUBTRACT;
        case GLFW_KEY_KP_ADD: return KeyCode::KP_ADD;
        case GLFW_KEY_KP_ENTER: return KeyCode::KP_ENTER;
        case GLFW_KEY_KP_EQUAL: return KeyCode::KP_EQUAL;
        case GLFW_KEY_LEFT_SHIFT: return KeyCode::LEFT_SHIFT;
        case GLFW_KEY_LEFT_CONTROL: return KeyCode::LEFT_CONTROL;
        case GLFW_KEY_LEFT_ALT: return KeyCode::LEFT_ALT;
        case GLFW_KEY_LEFT_SUPER: return KeyCode::LEFT_SUPER;
        case GLFW_KEY_RIGHT_SHIFT: return KeyCode::RIGHT_SHIFT;
        case GLFW_KEY_RIGHT_CONTROL: return KeyCode::RIGHT_CONTROL;
        case GLFW_KEY_RIGHT_ALT: return KeyCode::RIGHT_ALT;
        case GLFW_KEY_RIGHT_SUPER: return KeyCode::RIGHT_SUPER;
        case GLFW_KEY_MENU: return KeyCode::MENU;
        default: return KeyCode::UNKNOWN;
    }
}

constexpr KeyAction from_glfw_action(int action) {
    switch (action) {
        case GLFW_RELEASE: return KeyAction::RELEASE;
        case GLFW_PRESS: return KeyAction::PRESS;
        case GLFW_REPEAT: return KeyAction::REPEAT;
        default: return KeyAction::UNKNOWN;
    }
}

constexpr MouseButton from_glfw_button(int button) {
    switch (button) {
        case GLFW_MOUSE_BUTTON_1: return MouseButton::BUTTON_1;
        case GLFW_MOUSE_BUTTON_2: return MouseButton::BUTTON_2;
        case GLFW_MOUSE_BUTTON_3: return MouseButton::BUTTON_3;
        case GLFW_MOUSE_BUTTON_4: return MouseButton::BUTTON_4;
        case GLFW_MOUSE_BUTTON_5: return MouseButton::BUTTON_5;
        case GLFW_MOUSE_BUTTON_6: return MouseButton::BUTTON_6;
        case GLFW_MOUSE_BUTTON_7: return MouseButton::BUTTON_7;
        case GLFW_MOUSE_BUTTON_8: return MouseButton::BUTTON_8;
        default: return MouseButton::UNKNOWN;
    }
}

constexpr uint32_t key_mod_mask() {
    uint32_t mask = GLFW_MOD_SHIFT | GLFW_MOD_CONTROL | GLFW_MOD_ALT | GLFW_MOD_SUPER;
#ifdef GLFW_MOD_CAPS_LOCK
    mask |= GLFW_MOD_CAPS_LOCK;
#endif
#ifdef GLFW_MOD_NUM_LOCK
    mask |= GLFW_MOD_NUM_LOCK;
#endif
    return mask;
}

constexpr KeyMod from_glfw_mods(int mods) {
    return static_cast<KeyMod>(static_cast<uint32_t>(mods) & key_mod_mask());
}

} // namespace rtr::input
