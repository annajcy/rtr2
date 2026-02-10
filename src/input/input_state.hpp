#pragma once

#include <cstddef>
#include <unordered_map>

#include "input/input_types.hpp"

namespace rtr::input {

struct EnumClassHash {
    template <typename T>
    std::size_t operator()(T value) const noexcept {
        return static_cast<std::size_t>(value);
    }
};

struct InputState {
private:
    std::unordered_map<KeyCode, KeyAction, EnumClassHash> m_keys{};
    std::unordered_map<MouseButton, KeyAction, EnumClassHash> m_mouse_buttons{};
    KeyMod m_key_mods{KeyMod::NONE};

    double m_mouse_x{};
    double m_mouse_y{};

    double m_mouse_dx{};
    double m_mouse_dy{};

    double m_mouse_scroll_dx{};
    double m_mouse_scroll_dy{};

public:
    InputState() = default;

    KeyAction key_action(KeyCode key) const {
        const auto it = m_keys.find(key);
        if (it == m_keys.end()) {
            return KeyAction::RELEASE;
        }
        return it->second;
    }

    KeyAction mouse_button_action(MouseButton button) const {
        const auto it = m_mouse_buttons.find(button);
        if (it == m_mouse_buttons.end()) {
            return KeyAction::RELEASE;
        }
        return it->second;
    }

    bool key_down(KeyCode key) const {
        const KeyAction action = key_action(key);
        return action == KeyAction::PRESS || action == KeyAction::REPEAT;
    }

    bool mouse_button_down(MouseButton button) const {
        const KeyAction action = mouse_button_action(button);
        return action == KeyAction::PRESS || action == KeyAction::REPEAT;
    }

    bool mod_down(KeyMod mod_mask) const {
        return has_mod(m_key_mods, mod_mask);
    }

    KeyMod mods() const {
        return m_key_mods;
    }

    double mouse_x() const { return m_mouse_x; }
    double mouse_y() const { return m_mouse_y; }

    double mouse_dx() const { return m_mouse_dx; }
    double mouse_dy() const { return m_mouse_dy; }

    double mouse_scroll_dx() const { return m_mouse_scroll_dx; }
    double mouse_scroll_dy() const { return m_mouse_scroll_dy; }

    void update_key(KeyCode key, KeyAction action, KeyMod mods) {
        m_key_mods = mods;
        if (key == KeyCode::UNKNOWN || action == KeyAction::UNKNOWN) {
            return;
        }
        m_keys[key] = action;
    }

    void update_mouse_button(MouseButton button, KeyAction action, KeyMod mods) {
        m_key_mods = mods;
        if (button == MouseButton::UNKNOWN || action == KeyAction::UNKNOWN) {
            return;
        }
        m_mouse_buttons[button] = action;
    }

    void update_mouse_position(double x, double y) {
        m_mouse_dx += x - m_mouse_x;
        m_mouse_dy += y - m_mouse_y;
        m_mouse_x = x;
        m_mouse_y = y;
    }

    void update_mouse_scroll(double x, double y) {
        m_mouse_scroll_dx += x;
        m_mouse_scroll_dy += y;
    }

    void reset_deltas() {
        m_mouse_dx = 0.0;
        m_mouse_dy = 0.0;
        m_mouse_scroll_dx = 0.0;
        m_mouse_scroll_dy = 0.0;
    }
};

} // namespace rtr::input
