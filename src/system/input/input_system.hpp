#pragma once

#include <functional>
#include <utility>

#include "system/input/input_state.hpp"
#include "rhi/window.hpp"
#include "utils/event_center.hpp"

namespace rtr::system::input {

class InputSystem {
public:
    using KeyEvent = utils::Event<KeyCode, KeyAction, KeyMod>;
    using MouseButtonEvent = utils::Event<MouseButton, KeyAction, KeyMod>;
    using MouseMoveEvent = utils::Event<double, double>;
    using MouseScrollEvent = utils::Event<double, double>;

    struct RawEventSource {
        utils::Event<int, int, int>* key_event{};
        utils::Event<int, int, int>* mouse_button_event{};
        utils::Event<double, double>* mouse_move_event{};
        utils::Event<double, double>* mouse_scroll_event{};
    };

private:
    InputState m_state{};
    std::function<bool(bool is_mouse)> m_is_intercept_capture{[](bool) { return false; }};

    KeyEvent m_key_event{};
    MouseButtonEvent m_mouse_button_event{};
    MouseMoveEvent m_mouse_move_event{};
    MouseScrollEvent m_mouse_scroll_event{};

    RawEventSource m_source{};

    utils::Event<int, int, int>::ActionHandle m_key_handle{};
    utils::Event<int, int, int>::ActionHandle m_mouse_button_handle{};
    utils::Event<double, double>::ActionHandle m_mouse_move_handle{};
    utils::Event<double, double>::ActionHandle m_mouse_scroll_handle{};

public:
    explicit InputSystem(rhi::Window* window)
        : InputSystem(make_window_source(window)) {}

    explicit InputSystem(const RawEventSource& source) {
        attach(source);
    }

    ~InputSystem() {
        detach();
    }

    InputSystem(const InputSystem&) = delete;
    InputSystem& operator=(const InputSystem&) = delete;

    InputSystem(InputSystem&&) = delete;
    InputSystem& operator=(InputSystem&&) = delete;

    void begin_frame() {}

    void end_frame() {
        m_state.reset_deltas();
    }

    const InputState& state() const {
        return m_state;
    }

    void set_is_intercept_capture(std::function<bool(bool is_mouse)> is_intercept_capture) {
        if (!is_intercept_capture) {
            m_is_intercept_capture = [](bool) { return false; };
            return;
        }
        m_is_intercept_capture = std::move(is_intercept_capture);
    }

    KeyEvent& on_key() { return m_key_event; }
    MouseButtonEvent& on_mouse_button() { return m_mouse_button_event; }
    MouseMoveEvent& on_mouse_move() { return m_mouse_move_event; }
    MouseScrollEvent& on_mouse_scroll() { return m_mouse_scroll_event; }

    void handle_key_raw(int key, int action, int mods) {
        if (is_intercept_capture(false)) {
            return;
        }

        const KeyCode mapped_key = from_glfw_key(key);
        const KeyAction mapped_action = from_glfw_action(action);
        const KeyMod mapped_mods = from_glfw_mods(mods);
        m_state.update_key(mapped_key, mapped_action, mapped_mods);

        if (mapped_action == KeyAction::UNKNOWN) {
            return;
        }
        m_key_event.execute(mapped_key, mapped_action, mapped_mods);
    }

    void handle_mouse_button_raw(int button, int action, int mods) {
        if (is_intercept_capture(true)) {
            return;
        }

        const MouseButton mapped_button = from_glfw_button(button);
        const KeyAction mapped_action = from_glfw_action(action);
        const KeyMod mapped_mods = from_glfw_mods(mods);
        m_state.update_mouse_button(mapped_button, mapped_action, mapped_mods);

        if (mapped_action == KeyAction::UNKNOWN) {
            return;
        }
        m_mouse_button_event.execute(mapped_button, mapped_action, mapped_mods);
    }

    void handle_mouse_move_raw(double x, double y) {
        if (is_intercept_capture(true)) {
            return;
        }

        m_state.update_mouse_position(x, y);
        m_mouse_move_event.execute(x, y);
    }

    void handle_mouse_scroll_raw(double x, double y) {
        if (is_intercept_capture(true)) {
            return;
        }

        m_state.update_mouse_scroll(x, y);
        m_mouse_scroll_event.execute(x, y);
    }

private:
    static RawEventSource make_window_source(rhi::Window* window) {
        if (!window) {
            return RawEventSource{};
        }

        return RawEventSource{
            .key_event = &window->key_event(),
            .mouse_button_event = &window->mouse_button_event(),
            .mouse_move_event = &window->mouse_move_event(),
            .mouse_scroll_event = &window->mouse_scroll_event(),
        };
    }

    bool is_intercept_capture(bool is_mouse) const {
        return m_is_intercept_capture ? m_is_intercept_capture(is_mouse) : false;
    }

    void attach(const RawEventSource& source) {
        detach();
        m_source = source;

        if (m_source.key_event) {
            m_key_handle = m_source.key_event->add([this](int key, int action, int mods) {
                handle_key_raw(key, action, mods);
            });
        }

        if (m_source.mouse_button_event) {
            m_mouse_button_handle = m_source.mouse_button_event->add([this](int button, int action, int mods) {
                handle_mouse_button_raw(button, action, mods);
            });
        }

        if (m_source.mouse_move_event) {
            m_mouse_move_handle = m_source.mouse_move_event->add([this](double x, double y) {
                handle_mouse_move_raw(x, y);
            });
        }

        if (m_source.mouse_scroll_event) {
            m_mouse_scroll_handle = m_source.mouse_scroll_event->add([this](double x, double y) {
                handle_mouse_scroll_raw(x, y);
            });
        }
    }

    void detach() {
        if (m_source.key_event && m_key_handle != 0) {
            m_source.key_event->remove(m_key_handle);
        }
        if (m_source.mouse_button_event && m_mouse_button_handle != 0) {
            m_source.mouse_button_event->remove(m_mouse_button_handle);
        }
        if (m_source.mouse_move_event && m_mouse_move_handle != 0) {
            m_source.mouse_move_event->remove(m_mouse_move_handle);
        }
        if (m_source.mouse_scroll_event && m_mouse_scroll_handle != 0) {
            m_source.mouse_scroll_event->remove(m_mouse_scroll_handle);
        }

        m_source = RawEventSource{};
        m_key_handle = 0;
        m_mouse_button_handle = 0;
        m_mouse_move_handle = 0;
        m_mouse_scroll_handle = 0;
    }
};

} // namespace rtr::system::input
