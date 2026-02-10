#pragma once

#include <cstdlib>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#define GLFW_INCLUDE_VULKAN
#include "GLFW/glfw3.h"

#include "utils/event_center.hpp"
#include "vulkan/vulkan_raii.hpp"

namespace rtr::rhi {

class Window {
public:
    using WindowResizeEvent = utils::Event<int, int>;
    using KeyEvent = utils::Event<int, int, int>;
    using MouseButtonEvent = utils::Event<int, int, int>;
    using MouseMoveEvent = utils::Event<double, double>;
    using MouseScrollEvent = utils::Event<double, double>;

private:
    int m_width{800};
    int m_height{600};
    std::string m_title{"WindowGLFW"};
    GLFWwindow* m_window{nullptr};
    WindowResizeEvent m_window_resize_event{};
    KeyEvent m_key_event{};
    MouseButtonEvent m_mouse_button_event{};
    MouseMoveEvent m_mouse_move_event{};
    MouseScrollEvent m_mouse_scroll_event{};

public:
    Window(int width, int height, const std::string& title)
        : m_width(width), m_height(height), m_title(title) {
        if (!glfwInit()) {
            throw std::runtime_error("Failed to initialize GLFW");
        }

        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

        m_window = glfwCreateWindow(
            m_width,
            m_height,
            m_title.c_str(),
            nullptr,
            nullptr
        );
        if (!m_window) {
            glfwTerminate();
            throw std::runtime_error("Failed to create GLFW window");
        }

        glfwSetWindowUserPointer(m_window, this);
        glfwSetFramebufferSizeCallback(m_window, framebuffer_size_callback);
        glfwSetKeyCallback(m_window, key_callback);
        glfwSetMouseButtonCallback(m_window, mouse_button_callback);
        glfwSetCursorPosCallback(m_window, mouse_move_callback);
        glfwSetScrollCallback(m_window, mouse_scroll_callback);
    }

    ~Window() {
        if (m_window) {
            glfwDestroyWindow(m_window);
        }
        glfwTerminate();
    }

    GLFWwindow* window() const {
        return m_window;
    }

    std::string title() const {
        return m_title;
    }

    std::pair<int, int> framebuffer_size() const {
        int width{};
        int height{};
        glfwGetFramebufferSize(m_window, &width, &height);
        return {width, height};
    }

    const int& width() const {
        return m_width;
    }

    const int& height() const {
        return m_height;
    }

    bool is_should_close() const {
        return glfwWindowShouldClose(m_window);
    }

    void close() const {
        glfwSetWindowShouldClose(m_window, GLFW_TRUE);
    }

    void poll_events() const {
        glfwPollEvents();
    }

    void wait_events() const {
        glfwWaitEvents();
    }

    WindowResizeEvent& window_resize_event() { return m_window_resize_event; }
    KeyEvent& key_event() { return m_key_event; }
    MouseButtonEvent& mouse_button_event() { return m_mouse_button_event; }
    MouseMoveEvent& mouse_move_event() { return m_mouse_move_event; }
    MouseScrollEvent& mouse_scroll_event() { return m_mouse_scroll_event; }

    std::vector<std::string> required_extensions() const {
        uint32_t glfw_extension_count = 0;
        const char** glfw_extensions = glfwGetRequiredInstanceExtensions(&glfw_extension_count);

        std::vector<std::string> extensions;
        extensions.reserve(glfw_extension_count);
        for (uint32_t i = 0; i < glfw_extension_count; i++) {
            extensions.emplace_back(glfw_extensions[i]);
        }

        return extensions;
    }

    std::optional<VkSurfaceKHR> create_vk_surface(const vk::raii::Instance& instance) const {
        VkSurfaceKHR surface{};
        if (glfwCreateWindowSurface(*instance, m_window, nullptr, &surface) != VK_SUCCESS) {
            return std::nullopt;
        }
        return surface;
    }

private:
    static Window* from_glfw_window(GLFWwindow* window) {
        return static_cast<Window*>(glfwGetWindowUserPointer(window));
    }

    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
        (void)scancode;
        auto* self = from_glfw_window(window);
        if (!self) {
            return;
        }
        self->m_key_event.execute(key, action, mods);
    }

    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
        auto* self = from_glfw_window(window);
        if (!self) {
            return;
        }
        self->m_mouse_button_event.execute(button, action, mods);
    }

    static void mouse_move_callback(GLFWwindow* window, double x, double y) {
        auto* self = from_glfw_window(window);
        if (!self) {
            return;
        }
        self->m_mouse_move_event.execute(x, y);
    }

    static void mouse_scroll_callback(GLFWwindow* window, double x, double y) {
        auto* self = from_glfw_window(window);
        if (!self) {
            return;
        }
        self->m_mouse_scroll_event.execute(x, y);
    }

    static void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
        auto* self = from_glfw_window(window);
        if (!self) {
            return;
        }

        self->m_width = width;
        self->m_height = height;
        self->m_window_resize_event.execute(width, height);
    }
};

} // namespace rtr::rhi
