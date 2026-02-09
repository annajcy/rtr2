#pragma once

#include <cstdlib>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#define GLFW_INCLUDE_VULKAN
#include "GLFW/glfw3.h"

#include "vulkan/vulkan_raii.hpp"

namespace rtr::rhi {

class Window {
private:
    int m_width{800};
    int m_height{600};
    std::string m_title{"WindowGLFW"};
    GLFWwindow* m_window{nullptr};

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

    void poll_events() const {
        glfwPollEvents();
    }

    void wait_events() const {
        glfwWaitEvents();
    }

    void set_user_pointer(void* pointer) {
        glfwSetWindowUserPointer(m_window, pointer);
    }

    void set_framebuffer_size_callback(GLFWframebuffersizefun callback) {
        glfwSetFramebufferSizeCallback(m_window, callback);
    }

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
};

} // namespace rtr::rhi
