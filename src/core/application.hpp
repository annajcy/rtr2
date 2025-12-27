#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "window.hpp"
#include "context.hpp"
#include "device.hpp"
#include "command.hpp"
#include "renderer.hpp"
#include "render_pipeline.hpp"

#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vulkan_structs.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace rtr::core {

constexpr uint32_t WIDTH = 800;
constexpr uint32_t HEIGHT = 600;

constexpr uint32_t MAX_FRAMES_IN_FLIGHT = 2;

//const std::string shader_output_dir = "C:\\Users\\annaj\\Desktop\\codebase\\lightmap_compression\\build\\Debug\\shaders\\compiled\\";
//const std::string shader_output_dir = "/home/annaj/codebase/lightmap_compression/build/Debug/shaders/compiled/";
const std::string shader_output_dir = "/Users/jinceyang/Desktop/codebase/graphics/rtr2/build/Debug/shaders/compiled/";
const std::string vertex_shader_filename = "vert_buffer_vert.spv";
const std::string fragment_shader_filename = "vert_buffer_frag.spv";

const std::string model_path = "assets/models/stanford_bunny.obj";

class Application {
private:
    std::unique_ptr<Window> m_window{};
    std::unique_ptr<Context> m_context{};
    std::unique_ptr<Device> m_device{};
    
    // Renderer now manages swapchain, command pool, and sync objects
    std::unique_ptr<Renderer> m_renderer{};
    std::unique_ptr<RenderPipeline> m_render_pipeline{};

public:
    Application() {
        m_window = std::make_unique<Window>(WIDTH, HEIGHT, "RTR Application");
        m_window->set_user_pointer(this);
        m_window->set_framebuffer_size_callback(framebuffer_resize_callback);

        m_context = std::make_unique<Context>(m_window.get());
        m_device = std::make_unique<Device>(m_context.get());
        
        // Create renderer (manages swapchain, command buffers, sync)
        m_renderer = std::make_unique<Renderer>(
            m_device.get(),
            m_window.get(),
            MAX_FRAMES_IN_FLIGHT
        );

        // Build render pipeline (owns shaders/buffers/descriptor sets/pipeline state)
        m_render_pipeline = std::make_unique<RenderPipeline>(m_device.get(), m_renderer.get());
        m_render_pipeline->initialize(
            shader_output_dir,
            vertex_shader_filename,
            fragment_shader_filename,
            model_path);
    }

    void run() { loop(); }

    ~Application() = default;

    void loop() {
        while (!m_window->is_should_close()) {
            m_window->poll_events();
            
            // Execute linear pipeline
            m_renderer->draw_frame([this](FrameContext& ctx) {
                this->m_render_pipeline->execute_frame(ctx);
            });
        }
        
        m_device->device().waitIdle();
    }

    static void framebuffer_resize_callback(GLFWwindow* window, int width, int height) {
        auto app = reinterpret_cast<Application*>(glfwGetWindowUserPointer(window));
        if (!app) { return; }
        app->m_renderer->on_window_resized(width, height);
    }
};

} // namespace rtr
