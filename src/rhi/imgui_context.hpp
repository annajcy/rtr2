#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_vulkan.h"

#include "rhi/context.hpp"
#include "rhi/device.hpp"
#include "rhi/window.hpp"

namespace rtr::rhi {

class ImGuiContext {
private:
    Device* m_device{};
    Context* m_context{};
    Window* m_window{};

    uint32_t m_image_count{};
    vk::Format m_color_format{vk::Format::eUndefined};
    vk::Format m_depth_format{vk::Format::eUndefined};
    vk::raii::DescriptorPool m_descriptor_pool{nullptr};

    bool m_context_initialized{false};
    bool m_glfw_backend_initialized{false};
    bool m_vulkan_backend_initialized{false};
    bool m_initialized{false};

public:
    ImGuiContext(
        Device* device,
        Context* context,
        Window* window,
        uint32_t image_count,
        vk::Format color_format,
        vk::Format depth_format
    )
        : m_device(device),
          m_context(context),
          m_window(window),
          m_image_count(image_count),
          m_color_format(color_format),
          m_depth_format(depth_format) {
        if (!m_device || !m_context || !m_window) {
            throw std::runtime_error("ImGuiContext requires valid runtime context.");
        }
        if (m_image_count < 2) {
            throw std::runtime_error("ImGuiContext requires image_count >= 2.");
        }

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        m_context_initialized = true;

        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        io.ConfigFlags &= ~ImGuiConfigFlags_ViewportsEnable;
        ImGui::StyleColorsDark();
        setup_fonts();

        create_descriptor_pool();

        if (!ImGui_ImplGlfw_InitForVulkan(m_window->window(), true)) {
            throw std::runtime_error("ImGui_ImplGlfw_InitForVulkan failed.");
        }
        m_glfw_backend_initialized = true;

        init_vulkan_backend();
        m_initialized = true;
    }

    ~ImGuiContext() {
        if (!m_initialized) {
            return;
        }

        m_device->wait_idle();
        if (m_vulkan_backend_initialized) {
            ImGui_ImplVulkan_Shutdown();
            m_vulkan_backend_initialized = false;
        }
        if (m_glfw_backend_initialized) {
            ImGui_ImplGlfw_Shutdown();
            m_glfw_backend_initialized = false;
        }
        if (m_context_initialized) {
            ImGui::DestroyContext();
            m_context_initialized = false;
        }
        m_descriptor_pool.reset();
        m_initialized = false;
    }

    ImGuiContext(const ImGuiContext&) = delete;
    ImGuiContext& operator=(const ImGuiContext&) = delete;

    void begin_frame() {
        if (!m_initialized) {
            return;
        }

        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
    }

    ImDrawData* prepare_draw_data() {
        if (!m_initialized) {
            return nullptr;
        }
        ImGui::Render();
        return ImGui::GetDrawData();
    }

    void render_draw_data(const vk::raii::CommandBuffer& command_buffer, ImDrawData* draw_data) {
        if (!m_initialized || draw_data == nullptr) {
            return;
        }
        ImGui_ImplVulkan_RenderDrawData(draw_data, *command_buffer);
    }

    void on_swapchain_recreated(uint32_t image_count, vk::Format color_format, vk::Format depth_format) {
        if (!m_initialized) {
            return;
        }
        if (image_count < 2) {
            throw std::runtime_error("ImGuiContext requires image_count >= 2.");
        }

        const bool image_count_changed = m_image_count != image_count;
        const bool format_changed = m_color_format != color_format || m_depth_format != depth_format;
        if (!image_count_changed && !format_changed) {
            return;
        }

        m_image_count = image_count;
        m_color_format = color_format;
        m_depth_format = depth_format;

        if (format_changed) {
            m_device->wait_idle();
            if (m_vulkan_backend_initialized) {
                ImGui_ImplVulkan_Shutdown();
                m_vulkan_backend_initialized = false;
            }
            init_vulkan_backend();
            return;
        }

        ImGui_ImplVulkan_SetMinImageCount(m_image_count);
    }

    bool wants_capture_mouse() const {
        if (!m_initialized) {
            return false;
        }
        return ImGui::GetIO().WantCaptureMouse;
    }

    bool wants_capture_keyboard() const {
        if (!m_initialized) {
            return false;
        }
        return ImGui::GetIO().WantCaptureKeyboard;
    }

private:
    void init_vulkan_backend() {
        ImGui_ImplVulkan_InitInfo init_info{};
        init_info.ApiVersion = VK_API_VERSION_1_3;
        init_info.Instance = *m_context->instance();
        init_info.PhysicalDevice = *m_device->physical_device();
        init_info.Device = *m_device->device();
        init_info.QueueFamily = m_device->queue_family_index();
        init_info.Queue = *m_device->queue();
        init_info.PipelineCache = VK_NULL_HANDLE;
        init_info.DescriptorPool = *m_descriptor_pool;
        init_info.Subpass = 0;
        init_info.MinImageCount = m_image_count;
        init_info.ImageCount = m_image_count;
        init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
        init_info.UseDynamicRendering = true;

        vk::PipelineRenderingCreateInfo pipeline_rendering{};
        pipeline_rendering.colorAttachmentCount = 1;
        pipeline_rendering.pColorAttachmentFormats = &m_color_format;
        pipeline_rendering.depthAttachmentFormat = m_depth_format;
        init_info.PipelineRenderingCreateInfo = pipeline_rendering;

        if (!ImGui_ImplVulkan_Init(&init_info)) {
            throw std::runtime_error("ImGui_ImplVulkan_Init failed.");
        }
        m_vulkan_backend_initialized = true;
    }

    void setup_fonts() {
        ImGuiIO& io = ImGui::GetIO();
        ImFontConfig config;
        config.FontNo = 0;
        const char* font_path = "/Users/jinceyang/Desktop/codebase/graphics/rtr2/assets/fonts/Arial.ttf";
        if (!io.Fonts->AddFontFromFileTTF(
                font_path,
                15.0f,
                &config,
                io.Fonts->GetGlyphRangesChineseFull())) {
            throw std::runtime_error("Failed to load font: /Users/jinceyang/Desktop/codebase/graphics/rtr2/assets/fonts/Arial.ttf");
        }
    }

    void create_descriptor_pool() {
        std::array<vk::DescriptorPoolSize, 11> pool_sizes = {{
            {vk::DescriptorType::eSampler, 1000},
            {vk::DescriptorType::eCombinedImageSampler, 1000},
            {vk::DescriptorType::eSampledImage, 1000},
            {vk::DescriptorType::eStorageImage, 1000},
            {vk::DescriptorType::eUniformTexelBuffer, 1000},
            {vk::DescriptorType::eStorageTexelBuffer, 1000},
            {vk::DescriptorType::eUniformBuffer, 1000},
            {vk::DescriptorType::eStorageBuffer, 1000},
            {vk::DescriptorType::eUniformBufferDynamic, 1000},
            {vk::DescriptorType::eStorageBufferDynamic, 1000},
            {vk::DescriptorType::eInputAttachment, 1000},
        }};

        vk::DescriptorPoolCreateInfo pool_info{};
        pool_info.flags = vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet;
        pool_info.maxSets = 1000 * static_cast<uint32_t>(pool_sizes.size());
        pool_info.poolSizeCount = static_cast<uint32_t>(pool_sizes.size());
        pool_info.pPoolSizes = pool_sizes.data();

        m_descriptor_pool = vk::raii::DescriptorPool(m_device->device(), pool_info);
    }
};

} // namespace rtr::rhi
