#pragma once

#include <array>
#include <cstdint>

#include "command.hpp"
#include "device.hpp"
#include "renderer.hpp"
#include "window.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_vulkan.h"

namespace rtr::core {

class ImGuiLayer {
public:
    ImGuiLayer(Device* device, Renderer* renderer, Window* window)
        : m_device(device), m_renderer(renderer), m_window(window) {}

    ~ImGuiLayer() { shutdown(); }

    void initialize() {
        if (m_initialized) {
            return;
        }

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGui::StyleColorsDark();

        create_descriptor_pool();

        ImGui_ImplGlfw_InitForVulkan(m_window->window(), true);

        ImGui_ImplVulkan_InitInfo init_info{};
        init_info.ApiVersion = VK_API_VERSION_1_3;
        init_info.Instance = *m_device->context()->instance();
        init_info.PhysicalDevice = *m_device->physical_device();
        init_info.Device = *m_device->device();
        init_info.QueueFamily = m_device->queue_family_index();
        init_info.Queue = *m_device->queue();
        init_info.PipelineCache = VK_NULL_HANDLE;
        init_info.DescriptorPool = *m_descriptor_pool;
        init_info.Subpass = 0;
        init_info.MinImageCount = m_renderer->image_count();
        init_info.ImageCount = m_renderer->image_count();
        init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
        init_info.UseDynamicRendering = true;
        vk::PipelineRenderingCreateInfo pipeline_rendering{};
        const vk::Format color_format = m_renderer->render_format();
        pipeline_rendering.colorAttachmentCount = 1;
        pipeline_rendering.pColorAttachmentFormats = &color_format;
        pipeline_rendering.depthAttachmentFormat = m_renderer->depth_format();
        init_info.PipelineRenderingCreateInfo = pipeline_rendering;

        ImGui_ImplVulkan_Init(&init_info);

        m_last_image_count = m_renderer->image_count();
        m_initialized = true;
    }

    void shutdown() {
        if (!m_initialized) {
            return;
        }

        m_device->device().waitIdle();
        ImGui_ImplVulkan_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        m_descriptor_pool.reset();
        m_initialized = false;
    }

    void begin_frame() {
        if (!m_initialized) {
            return;
        }

        // Update min image count after swapchain recreation
        if (m_renderer->image_count() != m_last_image_count) {
            m_last_image_count = m_renderer->image_count();
            ImGui_ImplVulkan_SetMinImageCount(m_last_image_count);
        }

        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
    }

    void render_draw_data(const vk::raii::CommandBuffer& command_buffer) {
        if (!m_initialized) {
            return;
        }

        ImGui::Render();
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), *command_buffer);
    }

private:
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

    Device* m_device;
    Renderer* m_renderer;
    Window* m_window;
    vk::raii::DescriptorPool m_descriptor_pool{nullptr};
    bool m_initialized{false};
    uint32_t m_last_image_count{0};
};

} // namespace rtr::core
