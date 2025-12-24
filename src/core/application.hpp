#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <chrono>

#include "vk/vk_memory_buffer.hpp"
#include "vk/vk_surface.hpp"
#include "vk/vk_physical_device_picker.hpp"
#include "vk/vk_device.hpp"
#include "vk/vk_swapchain.hpp"

#include "utils/file_loder.hpp"

#include "window.hpp"
#include "context.hpp"

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

struct FrameSynchronizationObjects {
    vk::raii::Semaphore image_available_semaphore{nullptr};
    vk::raii::Semaphore render_finished_semaphore{nullptr};
    vk::raii::Fence submit_fence{nullptr};
    vk::raii::Fence present_fence{nullptr};
};

struct UniformBufferObject {
    alignas(16) glm::mat4 model;
    alignas(16) glm::mat4 view;
    alignas(16) glm::mat4 proj;

    static vk::DescriptorSetLayoutBinding get_descriptor_set_layout_binding(uint32_t binding = 0) {
        vk::DescriptorSetLayoutBinding layout_binding{};
        layout_binding.binding = binding;
        layout_binding.descriptorType = vk::DescriptorType::eUniformBuffer;
        layout_binding.descriptorCount = 1;
        layout_binding.stageFlags = vk::ShaderStageFlagBits::eVertex;
        layout_binding.pImmutableSamplers = nullptr;
        return layout_binding;
    }
};

struct Vertex {
    glm::vec2 pos;
    glm::vec3 color;

    static vk::VertexInputBindingDescription get_binding_description() {
        vk::VertexInputBindingDescription binding_description{};
        binding_description.binding = 0;
        binding_description.stride = sizeof(Vertex);
        binding_description.inputRate = vk::VertexInputRate::eVertex;
        return binding_description;
    }

    static std::array<vk::VertexInputAttributeDescription, 2> get_attribute_descriptions() {
        std::array<vk::VertexInputAttributeDescription, 2> attribute_descriptions{};

        attribute_descriptions[0].binding = 0;
        attribute_descriptions[0].location = 0;
        attribute_descriptions[0].format = vk::Format::eR32G32Sfloat;
        attribute_descriptions[0].offset = offsetof(Vertex, pos);

        attribute_descriptions[1].binding = 0;
        attribute_descriptions[1].location = 1;
        attribute_descriptions[1].format = vk::Format::eR32G32B32Sfloat;
        attribute_descriptions[1].offset = offsetof(Vertex, color);

        return attribute_descriptions;
    }
};

const std::vector<Vertex> vertices = {
    {{-0.5f, -0.5f}, {1.0f, 0.0f, 0.0f}},
    {{0.5f, -0.5f}, {0.0f, 1.0f, 0.0f}},
    {{0.5f, 0.5f}, {0.0f, 0.0f, 1.0f}},
    {{-0.5f, 0.5f}, {1.0f, 1.0f, 1.0f}}
};

const std::vector<uint16_t> indices = {0, 1, 2, 2, 3, 0};

class Application {
private:
    std::unique_ptr<Window> m_window{};
    std::unique_ptr<Context> m_context{};

    vk::raii::PhysicalDevice m_physical_device{nullptr};
    vk::raii::Device m_device{nullptr};
    vk::raii::Queue m_queue{nullptr};

    vk::raii::SwapchainKHR m_swapchain{nullptr};
    std::vector<vk::Image> m_swapchain_images{};
    std::vector<vk::raii::ImageView> m_swapchain_image_views{};
    vk::Extent2D m_swapchain_extent{};
    vk::Format m_swapchain_image_format{};

    vk::raii::CommandPool m_command_pool{nullptr};
    std::vector<vk::raii::CommandBuffer> m_command_buffers{};

    vk::raii::ShaderModule m_vertex_shader_module{nullptr};
    vk::raii::ShaderModule m_fragment_shader_module{nullptr};
    vk::raii::DescriptorSetLayout m_descriptor_set_layout{nullptr};
    vk::raii::PipelineLayout m_pipeline_layout{nullptr};
    vk::raii::Pipeline m_pipeline{nullptr};

    vk::raii::Buffer m_vertex_buffer{nullptr};
    vk::raii::DeviceMemory m_vertex_buffer_memory{nullptr};
    vk::raii::Buffer m_index_buffer{nullptr};
    vk::raii::DeviceMemory m_index_buffer_memory{nullptr};

    vk::raii::DescriptorPool m_descriptor_pool{nullptr};
    std::vector<vk::raii::Buffer> m_uniform_buffers{};
    std::vector<vk::raii::DeviceMemory> m_uniform_buffers_memory{};
    std::vector<void*> m_uniform_buffers_mapped_ptr{};
    std::vector<vk::raii::DescriptorSet> m_descriptor_sets{};

    std::vector<FrameSynchronizationObjects> m_frame_sync_objects{};

    bool m_framebuffer_resized = false;
    uint32_t m_current_frame = 0;
    int m_queue_family_index{-1};

    std::vector<std::string> m_required_device_extensions = {
        vk::KHRSwapchainExtensionName,
        vk::KHRSpirv14ExtensionName,
        vk::KHRSynchronization2ExtensionName,
        vk::KHRCreateRenderpass2ExtensionName,
        "VK_EXT_swapchain_maintenance1",
#if defined(__APPLE__)
        "VK_KHR_portability_subset",
        "VK_KHR_dynamic_rendering"
#endif
    };

    rtr::VKPhysicalDevicePickerApiVersionRule m_api_version_rule{
#if defined(__APPLE__)
        vk::ApiVersion12
#else
        vk::ApiVersion13
#endif
    };

    rtr::VKPhysicalDevicePickerDeviceExtensionRule m_device_extension_rule{
        m_required_device_extensions
    };

    rtr::VKPhysicalDevicePickerQueueBitsChecker m_queue_bits_checker{
        vk::QueueFlagBits::eGraphics | vk::QueueFlagBits::eCompute | vk::QueueFlagBits::eTransfer
    };

#if defined(__APPLE__)
    rtr::VKPhysicalDevicePickerFeatureRule<
        vk::PhysicalDeviceFeatures2,
        vk::PhysicalDeviceDynamicRenderingFeatures,
        vk::PhysicalDeviceSynchronization2Features,
        vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT,
        vk::PhysicalDeviceVulkan11Features,
        vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT
    >m_feature_rule{[](const auto &feature_chain) {
            
            const auto &dynamic_rendering_features = feature_chain.template get<vk::PhysicalDeviceDynamicRenderingFeatures>();
            if (!dynamic_rendering_features.dynamicRendering) {
                std::cout << "PhysicalDeviceDynamicRenderingFeatures.dynamicRendering not supported" << std::endl;
                return false;
            }
            const auto &dynamic_state_features = feature_chain.template get<vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT>();
            if (!dynamic_state_features.extendedDynamicState) {
                std::cout << "PhysicalDeviceExtendedDynamicStateFeaturesEXT.extendedDynamicState not supported" << std::endl;
                return false;
            }
            const auto &synchronization2_features = feature_chain.template get<vk::PhysicalDeviceSynchronization2Features>();
            if (!synchronization2_features.synchronization2) {
                std::cout << "PhysicalDeviceSynchronization2Features.synchronization2 not supported" << std::endl;
                return false;
            }
            const auto &vulkan11_features = feature_chain.template get<vk::PhysicalDeviceVulkan11Features>();
            if (!vulkan11_features.shaderDrawParameters) {
                std::cout << "PhysicalDeviceVulkan11Features.shaderDrawParameters not supported" << std::endl;
                return false;
            }
            const auto &swapchain_maintenance_features = feature_chain.template get<vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT>();
            if (!swapchain_maintenance_features.swapchainMaintenance1) {
                std::cout << "PhysicalDeviceSwapchainMaintenance1FeaturesEXT.swapchainMaintenance1 not supported" << std::endl;
                return false;
            }
            return true;
        }};
#else
    rtr::VKPhysicalDevicePickerFeatureRule<
        vk::PhysicalDeviceFeatures2,
        vk::PhysicalDeviceVulkan11Features,
        vk::PhysicalDeviceVulkan13Features,
        vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT,
        vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT>
        m_feature_rule{[](const auto &feature_chain) {
            const auto &vulkan13_features = feature_chain.template get<vk::PhysicalDeviceVulkan13Features>();
            if (!vulkan13_features.dynamicRendering) {
                std::cout << "PhysicalDeviceVulkan13Features.dynamicRendering not supported" << std::endl;
                return false;
            }
            if (!vulkan13_features.synchronization2) {
                std::cout << "PhysicalDeviceVulkan13Features.synchronization2 not supported" << std::endl;
                return false;
            }
            const auto &dynamic_state_features = feature_chain.template get<vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT>();
            if (!dynamic_state_features.extendedDynamicState) {
                std::cout << "PhysicalDeviceExtendedDynamicStateFeaturesEXT.extendedDynamicState not supported" << std::endl;
                return false;
            }
            const auto &vulkan11_features = feature_chain.template get<vk::PhysicalDeviceVulkan11Features>();
            if (!vulkan11_features.shaderDrawParameters) {
                std::cout << "PhysicalDeviceVulkan11Features.shaderDrawParameters not supported" << std::endl;
                return false;
            }
            const auto &swapchain_maintenance_features = feature_chain.template get<vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT>();
            if (!swapchain_maintenance_features.swapchainMaintenance1) {
                std::cout << "PhysicalDeviceSwapchainMaintenance1FeaturesEXT.swapchainMaintenance1 not supported" << std::endl;
                return false;
            }
            return true;
        }};
#endif

#if defined(__APPLE__)
    using DeviceFeatureChainType = vk::StructureChain<
        vk::PhysicalDeviceFeatures2,
        vk::PhysicalDeviceDynamicRenderingFeatures,
        vk::PhysicalDeviceSynchronization2Features,
        vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT,
        vk::PhysicalDeviceVulkan11Features,
        vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT>;

    std::function<DeviceFeatureChainType()> m_device_feature_chain_generator = []() {
        vk::PhysicalDeviceDynamicRenderingFeatures dynamic_rendering_features{};
        dynamic_rendering_features.dynamicRendering = true;

        vk::PhysicalDeviceSynchronization2Features synchronization2_features{};
        synchronization2_features.synchronization2 = true;

        vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT extended_dynamic_state_features{};
        extended_dynamic_state_features.extendedDynamicState = true;

        vk::PhysicalDeviceVulkan11Features vulkan11_features{};
        vulkan11_features.shaderDrawParameters = true;

        vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT swapchain_maintenance_features{};
        swapchain_maintenance_features.swapchainMaintenance1 = true;

        vk::PhysicalDeviceFeatures2 physical_device_features2{};

        return DeviceFeatureChainType{
            physical_device_features2,
            dynamic_rendering_features,
            synchronization2_features,
            extended_dynamic_state_features,
            vulkan11_features,
            swapchain_maintenance_features};
    };
#else
    using DeviceFeatureChainType = vk::StructureChain<
        vk::PhysicalDeviceFeatures2,
        vk::PhysicalDeviceVulkan11Features,
        vk::PhysicalDeviceVulkan13Features,
        vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT,
        vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT>;

    std::function<DeviceFeatureChainType()> m_device_feature_chain_generator = []() {
        vk::PhysicalDeviceVulkan13Features vulkan13_features{};
        vulkan13_features.dynamicRendering = true;
        vulkan13_features.synchronization2 = true;

        vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT extended_dynamic_state_features{};
        extended_dynamic_state_features.extendedDynamicState = true;

        vk::PhysicalDeviceVulkan11Features vulkan11_features{};
        vulkan11_features.shaderDrawParameters = true;

        vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT swapchain_maintenance_features{};
        swapchain_maintenance_features.swapchainMaintenance1 = true;

        vk::PhysicalDeviceFeatures2 physical_device_features2{};

        return DeviceFeatureChainType{
            physical_device_features2,
            vulkan11_features,
            vulkan13_features,
            extended_dynamic_state_features,
            swapchain_maintenance_features};
    };
#endif

public:
    Application() {
        m_window = std::make_unique<Window>(
            WIDTH,
            HEIGHT,
            "RTR Application"
        );

        m_context = std::make_unique<Context>(
            m_window.get(),
#if !defined(NDEBUG)
            true
#endif
        );

        pick_physical_device();
        create_device();
        create_queue();
        create_swapchain();
        create_shader_modules();
        create_descriptor_set_layout();
        create_pipeline();
        create_buffers();
        create_uniform_buffers();
        create_descriptor_pool();
        create_descriptor_sets();
        create_command_pool();
        create_frame_command_buffers();
        create_sync_objects();
    }

    void run() { loop(); }

    ~Application() = default;

private:
    void loop() {
        while (!m_context->window().is_should_close()) {
            m_context->window().poll_events();
            draw_frame();
        }
        
        m_device.waitIdle();
        
    }

    struct TransitionImageLayoutInfo {
        vk::ImageLayout layout;
        vk::AccessFlagBits2 access_mask;
        vk::PipelineStageFlagBits2 stage;
    };

    void record_command_buffer(const vk::raii::CommandBuffer &command_buffer, const vk::raii::DescriptorSet &descriptor_set, uint32_t image_index) {
        vk::CommandBufferBeginInfo begin_info{};
        begin_info.flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit;
        command_buffer.begin(begin_info);

        TransitionImageLayoutInfo old_layout_info{
            vk::ImageLayout::eUndefined,
            vk::AccessFlagBits2::eNone,
            vk::PipelineStageFlagBits2::eTopOfPipe};

        TransitionImageLayoutInfo new_layout_info{
            vk::ImageLayout::eColorAttachmentOptimal,
            vk::AccessFlagBits2::eColorAttachmentWrite,
            vk::PipelineStageFlagBits2::eColorAttachmentOutput};

        transition_image_layout(
            command_buffer,
            m_swapchain_images.at(image_index),
            old_layout_info,
            new_layout_info);

        //vk::ClearValue clear_value = vk::ClearValue{vk::ClearColorValue{0.0f, 0.0f, 0.0f, 1.0f}};
        vk::ClearValue clear_value = vk::ClearValue{vk::ClearColorValue{1.0f, 0.0f, 1.0f, 1.0f}}; 
        vk::RenderingAttachmentInfo color_attachment_info{};
        color_attachment_info.imageView = *m_swapchain_image_views.at(image_index);
        color_attachment_info.imageLayout = vk::ImageLayout::eColorAttachmentOptimal;
        color_attachment_info.loadOp = vk::AttachmentLoadOp::eClear;
        color_attachment_info.storeOp = vk::AttachmentStoreOp::eStore;
        color_attachment_info.clearValue = clear_value;

        vk::RenderingInfo rendering_info{};
        rendering_info.renderArea.offset = vk::Offset2D{0, 0};
        rendering_info.renderArea.extent = m_swapchain_extent;
        rendering_info.layerCount = 1;
        rendering_info.colorAttachmentCount = 1;
        rendering_info.pColorAttachments = &color_attachment_info;

        command_buffer.beginRendering(rendering_info);

        command_buffer.bindPipeline(
            vk::PipelineBindPoint::eGraphics,
            *m_pipeline);

        vk::Buffer vertex_buffers[] = {*m_vertex_buffer};
        vk::DeviceSize offsets[] = {0};
        command_buffer.bindVertexBuffers(0, vertex_buffers, offsets);

        command_buffer.bindIndexBuffer(
            *m_index_buffer,
            0,
            vk::IndexType::eUint16);

        command_buffer.bindDescriptorSets(
            vk::PipelineBindPoint::eGraphics,
            *m_pipeline_layout,
            0,
            *descriptor_set,
            {});

        vk::Viewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = static_cast<float>(m_swapchain_extent.width);
        viewport.height = static_cast<float>(m_swapchain_extent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        command_buffer.setViewport(0, viewport);

        vk::Rect2D scissor{};
        scissor.offset = vk::Offset2D{0, 0};
        scissor.extent = m_swapchain_extent;
        command_buffer.setScissor(0, scissor);

        command_buffer.drawIndexed(
            static_cast<uint32_t>(indices.size()),
            1,
            0,
            0,
            0);

        command_buffer.endRendering();

        old_layout_info.layout = vk::ImageLayout::eColorAttachmentOptimal;
        old_layout_info.access_mask = vk::AccessFlagBits2::eColorAttachmentWrite;
        old_layout_info.stage = vk::PipelineStageFlagBits2::eColorAttachmentOutput;

        new_layout_info.layout = vk::ImageLayout::ePresentSrcKHR;
        new_layout_info.access_mask = vk::AccessFlagBits2::eNone;
        new_layout_info.stage = vk::PipelineStageFlagBits2::eBottomOfPipe;

        transition_image_layout(
            command_buffer,
            m_swapchain_images.at(image_index),
            old_layout_info,
            new_layout_info);

        command_buffer.end();
    }

    void transition_image_layout(
        const vk::raii::CommandBuffer &command_buffer,
        const vk::Image &image,
        const TransitionImageLayoutInfo &old_layout_info,
        const TransitionImageLayoutInfo &new_layout_info) {
        vk::ImageMemoryBarrier2 image_memory_barrier{};
        image_memory_barrier.srcStageMask = old_layout_info.stage;
        image_memory_barrier.dstStageMask = new_layout_info.stage;
        image_memory_barrier.srcAccessMask = old_layout_info.access_mask;
        image_memory_barrier.dstAccessMask = new_layout_info.access_mask;
        image_memory_barrier.oldLayout = old_layout_info.layout;
        image_memory_barrier.newLayout = new_layout_info.layout;
        image_memory_barrier.image = image;
        image_memory_barrier.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
        image_memory_barrier.subresourceRange.baseMipLevel = 0;
        image_memory_barrier.subresourceRange.levelCount = 1;
        image_memory_barrier.subresourceRange.baseArrayLayer = 0;
        image_memory_barrier.subresourceRange.layerCount = 1;

        vk::DependencyInfo dependency_info{};
        dependency_info.imageMemoryBarrierCount = 1;
        dependency_info.pImageMemoryBarriers = &image_memory_barrier;

        command_buffer.pipelineBarrier2(dependency_info);
    }

    void update_uniform_buffer(void* mapped_ptr) {
        static auto start_time = std::chrono::high_resolution_clock::now();

        auto current_time = std::chrono::high_resolution_clock::now();
        float time = std::chrono::duration<float, std::chrono::seconds::period>(current_time - start_time).count();

        UniformBufferObject ubo{};
        ubo.model = glm::rotate(
            glm::mat4(1.0f),
            time * glm::radians(90.0f),
            glm::vec3(0.0f, 0.0f, 1.0f));
        ubo.view = glm::lookAt(
            glm::vec3(2.0f, 2.0f, 2.0f),
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 0.0f, 1.0f));
        ubo.proj = glm::perspective(
            glm::radians(45.0f),
            static_cast<float>(m_swapchain_extent.width) / static_cast<float>(m_swapchain_extent.height),
            0.1f,
            10.0f);
        ubo.proj[1][1] *= -1;

        std::memcpy(mapped_ptr, &ubo, sizeof(ubo));
    }

    void draw_frame() {
        auto &current_sync_objects = m_frame_sync_objects[m_current_frame];

        //update uniform buffer
        auto &uniform_buffer_mapped_ptr = m_uniform_buffers_mapped_ptr[m_current_frame];
        update_uniform_buffer(uniform_buffer_mapped_ptr);

        // acquire image
        std::array<vk::Fence, 2> wait_fences = {
            *current_sync_objects.submit_fence,
            *current_sync_objects.present_fence};

        if (vk::Result::eTimeout == m_device.waitForFences(
                                           wait_fences,
                                           true,
                                           UINT64_MAX)) {
            std::cout << "Wait for fence timed out." << std::endl;
        }

        auto [result, image_index] = m_swapchain.acquireNextImage(
            UINT64_MAX,
            *current_sync_objects.image_available_semaphore,
            nullptr);

        if (result == vk::Result::eErrorOutOfDateKHR) {
            m_framebuffer_resized = false;
            recreate_swapchain();
            return;
        }

        if (result != vk::Result::eSuccess && result != vk::Result::eSuboptimalKHR) {
            throw std::runtime_error("Failed to acquire swapchain image.");
        }

        bool recreate_required = (result == vk::Result::eSuboptimalKHR);

        //record command buffer
        auto &current_descriptor_set = m_descriptor_sets.at(m_current_frame);
        auto &command_buffer = m_command_buffers.at(m_current_frame);
        command_buffer.reset();

        record_command_buffer(
            command_buffer,
            current_descriptor_set,
            image_index
        );

        m_device.resetFences(wait_fences);

        vk::PipelineStageFlags wait_dst_stage_flag = vk::PipelineStageFlagBits::eColorAttachmentOutput;

        vk::Semaphore wait_semaphores[] = {
            *current_sync_objects.image_available_semaphore};

        vk::CommandBuffer command_buffers[] = {
            *command_buffer};

        vk::Semaphore signal_semaphores[] = {
            *current_sync_objects.render_finished_semaphore};

        vk::SubmitInfo submit_info{};
        submit_info.waitSemaphoreCount = 1;
        submit_info.pWaitSemaphores = wait_semaphores;
        submit_info.pWaitDstStageMask = &wait_dst_stage_flag;
        submit_info.commandBufferCount = 1;
        submit_info.pCommandBuffers = command_buffers;
        submit_info.signalSemaphoreCount = 1;
        submit_info.pSignalSemaphores = signal_semaphores;

        m_queue.submit(submit_info, *current_sync_objects.submit_fence);

        // present image
        vk::Fence present_fences[] = {
            *current_sync_objects.present_fence};

        vk::SwapchainPresentFenceInfoEXT present_fence_info{};
        present_fence_info.swapchainCount = 1;
        present_fence_info.pFences = present_fences;

        vk::SwapchainKHR swapchains[] = {
            *m_swapchain};

        vk::PresentInfoKHR present_info{};
        present_info.waitSemaphoreCount = 1;
        present_info.pWaitSemaphores = signal_semaphores;
        present_info.swapchainCount = 1;
        present_info.pSwapchains = swapchains;
        present_info.pImageIndices = &image_index;

        vk::StructureChain<vk::PresentInfoKHR, vk::SwapchainPresentFenceInfoEXT> present_info_chain{
            present_info,
            present_fence_info
        };

        vk::Result present_result = vk::Result::eSuccess;
        try {
            present_result = m_queue.presentKHR(
                present_info_chain.get<vk::PresentInfoKHR>());
        } catch (const vk::OutOfDateKHRError &) {
            present_result = vk::Result::eErrorOutOfDateKHR;
        }

        if (present_result == vk::Result::eErrorOutOfDateKHR || present_result == vk::Result::eSuboptimalKHR) {
            recreate_required = true;
            if (present_result == vk::Result::eErrorOutOfDateKHR) {
                std::cout << "Swapchain out of date during presentation." << std::endl;
            } else {
                std::cout << "Swapchain suboptimal during presentation." << std::endl;
            }
        } else if (present_result != vk::Result::eSuccess) {
            throw std::runtime_error("Failed to present swapchain image.");
        }

        if (recreate_required || m_framebuffer_resized) {
            m_framebuffer_resized = false;
            recreate_swapchain();
        }

        m_current_frame = (m_current_frame + 1) % MAX_FRAMES_IN_FLIGHT;
    }

    void pick_physical_device() {
        rtr::VKPhysicalDevicePickerQueuePresentChecker queue_present_checker{
            m_context->surface()
        };

        rtr::VKPhysicalDevicePickerQueueRule queue_rule{
            m_queue_family_index,
            {std::cref(m_queue_bits_checker), std::cref(queue_present_checker)}};

        auto physical_devices = m_context->instance().enumeratePhysicalDevices();
        if (auto _physical_device = rtr::pick_physical_device(
                physical_devices,
                m_api_version_rule,
                queue_rule,
                m_device_extension_rule,
                m_feature_rule)) {
            m_physical_device = std::move(_physical_device.value());
            std::cout << "Physical device selected as: " << m_physical_device.getProperties().deviceName << std::endl;
        } else {
            throw std::runtime_error("Failed to select a suitable physical device.");
        }
    }

    void create_device() {
        auto device_result = rtr::make_device(
            m_physical_device,
            m_required_device_extensions,
            m_device_feature_chain_generator(),
            static_cast<uint32_t>(m_queue_family_index));

        if (!device_result.has_value()) {
            throw std::runtime_error("Failed to create logical device.");
        }

        m_device = std::move(device_result.value());
    }

    void create_queue() {
        m_queue = m_device.getQueue(
            static_cast<uint32_t>(m_queue_family_index),
            0);
    }

    void create_swapchain() {
        auto surface_formats = m_physical_device.getSurfaceFormatsKHR(*m_context->surface());
        auto surface_format = rtr::select_surface_format(
            surface_formats,
            [](vk::SurfaceFormatKHR format) {
                return format.format == vk::Format::eB8G8R8A8Srgb &&
                       format.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear;
            });

        auto present_mode = rtr::select_present_mode(
            m_physical_device.getSurfacePresentModesKHR(*m_context->surface()),
            vk::PresentModeKHR::eMailbox);

        auto [extent_x, extent_y] = m_context->window().framebuffer_size();

        auto capabilities = m_physical_device.getSurfaceCapabilitiesKHR(*m_context->surface());
        auto [extent, image_count] = rtr::select_swapchain_image_property(
            capabilities,
            static_cast<uint32_t>(extent_x),
            static_cast<uint32_t>(extent_y),
            3);

        std::cout << image_count << " swapchain images will be created with extent ("
                  << extent.width << ", " << extent.height << ")." << std::endl;

        vk::SwapchainCreateInfoKHR swapchain_create_info{};
        swapchain_create_info.surface = *m_context->surface();
        swapchain_create_info.minImageCount = image_count;
        swapchain_create_info.imageFormat = surface_format.format;
        swapchain_create_info.imageColorSpace = surface_format.colorSpace;
        swapchain_create_info.imageExtent = extent;
        swapchain_create_info.imageArrayLayers = 1;
        swapchain_create_info.imageUsage = vk::ImageUsageFlagBits::eColorAttachment;
        swapchain_create_info.imageSharingMode = vk::SharingMode::eExclusive;
        swapchain_create_info.preTransform = capabilities.currentTransform;
        swapchain_create_info.compositeAlpha = vk::CompositeAlphaFlagBitsKHR::eOpaque;
        swapchain_create_info.presentMode = present_mode;
        swapchain_create_info.clipped = true;

        auto swapchain_result = rtr::make_swapchain_with_image_views(
            m_device,
            swapchain_create_info);

        if (!swapchain_result.has_value()) {
            throw std::runtime_error("Failed to create swapchain.");
        }

        m_swapchain = std::move(swapchain_result->first);
        m_swapchain_image_views = std::move(swapchain_result->second);
        m_swapchain_images = m_swapchain.getImages();
        m_swapchain_extent = extent;
        m_swapchain_image_format = surface_format.format;
    }

    void cleanup_swapchain() {
        m_swapchain_image_views.clear();
        m_swapchain_images.clear();
        m_swapchain.clear();
    }

    void recreate_swapchain() {
        auto [width, height] = m_context->window().framebuffer_size();
        while (width == 0 || height == 0) {
            m_context->window().wait_events();
            std::tie(width, height) = m_context->window().framebuffer_size();
        }

        m_device.waitIdle();

        cleanup_swapchain();
        create_swapchain();
    }

    void create_shader_modules() {
        auto vertex_shader_code = rtr::utils::read_file(shader_output_dir + vertex_shader_filename);
        auto fragment_shader_code = rtr::utils::read_file(shader_output_dir + fragment_shader_filename);

        vk::ShaderModuleCreateInfo vertex_shader_module_create_info{};
        vertex_shader_module_create_info.codeSize = vertex_shader_code.size();
        vertex_shader_module_create_info.pCode = reinterpret_cast<const uint32_t *>(vertex_shader_code.data());

        vk::ShaderModuleCreateInfo fragment_shader_module_create_info{};
        fragment_shader_module_create_info.codeSize = fragment_shader_code.size();
        fragment_shader_module_create_info.pCode = reinterpret_cast<const uint32_t *>(fragment_shader_code.data());

        m_vertex_shader_module = vk::raii::ShaderModule{
            m_device,
            vertex_shader_module_create_info
        };

        m_fragment_shader_module = vk::raii::ShaderModule{
            m_device,
            fragment_shader_module_create_info
        };
    }

    void create_descriptor_set_layout() {
        vk::DescriptorSetLayoutBinding ubo_layout_binding = UniformBufferObject::get_descriptor_set_layout_binding(0);

        std::vector<vk::DescriptorSetLayoutBinding> layout_bindings = {
            ubo_layout_binding
        };

        vk::DescriptorSetLayoutCreateInfo layout_create_info{};
        layout_create_info.bindingCount = static_cast<uint32_t>(layout_bindings.size());
        layout_create_info.pBindings = layout_bindings.data();

        m_descriptor_set_layout = vk::raii::DescriptorSetLayout{
            m_device,
            layout_create_info
        };
    }

    void create_uniform_buffers() {
        m_uniform_buffers.clear();
        m_uniform_buffers_memory.clear();
        m_uniform_buffers_mapped_ptr.clear();

        for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; i ++) {
            vk::DeviceSize buffer_size = sizeof(UniformBufferObject);
            if (auto mapped_buffer_with_memory_opt = rtr::make_mapped_buffer_with_memory(
                m_device,
                m_physical_device,
                buffer_size,
                vk::BufferUsageFlagBits::eUniformBuffer, 
                vk::MemoryPropertyFlags {})
            ) {
                auto [uniform_buffer, uniform_buffer_memory, mapped_ptr] = std::move(mapped_buffer_with_memory_opt.value());
                m_uniform_buffers.emplace_back(std::move(uniform_buffer));
                m_uniform_buffers_memory.emplace_back(std::move(uniform_buffer_memory));
                m_uniform_buffers_mapped_ptr.emplace_back(mapped_ptr);
            } else {
                throw std::runtime_error("Failed to create uniform buffer.");
            }
        }
    }

    void create_descriptor_pool() {

        std::vector<vk::DescriptorPoolSize> pool_sizes = {
            {vk::DescriptorType::eUniformBuffer, static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT)}
        };

        vk::DescriptorPoolCreateInfo pool_create_info{};
        pool_create_info.maxSets = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);
        pool_create_info.poolSizeCount = static_cast<uint32_t>(pool_sizes.size());
        pool_create_info.pPoolSizes = pool_sizes.data();
        pool_create_info.flags = vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet;

        m_descriptor_pool = vk::raii::DescriptorPool{
            m_device,
            pool_create_info
        };
    }

    void create_descriptor_sets() {
        std::vector<vk::DescriptorSetLayout> layouts(MAX_FRAMES_IN_FLIGHT, *m_descriptor_set_layout);

        vk::DescriptorSetAllocateInfo alloc_info{};
        alloc_info.descriptorPool = *m_descriptor_pool;
        alloc_info.descriptorSetCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);
        alloc_info.pSetLayouts = layouts.data();

        m_descriptor_sets = m_device.allocateDescriptorSets(alloc_info);

        for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
            vk::DescriptorBufferInfo buffer_info{};
            buffer_info.buffer = *m_uniform_buffers[i];
            buffer_info.offset = 0;
            buffer_info.range = sizeof(UniformBufferObject);

            vk::WriteDescriptorSet descriptor_write{};
            descriptor_write.dstSet = m_descriptor_sets[i];
            descriptor_write.dstBinding = 0;
            descriptor_write.dstArrayElement = 0;
            descriptor_write.descriptorType = vk::DescriptorType::eUniformBuffer;
            descriptor_write.descriptorCount = 1;
            descriptor_write.pBufferInfo = &buffer_info;

            m_device.updateDescriptorSets(
                descriptor_write,
                {}
            );
        }
    }

    void create_pipeline() {
        vk::PipelineShaderStageCreateInfo fragment_shader_stage_info{};
        fragment_shader_stage_info.stage = vk::ShaderStageFlagBits::eFragment;
        fragment_shader_stage_info.module = *m_fragment_shader_module;
        fragment_shader_stage_info.pName = "main";

        vk::PipelineShaderStageCreateInfo vertex_shader_stage_info{};
        vertex_shader_stage_info.stage = vk::ShaderStageFlagBits::eVertex;
        vertex_shader_stage_info.module = *m_vertex_shader_module;
        vertex_shader_stage_info.pName = "main";

        std::vector<vk::PipelineShaderStageCreateInfo> shader_stage_infos = {
            vertex_shader_stage_info,
            fragment_shader_stage_info};

        vk::PipelineVertexInputStateCreateInfo vertex_input_info{};
        auto vertex_binding_description = Vertex::get_binding_description();

        std::vector<vk::VertexInputBindingDescription> binding_descriptions = {
            vertex_binding_description};

        vertex_input_info.vertexBindingDescriptionCount = static_cast<uint32_t>(binding_descriptions.size());
        vertex_input_info.pVertexBindingDescriptions = binding_descriptions.data();

        auto attribute_descriptions = Vertex::get_attribute_descriptions();

        std::vector<vk::VertexInputAttributeDescription> attribute_descs = {
            attribute_descriptions[0],
            attribute_descriptions[1]
        };

        vertex_input_info.vertexAttributeDescriptionCount = static_cast<uint32_t>(attribute_descs.size());
        vertex_input_info.pVertexAttributeDescriptions = attribute_descs.data();

        vk::PipelineInputAssemblyStateCreateInfo input_assembly_info{};
        input_assembly_info.topology = vk::PrimitiveTopology::eTriangleList;

        vk::PipelineViewportStateCreateInfo viewport_info{};
        viewport_info.viewportCount = 1;
        viewport_info.scissorCount = 1;

        vk::PipelineRasterizationStateCreateInfo rasterization_info{};
        rasterization_info.depthClampEnable = VK_FALSE;
        rasterization_info.rasterizerDiscardEnable = VK_FALSE;
        rasterization_info.polygonMode = vk::PolygonMode::eFill;
        rasterization_info.cullMode = vk::CullModeFlagBits::eNone;
        rasterization_info.frontFace = vk::FrontFace::eCounterClockwise;
        rasterization_info.depthBiasEnable = VK_FALSE;
        rasterization_info.lineWidth = 1.0f;

        vk::PipelineMultisampleStateCreateInfo multisample_info{};
        multisample_info.rasterizationSamples = vk::SampleCountFlagBits::e1;

        vk::PipelineColorBlendAttachmentState color_blend_attachment{};
        color_blend_attachment.blendEnable = VK_FALSE;
        color_blend_attachment.colorWriteMask =
            vk::ColorComponentFlagBits::eR |
            vk::ColorComponentFlagBits::eG |
            vk::ColorComponentFlagBits::eB |
            vk::ColorComponentFlagBits::eA;

        std::vector<vk::PipelineColorBlendAttachmentState> color_blend_attachments = {
            color_blend_attachment};

        vk::PipelineColorBlendStateCreateInfo color_blend_state{};
        color_blend_state.logicOpEnable = VK_FALSE;
        color_blend_state.logicOp = vk::LogicOp::eCopy;
        color_blend_state.attachmentCount = static_cast<uint32_t>(color_blend_attachments.size());
        color_blend_state.pAttachments = color_blend_attachments.data();

        std::vector<vk::DynamicState> dynamic_states = {
            vk::DynamicState::eViewport,
            vk::DynamicState::eScissor};

        vk::PipelineDynamicStateCreateInfo dynamic_state_info{};
        dynamic_state_info.dynamicStateCount = static_cast<uint32_t>(dynamic_states.size());
        dynamic_state_info.pDynamicStates = dynamic_states.data();

        vk::PipelineLayoutCreateInfo pipeline_layout_info{};
        std::vector<vk::DescriptorSetLayout> set_layouts = {
            *m_descriptor_set_layout
        };
        pipeline_layout_info.setLayoutCount = static_cast<uint32_t>(set_layouts.size());
        pipeline_layout_info.pSetLayouts = set_layouts.data();

        m_pipeline_layout = vk::raii::PipelineLayout{
            m_device,
            pipeline_layout_info};

        vk::GraphicsPipelineCreateInfo graphics_pipeline_create_info{};
        graphics_pipeline_create_info.stageCount = static_cast<uint32_t>(shader_stage_infos.size());
        graphics_pipeline_create_info.pStages = shader_stage_infos.data();
        graphics_pipeline_create_info.pVertexInputState = &vertex_input_info;
        graphics_pipeline_create_info.pInputAssemblyState = &input_assembly_info;
        graphics_pipeline_create_info.pViewportState = &viewport_info;
        graphics_pipeline_create_info.pRasterizationState = &rasterization_info;
        graphics_pipeline_create_info.pMultisampleState = &multisample_info;
        graphics_pipeline_create_info.pColorBlendState = &color_blend_state;
        graphics_pipeline_create_info.pDynamicState = &dynamic_state_info;
        graphics_pipeline_create_info.layout = *m_pipeline_layout;
        graphics_pipeline_create_info.renderPass = VK_NULL_HANDLE;

        vk::PipelineRenderingCreateInfo pipeline_rendering_info{};

        std::vector<vk::Format> color_attachment_formats = {
            m_swapchain_image_format};

        pipeline_rendering_info.colorAttachmentCount = static_cast<uint32_t>(color_attachment_formats.size());
        pipeline_rendering_info.pColorAttachmentFormats = color_attachment_formats.data();

        vk::StructureChain<
            vk::GraphicsPipelineCreateInfo,
            vk::PipelineRenderingCreateInfo
        > pipeline_info_chain{
            graphics_pipeline_create_info,
            pipeline_rendering_info
        };

        m_pipeline = vk::raii::Pipeline{
            m_device,
            nullptr,
            pipeline_info_chain.get<vk::GraphicsPipelineCreateInfo>()
        };
    }

    void create_buffers() {
        vk::DeviceSize vertex_buffer_size = sizeof(vertices[0]) * vertices.size();
        vk::DeviceSize index_buffer_size = sizeof(indices[0]) * indices.size();
        vk::DeviceSize staging_buffer_size = vertex_buffer_size + index_buffer_size;

        auto staging_buffer = rtr::make_buffer_with_memory(
            m_device,
            m_physical_device,
            staging_buffer_size,
            vk::BufferUsageFlagBits::eTransferSrc,
            vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);

        if (!staging_buffer.has_value()) {
            throw std::runtime_error("Failed to create staging buffer.");
        }

        auto vertex_buffer = rtr::make_buffer_with_memory(
            m_device,
            m_physical_device,
            vertex_buffer_size,
            vk::BufferUsageFlagBits::eVertexBuffer | vk::BufferUsageFlagBits::eTransferDst,
            vk::MemoryPropertyFlagBits::eDeviceLocal);

        if (!vertex_buffer.has_value()) {
            throw std::runtime_error("Failed to create vertex buffer.");
        }

        m_vertex_buffer = std::move(vertex_buffer->first);
        m_vertex_buffer_memory = std::move(vertex_buffer->second);

        auto index_buffer = rtr::make_buffer_with_memory(
            m_device,
            m_physical_device,
            index_buffer_size,
            vk::BufferUsageFlagBits::eIndexBuffer | vk::BufferUsageFlagBits::eTransferDst,
            vk::MemoryPropertyFlagBits::eDeviceLocal);

        if (!index_buffer.has_value()) {
            throw std::runtime_error("Failed to create index buffer.");
        }

        m_index_buffer = std::move(index_buffer->first);
        m_index_buffer_memory = std::move(index_buffer->second);

        rtr::map_memory(
            staging_buffer->second,
            staging_buffer_size,
            0,
            [&](void *data) {
                std::memcpy(
                    data,
                    vertices.data(),
                    static_cast<size_t>(vertex_buffer_size));
                std::memcpy(
                    static_cast<char *>(data) + vertex_buffer_size,
                    indices.data(),
                    static_cast<size_t>(index_buffer_size));
            });

        copy_buffer(
            *staging_buffer->first,
            *m_vertex_buffer,
            vertex_buffer_size);

        copy_buffer(
            *staging_buffer->first,
            *m_index_buffer,
            index_buffer_size,
            vertex_buffer_size,
            0);
    }

    void create_command_pool() {
        vk::CommandPoolCreateInfo command_pool_create_info{};
        command_pool_create_info.flags = vk::CommandPoolCreateFlagBits::eResetCommandBuffer;
        command_pool_create_info.queueFamilyIndex = static_cast<uint32_t>(m_queue_family_index);

        m_command_pool = vk::raii::CommandPool{
            m_device,
            command_pool_create_info};
    }

    void create_frame_command_buffers() {
        vk::CommandBufferAllocateInfo command_buffer_allocate_info{};
        command_buffer_allocate_info.commandPool = *m_command_pool;
        command_buffer_allocate_info.level = vk::CommandBufferLevel::ePrimary;
        command_buffer_allocate_info.commandBufferCount = MAX_FRAMES_IN_FLIGHT;

        auto allocated_buffers = m_device.allocateCommandBuffers(command_buffer_allocate_info);
        m_command_buffers.clear();
        m_command_buffers.reserve(allocated_buffers.size());
        for (auto &buffer : allocated_buffers) {
            m_command_buffers.emplace_back(std::move(buffer));
        }
    }

    void create_sync_objects() {
        m_frame_sync_objects.clear();
        m_frame_sync_objects.resize(MAX_FRAMES_IN_FLIGHT);

        vk::SemaphoreCreateInfo semaphore_create_info{};
        vk::FenceCreateInfo fence_create_info{};
        fence_create_info.flags = vk::FenceCreateFlagBits::eSignaled;

        for (auto &sync_objects : m_frame_sync_objects) {
            sync_objects.image_available_semaphore = vk::raii::Semaphore{
                m_device,
                semaphore_create_info};

            sync_objects.render_finished_semaphore = vk::raii::Semaphore{
                m_device,
                semaphore_create_info};

            sync_objects.submit_fence = vk::raii::Fence{
                m_device,
                fence_create_info};

            sync_objects.present_fence = vk::raii::Fence{
                m_device,
                fence_create_info};
        }
    }

    void copy_buffer(
        vk::Buffer src,
        vk::Buffer dst,
        vk::DeviceSize size,
        vk::DeviceSize src_offset = 0,
        vk::DeviceSize dst_offset = 0) {
        vk::CommandPoolCreateInfo command_pool_info{};
        command_pool_info.flags = vk::CommandPoolCreateFlagBits::eTransient;
        command_pool_info.queueFamilyIndex = static_cast<uint32_t>(m_queue_family_index);

        vk::raii::CommandPool command_pool{
            m_device,
            command_pool_info};

        vk::CommandBufferAllocateInfo command_buffer_allocate_info{};
        command_buffer_allocate_info.commandPool = *command_pool;
        command_buffer_allocate_info.level = vk::CommandBufferLevel::ePrimary;
        command_buffer_allocate_info.commandBufferCount = 1;

        auto command_buffers = m_device.allocateCommandBuffers(command_buffer_allocate_info);
        auto &command_buffer = command_buffers.front();

        vk::CommandBufferBeginInfo begin_info{};
        begin_info.flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit;
        command_buffer.begin(begin_info);

        vk::BufferCopy buffer_copy{};
        buffer_copy.srcOffset = src_offset;
        buffer_copy.dstOffset = dst_offset;
        buffer_copy.size = size;
        command_buffer.copyBuffer(src, dst, buffer_copy);

        command_buffer.end();

        vk::CommandBuffer raw_command_buffer = *command_buffer;

        vk::SubmitInfo submit_info{};
        submit_info.commandBufferCount = 1;
        submit_info.pCommandBuffers = &raw_command_buffer;

        m_queue.submit(submit_info, vk::Fence{});
        m_queue.waitIdle();
    }

    static void framebuffer_resize_callback(GLFWwindow *window, int width, int height) {
        auto app = reinterpret_cast<Application *>(glfwGetWindowUserPointer(window));
        if (!app) {
            return;
        }

        std::cout << "Framebuffer resized to (" << width << ", " << height << ")." << std::endl;

        app->m_framebuffer_resized = true;
    }
};

} // namespace rtr
