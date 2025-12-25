#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <chrono>

#include "utils/file_loder.hpp"

#include "window.hpp"
#include "context.hpp"
#include "device.hpp"
#include "swap_chain.hpp"
#include "buffer.hpp"
#include "command_pool.hpp"
#include "descriptor.hpp"
#include "shader_module.hpp"

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
    std::unique_ptr<Device> m_device{};
    std::unique_ptr<SwapChain> m_swapchain{};

    std::unique_ptr<CommandPool> m_command_pool{nullptr};
    std::vector<vk::raii::CommandBuffer> m_command_buffers{};

    std::unique_ptr<Buffer> m_vertex_buffer{nullptr};
    std::unique_ptr<Buffer> m_index_buffer{nullptr};

    std::unique_ptr<ShaderModule> m_vertex_shader_module{nullptr};
    std::unique_ptr<ShaderModule> m_fragment_shader_module{nullptr};

    vk::raii::PipelineLayout m_pipeline_layout{nullptr};
    vk::raii::Pipeline m_pipeline{nullptr};

    std::vector<std::unique_ptr<Buffer>> m_uniform_buffers{};
    std::unique_ptr<DescriptorSetLayout> m_descriptor_set_layout{nullptr};
    std::unique_ptr<DescriptorPool> m_descriptor_pool{nullptr};
    std::vector<vk::raii::DescriptorSet> m_descriptor_sets{};

    std::vector<FrameSynchronizationObjects> m_frame_sync_objects{};

    bool m_framebuffer_resized = false;
    uint32_t m_current_frame = 0;


public:
    Application() {
        m_window = std::make_unique<Window>(WIDTH, HEIGHT, "RTR Application");
        m_window->set_framebuffer_size_callback(framebuffer_resize_callback);

        m_context = std::make_unique<Context>(m_window.get());
        m_device = std::make_unique<Device>(m_context.get());
        m_swapchain = std::make_unique<SwapChain>(m_device.get());

        // set shader modules
        m_vertex_shader_module = std::make_unique<ShaderModule>(
            ShaderModule::from_file(
                m_device.get(),
                shader_output_dir + vertex_shader_filename,
                vk::ShaderStageFlagBits::eVertex
            )
        );

        m_fragment_shader_module = std::make_unique<ShaderModule>(
            ShaderModule::from_file(
                m_device.get(),
                shader_output_dir + fragment_shader_filename,
                vk::ShaderStageFlagBits::eFragment
            )
        );

         // Create vertex and index buffers
        vk::DeviceSize vertex_buffer_size = sizeof(vertices[0]) * vertices.size();
        vk::DeviceSize index_buffer_size = sizeof(indices[0]) * indices.size();

        m_vertex_buffer = std::make_unique<Buffer>(
            Buffer::create_device_local_with_data(
                m_device.get(),
                vertices.data(),
                vertex_buffer_size,
                vk::BufferUsageFlagBits::eVertexBuffer
            )
        );

        m_index_buffer = std::make_unique<Buffer>(
            Buffer::create_device_local_with_data(
                m_device.get(),
                indices.data(),
                index_buffer_size,
                vk::BufferUsageFlagBits::eIndexBuffer
            )
        );

        // Create uniform buffers 
        m_uniform_buffers.clear();
        m_uniform_buffers.reserve(MAX_FRAMES_IN_FLIGHT);
        for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; i ++) {
            vk::DeviceSize buffer_size = sizeof(UniformBufferObject);
            auto uniform_buffer = std::make_unique<Buffer>(
                Buffer::create_host_visible_buffer(
                    m_device.get(),
                    buffer_size,
                    vk::BufferUsageFlagBits::eUniformBuffer
                )
            );
            uniform_buffer->map();
            m_uniform_buffers.emplace_back(std::move(uniform_buffer));
        }
        // Create descriptor set layout
        m_descriptor_set_layout = std::make_unique<DescriptorSetLayout>(
            DescriptorSetLayout::Builder()
                .add_binding(0, vk::DescriptorType::eUniformBuffer, 
                           vk::ShaderStageFlagBits::eVertex)
                .build(m_device.get())
        );
        // Create descriptor pool 
        m_descriptor_pool = std::make_unique<DescriptorPool>(
            DescriptorPool::Builder()
                .add_layout(*m_descriptor_set_layout, MAX_FRAMES_IN_FLIGHT)
                .set_flags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet)
                .build(m_device.get())
        );
        // Allocate and update descriptor sets
        m_descriptor_sets = m_descriptor_pool->allocate_multiple(
            *m_descriptor_set_layout, 
            MAX_FRAMES_IN_FLIGHT
        );

        for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
            DescriptorWriter()
                .write_buffer(
                    0, 
                    *m_uniform_buffers[i]->buffer(), 
                    0, 
                    sizeof(UniformBufferObject)
                )
                .update(m_device.get(), *m_descriptor_sets[i]);
        }
        create_pipeline();
       
        m_command_pool = std::make_unique<CommandPool>(m_device.get(), vk::CommandPoolCreateFlagBits::eResetCommandBuffer);
        m_command_buffers = m_command_pool->allocate_command_buffers(MAX_FRAMES_IN_FLIGHT);
        create_sync_objects();
    }

    void run() { loop(); }

    ~Application() = default;

private:
    void loop() {
        while (!m_window->is_should_close()) {
            m_window->poll_events();
            draw_frame();
        }
        
        m_device->device().waitIdle();
        
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
            m_swapchain->images().at(image_index),
            old_layout_info,
            new_layout_info);

        //vk::ClearValue clear_value = vk::ClearValue{vk::ClearColorValue{0.0f, 0.0f, 0.0f, 1.0f}};
        vk::ClearValue clear_value = vk::ClearValue{vk::ClearColorValue{1.0f, 0.0f, 1.0f, 1.0f}}; 
        vk::RenderingAttachmentInfo color_attachment_info{};
        color_attachment_info.imageView = *m_swapchain->image_views().at(image_index);
        color_attachment_info.imageLayout = vk::ImageLayout::eColorAttachmentOptimal;
        color_attachment_info.loadOp = vk::AttachmentLoadOp::eClear;
        color_attachment_info.storeOp = vk::AttachmentStoreOp::eStore;
        color_attachment_info.clearValue = clear_value;

        vk::RenderingInfo rendering_info{};
        rendering_info.renderArea.offset = vk::Offset2D{0, 0};
        rendering_info.renderArea.extent = m_swapchain->extent();
        rendering_info.layerCount = 1;
        rendering_info.colorAttachmentCount = 1;
        rendering_info.pColorAttachments = &color_attachment_info;

        command_buffer.beginRendering(rendering_info);

        command_buffer.bindPipeline(
            vk::PipelineBindPoint::eGraphics,
            *m_pipeline);

        vk::Buffer vertex_buffers[] = {*m_vertex_buffer->buffer()};
        vk::DeviceSize offsets[] = {0};
        command_buffer.bindVertexBuffers(0, vertex_buffers, offsets);

        command_buffer.bindIndexBuffer(
            *m_index_buffer->buffer(),
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
        viewport.width = static_cast<float>(m_swapchain->extent().width);
        viewport.height = static_cast<float>(m_swapchain->extent().height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        command_buffer.setViewport(0, viewport);

        vk::Rect2D scissor{};
        scissor.offset = vk::Offset2D{0, 0};
        scissor.extent = m_swapchain->extent();
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
            m_swapchain->images().at(image_index),
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
            static_cast<float>(m_swapchain->extent().width) / static_cast<float>(m_swapchain->extent().height),
            0.1f,
            10.0f);
        ubo.proj[1][1] *= -1;

        std::memcpy(mapped_ptr, &ubo, sizeof(ubo));
    }

    void draw_frame() {
        auto &current_sync_objects = m_frame_sync_objects[m_current_frame];

        //update uniform buffer
        update_uniform_buffer(m_uniform_buffers[m_current_frame]->mapped_data());

        // acquire image
        std::array<vk::Fence, 2> wait_fences = {
            *current_sync_objects.submit_fence,
            *current_sync_objects.present_fence};

        if (vk::Result::eTimeout == m_device->device().waitForFences(
                                           wait_fences,
                                           true,
                                           UINT64_MAX)) {
            std::cout << "Wait for fence timed out." << std::endl;
        }

        auto [result, image_index] = m_swapchain->acquire_next_image(
            current_sync_objects.image_available_semaphore);

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

        m_device->device().resetFences(wait_fences);

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

        m_device->queue().submit(submit_info, *current_sync_objects.submit_fence);

        // present image
        vk::Result present_result = m_swapchain->present(
            image_index, 
            current_sync_objects.render_finished_semaphore,
            current_sync_objects.present_fence
        );

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

    void recreate_swapchain() {
        m_swapchain->recreate();
    }

    void create_pipeline() {
        std::vector<vk::PipelineShaderStageCreateInfo> shader_stage_infos = {
            m_vertex_shader_module->stage_create_info(),
            m_fragment_shader_module->stage_create_info()
        };

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
            *m_descriptor_set_layout->layout()
        };
        pipeline_layout_info.setLayoutCount = static_cast<uint32_t>(set_layouts.size());
        pipeline_layout_info.pSetLayouts = set_layouts.data();

        m_pipeline_layout = vk::raii::PipelineLayout{
            m_device->device(),
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
            m_swapchain->image_format()};

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
            m_device->device(),
            nullptr,
            pipeline_info_chain.get<vk::GraphicsPipelineCreateInfo>()
        };
    }


    void create_sync_objects() {
        m_frame_sync_objects.clear();
        m_frame_sync_objects.resize(MAX_FRAMES_IN_FLIGHT);

        vk::SemaphoreCreateInfo semaphore_create_info{};
        vk::FenceCreateInfo fence_create_info{};
        fence_create_info.flags = vk::FenceCreateFlagBits::eSignaled;

        for (auto &sync_objects : m_frame_sync_objects) {
            sync_objects.image_available_semaphore = vk::raii::Semaphore{
                m_device->device(),
                semaphore_create_info};

            sync_objects.render_finished_semaphore = vk::raii::Semaphore{
                m_device->device(),
                semaphore_create_info};

            sync_objects.submit_fence = vk::raii::Fence{
                m_device->device(),
                fence_create_info};

            sync_objects.present_fence = vk::raii::Fence{
                m_device->device(),
                fence_create_info};
        }
    }

    static void framebuffer_resize_callback(GLFWwindow *window, int width, int height) {
        auto app = reinterpret_cast<Application *>(glfwGetWindowUserPointer(window));
        if (!app) { return; }
        std::cout << "Framebuffer resized to (" << width << ", " << height << ")." << std::endl;
        app->m_framebuffer_resized = true;
    }
};

} // namespace rtr
