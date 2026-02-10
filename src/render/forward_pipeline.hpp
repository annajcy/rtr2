#pragma once

#include <array>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rhi/buffer.hpp"
#include "rhi/descriptor.hpp"
#include "render/imgui_layer.hpp"
#include "render/mesh.hpp"
#include "render/renderer.hpp"
#include "rhi/shader_module.hpp"
#include "rhi/texture.hpp"
#include "render/render_graph.hpp"
#include "vulkan/vulkan.hpp"

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace rtr::render {

struct ForwardPipelineConfig {
    std::string shader_output_dir{"/Users/jinceyang/Desktop/codebase/graphics/rtr2/build/Debug/shaders/compiled/"};
    std::string vertex_shader_filename{"vert_buffer_vert.spv"};
    std::string fragment_shader_filename{"vert_buffer_frag.spv"};
    std::string model_path{"/Users/jinceyang/Desktop/codebase/graphics/rtr2/assets/models/spot.obj"};
    std::string texture_path{"/Users/jinceyang/Desktop/codebase/graphics/rtr2/assets/textures/spot_texture.png"};
};

struct UniformBufferObject {
    alignas(16) glm::mat4 model;
    alignas(16) glm::mat4 view;
    alignas(16) glm::mat4 proj;
    alignas(16) glm::mat4 normal;
};

class ForwardPass final : public render::IRenderPass {
private:
    using BuildUiCallback = std::function<void()>;

    render::Renderer* m_renderer{};
    render::Mesh* m_mesh{};
    render::ImGuiLayer* m_imgui_layer{};
    vk::raii::PipelineLayout* m_pipeline_layout{};
    vk::raii::Pipeline* m_pipeline{};
    BuildUiCallback m_build_ui{};

    std::vector<render::ResourceDependency> m_dependencies{
        {"uniform", render::ResourceAccess::eWrite},
        {"per_frame", render::ResourceAccess::eRead},
        {"texture", render::ResourceAccess::eRead},
        {"swapchain_color", render::ResourceAccess::eReadWrite},
        {"depth", render::ResourceAccess::eReadWrite}
    };

public:
    ForwardPass(
        render::Renderer* renderer,
        render::Mesh* mesh,
        render::ImGuiLayer* imgui_layer,
        vk::raii::PipelineLayout* pipeline_layout,
        vk::raii::Pipeline* pipeline
    )
        : m_renderer(renderer),
          m_mesh(mesh),
          m_imgui_layer(imgui_layer),
          m_pipeline_layout(pipeline_layout),
          m_pipeline(pipeline) {}

    void set_ui_builder(BuildUiCallback build_ui) {
        m_build_ui = std::move(build_ui);
    }

    std::string_view name() const override { return "forward_main"; }
    const std::vector<render::ResourceDependency>& dependencies() const override {
        return m_dependencies;
    }

    std::unique_ptr<render::IPassResources> create_resources() const override {
        return std::make_unique<render::EmptyPassResources>();
    }

    void execute(render::FrameContext& ctx, render::IPassResources& /*resources*/) override {
        m_imgui_layer->begin_frame();
        if (m_build_ui) {
            m_build_ui();
        }
        ImDrawData* imgui_draw_data = m_imgui_layer->prepare_draw_data();

        update_uniform_buffer(ctx);

        ctx.cmd().record([&](rhi::CommandBuffer& cb) {
            auto& cmd = cb.command_buffer();

            vk::ClearValue clear_value = vk::ClearValue{vk::ClearColorValue{0.0f, 0.0f, 0.0f, 1.0f}};
            vk::RenderingAttachmentInfo color_attachment_info{};
            color_attachment_info.imageView = *ctx.swapchain_image_view();
            color_attachment_info.imageLayout = vk::ImageLayout::eColorAttachmentOptimal;
            color_attachment_info.loadOp = vk::AttachmentLoadOp::eClear;
            color_attachment_info.storeOp = vk::AttachmentStoreOp::eStore;
            color_attachment_info.clearValue = clear_value;

            vk::ClearValue depth_clear{vk::ClearDepthStencilValue{1.0f, 0}};
            vk::RenderingAttachmentInfo depth_attachment_info{};
            depth_attachment_info.imageView = *ctx.depth_image().image_view();
            depth_attachment_info.imageLayout = vk::ImageLayout::eDepthAttachmentOptimal;
            depth_attachment_info.loadOp = vk::AttachmentLoadOp::eClear;
            depth_attachment_info.storeOp = vk::AttachmentStoreOp::eStore;
            depth_attachment_info.clearValue = depth_clear;

            vk::RenderingInfo rendering_info{};
            rendering_info.renderArea.offset = vk::Offset2D{0, 0};
            rendering_info.renderArea.extent = m_renderer->frame_scheduler().render_extent();
            rendering_info.layerCount = 1;
            rendering_info.colorAttachmentCount = 1;
            rendering_info.pColorAttachments = &color_attachment_info;
            rendering_info.pDepthAttachment = &depth_attachment_info;

            vk::ImageMemoryBarrier2 to_color{};
            to_color.srcStageMask = vk::PipelineStageFlagBits2::eTopOfPipe;
            to_color.dstStageMask = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
            to_color.srcAccessMask = vk::AccessFlagBits2::eNone;
            to_color.dstAccessMask = vk::AccessFlagBits2::eColorAttachmentWrite;
            to_color.oldLayout = vk::ImageLayout::eUndefined;
            to_color.newLayout = vk::ImageLayout::eColorAttachmentOptimal;
            to_color.image = ctx.swapchain_image();
            to_color.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
            to_color.subresourceRange.baseMipLevel = 0;
            to_color.subresourceRange.levelCount = 1;
            to_color.subresourceRange.baseArrayLayer = 0;
            to_color.subresourceRange.layerCount = 1;

            vk::ImageMemoryBarrier2 to_depth{};
            to_depth.srcStageMask = vk::PipelineStageFlagBits2::eEarlyFragmentTests | vk::PipelineStageFlagBits2::eLateFragmentTests;
            to_depth.dstStageMask = vk::PipelineStageFlagBits2::eEarlyFragmentTests | vk::PipelineStageFlagBits2::eLateFragmentTests;
            to_depth.srcAccessMask = vk::AccessFlagBits2::eDepthStencilAttachmentWrite;
            to_depth.dstAccessMask = vk::AccessFlagBits2::eDepthStencilAttachmentWrite;
            to_depth.oldLayout = vk::ImageLayout::eUndefined;
            to_depth.newLayout = vk::ImageLayout::eDepthAttachmentOptimal;
            to_depth.image = *ctx.depth_image().image();
            to_depth.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eDepth;
            to_depth.subresourceRange.baseMipLevel = 0;
            to_depth.subresourceRange.levelCount = 1;
            to_depth.subresourceRange.baseArrayLayer = 0;
            to_depth.subresourceRange.layerCount = 1;

            std::array<vk::ImageMemoryBarrier2, 2> barriers = {to_color, to_depth};
            vk::DependencyInfo to_depth_color_dep{};
            to_depth_color_dep.imageMemoryBarrierCount = static_cast<uint32_t>(barriers.size());
            to_depth_color_dep.pImageMemoryBarriers = barriers.data();
            cmd.pipelineBarrier2(to_depth_color_dep);

            cmd.beginRendering(rendering_info);
            cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *m_pipeline);

            std::vector<vk::Buffer> vertex_buffers = {m_mesh->vertex_buffer()};
            std::vector<vk::DeviceSize> offsets = {0};
            cmd.bindVertexBuffers(0, vertex_buffers, offsets);
            cmd.bindIndexBuffer(m_mesh->index_buffer(), 0, vk::IndexType::eUint32);

            cmd.bindDescriptorSets(
                vk::PipelineBindPoint::eGraphics,
                **m_pipeline_layout,
                0,
                *ctx.get_descriptor_set("per_frame"),
                {}
            );
            cmd.bindDescriptorSets(
                vk::PipelineBindPoint::eGraphics,
                **m_pipeline_layout,
                1,
                *ctx.get_descriptor_set("texture"),
                {}
            );

            vk::Viewport viewport{};
            viewport.x = 0.0f;
            viewport.y = 0.0f;
            viewport.width = static_cast<float>(m_renderer->frame_scheduler().render_extent().width);
            viewport.height = static_cast<float>(m_renderer->frame_scheduler().render_extent().height);
            viewport.minDepth = 0.0f;
            viewport.maxDepth = 1.0f;
            cmd.setViewport(0, viewport);

            vk::Rect2D scissor{};
            scissor.offset = vk::Offset2D{0, 0};
            scissor.extent = m_renderer->frame_scheduler().render_extent();
            cmd.setScissor(0, scissor);

            cmd.drawIndexed(m_mesh->index_count(), 1, 0, 0, 0);
            m_imgui_layer->render_draw_data(cmd, imgui_draw_data);
            cmd.endRendering();

            vk::ImageMemoryBarrier2 to_present{};
            to_present.srcStageMask = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
            to_present.dstStageMask = vk::PipelineStageFlagBits2::eBottomOfPipe;
            to_present.srcAccessMask = vk::AccessFlagBits2::eColorAttachmentWrite;
            to_present.dstAccessMask = vk::AccessFlagBits2::eNone;
            to_present.oldLayout = vk::ImageLayout::eColorAttachmentOptimal;
            to_present.newLayout = vk::ImageLayout::ePresentSrcKHR;
            to_present.image = ctx.swapchain_image();
            to_present.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
            to_present.subresourceRange.baseMipLevel = 0;
            to_present.subresourceRange.levelCount = 1;
            to_present.subresourceRange.baseArrayLayer = 0;
            to_present.subresourceRange.layerCount = 1;

            vk::DependencyInfo to_present_dep{};
            to_present_dep.imageMemoryBarrierCount = 1;
            to_present_dep.pImageMemoryBarriers = &to_present;
            cmd.pipelineBarrier2(to_present_dep);
        }, vk::CommandBufferUsageFlagBits::eOneTimeSubmit);
    }

private:
    void update_uniform_buffer(render::FrameContext& ctx) {
        static auto start_time = std::chrono::high_resolution_clock::now();
        auto current_time = std::chrono::high_resolution_clock::now();
        float time = std::chrono::duration<float, std::chrono::seconds::period>(current_time - start_time).count();

        UniformBufferObject ubo{};
        glm::mat4 model = glm::scale(glm::mat4(1.0f), glm::vec3(1.0f));
        ubo.model = glm::rotate(model, time * glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        ubo.view = glm::lookAt(
            glm::vec3(0.0f, 0.0f, -3.0f),
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 1.0f, 0.0f)
        );
        auto extent = m_renderer->frame_scheduler().render_extent();
        ubo.proj = glm::perspective(
            glm::radians(45.0f),
            static_cast<float>(extent.width) / static_cast<float>(extent.height),
            0.1f,
            10.0f
        );
        ubo.proj[1][1] *= -1;
        ubo.normal = glm::transpose(glm::inverse(ubo.model));

        std::memcpy(ctx.get_buffer("uniform").mapped_data(), &ubo, sizeof(ubo));
    }
};

class ForwardPipeline : public IFrameResourceBinder {
private:
    using BuildUiCallback = std::function<void()>;

    rhi::Device* m_device{};
    render::Renderer* m_renderer{};
    std::unique_ptr<render::ImGuiLayer> m_imgui_layer{nullptr};

    vk::raii::PipelineLayout m_pipeline_layout{nullptr};
    vk::raii::Pipeline m_pipeline{nullptr};

    vk::DeviceSize m_uniform_buffer_size{0};
    uint32_t m_frame_count{0};

    std::unique_ptr<rhi::ShaderModule> m_vertex_shader_module{nullptr};
    std::unique_ptr<rhi::ShaderModule> m_fragment_shader_module{nullptr};
    std::unique_ptr<render::Mesh> m_mesh{nullptr};

    std::vector<std::unique_ptr<rhi::Buffer>> m_uniform_buffers{};
    std::unique_ptr<rhi::DescriptorSystem> m_descriptor_system{nullptr};
    std::unique_ptr<rhi::Image> m_texture_image{nullptr};
    std::unique_ptr<rhi::Sampler> m_texture_sampler{nullptr};

    std::unique_ptr<render::ForwardPass> m_forward_pass{nullptr};

public:
    ForwardPipeline(
        render::Renderer* renderer,
        const ForwardPipelineConfig& config = {}
    )
        : m_renderer(renderer) {
        if (!m_renderer) {
            throw std::runtime_error("ForwardPipeline requires valid renderer.");
        }
        m_device = &m_renderer->device();
        m_uniform_buffer_size = sizeof(UniformBufferObject);
        m_frame_count = m_renderer->frame_scheduler().max_frames_in_flight();

        m_imgui_layer = std::make_unique<render::ImGuiLayer>(m_renderer);

        m_vertex_shader_module = std::make_unique<rhi::ShaderModule>(
            rhi::ShaderModule::from_file(
                m_device,
                config.shader_output_dir + config.vertex_shader_filename,
                vk::ShaderStageFlagBits::eVertex
            )
        );
        m_fragment_shader_module = std::make_unique<rhi::ShaderModule>(
            rhi::ShaderModule::from_file(
                m_device,
                config.shader_output_dir + config.fragment_shader_filename,
                vk::ShaderStageFlagBits::eFragment
            )
        );

        m_mesh = std::make_unique<render::Mesh>(render::Mesh::from_obj(m_device, config.model_path));
        m_texture_image = std::make_unique<rhi::Image>(
            rhi::Image::create_image_from_file(m_device, config.texture_path, true)
        );
        m_texture_sampler = std::make_unique<rhi::Sampler>(rhi::Sampler::create_default(m_device));

        m_uniform_buffers.reserve(m_frame_count);
        for (uint32_t i = 0; i < m_frame_count; ++i) {
            auto buffer = std::make_unique<rhi::Buffer>(
                rhi::Buffer::create_host_visible_buffer(
                    m_device,
                    m_uniform_buffer_size,
                    vk::BufferUsageFlagBits::eUniformBuffer
                )
            );
            buffer->map();
            m_uniform_buffers.emplace_back(std::move(buffer));
        }

        m_descriptor_system = std::make_unique<rhi::DescriptorSystem>(
            rhi::DescriptorSystem::Builder(m_device)
                .add_set("per_frame", 0, m_frame_count, [](rhi::DescriptorSetLayout::Builder& builder) {
                    builder.add_binding(0, vk::DescriptorType::eUniformBuffer, vk::ShaderStageFlagBits::eVertex);
                })
                .add_set("texture", 1, 1, [](rhi::DescriptorSetLayout::Builder& builder) {
                    builder.add_binding(0, vk::DescriptorType::eCombinedImageSampler, vk::ShaderStageFlagBits::eFragment);
                })
                .build()
        );

        m_descriptor_system->update_set(
            "per_frame",
            [this](rhi::DescriptorWriter& writer, uint32_t index) {
                writer.write_buffer(0, *m_uniform_buffers[index]->buffer(), 0, m_uniform_buffer_size);
            }).update_set(
            "texture",
            [this](rhi::DescriptorWriter& writer, uint32_t /*index*/) {
                writer.write_combined_image(
                    0,
                    *m_texture_image->image_view(),
                    *m_texture_sampler->sampler(),
                    vk::ImageLayout::eShaderReadOnlyOptimal
                );
            }
        );

        m_renderer->register_frame_resource_binder(*this);

        auto layout_info = rhi::DescriptorSystem::make_pipeline_layout_info(*m_descriptor_system);
        m_pipeline_layout = vk::raii::PipelineLayout{m_device->device(), layout_info.info};

        std::vector<vk::PipelineShaderStageCreateInfo> shader_stage_infos = {
            m_vertex_shader_module->stage_create_info(),
            m_fragment_shader_module->stage_create_info()
        };

        auto vertex_input_state = render::Mesh::vertex_input_state();
        vk::PipelineVertexInputStateCreateInfo vertex_input_info{};
        vertex_input_info.vertexBindingDescriptionCount = static_cast<uint32_t>(vertex_input_state.bindings.size());
        vertex_input_info.pVertexBindingDescriptions = vertex_input_state.bindings.data();
        vertex_input_info.vertexAttributeDescriptionCount = static_cast<uint32_t>(vertex_input_state.attributes.size());
        vertex_input_info.pVertexAttributeDescriptions = vertex_input_state.attributes.data();

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

        vk::PipelineDepthStencilStateCreateInfo depth_info{};
        depth_info.depthTestEnable = VK_TRUE;
        depth_info.depthWriteEnable = VK_TRUE;
        depth_info.depthCompareOp = vk::CompareOp::eLess;

        vk::PipelineColorBlendAttachmentState color_blend_attachment{};
        color_blend_attachment.blendEnable = VK_FALSE;
        color_blend_attachment.colorWriteMask =
            vk::ColorComponentFlagBits::eR |
            vk::ColorComponentFlagBits::eG |
            vk::ColorComponentFlagBits::eB |
            vk::ColorComponentFlagBits::eA;

        vk::PipelineColorBlendStateCreateInfo color_blend_state{};
        color_blend_state.logicOpEnable = VK_FALSE;
        color_blend_state.logicOp = vk::LogicOp::eCopy;
        color_blend_state.attachmentCount = 1;
        color_blend_state.pAttachments = &color_blend_attachment;

        std::vector<vk::DynamicState> dynamic_states = {
            vk::DynamicState::eViewport,
            vk::DynamicState::eScissor
        };
        vk::PipelineDynamicStateCreateInfo dynamic_state_info{};
        dynamic_state_info.dynamicStateCount = static_cast<uint32_t>(dynamic_states.size());
        dynamic_state_info.pDynamicStates = dynamic_states.data();

        vk::GraphicsPipelineCreateInfo graphics_pipeline_create_info{};
        graphics_pipeline_create_info.stageCount = static_cast<uint32_t>(shader_stage_infos.size());
        graphics_pipeline_create_info.pStages = shader_stage_infos.data();
        graphics_pipeline_create_info.pVertexInputState = &vertex_input_info;
        graphics_pipeline_create_info.pInputAssemblyState = &input_assembly_info;
        graphics_pipeline_create_info.pViewportState = &viewport_info;
        graphics_pipeline_create_info.pRasterizationState = &rasterization_info;
        graphics_pipeline_create_info.pMultisampleState = &multisample_info;
        graphics_pipeline_create_info.pDepthStencilState = &depth_info;
        graphics_pipeline_create_info.pColorBlendState = &color_blend_state;
        graphics_pipeline_create_info.pDynamicState = &dynamic_state_info;
        graphics_pipeline_create_info.layout = *m_pipeline_layout;
        graphics_pipeline_create_info.renderPass = VK_NULL_HANDLE;

        vk::PipelineRenderingCreateInfo pipeline_rendering_info{};
        vk::Format color_attachment_format = m_renderer->frame_scheduler().render_format();
        pipeline_rendering_info.colorAttachmentCount = 1;
        pipeline_rendering_info.pColorAttachmentFormats = &color_attachment_format;
        pipeline_rendering_info.depthAttachmentFormat = m_renderer->frame_scheduler().depth_format();

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

        m_forward_pass = std::make_unique<render::ForwardPass>(
            m_renderer,
            m_mesh.get(),
            m_imgui_layer.get(),
            &m_pipeline_layout,
            &m_pipeline
        );
    }

    ~ForwardPipeline() override {
        if (m_renderer != nullptr) {
            m_renderer->unregister_frame_resource_binder(*this);
        }
    }

    void bind_static_resources(render::ResourceRegistries& registries) override {
        registries.registry<vk::raii::DescriptorSet>().set_global_resource(
            "texture", 
            m_descriptor_system->get_set("texture", 0)
        );
    }

    void bind_frame_resources(uint32_t frame_index, render::ResourceRegistries& registries) override {
        registries.registry<rhi::Buffer>().set_frame_resource(
            frame_index, 
            "uniform", 
            *m_uniform_buffers.at(frame_index)
        );
        registries.registry<vk::raii::DescriptorSet>().set_frame_resource(
            frame_index,
            "per_frame",
            m_descriptor_system->get_set("per_frame", frame_index)
        );
    }

    void execute_frame(render::FrameContext& ctx) {
        if (!m_forward_pass) {
            throw std::runtime_error("Forward pass is not initialized.");
        }
        render::EmptyPassResources resources{};
        m_forward_pass->execute(ctx, resources);
    }

    void set_imgui_ui_builder(BuildUiCallback build_ui) {
        if (!m_forward_pass) {
            throw std::runtime_error("Forward pass is not initialized.");
        }
        m_forward_pass->set_ui_builder(std::move(build_ui));
    }
};

} // namespace rtr::render
