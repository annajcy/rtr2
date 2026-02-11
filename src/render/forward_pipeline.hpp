#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rhi/buffer.hpp"
#include "rhi/descriptor.hpp"
#include "rhi/shader_module.hpp"
#include "rhi/texture.hpp"
#include "render/mesh.hpp"
#include "render/pipeline.hpp"
#include "render/render_pass.hpp"
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
public:
    struct FrameResources {
        rhi::Buffer* uniform_buffer{};
        rhi::Image* depth_image{};
        vk::raii::DescriptorSet* per_frame_set{};
    };

private:
    render::Mesh* m_mesh{};
    vk::raii::PipelineLayout* m_pipeline_layout{};
    vk::raii::Pipeline* m_pipeline{};
    FrameResources m_frame_resources{};
    rhi::DescriptorSystem* m_descriptor_system{};

    std::vector<render::ResourceDependency> m_dependencies{
        {"uniform", render::ResourceAccess::eWrite},
        {"per_frame", render::ResourceAccess::eRead},
        {"texture", render::ResourceAccess::eRead},
        {"swapchain_color", render::ResourceAccess::eReadWrite},
        {"depth", render::ResourceAccess::eReadWrite}
    };

public:
    ForwardPass(
        render::Mesh* mesh,
        vk::raii::PipelineLayout* pipeline_layout,
        vk::raii::Pipeline* pipeline,
        rhi::DescriptorSystem* descriptor_system
    )
        : m_mesh(mesh),
          m_pipeline_layout(pipeline_layout),
          m_pipeline(pipeline),
          m_descriptor_system(descriptor_system) {}

    std::string_view name() const override { return "forward_main"; }

    const std::vector<render::ResourceDependency>& dependencies() const override {
        return m_dependencies;
    }

    void bind_frame_resources(const FrameResources& resources) {
        if (resources.uniform_buffer == nullptr ||
            resources.depth_image == nullptr ||
            resources.per_frame_set == nullptr) {
            throw std::runtime_error("ForwardPass frame resources are incomplete.");
        }
        m_frame_resources = resources;
    }

    void execute(render::FrameContext& ctx) override {
        if (m_frame_resources.uniform_buffer == nullptr ||
            m_frame_resources.depth_image == nullptr ||
            m_frame_resources.per_frame_set == nullptr) {
            throw std::runtime_error("ForwardPass frame resources are not bound.");
        }

        update_uniform_buffer(*m_frame_resources.uniform_buffer, ctx.render_extent());

        auto& cmd = ctx.cmd().command_buffer();
        rhi::Image& depth_image = *m_frame_resources.depth_image;

        vk::ClearValue clear_value = vk::ClearValue{vk::ClearColorValue{0.0f, 0.0f, 0.0f, 1.0f}};
        vk::RenderingAttachmentInfo color_attachment_info{};
        color_attachment_info.imageView = *ctx.swapchain_image_view();
        color_attachment_info.imageLayout = vk::ImageLayout::eColorAttachmentOptimal;
        color_attachment_info.loadOp = vk::AttachmentLoadOp::eClear;
        color_attachment_info.storeOp = vk::AttachmentStoreOp::eStore;
        color_attachment_info.clearValue = clear_value;

        vk::ClearValue depth_clear{vk::ClearDepthStencilValue{1.0f, 0}};
        vk::RenderingAttachmentInfo depth_attachment_info{};
        depth_attachment_info.imageView = *depth_image.image_view();
        depth_attachment_info.imageLayout = vk::ImageLayout::eDepthAttachmentOptimal;
        depth_attachment_info.loadOp = vk::AttachmentLoadOp::eClear;
        depth_attachment_info.storeOp = vk::AttachmentStoreOp::eStore;
        depth_attachment_info.clearValue = depth_clear;

        vk::RenderingInfo rendering_info{};
        rendering_info.renderArea.offset = vk::Offset2D{0, 0};
        rendering_info.renderArea.extent = ctx.render_extent();
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
        to_depth.image = *depth_image.image();
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
            **m_frame_resources.per_frame_set,
            {}
        );
        cmd.bindDescriptorSets(
            vk::PipelineBindPoint::eGraphics,
            **m_pipeline_layout,
            1,
            *m_descriptor_system->get_set("texture", 0),
            {}
        );

        vk::Viewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = static_cast<float>(ctx.render_extent().width);
        viewport.height = static_cast<float>(ctx.render_extent().height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        cmd.setViewport(0, viewport);

        vk::Rect2D scissor{};
        scissor.offset = vk::Offset2D{0, 0};
        scissor.extent = ctx.render_extent();
        cmd.setScissor(0, scissor);

        cmd.drawIndexed(m_mesh->index_count(), 1, 0, 0, 0);
        cmd.endRendering();
    }

private:
    void update_uniform_buffer(rhi::Buffer& uniform_buffer, const vk::Extent2D& extent) {
        static const auto start_time = std::chrono::high_resolution_clock::now();
        const auto current_time = std::chrono::high_resolution_clock::now();
        const float time = std::chrono::duration<float, std::chrono::seconds::period>(current_time - start_time).count();

        UniformBufferObject ubo{};
        glm::mat4 model = glm::scale(glm::mat4(1.0f), glm::vec3(1.0f));
        ubo.model = glm::rotate(model, time * glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        ubo.view = glm::lookAt(
            glm::vec3(0.0f, 0.0f, -3.0f),
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 1.0f, 0.0f)
        );
        ubo.proj = glm::perspective(
            glm::radians(45.0f),
            static_cast<float>(extent.width) / static_cast<float>(extent.height),
            0.1f,
            10.0f
        );
        ubo.proj[1][1] *= -1;
        ubo.normal = glm::transpose(glm::inverse(ubo.model));

        std::memcpy(uniform_buffer.mapped_data(), &ubo, sizeof(ubo));
    }
};

class ForwardPipeline : public RenderPipelineBase {
private:
    vk::raii::PipelineLayout m_pipeline_layout{nullptr};
    vk::raii::Pipeline m_pipeline{nullptr};

    std::unique_ptr<rhi::ShaderModule> m_vertex_shader_module{nullptr};
    std::unique_ptr<rhi::ShaderModule> m_fragment_shader_module{nullptr};
    std::unique_ptr<render::Mesh> m_mesh{nullptr};

    vk::DeviceSize m_uniform_buffer_size{0};
    std::vector<std::unique_ptr<rhi::Buffer>> m_uniform_buffers{};
    std::vector<std::unique_ptr<rhi::Image>> m_depth_images{};

    std::unique_ptr<rhi::DescriptorSystem> m_descriptor_system{nullptr};
    std::unique_ptr<rhi::Image> m_texture_image{nullptr};
    std::unique_ptr<rhi::Sampler> m_texture_sampler{nullptr};

    std::unique_ptr<render::ForwardPass> m_forward_pass{nullptr};
    std::unique_ptr<render::ImGUIPass> m_imgui_pass{nullptr};

public:
    ForwardPipeline(
        const render::PipelineRuntime& runtime,
        const ForwardPipelineConfig& config = {}
    )
        : RenderPipelineBase(runtime) {
        m_uniform_buffer_size = sizeof(UniformBufferObject);

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

        m_uniform_buffers = make_per_frame_mapped_uniform_buffers(m_uniform_buffer_size);

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

        auto layout_info = rhi::DescriptorSystem::make_pipeline_layout_info(*m_descriptor_system);
        m_pipeline_layout = vk::raii::PipelineLayout{m_device->device(), layout_info.info};

        create_graphics_pipeline();

        m_forward_pass = std::make_unique<render::ForwardPass>(
            m_mesh.get(),
            &m_pipeline_layout,
            &m_pipeline,
            m_descriptor_system.get()
        );

        m_imgui_pass = std::make_unique<render::ImGUIPass>(
            m_device,
            m_context,
            m_window,
            m_image_count,
            m_color_format,
            m_depth_format
        );
    }

    ~ForwardPipeline() override = default;

    ImGUIPass& imgui_pass() {
        if (!m_imgui_pass) {
            throw std::runtime_error("ForwardPipeline imgui pass is not initialized.");
        }
        return *m_imgui_pass;
    }

    const ImGUIPass& imgui_pass() const {
        if (!m_imgui_pass) {
            throw std::runtime_error("ForwardPipeline imgui pass is not initialized.");
        }
        return *m_imgui_pass;
    }

    void on_resize(int /*width*/, int /*height*/) override {}

    void handle_swapchain_state_change(
        const FrameScheduler::SwapchainState& /*state*/,
        const SwapchainChangeSummary& diff
    ) override {
        if (diff.extent_or_depth_changed()) {
            create_depth_images();
        }
        if (diff.color_or_depth_changed()) {
            create_graphics_pipeline();
        }

        if (m_imgui_pass) {
            m_imgui_pass->on_swapchain_recreated(m_image_count, m_color_format, m_depth_format);
        }
    }

    void render(FrameContext& ctx) override {
        const auto extent = ctx.render_extent();
        if (extent.width == 0 || extent.height == 0) {
            return;
        }
        const uint32_t frame_index = ctx.frame_index();
        if (frame_index >= m_uniform_buffers.size() || frame_index >= m_depth_images.size()) {
            throw std::runtime_error("ForwardPipeline frame resources are not ready.");
        }
        if (!m_forward_pass || !m_imgui_pass) {
            throw std::runtime_error("Forward pipeline passes are not initialized.");
        }

        m_forward_pass->bind_frame_resources(ForwardPass::FrameResources{
            .uniform_buffer = m_uniform_buffers[frame_index].get(),
            .depth_image = m_depth_images[frame_index].get(),
            .per_frame_set = &m_descriptor_system->get_set("per_frame", frame_index)
        });
        m_forward_pass->execute(ctx);

        m_imgui_pass->bind_frame_resources(ImGUIPass::FrameResources{
            .depth_image = m_depth_images[frame_index].get()
        });
        m_imgui_pass->execute(ctx);
    }

private:
    void create_depth_images() {
        if (!has_valid_extent()) {
            return;
        }
        m_depth_images = make_per_frame_depth_images(m_swapchain_extent, m_depth_format);
    }

    void create_graphics_pipeline() {
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
        vk::Format color_attachment_format = m_color_format;
        pipeline_rendering_info.colorAttachmentCount = 1;
        pipeline_rendering_info.pColorAttachmentFormats = &color_attachment_format;
        pipeline_rendering_info.depthAttachmentFormat = m_depth_format;

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
};

} // namespace rtr::render
