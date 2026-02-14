#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rtr/framework/core/world.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/rhi/buffer.hpp"
#include "rtr/rhi/descriptor.hpp"
#include "rtr/rhi/mesh.hpp"
#include "rtr/rhi/shader_module.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view_builder.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view.hpp"
#include "rtr/system/render/pipeline.hpp"
#include "rtr/system/render/render_pass.hpp"
#include "rtr/utils/log.hpp"
#include "vulkan/vulkan.hpp"

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

namespace rtr::system::render {

struct ForwardPipelineConfig {
    std::string shader_output_dir{"/Users/jinceyang/Desktop/codebase/graphics/rtr2/build/Debug/shaders/compiled/"};
    std::string vertex_shader_filename{"vert_buffer_vert.spv"};
    std::string fragment_shader_filename{"vert_buffer_frag.spv"};
};

struct UniformBufferObject {
    alignas(16) glm::mat4 model;
    alignas(16) glm::mat4 view;
    alignas(16) glm::mat4 proj;
    alignas(16) glm::mat4 normal;
    alignas(16) glm::vec4 base_color;
};

class ForwardPass final : public render::IRenderPass {
public:
    struct DrawItem {
        rhi::Mesh* mesh{};
        vk::raii::DescriptorSet* per_object_set{};
    };

    struct RenderPassResources {
        rhi::Image* depth_image{};
        std::vector<DrawItem> draw_items{};
    };

private:
    vk::raii::PipelineLayout* m_pipeline_layout{};
    vk::raii::Pipeline* m_pipeline{};
    RenderPassResources m_render_pass_resources{};

    std::vector<render::ResourceDependency> m_dependencies{
        {"forward.per_object", render::ResourceAccess::eRead},
        {"swapchain_color", render::ResourceAccess::eReadWrite},
        {"depth", render::ResourceAccess::eReadWrite}
    };

public:
    ForwardPass(
        vk::raii::PipelineLayout* pipeline_layout,
        vk::raii::Pipeline* pipeline
    )
        : m_pipeline_layout(pipeline_layout),
          m_pipeline(pipeline) {}

    std::string_view name() const override { return "forward_main"; }

    const std::vector<render::ResourceDependency>& dependencies() const override {
        return m_dependencies;
    }

    void bind_render_pass_resources(RenderPassResources resources) {
        if (resources.depth_image == nullptr) {
            throw std::runtime_error("ForwardPass frame resources are incomplete.");
        }
        for (const auto& item : resources.draw_items) {
            if (item.mesh == nullptr ||
                item.per_object_set == nullptr) {
                throw std::runtime_error("ForwardPass draw item resources are incomplete.");
            }
        }
        m_render_pass_resources = std::move(resources);
    }

    void execute(render::FrameContext& ctx) override {
        if (m_render_pass_resources.depth_image == nullptr) {
            throw std::runtime_error("ForwardPass frame resources are not bound.");
        }

        auto& cmd = ctx.cmd().command_buffer();
        rhi::Image& depth_image = *m_render_pass_resources.depth_image;

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

        for (const auto& item : m_render_pass_resources.draw_items) {
            std::vector<vk::Buffer> vertex_buffers = {item.mesh->vertex_buffer()};
            std::vector<vk::DeviceSize> offsets = {0};
            cmd.bindVertexBuffers(0, vertex_buffers, offsets);
            cmd.bindIndexBuffer(item.mesh->index_buffer(), 0, vk::IndexType::eUint32);

            cmd.bindDescriptorSets(
                vk::PipelineBindPoint::eGraphics,
                **m_pipeline_layout,
                0,
                **item.per_object_set,
                {}
            );

            cmd.drawIndexed(item.mesh->index_count(), 1, 0, 0, 0);
        }

        cmd.endRendering();
    }
};

class ForwardPipeline : public RenderPipelineBase,
                        public IFramePreparePipeline,
                        public IResourceAwarePipeline,
                        public IImGuiOverlayPipeline {
private:
    static constexpr uint32_t kMaxRenderables = 256;

    vk::raii::PipelineLayout m_pipeline_layout{nullptr};
    vk::raii::Pipeline m_pipeline{nullptr};

    std::unique_ptr<rhi::ShaderModule> m_vertex_shader_module{nullptr};
    std::unique_ptr<rhi::ShaderModule> m_fragment_shader_module{nullptr};

    std::optional<ForwardSceneView> m_scene_view{};

    vk::DeviceSize m_uniform_buffer_size{0};
    std::vector<std::vector<std::unique_ptr<rhi::Buffer>>> m_object_uniform_buffers{};
    std::vector<std::vector<vk::raii::DescriptorSet>> m_object_sets{};
    std::vector<std::unique_ptr<rhi::Image>> m_depth_images{};

    std::unique_ptr<rhi::DescriptorSetLayout> m_per_object_layout{nullptr};
    std::unique_ptr<rhi::DescriptorPool> m_descriptor_pool{nullptr};

    resource::ResourceManager* m_resource_manager{};

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

        rhi::DescriptorSetLayout::Builder per_object_layout_builder;
        per_object_layout_builder.add_binding(
            0,
            vk::DescriptorType::eUniformBuffer,
            vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment
        );
        m_per_object_layout = std::make_unique<rhi::DescriptorSetLayout>(
            per_object_layout_builder.build(m_device)
        );

        rhi::DescriptorPool::Builder descriptor_pool_builder;
        descriptor_pool_builder
            .add_layout(*m_per_object_layout, kMaxRenderables * m_frame_count)
            .set_flags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
        m_descriptor_pool = std::make_unique<rhi::DescriptorPool>(
            descriptor_pool_builder.build(m_device)
        );

        create_per_object_resources();
        std::array<vk::DescriptorSetLayout, 1> set_layouts{
            *m_per_object_layout->layout()
        };
        vk::PipelineLayoutCreateInfo pipeline_layout_info{};
        pipeline_layout_info.setLayoutCount = static_cast<uint32_t>(set_layouts.size());
        pipeline_layout_info.pSetLayouts = set_layouts.data();
        m_pipeline_layout = vk::raii::PipelineLayout{m_device->device(), pipeline_layout_info};

        create_graphics_pipeline();

        m_forward_pass = std::make_unique<render::ForwardPass>(
            &m_pipeline_layout,
            &m_pipeline
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

    ~ForwardPipeline() override {
        // Descriptor sets must be destroyed before the descriptor pool.
        m_forward_pass.reset();
        m_imgui_pass.reset();
        m_object_sets.clear();
        m_descriptor_pool.reset();
    }

    void set_resource_manager(resource::ResourceManager* manager) override {
        m_resource_manager = manager;
    }

    void prepare_frame(const FramePrepareContext& ctx) override {
        auto* active_scene = ctx.world.active_scene();
        if (active_scene == nullptr) {
            throw std::runtime_error("ForwardPipeline prepare_frame requires an active scene.");
        }
        set_scene_view(build_forward_scene_view(*active_scene, ctx.resources));
    }

    void set_scene_view(ForwardSceneView scene_view) {
        m_scene_view = std::move(scene_view);
    }

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

    void set_imgui_overlay(std::shared_ptr<IImGuiOverlay> overlay) override {
        if (!m_imgui_pass) {
            throw std::runtime_error("ForwardPipeline imgui pass is not initialized.");
        }
        m_imgui_pass->set_overlay(std::move(overlay));
    }

    void clear_imgui_overlay() override {
        if (!m_imgui_pass) {
            return;
        }
        m_imgui_pass->clear_overlay();
    }

    bool wants_imgui_capture_mouse() const override {
        if (!m_imgui_pass) {
            return false;
        }
        return m_imgui_pass->wants_capture_mouse();
    }

    bool wants_imgui_capture_keyboard() const override {
        if (!m_imgui_pass) {
            return false;
        }
        return m_imgui_pass->wants_capture_keyboard();
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
        if (m_resource_manager == nullptr) {
            throw std::runtime_error("ForwardPipeline requires resource manager before render().");
        }
        if (!m_scene_view.has_value()) {
            throw std::runtime_error("ForwardPipeline requires scene view before render().");
        }

        const uint32_t frame_index = ctx.frame_index();
        if (frame_index >= m_object_uniform_buffers.size() ||
            frame_index >= m_object_sets.size() ||
            frame_index >= m_depth_images.size()) {
            throw std::runtime_error("ForwardPipeline frame resources are not ready.");
        }

        if (!m_forward_pass || !m_imgui_pass) {
            throw std::runtime_error("Forward pipeline passes are not initialized.");
        }

        const auto& scene_view = m_scene_view.value();
        if (scene_view.renderables.size() > kMaxRenderables) {
            throw std::runtime_error("Renderable count exceeds preallocated ForwardPipeline capacity.");
        }

        glm::mat4 proj = scene_view.camera.proj;
        proj[1][1] *= -1;

        auto& frame_uniform_buffers = m_object_uniform_buffers[frame_index];
        auto& frame_sets = m_object_sets[frame_index];

        std::vector<ForwardPass::DrawItem> draw_items{};
        draw_items.reserve(scene_view.renderables.size());

        for (std::size_t i = 0; i < scene_view.renderables.size(); ++i) {
            const auto& renderable = scene_view.renderables[i];
            auto& mesh = require_mesh(renderable.mesh);

            UniformBufferObject ubo{};
            ubo.model = renderable.model;
            ubo.view = scene_view.camera.view;
            ubo.proj = proj;
            ubo.normal = renderable.normal;
            ubo.base_color = renderable.base_color;

            std::memcpy(frame_uniform_buffers[i]->mapped_data(), &ubo, sizeof(ubo));

            draw_items.emplace_back(ForwardPass::DrawItem{
                .mesh = &mesh,
                .per_object_set = &frame_sets[i]
            });
        }

        m_forward_pass->bind_render_pass_resources(ForwardPass::RenderPassResources{
            .depth_image = m_depth_images[frame_index].get(),
            .draw_items = std::move(draw_items)
        });
        m_forward_pass->execute(ctx);

        m_imgui_pass->bind_render_pass_resources(ImGUIPass::RenderPassResources{
            .depth_image = m_depth_images[frame_index].get()
        });
        m_imgui_pass->execute(ctx);
    }

private:
    rhi::Mesh& require_mesh(resource::MeshHandle mesh_handle) {
        auto logger = utils::get_logger("render.pipeline.forward");
        if (!mesh_handle.is_valid()) {
            logger->error("Renderable mesh handle is invalid.");
            throw std::runtime_error("Renderable mesh handle is invalid.");
        }
        if (m_resource_manager == nullptr) {
            logger->error("ForwardPipeline missing resource manager while requesting mesh handle={}.", mesh_handle.value);
            throw std::runtime_error("ForwardPipeline missing resource manager.");
        }
        return m_resource_manager->require_mesh_rhi(mesh_handle, m_device);
    }

    void create_per_object_resources() {
        m_object_uniform_buffers.clear();
        m_object_sets.clear();
        m_object_uniform_buffers.resize(m_frame_count);
        m_object_sets.resize(m_frame_count);

        for (uint32_t frame = 0; frame < m_frame_count; ++frame) {
            auto& frame_buffers = m_object_uniform_buffers[frame];
            frame_buffers.reserve(kMaxRenderables);
            for (uint32_t slot = 0; slot < kMaxRenderables; ++slot) {
                auto buffer = std::make_unique<rhi::Buffer>(
                    rhi::Buffer::create_host_visible_buffer(
                        m_device,
                        m_uniform_buffer_size,
                        vk::BufferUsageFlagBits::eUniformBuffer
                    )
                );
                buffer->map();
                frame_buffers.emplace_back(std::move(buffer));
            }

            m_object_sets[frame] = m_descriptor_pool->allocate_multiple(*m_per_object_layout, kMaxRenderables);
            for (uint32_t slot = 0; slot < kMaxRenderables; ++slot) {
                rhi::DescriptorWriter writer;
                writer.write_buffer(
                    0,
                    *frame_buffers[slot]->buffer(),
                    0,
                    m_uniform_buffer_size
                );
                writer.update(m_device, *m_object_sets[frame][slot]);
            }
        }
    }

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

        auto vertex_input_state = rhi::Mesh::vertex_input_state();
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

} // namespace rtr::system::render
