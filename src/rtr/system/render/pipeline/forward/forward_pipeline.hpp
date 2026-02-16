#pragma once

#include <pbpt/math/math.h>

#include <array>
#include <cstddef>
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
#include "rtr/system/render/frame_color_source.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view_builder.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view.hpp"
#include "rtr/system/render/pipeline.hpp"
#include "rtr/system/render/render_pass.hpp"
#include "rtr/utils/log.hpp"
#include "vulkan/vulkan.hpp"

namespace rtr::system::render {

struct ForwardPipelineConfig {
    std::string shader_output_dir{"/Users/jinceyang/Desktop/codebase/graphics/rtr2/build/Debug/shaders/compiled/"};
    std::string vertex_shader_filename{"vert_buffer_vert.spv"};
    std::string fragment_shader_filename{"vert_buffer_frag.spv"};
};

struct GpuMat4 {
    // Stored as contiguous row-major values for an explicit CPU->GPU contract.
    alignas(16) std::array<float, 16> values{};
};

struct UniformBufferObjectGpu {
    alignas(16) GpuMat4 model{};
    alignas(16) GpuMat4 view{};
    alignas(16) GpuMat4 proj{};
    alignas(16) GpuMat4 normal{};
    alignas(16) std::array<float, 4> base_color{};
};

inline GpuMat4 pack_mat4_row_major(const pbpt::math::mat4& matrix) {
    GpuMat4 packed{};
    std::size_t index = 0;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            packed.values[index++] = static_cast<float>(matrix[row][col]);
        }
    }
    return packed;
}

class ForwardPass final : public render::IRenderPass {
public:
    struct DrawItem {
        rhi::Mesh* mesh{};
        vk::raii::DescriptorSet* per_object_set{};
    };

    struct RenderPassResources {
        rhi::Image* color_image{};
        vk::ImageLayout* color_layout{};
        rhi::Image* depth_image{};
        vk::Extent2D extent{};
        std::vector<DrawItem> draw_items{};
    };

private:
    vk::raii::PipelineLayout* m_pipeline_layout{};
    vk::raii::Pipeline* m_pipeline{};
    RenderPassResources m_render_pass_resources{};

    std::vector<render::ResourceDependency> m_dependencies{
        {"forward.per_object", render::ResourceAccess::eRead},
        {"offscreen_color", render::ResourceAccess::eReadWrite},
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
        if (resources.color_image == nullptr ||
            resources.color_layout == nullptr ||
            resources.depth_image == nullptr ||
            resources.extent.width == 0 ||
            resources.extent.height == 0) {
            throw std::runtime_error("ForwardPass frame resources are incomplete.");
        }
        for (const auto& item : resources.draw_items) {
            if (item.mesh == nullptr || item.per_object_set == nullptr) {
                throw std::runtime_error("ForwardPass draw item resources are incomplete.");
            }
        }
        m_render_pass_resources = std::move(resources);
    }

    void execute(render::FrameContext& ctx) override {
        if (m_render_pass_resources.color_image == nullptr ||
            m_render_pass_resources.color_layout == nullptr ||
            m_render_pass_resources.depth_image == nullptr) {
            throw std::runtime_error("ForwardPass frame resources are not bound.");
        }

        auto& cmd = ctx.cmd().command_buffer();
        rhi::Image& color_image = *m_render_pass_resources.color_image;
        rhi::Image& depth_image = *m_render_pass_resources.depth_image;

        vk::ImageMemoryBarrier2 to_color{};
        to_color.srcStageMask =
            (*m_render_pass_resources.color_layout == vk::ImageLayout::eUndefined)
                ? vk::PipelineStageFlagBits2::eTopOfPipe
                : vk::PipelineStageFlagBits2::eAllCommands;
        to_color.dstStageMask = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        to_color.srcAccessMask =
            (*m_render_pass_resources.color_layout == vk::ImageLayout::eUndefined)
                ? vk::AccessFlagBits2::eNone
                : vk::AccessFlagBits2::eMemoryRead | vk::AccessFlagBits2::eMemoryWrite;
        to_color.dstAccessMask = vk::AccessFlagBits2::eColorAttachmentWrite;
        to_color.oldLayout = *m_render_pass_resources.color_layout;
        to_color.newLayout = vk::ImageLayout::eColorAttachmentOptimal;
        to_color.image = *color_image.image();
        to_color.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
        to_color.subresourceRange.baseMipLevel = 0;
        to_color.subresourceRange.levelCount = 1;
        to_color.subresourceRange.baseArrayLayer = 0;
        to_color.subresourceRange.layerCount = 1;

        vk::ImageMemoryBarrier2 to_depth{};
        to_depth.srcStageMask = vk::PipelineStageFlagBits2::eTopOfPipe;
        to_depth.dstStageMask = vk::PipelineStageFlagBits2::eEarlyFragmentTests | vk::PipelineStageFlagBits2::eLateFragmentTests;
        to_depth.srcAccessMask = vk::AccessFlagBits2::eNone;
        to_depth.dstAccessMask = vk::AccessFlagBits2::eDepthStencilAttachmentWrite;
        to_depth.oldLayout = vk::ImageLayout::eUndefined;
        to_depth.newLayout = vk::ImageLayout::eDepthAttachmentOptimal;
        to_depth.image = *depth_image.image();
        to_depth.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eDepth;
        to_depth.subresourceRange.baseMipLevel = 0;
        to_depth.subresourceRange.levelCount = 1;
        to_depth.subresourceRange.baseArrayLayer = 0;
        to_depth.subresourceRange.layerCount = 1;

        std::array<vk::ImageMemoryBarrier2, 2> to_render_barriers = {to_color, to_depth};
        vk::DependencyInfo to_render_dep{};
        to_render_dep.imageMemoryBarrierCount = static_cast<uint32_t>(to_render_barriers.size());
        to_render_dep.pImageMemoryBarriers = to_render_barriers.data();
        cmd.pipelineBarrier2(to_render_dep);

        vk::ClearValue clear_value = vk::ClearValue{vk::ClearColorValue{0.0f, 0.0f, 0.0f, 1.0f}};
        vk::RenderingAttachmentInfo color_attachment_info{};
        color_attachment_info.imageView = *color_image.image_view();
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
        rendering_info.renderArea.extent = m_render_pass_resources.extent;
        rendering_info.layerCount = 1;
        rendering_info.colorAttachmentCount = 1;
        rendering_info.pColorAttachments = &color_attachment_info;
        rendering_info.pDepthAttachment = &depth_attachment_info;

        cmd.beginRendering(rendering_info);
        cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *m_pipeline);

        vk::Viewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = static_cast<float>(m_render_pass_resources.extent.width);
        viewport.height = static_cast<float>(m_render_pass_resources.extent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        cmd.setViewport(0, viewport);

        vk::Rect2D scissor{};
        scissor.offset = vk::Offset2D{0, 0};
        scissor.extent = m_render_pass_resources.extent;
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
        *m_render_pass_resources.color_layout = vk::ImageLayout::eColorAttachmentOptimal;
    }
};

class ForwardPipeline : public RenderPipelineBase,
                        public IFramePreparePipeline,
                        public IResourceAwarePipeline,
                        public IFrameColorSource,
                        public ISceneViewportSink {
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

    std::vector<std::unique_ptr<rhi::Image>> m_color_images{};
    std::vector<vk::ImageLayout> m_color_image_layouts{};
    std::vector<std::unique_ptr<rhi::Image>> m_depth_images{};
    vk::Extent2D m_scene_target_extent{};
    vk::Extent2D m_requested_scene_extent{};
    bool m_scene_extent_dirty{false};

    std::unique_ptr<rhi::DescriptorSetLayout> m_per_object_layout{nullptr};
    std::unique_ptr<rhi::DescriptorPool> m_descriptor_pool{nullptr};

    resource::ResourceManager* m_resource_manager{};

    std::unique_ptr<render::ForwardPass> m_forward_pass{nullptr};

public:
    ForwardPipeline(
        const render::PipelineRuntime& runtime,
        const ForwardPipelineConfig& config = {}
    )
        : RenderPipelineBase(runtime) {
        m_uniform_buffer_size = sizeof(UniformBufferObjectGpu);

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
    }

    ~ForwardPipeline() override {
        m_forward_pass.reset();
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

    void set_scene_viewport_extent(vk::Extent2D extent) override {
        if (extent.width == 0 || extent.height == 0) {
            return;
        }
        if (m_requested_scene_extent.width == extent.width &&
            m_requested_scene_extent.height == extent.height) {
            return;
        }
        m_requested_scene_extent = extent;
        m_scene_extent_dirty = true;
    }

    FrameColorSourceView frame_color_source_view(uint32_t frame_index) const override {
        if (frame_index >= m_color_images.size() ||
            frame_index >= m_color_image_layouts.size() ||
            m_color_images[frame_index] == nullptr) {
            return {};
        }

        return FrameColorSourceView{
            .image_view = *m_color_images[frame_index]->image_view(),
            .image_layout = m_color_image_layouts[frame_index],
            .extent = vk::Extent2D{
                m_color_images[frame_index]->width(),
                m_color_images[frame_index]->height()
            }
        };
    }

    void on_resize(int /*width*/, int /*height*/) override {}

    void handle_swapchain_state_change(
        const FrameScheduler::SwapchainState& /*state*/,
        const SwapchainChangeSummary& diff
    ) override {
        if (diff.color_or_depth_changed()) {
            create_graphics_pipeline();
        }
        if (diff.extent_or_depth_changed()) {
            m_scene_extent_dirty = true;
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

        ensure_scene_targets(extent);

        const uint32_t frame_index = ctx.frame_index();
        if (frame_index >= m_object_uniform_buffers.size() ||
            frame_index >= m_object_sets.size() ||
            frame_index >= m_depth_images.size() ||
            frame_index >= m_color_images.size() ||
            frame_index >= m_color_image_layouts.size()) {
            throw std::runtime_error("ForwardPipeline frame resources are not ready.");
        }

        if (!m_forward_pass) {
            throw std::runtime_error("Forward pipeline pass is not initialized.");
        }

        const auto& scene_view = m_scene_view.value();
        if (scene_view.renderables.size() > kMaxRenderables) {
            throw std::runtime_error("Renderable count exceeds preallocated ForwardPipeline capacity.");
        }

        pbpt::math::mat4 proj = scene_view.camera.proj;
        auto& frame_uniform_buffers = m_object_uniform_buffers[frame_index];
        auto& frame_sets = m_object_sets[frame_index];

        std::vector<ForwardPass::DrawItem> draw_items{};
        draw_items.reserve(scene_view.renderables.size());

        for (std::size_t i = 0; i < scene_view.renderables.size(); ++i) {
            const auto& renderable = scene_view.renderables[i];
            auto& mesh = require_mesh(renderable.mesh);

            UniformBufferObjectGpu ubo{};
            ubo.model = pack_mat4_row_major(renderable.model);
            ubo.view = pack_mat4_row_major(scene_view.camera.view);
            ubo.proj = pack_mat4_row_major(proj);
            ubo.normal = pack_mat4_row_major(renderable.normal);
            ubo.base_color = {
                static_cast<float>(renderable.base_color.x()),
                static_cast<float>(renderable.base_color.y()),
                static_cast<float>(renderable.base_color.z()),
                static_cast<float>(renderable.base_color.w())
            };

            std::memcpy(frame_uniform_buffers[i]->mapped_data(), &ubo, sizeof(ubo));

            draw_items.emplace_back(ForwardPass::DrawItem{
                .mesh = &mesh,
                .per_object_set = &frame_sets[i]
            });
        }

        m_forward_pass->bind_render_pass_resources(ForwardPass::RenderPassResources{
            .color_image = m_color_images[frame_index].get(),
            .color_layout = &m_color_image_layouts[frame_index],
            .depth_image = m_depth_images[frame_index].get(),
            .extent = m_scene_target_extent,
            .draw_items = std::move(draw_items)
        });
        m_forward_pass->execute(ctx);

        present_offscreen_to_swapchain(ctx, frame_index);
    }

private:
    rhi::Mesh& require_mesh(resource::MeshHandle mesh_handle) {
        auto logger = utils::get_logger("render.pipeline.forward");
        if (!mesh_handle.is_valid()) {
            logger->error("Renderable mesh handle is invalid.");
            throw std::runtime_error("Renderable mesh handle is invalid.");
        }
        if (m_resource_manager == nullptr) {
            logger->error("ForwardPipeline missing resource manager while requesting mesh handle={}", mesh_handle.value);
            throw std::runtime_error("ForwardPipeline missing resource manager.");
        }
        return m_resource_manager->require_gpu<rtr::resource::MeshResourceKind>(mesh_handle, m_device);
    }

    void ensure_scene_targets(vk::Extent2D fallback_extent) {
        vk::Extent2D desired_extent = fallback_extent;
        if (m_requested_scene_extent.width > 0 && m_requested_scene_extent.height > 0) {
            desired_extent = m_requested_scene_extent;
        }

        const bool need_recreate =
            m_scene_extent_dirty ||
            m_scene_target_extent.width != desired_extent.width ||
            m_scene_target_extent.height != desired_extent.height ||
            m_color_images.empty() ||
            m_depth_images.empty();
        if (!need_recreate) {
            return;
        }

        m_device->wait_idle();
        m_scene_target_extent = desired_extent;
        create_color_images();
        create_depth_images();
        m_scene_extent_dirty = false;
    }

    void present_offscreen_to_swapchain(FrameContext& ctx, uint32_t frame_index) {
        auto& cmd = ctx.cmd().command_buffer();
        auto& color_image = *m_color_images[frame_index];
        auto& color_layout = m_color_image_layouts[frame_index];

        vk::ImageMemoryBarrier2 offscreen_to_src{};
        offscreen_to_src.srcStageMask = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        offscreen_to_src.dstStageMask = vk::PipelineStageFlagBits2::eTransfer;
        offscreen_to_src.srcAccessMask = vk::AccessFlagBits2::eColorAttachmentWrite;
        offscreen_to_src.dstAccessMask = vk::AccessFlagBits2::eTransferRead;
        offscreen_to_src.oldLayout = color_layout;
        offscreen_to_src.newLayout = vk::ImageLayout::eTransferSrcOptimal;
        offscreen_to_src.image = *color_image.image();
        offscreen_to_src.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
        offscreen_to_src.subresourceRange.baseMipLevel = 0;
        offscreen_to_src.subresourceRange.levelCount = 1;
        offscreen_to_src.subresourceRange.baseArrayLayer = 0;
        offscreen_to_src.subresourceRange.layerCount = 1;

        vk::ImageMemoryBarrier2 swapchain_to_dst{};
        swapchain_to_dst.srcStageMask = vk::PipelineStageFlagBits2::eTopOfPipe;
        swapchain_to_dst.dstStageMask = vk::PipelineStageFlagBits2::eTransfer;
        swapchain_to_dst.srcAccessMask = vk::AccessFlagBits2::eNone;
        swapchain_to_dst.dstAccessMask = vk::AccessFlagBits2::eTransferWrite;
        swapchain_to_dst.oldLayout = vk::ImageLayout::eUndefined;
        swapchain_to_dst.newLayout = vk::ImageLayout::eTransferDstOptimal;
        swapchain_to_dst.image = ctx.swapchain_image();
        swapchain_to_dst.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
        swapchain_to_dst.subresourceRange.baseMipLevel = 0;
        swapchain_to_dst.subresourceRange.levelCount = 1;
        swapchain_to_dst.subresourceRange.baseArrayLayer = 0;
        swapchain_to_dst.subresourceRange.layerCount = 1;

        std::array<vk::ImageMemoryBarrier2, 2> to_blit_barriers = {
            offscreen_to_src,
            swapchain_to_dst
        };
        vk::DependencyInfo to_blit_dep{};
        to_blit_dep.imageMemoryBarrierCount = static_cast<uint32_t>(to_blit_barriers.size());
        to_blit_dep.pImageMemoryBarriers = to_blit_barriers.data();
        cmd.pipelineBarrier2(to_blit_dep);

        vk::ImageBlit2 blit{};
        blit.srcSubresource.aspectMask = vk::ImageAspectFlagBits::eColor;
        blit.srcSubresource.mipLevel = 0;
        blit.srcSubresource.baseArrayLayer = 0;
        blit.srcSubresource.layerCount = 1;
        blit.srcOffsets[0] = vk::Offset3D{0, 0, 0};
        blit.srcOffsets[1] = vk::Offset3D{
            static_cast<int32_t>(m_scene_target_extent.width),
            static_cast<int32_t>(m_scene_target_extent.height),
            1
        };
        blit.dstSubresource.aspectMask = vk::ImageAspectFlagBits::eColor;
        blit.dstSubresource.mipLevel = 0;
        blit.dstSubresource.baseArrayLayer = 0;
        blit.dstSubresource.layerCount = 1;
        blit.dstOffsets[0] = vk::Offset3D{0, 0, 0};
        blit.dstOffsets[1] = vk::Offset3D{
            static_cast<int32_t>(ctx.render_extent().width),
            static_cast<int32_t>(ctx.render_extent().height),
            1
        };

        vk::BlitImageInfo2 blit_info{};
        blit_info.srcImage = *color_image.image();
        blit_info.srcImageLayout = vk::ImageLayout::eTransferSrcOptimal;
        blit_info.dstImage = ctx.swapchain_image();
        blit_info.dstImageLayout = vk::ImageLayout::eTransferDstOptimal;
        blit_info.filter = vk::Filter::eLinear;
        blit_info.regionCount = 1;
        blit_info.pRegions = &blit;
        cmd.blitImage2(blit_info);

        vk::ImageMemoryBarrier2 swapchain_to_color{};
        swapchain_to_color.srcStageMask = vk::PipelineStageFlagBits2::eTransfer;
        swapchain_to_color.dstStageMask = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        swapchain_to_color.srcAccessMask = vk::AccessFlagBits2::eTransferWrite;
        swapchain_to_color.dstAccessMask = vk::AccessFlagBits2::eColorAttachmentWrite;
        swapchain_to_color.oldLayout = vk::ImageLayout::eTransferDstOptimal;
        swapchain_to_color.newLayout = vk::ImageLayout::eColorAttachmentOptimal;
        swapchain_to_color.image = ctx.swapchain_image();
        swapchain_to_color.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
        swapchain_to_color.subresourceRange.baseMipLevel = 0;
        swapchain_to_color.subresourceRange.levelCount = 1;
        swapchain_to_color.subresourceRange.baseArrayLayer = 0;
        swapchain_to_color.subresourceRange.layerCount = 1;

        vk::ImageMemoryBarrier2 offscreen_to_sampled{};
        offscreen_to_sampled.srcStageMask = vk::PipelineStageFlagBits2::eTransfer;
        offscreen_to_sampled.dstStageMask = vk::PipelineStageFlagBits2::eFragmentShader;
        offscreen_to_sampled.srcAccessMask = vk::AccessFlagBits2::eTransferRead;
        offscreen_to_sampled.dstAccessMask = vk::AccessFlagBits2::eShaderRead;
        offscreen_to_sampled.oldLayout = vk::ImageLayout::eTransferSrcOptimal;
        offscreen_to_sampled.newLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
        offscreen_to_sampled.image = *color_image.image();
        offscreen_to_sampled.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
        offscreen_to_sampled.subresourceRange.baseMipLevel = 0;
        offscreen_to_sampled.subresourceRange.levelCount = 1;
        offscreen_to_sampled.subresourceRange.baseArrayLayer = 0;
        offscreen_to_sampled.subresourceRange.layerCount = 1;

        std::array<vk::ImageMemoryBarrier2, 2> to_final_barriers = {
            swapchain_to_color,
            offscreen_to_sampled
        };
        vk::DependencyInfo to_final_dep{};
        to_final_dep.imageMemoryBarrierCount = static_cast<uint32_t>(to_final_barriers.size());
        to_final_dep.pImageMemoryBarriers = to_final_barriers.data();
        cmd.pipelineBarrier2(to_final_dep);

        color_layout = vk::ImageLayout::eShaderReadOnlyOptimal;
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

    void create_color_images() {
        m_color_images.clear();
        m_color_images.reserve(m_frame_count);
        m_color_image_layouts.assign(m_frame_count, vk::ImageLayout::eUndefined);

        const vk::ImageUsageFlags usage =
            vk::ImageUsageFlagBits::eColorAttachment |
            vk::ImageUsageFlagBits::eSampled |
            vk::ImageUsageFlagBits::eTransferSrc;

        for (uint32_t i = 0; i < m_frame_count; ++i) {
            m_color_images.emplace_back(std::make_unique<rhi::Image>(
                rhi::Image(
                    m_device,
                    m_scene_target_extent.width,
                    m_scene_target_extent.height,
                    m_color_format,
                    vk::ImageTiling::eOptimal,
                    usage,
                    vk::MemoryPropertyFlagBits::eDeviceLocal,
                    vk::ImageAspectFlagBits::eColor,
                    false
                )
            ));
        }
    }

    void create_depth_images() {
        if (m_scene_target_extent.width == 0 || m_scene_target_extent.height == 0) {
            return;
        }
        m_depth_images = make_per_frame_depth_images(m_scene_target_extent, m_depth_format);
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
