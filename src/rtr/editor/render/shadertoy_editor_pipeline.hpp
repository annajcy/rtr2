#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rtr/editor/core/editor_capture.hpp"
#include "rtr/editor/core/editor_host.hpp"
#include "rtr/editor/render/editor_imgui_pass.hpp"
#include "rtr/rhi/buffer.hpp"
#include "rtr/rhi/descriptor.hpp"
#include "rtr/rhi/shader_module.hpp"
#include "rtr/rhi/texture.hpp"
#include "rtr/system/render/frame_color_source.hpp"
#include "rtr/system/render/pipeline.hpp"
#include "rtr/system/render/pipeline/shadertoy/shadertoy_compute_pass.hpp"
#include "vulkan/vulkan.hpp"

// ============================================================================
// ShaderToyEditorPipeline (self-contained, composition-based)
//
// Sequence:
//   1. ComputePass — writes to an offscreen storage image.
//   2. Image barriers — offscreen to ShaderReadOnlyOptimal,
//                       swapchain to ColorAttachmentOptimal.
//   3. EditorImGuiPass — renders editor UI, scene view samples offscreen.
// ============================================================================

namespace rtr::editor::render {

class ShaderToyEditorPipeline final : public system::render::RenderPipelineBase,
                                      public system::render::IFrameColorSource,
                                      public system::render::ISceneViewportSink,
                                      public IEditorInputCaptureSource {
    struct OffscreenFrameResources {
        std::unique_ptr<rhi::Image> image{};
        vk::ImageLayout             layout{vk::ImageLayout::eUndefined};
    };

    vk::Format m_offscreen_format{vk::Format::eUndefined};

    std::unique_ptr<rhi::ShaderModule>        m_compute_shader_module{nullptr};
    std::unique_ptr<rhi::DescriptorSetLayout> m_compute_layout{nullptr};
    std::unique_ptr<rhi::DescriptorPool>      m_descriptor_pool{nullptr};
    std::vector<vk::raii::DescriptorSet>      m_compute_sets{};

    vk::raii::PipelineLayout m_compute_pipeline_layout{nullptr};
    vk::raii::Pipeline       m_compute_pipeline{nullptr};

    vk::DeviceSize                            m_uniform_buffer_size{0};
    std::vector<std::unique_ptr<rhi::Buffer>> m_uniform_buffers{};

    std::vector<OffscreenFrameResources> m_offscreen_frame_resources{};
    vk::Extent2D                         m_scene_target_extent{};
    vk::Extent2D                         m_requested_scene_extent{};
    bool                                 m_scene_extent_dirty{false};

    std::array<float, 4> m_params{1.0f, 0.0f, 0.0f, 0.0f};

    std::unique_ptr<system::render::ComputePass> m_compute_pass{nullptr};
    std::unique_ptr<EditorImGuiPass>             m_editor_pass{nullptr};

public:
    ShaderToyEditorPipeline(const system::render::PipelineRuntime& runtime, std::shared_ptr<EditorHost> editor_host)
        : system::render::RenderPipelineBase(runtime) {
        using namespace system::render;

        m_uniform_buffer_size = sizeof(ShaderToyUniformBufferObject);
        m_offscreen_format    = pick_offscreen_format();

        m_compute_shader_module = std::make_unique<rhi::ShaderModule>(rhi::ShaderModule::from_file(
            *m_device,
            "/Users/jinceyang/Desktop/codebase/graphics/rtr2/build/Debug/shaders/compiled/shadertoy_compute_comp.spv",
            vk::ShaderStageFlagBits::eCompute));

        m_uniform_buffers = make_per_frame_mapped_uniform_buffers(m_uniform_buffer_size);

        rhi::DescriptorSetLayout::Builder compute_layout_builder;
        compute_layout_builder.add_binding(0, vk::DescriptorType::eUniformBuffer, vk::ShaderStageFlagBits::eCompute)
            .add_binding(1, vk::DescriptorType::eStorageImage, vk::ShaderStageFlagBits::eCompute);
        m_compute_layout = std::make_unique<rhi::DescriptorSetLayout>(compute_layout_builder.build(*m_device));

        rhi::DescriptorPool::Builder pool_builder;
        pool_builder.add_layout(*m_compute_layout, m_frame_count)
            .set_flags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
        m_descriptor_pool = std::make_unique<rhi::DescriptorPool>(pool_builder.build(*m_device));

        m_compute_sets = m_descriptor_pool->allocate_multiple(*m_compute_layout, m_frame_count);

        {
            std::array<vk::DescriptorSetLayout, 1> layouts{*m_compute_layout->layout()};
            vk::PipelineLayoutCreateInfo           info{};
            info.setLayoutCount       = static_cast<uint32_t>(layouts.size());
            info.pSetLayouts          = layouts.data();
            m_compute_pipeline_layout = vk::raii::PipelineLayout{m_device->device(), info};
        }

        build_compute_pipeline();

        m_compute_pass = std::make_unique<ComputePass>(&m_compute_pipeline_layout, &m_compute_pipeline);
        m_editor_pass  = std::make_unique<EditorImGuiPass>(runtime, std::move(editor_host), this);
    }

    ~ShaderToyEditorPipeline() override = default;

    std::array<float, 4>&       params() { return m_params; }
    const std::array<float, 4>& params() const { return m_params; }

    // -----------------------------------------------------------------------
    // IEditorInputCaptureSource
    // -----------------------------------------------------------------------
    bool wants_imgui_capture_mouse() const override { return m_editor_pass->wants_capture_mouse(); }
    bool wants_imgui_capture_keyboard() const override { return m_editor_pass->wants_capture_keyboard(); }

    // -----------------------------------------------------------------------
    // IFrameColorSource
    // -----------------------------------------------------------------------
    system::render::FrameColorSourceView frame_color_source_view(uint32_t frame_index) const override {
        if (frame_index >= m_offscreen_frame_resources.size() || !m_offscreen_frame_resources[frame_index].image)
            return {};
        const auto& f = m_offscreen_frame_resources[frame_index];
        return system::render::FrameColorSourceView{.image_view   = *f.image->image_view(),
                                                    .image_layout = f.layout,
                                                    .extent       = {f.image->width(), f.image->height()}};
    }

    // -----------------------------------------------------------------------
    // ISceneViewportSink
    // -----------------------------------------------------------------------
    void set_scene_viewport_extent(vk::Extent2D extent) override {
        if (extent.width == 0 || extent.height == 0)
            return;
        if (m_requested_scene_extent.width == extent.width && m_requested_scene_extent.height == extent.height)
            return;
        m_requested_scene_extent = extent;
        m_scene_extent_dirty     = true;
    }

    // -----------------------------------------------------------------------
    // RenderPipelineBase
    // -----------------------------------------------------------------------
    void on_resize(int /*w*/, int /*h*/) override {}

    void handle_swapchain_state_change(const system::render::ActiveFrameScheduler::SwapchainState& state,
                                       const system::render::SwapchainChangeSummary&         diff) override {
        if (diff.extent_or_depth_changed())
            m_scene_extent_dirty = true;
        m_editor_pass->on_swapchain_recreated(state.image_count, state.color_format, state.depth_format);
    }

    // -----------------------------------------------------------------------
    // IRenderPipeline
    // -----------------------------------------------------------------------
    void render(system::render::FrameContext& ctx) override {
        using namespace system::render;

        const auto extent = ctx.render_extent();
        if (extent.width == 0 || extent.height == 0)
            return;

        ensure_scene_targets(extent);
        const uint32_t frame_index = ctx.frame_index();

        // --- 1. ComputePass ---
        auto& frame = m_offscreen_frame_resources[frame_index];
        m_compute_pass->execute(
            ctx,
            ComputePass::RenderPassResources{.uniform_buffer   = m_uniform_buffers[frame_index].get(),
                                             .offscreen_image  = frame.image.get(),
                                             .offscreen_layout = &frame.layout,
                                             .compute_set      = &m_compute_sets[frame_index],
                                             .i_params         = m_params});

        // --- 2. Image barriers ---
        auto& cmd = ctx.cmd().command_buffer();

        // offscreen: eGeneral → eShaderReadOnlyOptimal
        vk::ImageMemoryBarrier2 offscreen_to_sampled{};
        offscreen_to_sampled.srcStageMask     = vk::PipelineStageFlagBits2::eComputeShader;
        offscreen_to_sampled.dstStageMask     = vk::PipelineStageFlagBits2::eFragmentShader;
        offscreen_to_sampled.srcAccessMask    = vk::AccessFlagBits2::eShaderStorageWrite;
        offscreen_to_sampled.dstAccessMask    = vk::AccessFlagBits2::eShaderRead;
        offscreen_to_sampled.oldLayout        = frame.layout;
        offscreen_to_sampled.newLayout        = vk::ImageLayout::eShaderReadOnlyOptimal;
        offscreen_to_sampled.image            = *frame.image->image();
        offscreen_to_sampled.subresourceRange = {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};

        // swapchain: eUndefined → eColorAttachmentOptimal
        vk::ImageMemoryBarrier2 swapchain_to_color{};
        swapchain_to_color.srcStageMask     = vk::PipelineStageFlagBits2::eTopOfPipe;
        swapchain_to_color.dstStageMask     = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        swapchain_to_color.srcAccessMask    = vk::AccessFlagBits2::eNone;
        swapchain_to_color.dstAccessMask    = vk::AccessFlagBits2::eColorAttachmentWrite;
        swapchain_to_color.oldLayout        = vk::ImageLayout::eUndefined;
        swapchain_to_color.newLayout        = vk::ImageLayout::eColorAttachmentOptimal;
        swapchain_to_color.image            = ctx.swapchain_image();
        swapchain_to_color.subresourceRange = {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};

        std::array<vk::ImageMemoryBarrier2, 2> barriers{offscreen_to_sampled, swapchain_to_color};
        vk::DependencyInfo                     dep{};
        dep.imageMemoryBarrierCount = static_cast<uint32_t>(barriers.size());
        dep.pImageMemoryBarriers    = barriers.data();
        cmd.pipelineBarrier2(dep);

        frame.layout = vk::ImageLayout::eShaderReadOnlyOptimal;

        // --- 3. EditorImGuiPass ---
        auto source_view = frame_color_source_view(frame_index);
        m_editor_pass->execute(ctx, EditorImGuiPass::RenderPassResources{.scene_image_view   = source_view.image_view,
                                                                          .scene_image_layout = source_view.image_layout,
                                                                          .scene_extent       = source_view.extent});
    }

private:
    void ensure_scene_targets(vk::Extent2D fallback) {
        vk::Extent2D desired = fallback;
        if (m_requested_scene_extent.width > 0 && m_requested_scene_extent.height > 0)
            desired = m_requested_scene_extent;

        const bool need_recreate = m_scene_extent_dirty || m_scene_target_extent.width != desired.width ||
                                   m_scene_target_extent.height != desired.height ||
                                   m_offscreen_frame_resources.empty();
        if (!need_recreate)
            return;

        m_device->wait_idle();
        m_scene_target_extent = desired;
        create_offscreen_images();
        refresh_compute_descriptors();
        m_scene_extent_dirty = false;
    }

    vk::Format pick_offscreen_format() const {
        auto supports = [this](vk::Format fmt) {
            const auto f = m_device->physical_device().getFormatProperties(fmt).optimalTilingFeatures;
            return (f & vk::FormatFeatureFlagBits::eStorageImage) == vk::FormatFeatureFlagBits::eStorageImage &&
                   (f & vk::FormatFeatureFlagBits::eSampledImage) == vk::FormatFeatureFlagBits::eSampledImage;
        };
        if (supports(vk::Format::eR16G16B16A16Sfloat))
            return vk::Format::eR16G16B16A16Sfloat;
        if (supports(vk::Format::eR8G8B8A8Unorm))
            return vk::Format::eR8G8B8A8Unorm;
        throw std::runtime_error("No supported offscreen format.");
    }

    void build_compute_pipeline() {
        vk::ComputePipelineCreateInfo info{};
        info.stage         = m_compute_shader_module->stage_create_info();
        info.layout        = *m_compute_pipeline_layout;
        m_compute_pipeline = vk::raii::Pipeline{m_device->device(), nullptr, info};
    }

    void create_offscreen_images() {
        m_offscreen_frame_resources.clear();
        m_offscreen_frame_resources.reserve(m_frame_count);
        const vk::ImageUsageFlags usage = vk::ImageUsageFlagBits::eStorage | vk::ImageUsageFlagBits::eSampled;
        for (uint32_t i = 0; i < m_frame_count; ++i) {
            OffscreenFrameResources res{};
            res.image = std::make_unique<rhi::Image>(
                rhi::Image(*m_device, m_scene_target_extent.width, m_scene_target_extent.height, m_offscreen_format,
                           vk::ImageTiling::eOptimal, usage, vk::MemoryPropertyFlagBits::eDeviceLocal,
                           vk::ImageAspectFlagBits::eColor, false));
            res.layout = vk::ImageLayout::eUndefined;
            m_offscreen_frame_resources.push_back(std::move(res));
        }
    }

    void refresh_compute_descriptors() {
        for (uint32_t i = 0; i < m_frame_count; ++i) {
            rhi::DescriptorWriter w;
            w.write_buffer(0, *m_uniform_buffers[i]->buffer(), 0, m_uniform_buffer_size);
            w.write_storage_image(1, *m_offscreen_frame_resources[i].image->image_view(), vk::ImageLayout::eGeneral);
            w.update(*m_device, *m_compute_sets[i]);
        }
    }

    std::vector<std::unique_ptr<rhi::Buffer>> make_per_frame_mapped_uniform_buffers(vk::DeviceSize size) {
        std::vector<std::unique_ptr<rhi::Buffer>> buffers;
        buffers.reserve(m_frame_count);
        for (uint32_t i = 0; i < m_frame_count; ++i) {
            auto buf = std::make_unique<rhi::Buffer>(
                rhi::Buffer::create_host_visible_buffer(*m_device, size, vk::BufferUsageFlagBits::eUniformBuffer));
            buf->map();
            buffers.emplace_back(std::move(buf));
        }
        return buffers;
    }
};

}  // namespace rtr::editor::render
