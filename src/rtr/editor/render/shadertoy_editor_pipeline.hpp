#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
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
    struct ShaderToyFrameTargets {
        std::array<rhi::Image, rhi::kFramesInFlight> offscreen_images;
        std::array<vk::ImageLayout, rhi::kFramesInFlight> offscreen_layouts;

        ShaderToyFrameTargets(
            std::array<rhi::Image, rhi::kFramesInFlight>&& offscreen_images_in,
            std::array<vk::ImageLayout, rhi::kFramesInFlight>&& offscreen_layouts_in
        )
            : offscreen_images(std::move(offscreen_images_in)),
              offscreen_layouts(std::move(offscreen_layouts_in)) {}
    };

    vk::Format m_offscreen_format{vk::Format::eUndefined};

    rhi::ShaderModule m_compute_shader_module;
    rhi::DescriptorSetLayout m_compute_layout;
    rhi::DescriptorPool m_descriptor_pool;
    std::array<vk::raii::DescriptorSet, rhi::kFramesInFlight> m_compute_sets;

    vk::raii::PipelineLayout m_compute_pipeline_layout{nullptr};
    vk::raii::Pipeline m_compute_pipeline{nullptr};

    vk::DeviceSize m_uniform_buffer_size{0};
    std::array<rhi::Buffer, rhi::kFramesInFlight> m_uniform_buffers;

    std::optional<ShaderToyFrameTargets> m_frame_targets{};
    vk::Extent2D m_scene_target_extent{};
    vk::Extent2D m_requested_scene_extent{};
    bool m_scene_extent_dirty{false};

    std::array<float, 4> m_params{1.0f, 0.0f, 0.0f, 0.0f};

    system::render::ComputePass m_compute_pass;
    EditorImGuiPass m_editor_pass;

public:
    ShaderToyEditorPipeline(const system::render::PipelineRuntime& runtime, std::shared_ptr<EditorHost> editor_host)
        : system::render::RenderPipelineBase(runtime),
          m_offscreen_format(pick_offscreen_format()),
          m_compute_shader_module(build_shader_module(
              m_device,
              "/Users/jinceyang/Desktop/codebase/graphics/rtr2/build/Debug/shaders/compiled/shadertoy_compute_comp.spv",
              vk::ShaderStageFlagBits::eCompute
          )),
          m_compute_layout(build_compute_layout(m_device)),
          m_descriptor_pool(build_descriptor_pool(m_device, m_compute_layout, static_cast<uint32_t>(rhi::kFramesInFlight))),
          m_compute_sets(vector_to_frame_array(
              m_descriptor_pool.allocate_multiple(m_compute_layout, static_cast<uint32_t>(rhi::kFramesInFlight)),
              "ShaderToyEditorPipeline compute descriptor sets"
          )),
          m_compute_pipeline_layout(build_pipeline_layout(m_device, m_compute_layout)),
          m_uniform_buffer_size(sizeof(system::render::ShaderToyUniformBufferObject)),
          m_uniform_buffers(make_per_frame_mapped_uniform_buffers(m_uniform_buffer_size)),
          m_compute_pass(&m_compute_pipeline_layout, &m_compute_pipeline),
          m_editor_pass(runtime, std::move(editor_host), this) {
        build_compute_pipeline();
    }

    ~ShaderToyEditorPipeline() override = default;

    std::array<float, 4>& params() { return m_params; }
    const std::array<float, 4>& params() const { return m_params; }

    // -----------------------------------------------------------------------
    // IEditorInputCaptureSource
    // -----------------------------------------------------------------------
    bool wants_imgui_capture_mouse() const override { return m_editor_pass.wants_capture_mouse(); }
    bool wants_imgui_capture_keyboard() const override { return m_editor_pass.wants_capture_keyboard(); }

    // -----------------------------------------------------------------------
    // IFrameColorSource
    // -----------------------------------------------------------------------
    system::render::FrameColorSourceView frame_color_source_view(uint32_t frame_index) const override {
        if (!m_frame_targets.has_value()) {
            return {};
        }
        if (frame_index >= rhi::kFramesInFlight) {
            return {};
        }
        const auto& offscreen = m_frame_targets->offscreen_images[frame_index];
        return system::render::FrameColorSourceView{
            .image_view = *offscreen.image_view(),
            .image_layout = m_frame_targets->offscreen_layouts[frame_index],
            .extent = {offscreen.width(), offscreen.height()}
        };
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
        m_scene_extent_dirty = true;
    }

    // -----------------------------------------------------------------------
    // RenderPipelineBase
    // -----------------------------------------------------------------------
    void on_resize(int /*w*/, int /*h*/) override {}

    void handle_swapchain_state_change(const system::render::FrameScheduler::SwapchainState& state,
                                       const system::render::SwapchainChangeSummary& diff) override {
        if (diff.extent_or_depth_changed())
            m_scene_extent_dirty = true;
        m_editor_pass.on_swapchain_recreated(state.image_count, state.color_format, state.depth_format);
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
        auto& frame_targets = require_frame_targets();

        // --- 1. ComputePass ---
        m_compute_pass.execute(
            ctx,
            ComputePass::RenderPassResources{
                .uniform_buffer = m_uniform_buffers[frame_index],
                .offscreen_image = frame_targets.offscreen_images[frame_index],
                .offscreen_layout = frame_targets.offscreen_layouts[frame_index],
                .compute_set = m_compute_sets[frame_index],
                .i_params = m_params
            }
        );

        // --- 2. Image barriers ---
        auto& cmd = ctx.cmd().command_buffer();

        // offscreen: eGeneral → eShaderReadOnlyOptimal
        vk::ImageMemoryBarrier2 offscreen_to_sampled{};
        offscreen_to_sampled.srcStageMask = vk::PipelineStageFlagBits2::eComputeShader;
        offscreen_to_sampled.dstStageMask = vk::PipelineStageFlagBits2::eFragmentShader;
        offscreen_to_sampled.srcAccessMask = vk::AccessFlagBits2::eShaderStorageWrite;
        offscreen_to_sampled.dstAccessMask = vk::AccessFlagBits2::eShaderRead;
        offscreen_to_sampled.oldLayout = frame_targets.offscreen_layouts[frame_index];
        offscreen_to_sampled.newLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
        offscreen_to_sampled.image = *frame_targets.offscreen_images[frame_index].image();
        offscreen_to_sampled.subresourceRange = {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};

        // swapchain: eUndefined → eColorAttachmentOptimal
        vk::ImageMemoryBarrier2 swapchain_to_color{};
        swapchain_to_color.srcStageMask = vk::PipelineStageFlagBits2::eTopOfPipe;
        swapchain_to_color.dstStageMask = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        swapchain_to_color.srcAccessMask = vk::AccessFlagBits2::eNone;
        swapchain_to_color.dstAccessMask = vk::AccessFlagBits2::eColorAttachmentWrite;
        swapchain_to_color.oldLayout = vk::ImageLayout::eUndefined;
        swapchain_to_color.newLayout = vk::ImageLayout::eColorAttachmentOptimal;
        swapchain_to_color.image = ctx.swapchain_image();
        swapchain_to_color.subresourceRange = {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};

        std::array<vk::ImageMemoryBarrier2, 2> barriers{offscreen_to_sampled, swapchain_to_color};
        vk::DependencyInfo dep{};
        dep.imageMemoryBarrierCount = static_cast<uint32_t>(barriers.size());
        dep.pImageMemoryBarriers = barriers.data();
        cmd.pipelineBarrier2(dep);

        frame_targets.offscreen_layouts[frame_index] = vk::ImageLayout::eShaderReadOnlyOptimal;

        // --- 3. EditorImGuiPass ---
        auto source_view = frame_color_source_view(frame_index);
        m_editor_pass.execute(
            ctx,
            EditorImGuiPass::RenderPassResources{
                .scene_image_view = source_view.image_view,
                .scene_image_layout = source_view.image_layout,
                .scene_extent = source_view.extent
            }
        );
    }

private:
    static rhi::ShaderModule build_shader_module(
        rhi::Device& device,
        const std::string& shader_path,
        vk::ShaderStageFlagBits stage
    ) {
        return rhi::ShaderModule::from_file(device, shader_path, stage);
    }

    static rhi::DescriptorSetLayout build_compute_layout(rhi::Device& device) {
        rhi::DescriptorSetLayout::Builder compute_layout_builder;
        compute_layout_builder.add_binding(0, vk::DescriptorType::eUniformBuffer, vk::ShaderStageFlagBits::eCompute)
            .add_binding(1, vk::DescriptorType::eStorageImage, vk::ShaderStageFlagBits::eCompute);
        return compute_layout_builder.build(device);
    }

    static rhi::DescriptorPool build_descriptor_pool(
        rhi::Device& device,
        const rhi::DescriptorSetLayout& compute_layout,
        uint32_t frame_count
    ) {
        rhi::DescriptorPool::Builder pool_builder;
        pool_builder.add_layout(compute_layout, frame_count)
            .set_flags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
        return pool_builder.build(device);
    }

    static vk::raii::PipelineLayout build_pipeline_layout(
        rhi::Device& device,
        const rhi::DescriptorSetLayout& layout
    ) {
        std::array<vk::DescriptorSetLayout, 1> layouts{*layout.layout()};
        vk::PipelineLayoutCreateInfo info{};
        info.setLayoutCount = static_cast<uint32_t>(layouts.size());
        info.pSetLayouts = layouts.data();
        return vk::raii::PipelineLayout{device.device(), info};
    }

    ShaderToyFrameTargets& require_frame_targets() {
        if (!m_frame_targets.has_value()) {
            throw std::runtime_error("ShaderToyEditorPipeline frame targets are not initialized.");
        }
        return *m_frame_targets;
    }

    void ensure_scene_targets(vk::Extent2D fallback) {
        vk::Extent2D desired = fallback;
        if (m_requested_scene_extent.width > 0 && m_requested_scene_extent.height > 0)
            desired = m_requested_scene_extent;

        const bool need_recreate = m_scene_extent_dirty || m_scene_target_extent.width != desired.width ||
                                   m_scene_target_extent.height != desired.height || !m_frame_targets.has_value();
        if (!need_recreate)
            return;

        m_device.wait_idle();
        m_scene_target_extent = desired;
        m_frame_targets = create_frame_targets();
        refresh_compute_descriptors();
        m_scene_extent_dirty = false;
    }

    vk::Format pick_offscreen_format() const {
        auto supports = [this](vk::Format fmt) {
            const auto f = m_device.physical_device().getFormatProperties(fmt).optimalTilingFeatures;
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
        info.stage = m_compute_shader_module.stage_create_info();
        info.layout = *m_compute_pipeline_layout;
        m_compute_pipeline = vk::raii::Pipeline{m_device.device(), nullptr, info};
    }

    std::array<rhi::Image, rhi::kFramesInFlight> create_offscreen_images() const {
        const vk::ImageUsageFlags usage = vk::ImageUsageFlagBits::eStorage | vk::ImageUsageFlagBits::eSampled;
        return make_frame_array<rhi::Image>([&](uint32_t) {
            return rhi::Image(
                m_device,
                m_scene_target_extent.width,
                m_scene_target_extent.height,
                m_offscreen_format,
                vk::ImageTiling::eOptimal,
                usage,
                vk::MemoryPropertyFlagBits::eDeviceLocal,
                vk::ImageAspectFlagBits::eColor,
                false
            );
        });
    }

    static std::array<vk::ImageLayout, rhi::kFramesInFlight> create_initial_offscreen_layouts() {
        return make_frame_array<vk::ImageLayout>([](uint32_t) { return vk::ImageLayout::eUndefined; });
    }

    ShaderToyFrameTargets create_frame_targets() const {
        return ShaderToyFrameTargets{
            create_offscreen_images(),
            create_initial_offscreen_layouts()
        };
    }

    void refresh_compute_descriptors() {
        auto& frame_targets = require_frame_targets();
        for (uint32_t i = 0; i < rhi::kFramesInFlight; ++i) {
            rhi::DescriptorWriter w;
            w.write_buffer(0, *m_uniform_buffers[i].buffer(), 0, m_uniform_buffer_size);
            w.write_storage_image(1, *frame_targets.offscreen_images[i].image_view(), vk::ImageLayout::eGeneral);
            w.update(m_device, *m_compute_sets[i]);
        }
    }
};

}  // namespace rtr::editor::render
