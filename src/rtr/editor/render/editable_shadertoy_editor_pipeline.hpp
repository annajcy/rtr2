#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
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
#include "rtr/system/render/pipeline/shadertoy/editable_shadertoy_reload_controller.hpp"
#include "rtr/system/render/pipeline/shadertoy/shadertoy_compute_pass.hpp"
#include "rtr/system/render/render_pipeline.hpp"
#include "rtr/system/render/render_resource_state.hpp"
#include "rtr/system/render/scene_target_controller.hpp"
#include "vulkan/vulkan.hpp"

namespace rtr::editor::render {

struct EditableShaderToyEditorPipelineConfig {
    std::string initial_shader_source_path{"shaders/editable_shadertoy_compute.slang"};
    bool auto_reload_enabled{true};
};

class EditableShaderToyEditorPipeline final : public system::render::RenderPipeline,
                                              public IEditorInputCaptureSource {
private:
    struct ShaderToyFrameTargets {
        std::array<system::render::FrameTrackedImage, rhi::kFramesInFlight> offscreen_images;

        explicit ShaderToyFrameTargets(
            std::array<system::render::FrameTrackedImage, rhi::kFramesInFlight>&& offscreen_images_in
        )
            : offscreen_images(std::move(offscreen_images_in)) {}
    };

    vk::Format m_offscreen_format{vk::Format::eUndefined};

    std::optional<rhi::ShaderModule> m_compute_shader_module{};
    rhi::DescriptorSetLayout m_compute_layout;
    rhi::DescriptorPool m_descriptor_pool;
    std::array<vk::raii::DescriptorSet, rhi::kFramesInFlight> m_compute_sets;

    vk::raii::PipelineLayout m_compute_pipeline_layout{nullptr};
    vk::raii::Pipeline m_compute_pipeline{nullptr};

    vk::DeviceSize m_uniform_buffer_size{0};
    std::array<rhi::Buffer, rhi::kFramesInFlight> m_uniform_buffers;
    std::array<std::uint64_t, rhi::kFramesInFlight> m_compute_set_generation{};

    system::render::SceneTargetController<ShaderToyFrameTargets> m_scene_targets;

    std::array<float, 4> m_params{1.0f, 0.0f, 0.0f, 0.0f};
    system::render::EditableShaderToyReloadController m_reload_controller;

    system::render::ComputePass m_compute_pass;
    EditorImGuiPass m_editor_pass;

public:
    EditableShaderToyEditorPipeline(
        const system::render::PipelineRuntime& runtime,
        std::shared_ptr<EditorHost> editor_host,
        const EditableShaderToyEditorPipelineConfig& config = {}
    )
        : system::render::RenderPipeline(runtime),
          m_offscreen_format(pick_offscreen_format()),
          m_compute_layout(build_compute_layout(m_device)),
          m_descriptor_pool(build_descriptor_pool(m_device, m_compute_layout, static_cast<uint32_t>(rhi::kFramesInFlight))),
          m_compute_sets(vector_to_frame_array(
              m_descriptor_pool.allocate_multiple(m_compute_layout, static_cast<uint32_t>(rhi::kFramesInFlight)),
              "EditableShaderToyEditorPipeline compute descriptor sets"
          )),
          m_compute_pipeline_layout(build_pipeline_layout(m_device, m_compute_layout)),
          m_uniform_buffer_size(sizeof(system::render::ShaderToyUniformBufferObject)),
          m_uniform_buffers(make_per_frame_mapped_uniform_buffers(m_uniform_buffer_size)),
          m_scene_targets(*this, "EditableShaderToyEditorPipeline"),
          m_reload_controller(config.initial_shader_source_path, config.auto_reload_enabled),
          m_compute_pass(m_compute_pipeline_layout, m_compute_pipeline),
          m_editor_pass(runtime, std::move(editor_host), *this) {}

    ~EditableShaderToyEditorPipeline() override = default;

    std::array<float, 4>& params() { return m_params; }
    const std::array<float, 4>& params() const { return m_params; }

    const std::string& shader_source_path() const { return m_reload_controller.shader_source_path(); }
    void set_shader_source_path(std::string shader_source_path) {
        m_reload_controller.set_shader_source_path(std::move(shader_source_path));
    }

    bool auto_reload_enabled() const { return m_reload_controller.auto_reload_enabled(); }
    void set_auto_reload_enabled(bool enabled) { m_reload_controller.set_auto_reload_enabled(enabled); }

    void request_shader_reload() { m_reload_controller.request_reload(); }

    const system::render::EditableShaderToyReloadState& reload_state() const {
        return m_reload_controller.reload_state();
    }

    bool wants_imgui_capture_mouse() const override { return m_editor_pass.wants_capture_mouse(); }
    bool wants_imgui_capture_keyboard() const override { return m_editor_pass.wants_capture_keyboard(); }

    void on_resize(int /*w*/, int /*h*/) override {}

    void prepare_frame(const system::render::FramePrepareContext& /*ctx*/) override {
        const auto reload_result = m_reload_controller.check_for_reload();
        if (!reload_result.should_compile) {
            return;
        }

        const auto compile_result =
            system::render::compile_editable_shadertoy_shader(reload_result.resolved_path);
        if (!compile_result.ok) {
            invalidate_program();
            m_reload_controller.apply_compile_failure(
                reload_result.resolved_path,
                reload_result.write_time,
                compile_result.diagnostics
            );
            return;
        }

        try {
            wait_for_scene_target_rebuild();
            m_compute_shader_module.emplace(
                m_device,
                compile_result.spirv_code,
                vk::ShaderStageFlagBits::eCompute,
                compile_result.entry_point
            );
            build_compute_pipeline();
            m_reload_controller.apply_compile_success(
                reload_result.resolved_path,
                compile_result.last_write_time
            );
        } catch (const std::exception& e) {
            invalidate_program();
            m_reload_controller.apply_compile_failure(
                reload_result.resolved_path,
                compile_result.last_write_time,
                e.what()
            );
        }
    }

    void handle_swapchain_state_change(const system::render::FrameScheduler::SwapchainState& state,
                                       const system::render::SwapchainChangeSummary& diff) override {
        if (diff.depth_format_changed) {
            m_scene_targets.request_recreate();
        }
        if (diff.extent_changed) {
            m_scene_targets.on_swapchain_extent_changed();
        }
        m_editor_pass.on_swapchain_recreated(state.image_count, state.color_format, state.depth_format);
    }

    void render(system::render::FrameContext& ctx) override {
        using namespace system::render;

        const auto extent = ctx.render_extent();
        if (extent.width == 0 || extent.height == 0) {
            return;
        }

        auto& frame_targets = m_scene_targets.ensure(
            ctx.frame_index(),
            extent,
            [this](vk::Extent2D desired_extent) { return create_frame_targets(desired_extent); },
            [](ShaderToyFrameTargets&) {}
        );

        const uint32_t frame_index = ctx.frame_index();
        auto& tracked_offscreen = frame_targets.offscreen_images[frame_index];
        refresh_compute_descriptor(frame_index, frame_targets);

        if (reload_state().has_valid_program &&
            static_cast<vk::Pipeline>(m_compute_pipeline) != vk::Pipeline{}) {
            m_compute_pass.execute(
                ctx,
                ComputePass::RenderPassResources{
                    .uniform_buffer = m_uniform_buffers[frame_index],
                    .offscreen = tracked_offscreen.view(),
                    .compute_set = m_compute_sets[frame_index],
                    .i_params = m_params
                }
            );
        } else {
            clear_offscreen_black(ctx, tracked_offscreen);
        }

        transition_offscreen_to_sampled(ctx, tracked_offscreen);
        transition_swapchain_to_color(ctx);

        m_editor_pass.execute(
            ctx,
            EditorImGuiPass::RenderPassResources{
                .scene_image_view = *tracked_offscreen.image.image_view(),
                .scene_image_layout = tracked_offscreen.layout,
                .scene_extent = {tracked_offscreen.image.width(), tracked_offscreen.image.height()}
            }
        );
    }

private:
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

    vk::Format pick_offscreen_format() const {
        auto supports = [this](vk::Format fmt) {
            const auto f = m_device.physical_device().getFormatProperties(fmt).optimalTilingFeatures;
            return (f & vk::FormatFeatureFlagBits::eStorageImage) == vk::FormatFeatureFlagBits::eStorageImage &&
                   (f & vk::FormatFeatureFlagBits::eSampledImage) == vk::FormatFeatureFlagBits::eSampledImage;
        };
        if (supports(vk::Format::eR16G16B16A16Sfloat)) {
            return vk::Format::eR16G16B16A16Sfloat;
        }
        if (supports(vk::Format::eR8G8B8A8Unorm)) {
            return vk::Format::eR8G8B8A8Unorm;
        }
        throw std::runtime_error("No supported offscreen format.");
    }

    void build_compute_pipeline() {
        if (!m_compute_shader_module.has_value()) {
            throw std::runtime_error("Editable ShaderToy compute shader module is missing.");
        }

        vk::ComputePipelineCreateInfo info{};
        info.stage = m_compute_shader_module->stage_create_info();
        info.layout = *m_compute_pipeline_layout;
        m_compute_pipeline = vk::raii::Pipeline{m_device.device(), nullptr, info};
    }

    void invalidate_program() {
        wait_for_scene_target_rebuild();
        m_compute_pipeline = nullptr;
        m_compute_shader_module.reset();
    }

    static std::pair<vk::PipelineStageFlags2, vk::AccessFlags2> source_state_for_layout(
        vk::ImageLayout layout
    ) {
        switch (layout) {
        case vk::ImageLayout::eShaderReadOnlyOptimal:
            return {vk::PipelineStageFlagBits2::eFragmentShader, vk::AccessFlagBits2::eShaderRead};
        case vk::ImageLayout::eGeneral:
            return {vk::PipelineStageFlagBits2::eComputeShader, vk::AccessFlagBits2::eShaderStorageWrite};
        case vk::ImageLayout::eTransferDstOptimal:
            return {vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferWrite};
        default:
            return {vk::PipelineStageFlagBits2::eTopOfPipe, vk::AccessFlagBits2::eNone};
        }
    }

    void clear_offscreen_black(system::render::FrameContext& ctx, system::render::FrameTrackedImage& tracked_offscreen) {
        auto& cmd = ctx.cmd().command_buffer();
        const auto [src_stage, src_access] = source_state_for_layout(tracked_offscreen.layout);

        vk::ImageMemoryBarrier2 to_transfer{};
        to_transfer.srcStageMask = src_stage;
        to_transfer.dstStageMask = vk::PipelineStageFlagBits2::eTransfer;
        to_transfer.srcAccessMask = src_access;
        to_transfer.dstAccessMask = vk::AccessFlagBits2::eTransferWrite;
        to_transfer.oldLayout = tracked_offscreen.layout;
        to_transfer.newLayout = vk::ImageLayout::eTransferDstOptimal;
        to_transfer.image = *tracked_offscreen.image.image();
        to_transfer.subresourceRange = {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};

        vk::DependencyInfo dependency{};
        dependency.imageMemoryBarrierCount = 1;
        dependency.pImageMemoryBarriers = &to_transfer;
        cmd.pipelineBarrier2(dependency);

        const vk::ClearColorValue clear_color(std::array<float, 4>{0.0f, 0.0f, 0.0f, 1.0f});
        const vk::ImageSubresourceRange range{vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};
        cmd.clearColorImage(
            *tracked_offscreen.image.image(),
            vk::ImageLayout::eTransferDstOptimal,
            clear_color,
            range
        );
        tracked_offscreen.layout = vk::ImageLayout::eTransferDstOptimal;
    }

    void transition_offscreen_to_sampled(
        system::render::FrameContext& ctx,
        system::render::FrameTrackedImage& tracked_offscreen
    ) {
        auto& cmd = ctx.cmd().command_buffer();
        const auto [src_stage, src_access] = source_state_for_layout(tracked_offscreen.layout);

        vk::ImageMemoryBarrier2 offscreen_to_sampled{};
        offscreen_to_sampled.srcStageMask = src_stage;
        offscreen_to_sampled.dstStageMask = vk::PipelineStageFlagBits2::eFragmentShader;
        offscreen_to_sampled.srcAccessMask = src_access;
        offscreen_to_sampled.dstAccessMask = vk::AccessFlagBits2::eShaderRead;
        offscreen_to_sampled.oldLayout = tracked_offscreen.layout;
        offscreen_to_sampled.newLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
        offscreen_to_sampled.image = *tracked_offscreen.image.image();
        offscreen_to_sampled.subresourceRange = {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};

        vk::DependencyInfo dependency{};
        dependency.imageMemoryBarrierCount = 1;
        dependency.pImageMemoryBarriers = &offscreen_to_sampled;
        cmd.pipelineBarrier2(dependency);

        tracked_offscreen.layout = vk::ImageLayout::eShaderReadOnlyOptimal;
    }

    void transition_swapchain_to_color(system::render::FrameContext& ctx) {
        auto& cmd = ctx.cmd().command_buffer();

        vk::ImageMemoryBarrier2 swapchain_to_color{};
        swapchain_to_color.srcStageMask = vk::PipelineStageFlagBits2::eTopOfPipe;
        swapchain_to_color.dstStageMask = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        swapchain_to_color.srcAccessMask = vk::AccessFlagBits2::eNone;
        swapchain_to_color.dstAccessMask = vk::AccessFlagBits2::eColorAttachmentWrite;
        swapchain_to_color.oldLayout = vk::ImageLayout::eUndefined;
        swapchain_to_color.newLayout = vk::ImageLayout::eColorAttachmentOptimal;
        swapchain_to_color.image = ctx.swapchain_image();
        swapchain_to_color.subresourceRange = {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};

        vk::DependencyInfo dependency{};
        dependency.imageMemoryBarrierCount = 1;
        dependency.pImageMemoryBarriers = &swapchain_to_color;
        cmd.pipelineBarrier2(dependency);
    }

    std::array<system::render::FrameTrackedImage, rhi::kFramesInFlight> create_offscreen_images(
        vk::Extent2D scene_extent
    ) const {
        const vk::ImageUsageFlags usage =
            vk::ImageUsageFlagBits::eStorage | vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eTransferDst;
        return make_frame_array<system::render::FrameTrackedImage>([&](uint32_t) {
            return system::render::FrameTrackedImage{
                rhi::Image(
                    m_device,
                    scene_extent.width,
                    scene_extent.height,
                    m_offscreen_format,
                    vk::ImageTiling::eOptimal,
                    usage,
                    vk::MemoryPropertyFlagBits::eDeviceLocal,
                    vk::ImageAspectFlagBits::eColor,
                    false
                ),
                vk::ImageLayout::eUndefined
            };
        });
    }

    ShaderToyFrameTargets create_frame_targets(vk::Extent2D scene_extent) const {
        return ShaderToyFrameTargets{create_offscreen_images(scene_extent)};
    }

    void refresh_compute_descriptor(uint32_t frame_index, ShaderToyFrameTargets& frame_targets) {
        const auto generation = m_scene_targets.active_generation();
        if (m_compute_set_generation[frame_index] == generation) {
            return;
        }

        rhi::DescriptorWriter writer;
        writer.write_buffer(0, *m_uniform_buffers[frame_index].buffer(), 0, m_uniform_buffer_size);
        writer.write_storage_image(1, *frame_targets.offscreen_images[frame_index].image.image_view(), vk::ImageLayout::eGeneral);
        writer.update(m_device, *m_compute_sets[frame_index]);
        m_compute_set_generation[frame_index] = generation;
    }
};

}  // namespace rtr::editor::render
