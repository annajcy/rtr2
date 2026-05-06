#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>

#include "rtr/rhi/buffer.hpp"
#include "rtr/rhi/descriptor.hpp"
#include "rtr/rhi/shader_module.hpp"
#include "rtr/rhi/texture.hpp"
#include "rtr/system/render/pipeline/shadertoy/shadertoy_compute_pass.hpp"
#include "rtr/system/render/render_pipeline.hpp"
#include "rtr/system/render/render_resource_state.hpp"
#include "rtr/system/render/scene_target_controller.hpp"
#include "vulkan/vulkan.hpp"

namespace rtr::system::render {

struct ShaderToyPipelineConfig {
    std::string compute_shader_filename{"shadertoy_compute_comp.spv"};
};

class ShaderToyPipeline final : public RenderPipeline {
    struct ShaderToyFrameTargets {
        std::array<FrameTrackedImage, rhi::kFramesInFlight> offscreen_images;

        explicit ShaderToyFrameTargets(std::array<FrameTrackedImage, rhi::kFramesInFlight>&& offscreen_images_in)
            : offscreen_images(std::move(offscreen_images_in)) {}
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
    std::array<std::uint64_t, rhi::kFramesInFlight> m_compute_set_generation{};
    SceneTargetController<ShaderToyFrameTargets> m_scene_targets;
    std::array<float, 4> m_params{1.0f, 0.0f, 0.0f, 0.0f};

    ComputePass m_compute_pass;

public:
    ShaderToyPipeline(const render::PipelineRuntime& runtime, const ShaderToyPipelineConfig& config = {})
        : RenderPipeline(runtime),
          m_offscreen_format(pick_offscreen_format()),
          m_compute_shader_module(build_shader_module(
              m_device,
              resolve_shader_path(runtime, config.compute_shader_filename),
              vk::ShaderStageFlagBits::eCompute
          )),
          m_compute_layout(build_compute_layout(m_device)),
          m_descriptor_pool(build_descriptor_pool(m_device, m_compute_layout, static_cast<uint32_t>(rhi::kFramesInFlight))),
          m_compute_sets(vector_to_frame_array(
              m_descriptor_pool.allocate_multiple(m_compute_layout, static_cast<uint32_t>(rhi::kFramesInFlight)),
              "ShaderToyPipeline compute descriptor sets"
          )),
          m_compute_pipeline_layout(build_pipeline_layout(m_device, m_compute_layout)),
          m_uniform_buffer_size(sizeof(ShaderToyUniformBufferObject)),
          m_uniform_buffers(make_per_frame_mapped_uniform_buffers(m_uniform_buffer_size)),
          m_scene_targets(*this, "ShaderToyPipeline"),
          m_compute_pass(m_compute_pipeline_layout, m_compute_pipeline) {
        build_compute_pipeline();
    }

    ~ShaderToyPipeline() override = default;

    std::array<float, 4>& params() { return m_params; }
    const std::array<float, 4>& params() const { return m_params; }

    PipelineFinalOutput final_output(uint32_t frame_index) override {
        auto& frame_targets = m_scene_targets.require_targets();
        if (frame_index >= rhi::kFramesInFlight) {
            throw std::runtime_error("ShaderToyPipeline final output frame index out of range.");
        }
        return PipelineFinalOutput{
            .color = frame_targets.offscreen_images[frame_index].view(),
            .extent = m_scene_targets.scene_extent()
        };
    }

    void on_resize(int /*w*/, int /*h*/) override {}

    void handle_swapchain_state_change(const FrameScheduler::SwapchainState& /*state*/,
                                       const SwapchainChangeSummary& diff) override {
        if (diff.extent_changed) {
            m_scene_targets.on_swapchain_extent_changed();
        }
    }

    void render(FrameContext& ctx) override {
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

        m_compute_pass.execute(
            ctx,
            ComputePass::RenderPassResources{
                .uniform_buffer = m_uniform_buffers[frame_index],
                .offscreen = tracked_offscreen.view(),
                .compute_set = m_compute_sets[frame_index],
                .i_params = m_params
            }
        );

        auto& cmd = ctx.cmd().command_buffer();
        vk::ImageMemoryBarrier2 to_sampled{};
        to_sampled.srcStageMask = vk::PipelineStageFlagBits2::eComputeShader;
        to_sampled.dstStageMask = vk::PipelineStageFlagBits2::eFragmentShader;
        to_sampled.srcAccessMask = vk::AccessFlagBits2::eShaderStorageWrite;
        to_sampled.dstAccessMask = vk::AccessFlagBits2::eShaderRead;
        to_sampled.oldLayout = tracked_offscreen.layout;
        to_sampled.newLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
        to_sampled.image = *tracked_offscreen.image.image();
        to_sampled.subresourceRange = {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};

        vk::DependencyInfo dependency{};
        dependency.imageMemoryBarrierCount = 1;
        dependency.pImageMemoryBarriers = &to_sampled;
        cmd.pipelineBarrier2(dependency);
        tracked_offscreen.layout = vk::ImageLayout::eShaderReadOnlyOptimal;
    }

private:
    static rhi::ShaderModule build_shader_module(
        rhi::Device& device,
        const std::filesystem::path& shader_path,
        vk::ShaderStageFlagBits stage
    ) {
        return rhi::ShaderModule::from_file(device, shader_path.string(), stage);
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
        vk::ComputePipelineCreateInfo info{};
        info.stage = m_compute_shader_module.stage_create_info();
        info.layout = *m_compute_pipeline_layout;
        m_compute_pipeline = vk::raii::Pipeline{m_device.device(), nullptr, info};
    }

    std::array<FrameTrackedImage, rhi::kFramesInFlight> create_offscreen_images(vk::Extent2D scene_extent) const {
        const vk::ImageUsageFlags usage = vk::ImageUsageFlagBits::eStorage |
                                          vk::ImageUsageFlagBits::eSampled |
                                          vk::ImageUsageFlagBits::eTransferSrc;
        return make_frame_array<FrameTrackedImage>([&](uint32_t) {
            return FrameTrackedImage{
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

}  // namespace rtr::system::render
