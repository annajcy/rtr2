#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rtr/rhi/buffer.hpp"
#include "rtr/rhi/descriptor.hpp"
#include "rtr/rhi/shader_module.hpp"
#include "rtr/rhi/texture.hpp"
#include "rtr/system/render/pass/present_image_pass.hpp"
#include "rtr/system/render/pipeline/shadertoy/shadertoy_compute_pass.hpp"
#include "rtr/system/render/render_pipeline.hpp"
#include "rtr/system/render/render_resource_state.hpp"
#include "rtr/system/render/scene_target_controller.hpp"
#include "vulkan/vulkan.hpp"

// ============================================================================
// ShaderToyPipeline (self-contained, composition-based)
//
// Sequence: ComputePass (offscreen) → PresentImagePass (to swapchain).
// All barriers are handled by the passes themselves.
// ============================================================================

namespace rtr::system::render {

struct ShaderToyPipelineConfig {
    std::string compute_shader_filename{"shadertoy_compute_comp.spv"};
    std::string present_vertex_shader_filename{"shadertoy_present_vert.spv"};
    std::string present_fragment_shader_filename{"shadertoy_present_frag.spv"};
};

class ShaderToyPipeline final : public RenderPipeline {
    struct ShaderToyFrameTargets {
        std::array<FrameTrackedImage, rhi::kFramesInFlight> offscreen_images;
        std::array<rhi::Image, rhi::kFramesInFlight> depth_images;

        ShaderToyFrameTargets(
            std::array<FrameTrackedImage, rhi::kFramesInFlight>&& offscreen_images_in,
            std::array<rhi::Image, rhi::kFramesInFlight>&& depth_images_in
        )
            : offscreen_images(std::move(offscreen_images_in)),
              depth_images(std::move(depth_images_in)) {}
    };

    vk::Format m_offscreen_format{vk::Format::eUndefined};

    rhi::ShaderModule m_compute_shader_module;
    rhi::ShaderModule m_present_vertex_shader_module;
    rhi::ShaderModule m_present_fragment_shader_module;

    rhi::DescriptorSetLayout m_compute_layout;
    rhi::DescriptorSetLayout m_present_layout;
    rhi::DescriptorPool m_descriptor_pool;
    rhi::Sampler m_offscreen_sampler;

    std::array<vk::raii::DescriptorSet, rhi::kFramesInFlight> m_compute_sets;
    std::array<vk::raii::DescriptorSet, rhi::kFramesInFlight> m_present_sets;

    vk::raii::PipelineLayout m_compute_pipeline_layout{nullptr};
    vk::raii::Pipeline m_compute_pipeline{nullptr};
    vk::raii::PipelineLayout m_present_pipeline_layout{nullptr};
    vk::raii::Pipeline m_present_pipeline{nullptr};

    vk::DeviceSize m_uniform_buffer_size{0};
    std::array<rhi::Buffer, rhi::kFramesInFlight> m_uniform_buffers;
    SceneTargetController<ShaderToyFrameTargets> m_scene_targets;

    ComputePass m_compute_pass;
    PresentImagePass m_present_pass;

public:
    ShaderToyPipeline(const render::PipelineRuntime& runtime, const ShaderToyPipelineConfig& config = {})
        : RenderPipeline(runtime),
          m_offscreen_format(pick_offscreen_format()),
          m_compute_shader_module(build_shader_module(
              m_device,
              resolve_shader_path(runtime, config.compute_shader_filename),
              vk::ShaderStageFlagBits::eCompute
          )),
          m_present_vertex_shader_module(build_shader_module(
              m_device,
              resolve_shader_path(runtime, config.present_vertex_shader_filename),
              vk::ShaderStageFlagBits::eVertex
          )),
          m_present_fragment_shader_module(build_shader_module(
              m_device,
              resolve_shader_path(runtime, config.present_fragment_shader_filename),
              vk::ShaderStageFlagBits::eFragment
          )),
          m_compute_layout(build_compute_layout(m_device)),
          m_present_layout(build_present_layout(m_device)),
          m_descriptor_pool(build_descriptor_pool(m_device, m_compute_layout, m_present_layout,
                                                  static_cast<uint32_t>(rhi::kFramesInFlight))),
          m_offscreen_sampler(rhi::Sampler::create_default(m_device, 1)),
          m_uniform_buffer_size(sizeof(ShaderToyUniformBufferObject)),
          m_uniform_buffers(make_per_frame_mapped_uniform_buffers(m_uniform_buffer_size)),
          m_compute_sets(vector_to_frame_array(
              m_descriptor_pool.allocate_multiple(m_compute_layout, static_cast<uint32_t>(rhi::kFramesInFlight)),
              "ShaderToyPipeline compute descriptor sets"
          )),
          m_present_sets(vector_to_frame_array(
              m_descriptor_pool.allocate_multiple(m_present_layout, static_cast<uint32_t>(rhi::kFramesInFlight)),
              "ShaderToyPipeline present descriptor sets"
          )),
          m_compute_pipeline_layout(build_pipeline_layout(m_device, m_compute_layout)),
          m_present_pipeline_layout(build_pipeline_layout(m_device, m_present_layout)),
          m_scene_targets(*this, "ShaderToyPipeline"),
          m_compute_pass(m_compute_pipeline_layout, m_compute_pipeline),
          m_present_pass(m_present_pipeline_layout, m_present_pipeline) {
        build_compute_pipeline();
        build_present_pipeline();
    }

    ~ShaderToyPipeline() override = default;

    void on_resize(int /*w*/, int /*h*/) override {}

    void handle_swapchain_state_change(const FrameScheduler::SwapchainState& /*state*/,
                                       const SwapchainChangeSummary& diff) override {
        if (diff.depth_format_changed) {
            m_scene_targets.request_recreate();
        }
        if (diff.extent_changed) {
            m_scene_targets.on_swapchain_extent_changed();
        }
        if (diff.color_or_depth_changed()) {
            build_present_pipeline();
        }
    }

    void render(FrameContext& ctx) override {
        const auto extent = ctx.render_extent();
        if (extent.width == 0 || extent.height == 0)
            return;

        auto& frame_targets = m_scene_targets.ensure(
            extent,
            [this](vk::Extent2D desired_extent) { return create_frame_targets(desired_extent); },
            [this](ShaderToyFrameTargets& targets) { refresh_compute_descriptors(targets); }
        );

        const uint32_t frame_index = ctx.frame_index();

        auto& tracked_offscreen = frame_targets.offscreen_images[frame_index];

        m_compute_pass.execute(
            ctx,
            ComputePass::RenderPassResources{
                .uniform_buffer = m_uniform_buffers[frame_index],
                .offscreen = tracked_offscreen.view(),
                .compute_set = m_compute_sets[frame_index]
            }
        );

        update_present_descriptor(frame_index, frame_targets);

        m_present_pass.execute(
            ctx,
            PresentImagePass::RenderPassResources{
                .offscreen = tracked_offscreen.view(),
                .depth_image = frame_targets.depth_images[frame_index],
                .present_set = m_present_sets[frame_index]
            }
        );
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

    static rhi::DescriptorSetLayout build_present_layout(rhi::Device& device) {
        rhi::DescriptorSetLayout::Builder present_layout_builder;
        present_layout_builder.add_binding(0, vk::DescriptorType::eCombinedImageSampler, vk::ShaderStageFlagBits::eFragment);
        return present_layout_builder.build(device);
    }

    static rhi::DescriptorPool build_descriptor_pool(
        rhi::Device& device,
        const rhi::DescriptorSetLayout& compute_layout,
        const rhi::DescriptorSetLayout& present_layout,
        uint32_t frame_count
    ) {
        rhi::DescriptorPool::Builder pool_builder;
        pool_builder.add_layout(compute_layout, frame_count)
            .add_layout(present_layout, frame_count)
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

    void update_present_descriptor(uint32_t frame_index, ShaderToyFrameTargets& frame_targets) {
        auto& tracked = frame_targets.offscreen_images[frame_index];
        rhi::DescriptorWriter w;
        w.write_combined_image(0, *tracked.image.image_view(), *m_offscreen_sampler.sampler(),
                               vk::ImageLayout::eShaderReadOnlyOptimal);
        w.update(m_device, *m_present_sets[frame_index]);
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

    void build_present_pipeline() {
        std::vector<vk::PipelineShaderStageCreateInfo> stages = {
            m_present_vertex_shader_module.stage_create_info(),
            m_present_fragment_shader_module.stage_create_info()
        };

        vk::PipelineVertexInputStateCreateInfo vi{};
        vk::PipelineInputAssemblyStateCreateInfo ia{};
        ia.topology = vk::PrimitiveTopology::eTriangleList;

        vk::PipelineViewportStateCreateInfo vps{};
        vps.viewportCount = 1;
        vps.scissorCount = 1;

        vk::PipelineRasterizationStateCreateInfo rs{};
        rs.polygonMode = vk::PolygonMode::eFill;
        rs.cullMode = vk::CullModeFlagBits::eNone;
        rs.frontFace = vk::FrontFace::eCounterClockwise;
        rs.lineWidth = 1.0f;

        vk::PipelineMultisampleStateCreateInfo ms{};
        ms.rasterizationSamples = vk::SampleCountFlagBits::e1;

        vk::PipelineDepthStencilStateCreateInfo ds{};
        ds.depthCompareOp = vk::CompareOp::eAlways;

        vk::PipelineColorBlendAttachmentState cba{};
        cba.colorWriteMask = vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
                             vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA;
        vk::PipelineColorBlendStateCreateInfo cb{};
        cb.attachmentCount = 1;
        cb.pAttachments = &cba;

        std::vector<vk::DynamicState> dyn = {vk::DynamicState::eViewport, vk::DynamicState::eScissor};
        vk::PipelineDynamicStateCreateInfo dys{};
        dys.dynamicStateCount = static_cast<uint32_t>(dyn.size());
        dys.pDynamicStates = dyn.data();

        vk::GraphicsPipelineCreateInfo info{};
        info.stageCount = static_cast<uint32_t>(stages.size());
        info.pStages = stages.data();
        info.pVertexInputState = &vi;
        info.pInputAssemblyState = &ia;
        info.pViewportState = &vps;
        info.pRasterizationState = &rs;
        info.pMultisampleState = &ms;
        info.pDepthStencilState = &ds;
        info.pColorBlendState = &cb;
        info.pDynamicState = &dys;
        info.layout = *m_present_pipeline_layout;
        info.renderPass = VK_NULL_HANDLE;

        vk::PipelineRenderingCreateInfo ri{};
        vk::Format cfmt = m_color_format;
        ri.colorAttachmentCount = 1;
        ri.pColorAttachmentFormats = &cfmt;
        ri.depthAttachmentFormat = m_depth_format;

        vk::StructureChain<vk::GraphicsPipelineCreateInfo, vk::PipelineRenderingCreateInfo> chain{info, ri};
        m_present_pipeline =
            vk::raii::Pipeline{m_device.device(), nullptr, chain.get<vk::GraphicsPipelineCreateInfo>()};
    }

    std::array<FrameTrackedImage, rhi::kFramesInFlight> create_offscreen_images(vk::Extent2D scene_extent) const {
        const vk::ImageUsageFlags usage = vk::ImageUsageFlagBits::eStorage | vk::ImageUsageFlagBits::eSampled;
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
        return ShaderToyFrameTargets{
            create_offscreen_images(scene_extent),
            make_per_frame_depth_images(scene_extent, m_depth_format)
        };
    }

    void refresh_compute_descriptors(ShaderToyFrameTargets& frame_targets) {
        for (uint32_t i = 0; i < rhi::kFramesInFlight; ++i) {
            rhi::DescriptorWriter w;
            w.write_buffer(0, *m_uniform_buffers[i].buffer(), 0, m_uniform_buffer_size);
            w.write_storage_image(1, *frame_targets.offscreen_images[i].image.image_view(), vk::ImageLayout::eGeneral);
            w.update(m_device, *m_compute_sets[i]);
        }
    }
};

}  // namespace rtr::system::render
