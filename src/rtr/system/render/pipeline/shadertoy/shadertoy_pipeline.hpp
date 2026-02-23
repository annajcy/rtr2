#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rtr/rhi/buffer.hpp"
#include "rtr/rhi/descriptor.hpp"
#include "rtr/rhi/shader_module.hpp"
#include "rtr/rhi/texture.hpp"
#include "rtr/system/render/frame_color_source.hpp"
#include "rtr/system/render/pass/present_image_pass.hpp"
#include "rtr/system/render/pipeline.hpp"
#include "rtr/system/render/pipeline/shadertoy/shadertoy_compute_pass.hpp"
#include "vulkan/vulkan.hpp"

// ============================================================================
// ShaderToyPipeline (self-contained, composition-based)
//
// Sequence: ComputePass (offscreen) → PresentImagePass (to swapchain).
// All barriers are handled by the passes themselves.
// ============================================================================

namespace rtr::system::render {

struct ShaderToyPipelineConfig {
    std::string shader_output_dir{"/Users/jinceyang/Desktop/codebase/graphics/rtr2/build/Debug/shaders/compiled/"};
    std::string compute_shader_filename{"shadertoy_compute_comp.spv"};
    std::string present_vertex_shader_filename{"shadertoy_present_vert.spv"};
    std::string present_fragment_shader_filename{"shadertoy_present_frag.spv"};
};

class ShaderToyPipeline final : public RenderPipelineBase, public IFrameColorSource, public ISceneViewportSink {
    struct ShaderToyFrameTargets {
        std::array<rhi::Image, rhi::kFramesInFlight> offscreen_images;
        std::array<vk::ImageLayout, rhi::kFramesInFlight> offscreen_layouts;
        std::array<rhi::Image, rhi::kFramesInFlight> depth_images;

        ShaderToyFrameTargets(
            std::array<rhi::Image, rhi::kFramesInFlight>&& offscreen_images_in,
            std::array<vk::ImageLayout, rhi::kFramesInFlight>&& offscreen_layouts_in,
            std::array<rhi::Image, rhi::kFramesInFlight>&& depth_images_in
        )
            : offscreen_images(std::move(offscreen_images_in)),
              offscreen_layouts(std::move(offscreen_layouts_in)),
              depth_images(std::move(depth_images_in)) {}
    };

    vk::Format m_offscreen_format{vk::Format::eUndefined};

    rhi::ShaderModule m_compute_shader_module;
    rhi::ShaderModule m_present_vertex_shader_module;
    rhi::ShaderModule m_present_fragment_shader_module;

    rhi::DescriptorSetLayout m_compute_layout;
    rhi::DescriptorSetLayout m_present_layout;
    rhi::DescriptorPool      m_descriptor_pool;
    rhi::Sampler             m_offscreen_sampler;

    std::array<vk::raii::DescriptorSet, rhi::kFramesInFlight> m_compute_sets;
    std::array<vk::raii::DescriptorSet, rhi::kFramesInFlight> m_present_sets;

    vk::raii::PipelineLayout m_compute_pipeline_layout{nullptr};
    vk::raii::Pipeline       m_compute_pipeline{nullptr};
    vk::raii::PipelineLayout m_present_pipeline_layout{nullptr};
    vk::raii::Pipeline       m_present_pipeline{nullptr};

    vk::DeviceSize               m_uniform_buffer_size{0};
    std::array<rhi::Buffer, rhi::kFramesInFlight> m_uniform_buffers;
    std::optional<ShaderToyFrameTargets> m_frame_targets{};
    vk::Extent2D                         m_scene_target_extent{};
    vk::Extent2D                         m_requested_scene_extent{};
    bool                                 m_scene_extent_dirty{false};

    ComputePass      m_compute_pass;
    PresentImagePass m_present_pass;

public:
    ShaderToyPipeline(const render::PipelineRuntime& runtime, const ShaderToyPipelineConfig& config = {})
        : RenderPipelineBase(runtime),
          m_offscreen_format(pick_offscreen_format()),
          m_compute_shader_module(build_shader_module(m_device, config.shader_output_dir + config.compute_shader_filename,
                                                     vk::ShaderStageFlagBits::eCompute)),
          m_present_vertex_shader_module(build_shader_module(
              m_device, config.shader_output_dir + config.present_vertex_shader_filename, vk::ShaderStageFlagBits::eVertex
          )),
          m_present_fragment_shader_module(build_shader_module(
              m_device, config.shader_output_dir + config.present_fragment_shader_filename, vk::ShaderStageFlagBits::eFragment
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
          m_compute_pass(&m_compute_pipeline_layout, &m_compute_pipeline),
          m_present_pass(&m_present_pipeline_layout, &m_present_pipeline) {

        build_compute_pipeline();
        build_present_pipeline();

    }

    ~ShaderToyPipeline() override = default;

    void on_resize(int /*w*/, int /*h*/) override {}

    FrameColorSourceView frame_color_source_view(uint32_t frame_index) const override {
        if (!m_frame_targets.has_value()) {
            return {};
        }
        if (frame_index >= rhi::kFramesInFlight) {
            return {};
        }
        const auto& offscreen = m_frame_targets->offscreen_images[frame_index];
        return FrameColorSourceView{.image_view   = *offscreen.image_view(),
                                    .image_layout = m_frame_targets->offscreen_layouts[frame_index],
                                    .extent       = {offscreen.width(), offscreen.height()}};
    }

    void set_scene_viewport_extent(vk::Extent2D extent) override {
        if (extent.width == 0 || extent.height == 0)
            return;
        if (m_requested_scene_extent.width == extent.width && m_requested_scene_extent.height == extent.height)
            return;
        m_requested_scene_extent = extent;
        m_scene_extent_dirty     = true;
    }

    void handle_swapchain_state_change(const FrameScheduler::SwapchainState& /*state*/,
                                       const SwapchainChangeSummary& diff) override {
        if (diff.extent_or_depth_changed())
            m_scene_extent_dirty = true;
        if (diff.color_or_depth_changed())
            build_present_pipeline();
    }

    void render(FrameContext& ctx) override {
        // --- 1. Guard & lazy resize ---
        const auto extent = ctx.render_extent();
        if (extent.width == 0 || extent.height == 0)
            return;

        ensure_scene_targets(extent);
        const uint32_t frame_index = ctx.frame_index();
        auto& frame_targets = require_frame_targets();

        // --- 2. ComputePass: write to offscreen storage image ---
        m_compute_pass.execute(
            ctx,
            ComputePass::RenderPassResources{.uniform_buffer   = m_uniform_buffers[frame_index],
                                             .offscreen_image  = frame_targets.offscreen_images[frame_index],
                                             .offscreen_layout = frame_targets.offscreen_layouts[frame_index],
                                             .compute_set      = m_compute_sets[frame_index]});

        // Update present descriptor so the sampler points to the (possibly recreated) offscreen image
        update_present_descriptor(frame_index);

        // --- 3. PresentImagePass: sample offscreen → swapchain ---
        m_present_pass.execute(
            ctx,
            PresentImagePass::RenderPassResources{.offscreen_image  = frame_targets.offscreen_images[frame_index],
                                                  .offscreen_layout = frame_targets.offscreen_layouts[frame_index],
                                                  .depth_image      = frame_targets.depth_images[frame_index],
                                                  .present_set      = m_present_sets[frame_index]});
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
        vk::PipelineLayoutCreateInfo           info{};
        info.setLayoutCount       = static_cast<uint32_t>(layouts.size());
        info.pSetLayouts          = layouts.data();
        return vk::raii::PipelineLayout{device.device(), info};
    }

    ShaderToyFrameTargets& require_frame_targets() {
        if (!m_frame_targets.has_value()) {
            throw std::runtime_error("ShaderToyPipeline frame targets are not initialized.");
        }
        return *m_frame_targets;
    }

    void update_present_descriptor(uint32_t frame_index) {
        auto& frame_targets = require_frame_targets();
        auto& img = frame_targets.offscreen_images[frame_index];
        rhi::DescriptorWriter w;
        w.write_combined_image(0, *img.image_view(), *m_offscreen_sampler.sampler(),
                               vk::ImageLayout::eShaderReadOnlyOptimal);
        w.update(m_device, *m_present_sets[frame_index]);
    }

    void ensure_scene_targets(vk::Extent2D fallback) {
        vk::Extent2D desired = fallback;
        if (m_requested_scene_extent.width > 0 && m_requested_scene_extent.height > 0)
            desired = m_requested_scene_extent;

        const bool need_recreate = m_scene_extent_dirty || m_scene_target_extent.width != desired.width ||
                                   m_scene_target_extent.height != desired.height ||
                                   !m_frame_targets.has_value();
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
        info.stage         = m_compute_shader_module.stage_create_info();
        info.layout        = *m_compute_pipeline_layout;
        m_compute_pipeline = vk::raii::Pipeline{m_device.device(), nullptr, info};
    }

    void build_present_pipeline() {
        std::vector<vk::PipelineShaderStageCreateInfo> stages = {m_present_vertex_shader_module.stage_create_info(),
                                                                 m_present_fragment_shader_module.stage_create_info()};

        vk::PipelineVertexInputStateCreateInfo   vi{};
        vk::PipelineInputAssemblyStateCreateInfo ia{};
        ia.topology = vk::PrimitiveTopology::eTriangleList;

        vk::PipelineViewportStateCreateInfo vps{};
        vps.viewportCount = 1;
        vps.scissorCount  = 1;

        vk::PipelineRasterizationStateCreateInfo rs{};
        rs.polygonMode = vk::PolygonMode::eFill;
        rs.cullMode    = vk::CullModeFlagBits::eNone;
        rs.frontFace   = vk::FrontFace::eCounterClockwise;
        rs.lineWidth   = 1.0f;

        vk::PipelineMultisampleStateCreateInfo ms{};
        ms.rasterizationSamples = vk::SampleCountFlagBits::e1;

        vk::PipelineDepthStencilStateCreateInfo ds{};
        ds.depthCompareOp = vk::CompareOp::eAlways;

        vk::PipelineColorBlendAttachmentState cba{};
        cba.colorWriteMask = vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
                             vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA;
        vk::PipelineColorBlendStateCreateInfo cb{};
        cb.attachmentCount = 1;
        cb.pAttachments    = &cba;

        std::vector<vk::DynamicState>      dyn = {vk::DynamicState::eViewport, vk::DynamicState::eScissor};
        vk::PipelineDynamicStateCreateInfo dys{};
        dys.dynamicStateCount = static_cast<uint32_t>(dyn.size());
        dys.pDynamicStates    = dyn.data();

        vk::GraphicsPipelineCreateInfo info{};
        info.stageCount          = static_cast<uint32_t>(stages.size());
        info.pStages             = stages.data();
        info.pVertexInputState   = &vi;
        info.pInputAssemblyState = &ia;
        info.pViewportState      = &vps;
        info.pRasterizationState = &rs;
        info.pMultisampleState   = &ms;
        info.pDepthStencilState  = &ds;
        info.pColorBlendState    = &cb;
        info.pDynamicState       = &dys;
        info.layout              = *m_present_pipeline_layout;
        info.renderPass          = VK_NULL_HANDLE;

        vk::PipelineRenderingCreateInfo ri{};
        vk::Format                      cfmt = m_color_format;
        ri.colorAttachmentCount              = 1;
        ri.pColorAttachmentFormats           = &cfmt;
        ri.depthAttachmentFormat             = m_depth_format;

        vk::StructureChain<vk::GraphicsPipelineCreateInfo, vk::PipelineRenderingCreateInfo> chain{info, ri};
        m_present_pipeline =
            vk::raii::Pipeline{m_device.device(), nullptr, chain.get<vk::GraphicsPipelineCreateInfo>()};
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
            create_initial_offscreen_layouts(),
            make_per_frame_depth_images(m_scene_target_extent, m_depth_format)
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

}  // namespace rtr::system::render
