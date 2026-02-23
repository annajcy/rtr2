#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
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
    struct OffscreenFrameResources {
        std::unique_ptr<rhi::Image> image{};
        vk::ImageLayout             layout{vk::ImageLayout::eUndefined};
    };

    vk::Format m_offscreen_format{vk::Format::eUndefined};

    std::unique_ptr<rhi::ShaderModule> m_compute_shader_module{nullptr};
    std::unique_ptr<rhi::ShaderModule> m_present_vertex_shader_module{nullptr};
    std::unique_ptr<rhi::ShaderModule> m_present_fragment_shader_module{nullptr};

    std::unique_ptr<rhi::DescriptorSetLayout> m_compute_layout{nullptr};
    std::unique_ptr<rhi::DescriptorSetLayout> m_present_layout{nullptr};
    std::unique_ptr<rhi::DescriptorPool>      m_descriptor_pool{nullptr};
    std::unique_ptr<rhi::Sampler>             m_offscreen_sampler{nullptr};

    std::vector<vk::raii::DescriptorSet> m_compute_sets{};
    std::vector<vk::raii::DescriptorSet> m_present_sets{};

    vk::raii::PipelineLayout m_compute_pipeline_layout{nullptr};
    vk::raii::Pipeline       m_compute_pipeline{nullptr};
    vk::raii::PipelineLayout m_present_pipeline_layout{nullptr};
    vk::raii::Pipeline       m_present_pipeline{nullptr};

    vk::DeviceSize                            m_uniform_buffer_size{0};
    std::vector<std::unique_ptr<rhi::Buffer>> m_uniform_buffers{};
    std::vector<std::unique_ptr<rhi::Image>>  m_depth_images{};

    std::vector<OffscreenFrameResources> m_offscreen_frame_resources{};
    vk::Extent2D                         m_scene_target_extent{};
    vk::Extent2D                         m_requested_scene_extent{};
    bool                                 m_scene_extent_dirty{false};

    std::unique_ptr<ComputePass>      m_compute_pass{nullptr};
    std::unique_ptr<PresentImagePass> m_present_pass{nullptr};

public:
    ShaderToyPipeline(const render::PipelineRuntime& runtime, const ShaderToyPipelineConfig& config = {})
        : RenderPipelineBase(runtime) {
        m_uniform_buffer_size = sizeof(ShaderToyUniformBufferObject);
        m_offscreen_format    = pick_offscreen_format();

        m_compute_shader_module        = std::make_unique<rhi::ShaderModule>(rhi::ShaderModule::from_file(
            *m_device, config.shader_output_dir + config.compute_shader_filename, vk::ShaderStageFlagBits::eCompute));
        m_present_vertex_shader_module = std::make_unique<rhi::ShaderModule>(
            rhi::ShaderModule::from_file(*m_device, config.shader_output_dir + config.present_vertex_shader_filename,
                                         vk::ShaderStageFlagBits::eVertex));
        m_present_fragment_shader_module = std::make_unique<rhi::ShaderModule>(
            rhi::ShaderModule::from_file(*m_device, config.shader_output_dir + config.present_fragment_shader_filename,
                                         vk::ShaderStageFlagBits::eFragment));

        m_uniform_buffers   = make_per_frame_mapped_uniform_buffers(m_uniform_buffer_size);
        m_offscreen_sampler = std::make_unique<rhi::Sampler>(rhi::Sampler::create_default(*m_device, 1));

        rhi::DescriptorSetLayout::Builder compute_layout_builder;
        compute_layout_builder.add_binding(0, vk::DescriptorType::eUniformBuffer, vk::ShaderStageFlagBits::eCompute)
            .add_binding(1, vk::DescriptorType::eStorageImage, vk::ShaderStageFlagBits::eCompute);
        m_compute_layout = std::make_unique<rhi::DescriptorSetLayout>(compute_layout_builder.build(*m_device));

        rhi::DescriptorSetLayout::Builder present_layout_builder;
        present_layout_builder.add_binding(0, vk::DescriptorType::eCombinedImageSampler,
                                           vk::ShaderStageFlagBits::eFragment);
        m_present_layout = std::make_unique<rhi::DescriptorSetLayout>(present_layout_builder.build(*m_device));

        rhi::DescriptorPool::Builder pool_builder;
        pool_builder.add_layout(*m_compute_layout, m_frame_count)
            .add_layout(*m_present_layout, m_frame_count)
            .set_flags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
        m_descriptor_pool = std::make_unique<rhi::DescriptorPool>(pool_builder.build(*m_device));

        m_compute_sets = m_descriptor_pool->allocate_multiple(*m_compute_layout, m_frame_count);
        m_present_sets = m_descriptor_pool->allocate_multiple(*m_present_layout, m_frame_count);

        // Compute pipeline layout (set 0 = compute)
        {
            std::array<vk::DescriptorSetLayout, 1> layouts{*m_compute_layout->layout()};
            vk::PipelineLayoutCreateInfo           info{};
            info.setLayoutCount       = static_cast<uint32_t>(layouts.size());
            info.pSetLayouts          = layouts.data();
            m_compute_pipeline_layout = vk::raii::PipelineLayout{m_device->device(), info};
        }
        // Present pipeline layout (set 0 = present sampler)
        {
            std::array<vk::DescriptorSetLayout, 1> layouts{*m_present_layout->layout()};
            vk::PipelineLayoutCreateInfo           info{};
            info.setLayoutCount       = static_cast<uint32_t>(layouts.size());
            info.pSetLayouts          = layouts.data();
            m_present_pipeline_layout = vk::raii::PipelineLayout{m_device->device(), info};
        }

        build_compute_pipeline();
        build_present_pipeline();

        m_compute_pass = std::make_unique<ComputePass>(&m_compute_pipeline_layout, &m_compute_pipeline);
        m_present_pass = std::make_unique<PresentImagePass>(&m_present_pipeline_layout, &m_present_pipeline);
    }

    ~ShaderToyPipeline() override = default;

    void on_resize(int /*w*/, int /*h*/) override {}

    FrameColorSourceView frame_color_source_view(uint32_t frame_index) const override {
        if (frame_index >= m_offscreen_frame_resources.size() || !m_offscreen_frame_resources[frame_index].image)
            return {};
        const auto& f = m_offscreen_frame_resources[frame_index];
        return FrameColorSourceView{.image_view   = *f.image->image_view(),
                                    .image_layout = f.layout,
                                    .extent       = {f.image->width(), f.image->height()}};
    }

    void set_scene_viewport_extent(vk::Extent2D extent) override {
        if (extent.width == 0 || extent.height == 0)
            return;
        if (m_requested_scene_extent.width == extent.width && m_requested_scene_extent.height == extent.height)
            return;
        m_requested_scene_extent = extent;
        m_scene_extent_dirty     = true;
    }

    void handle_swapchain_state_change(const ActiveFrameScheduler::SwapchainState& /*state*/,
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

        // --- 2. ComputePass: write to offscreen storage image ---
        auto& frame = m_offscreen_frame_resources[frame_index];
        m_compute_pass->execute(
            ctx,
            ComputePass::RenderPassResources{.uniform_buffer   = m_uniform_buffers[frame_index].get(),
                                             .offscreen_image  = frame.image.get(),
                                             .offscreen_layout = &frame.layout,
                                             .compute_set      = &m_compute_sets[frame_index]});

        // Update present descriptor so the sampler points to the (possibly recreated) offscreen image
        update_present_descriptor(frame_index);

        // --- 3. PresentImagePass: sample offscreen → swapchain ---
        m_present_pass->execute(
            ctx,
            PresentImagePass::RenderPassResources{.offscreen_image  = frame.image.get(),
                                                  .offscreen_layout = &frame.layout,
                                                  .depth_image      = m_depth_images[frame_index].get(),
                                                  .present_set      = &m_present_sets[frame_index]});
    }

private:
    void update_present_descriptor(uint32_t frame_index) {
        auto* img = m_offscreen_frame_resources[frame_index].image.get();
        if (!img)
            return;
        rhi::DescriptorWriter w;
        w.write_combined_image(0, *img->image_view(), *m_offscreen_sampler->sampler(),
                               vk::ImageLayout::eShaderReadOnlyOptimal);
        w.update(*m_device, *m_present_sets[frame_index]);
    }

    void ensure_scene_targets(vk::Extent2D fallback) {
        vk::Extent2D desired = fallback;
        if (m_requested_scene_extent.width > 0 && m_requested_scene_extent.height > 0)
            desired = m_requested_scene_extent;

        const bool need_recreate = m_scene_extent_dirty || m_scene_target_extent.width != desired.width ||
                                   m_scene_target_extent.height != desired.height ||
                                   m_offscreen_frame_resources.empty() || m_depth_images.empty();
        if (!need_recreate)
            return;

        m_device->wait_idle();
        m_scene_target_extent = desired;
        create_offscreen_images();
        create_depth_images();
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

    void build_present_pipeline() {
        std::vector<vk::PipelineShaderStageCreateInfo> stages = {m_present_vertex_shader_module->stage_create_info(),
                                                                 m_present_fragment_shader_module->stage_create_info()};

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
            vk::raii::Pipeline{m_device->device(), nullptr, chain.get<vk::GraphicsPipelineCreateInfo>()};
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

    void create_depth_images() { m_depth_images = make_per_frame_depth_images(m_scene_target_extent, m_depth_format); }

    void refresh_compute_descriptors() {
        for (uint32_t i = 0; i < m_frame_count; ++i) {
            rhi::DescriptorWriter w;
            w.write_buffer(0, *m_uniform_buffers[i]->buffer(), 0, m_uniform_buffer_size);
            w.write_storage_image(1, *m_offscreen_frame_resources[i].image->image_view(), vk::ImageLayout::eGeneral);
            w.update(*m_device, *m_compute_sets[i]);
        }
    }
};

}  // namespace rtr::system::render
