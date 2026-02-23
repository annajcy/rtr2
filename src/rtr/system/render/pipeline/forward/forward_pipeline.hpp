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
#include "rtr/rhi/texture.hpp"
#include "rtr/system/render/frame_color_source.hpp"
#include "rtr/system/render/pass/present_pass.hpp"
#include "rtr/system/render/pipeline.hpp"
#include "rtr/system/render/pipeline/forward/forward_pass.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view_builder.hpp"
#include "rtr/system/render/render_pass.hpp"
#include "rtr/utils/log.hpp"
#include "vulkan/vulkan.hpp"

namespace rtr::system::render {

// ---------------------------------------------------------------------------
// Shared GPU data types (used by both ForwardPipeline and ForwardEditorPipeline)
// ---------------------------------------------------------------------------

struct GpuMat4 {
    alignas(16) std::array<float, 16> values{};
};

struct GpuPointLight {
    alignas(16) std::array<float, 3> position{};
    alignas(4) float intensity{0.0f};
    alignas(16) std::array<float, 3> color{};
    alignas(4) float range{0.0f};
    alignas(4) float specular_strength{0.0f};
    alignas(4) float shininess{0.0f};
    alignas(8) std::array<float, 2> padding{};
};

struct UniformBufferObjectGpu {
    alignas(16) GpuMat4 model{};
    alignas(16) GpuMat4 view{};
    alignas(16) GpuMat4 proj{};
    alignas(16) GpuMat4 normal{};
    alignas(16) std::array<float, 4> base_color{};
    alignas(16) std::array<GpuPointLight, 4> point_lights{};
    alignas(16) std::array<float, 3> camera_world_pos{};
    alignas(4) uint32_t point_light_count{0};
};

inline GpuMat4 pack_mat4_row_major(const pbpt::math::mat4& m) {
    GpuMat4     out{};
    std::size_t idx = 0;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out.values[idx++] = static_cast<float>(m[r][c]);
    return out;
}

struct ForwardPipelineConfig {
    std::string shader_output_dir{"/Users/jinceyang/Desktop/codebase/graphics/rtr2/build/Debug/shaders/compiled/"};
    std::string vertex_shader_filename{"vert_buffer_vert.spv"};
    std::string fragment_shader_filename{"vert_buffer_frag.spv"};
};

// ---------------------------------------------------------------------------
// ForwardPipeline
// Sequence: ForwardPass (offscreen) → PresentPass (blit to swapchain).
// ---------------------------------------------------------------------------
class ForwardPipeline final : public RenderPipelineBase,
                              public IFramePreparePipeline,
                              public IFrameColorSource,
                              public ISceneViewportSink {
    static constexpr uint32_t kMaxRenderables = 256;

    vk::raii::PipelineLayout m_pipeline_layout{nullptr};
    vk::raii::Pipeline       m_pipeline{nullptr};

    std::unique_ptr<rhi::ShaderModule> m_vertex_shader_module{nullptr};
    std::unique_ptr<rhi::ShaderModule> m_fragment_shader_module{nullptr};

    std::unique_ptr<rhi::DescriptorSetLayout> m_per_object_layout{nullptr};
    std::unique_ptr<rhi::DescriptorPool>      m_descriptor_pool{nullptr};

    vk::DeviceSize                                         m_uniform_buffer_size{0};
    std::vector<std::vector<std::unique_ptr<rhi::Buffer>>> m_object_uniform_buffers{};
    std::vector<std::vector<vk::raii::DescriptorSet>>      m_object_sets{};

    std::vector<std::unique_ptr<rhi::Image>> m_depth_images{};
    std::vector<std::unique_ptr<rhi::Image>> m_color_images{};
    std::vector<vk::ImageLayout>             m_color_image_layouts{};

    vk::Extent2D m_scene_target_extent{};
    vk::Extent2D m_requested_scene_extent{};
    bool         m_scene_extent_dirty{false};

    std::optional<ForwardSceneView> m_scene_view{};
    resource::ResourceManager*      m_resource_manager{};

    std::unique_ptr<render::ForwardPass> m_forward_pass{nullptr};
    std::unique_ptr<render::PresentPass> m_present_pass{nullptr};

public:
    ForwardPipeline(const render::PipelineRuntime& runtime, const ForwardPipelineConfig& config = {})
        : RenderPipelineBase(runtime) {
        m_uniform_buffer_size = sizeof(UniformBufferObjectGpu);

        m_vertex_shader_module   = std::make_unique<rhi::ShaderModule>(rhi::ShaderModule::from_file(
            *m_device, config.shader_output_dir + config.vertex_shader_filename, vk::ShaderStageFlagBits::eVertex));
        m_fragment_shader_module = std::make_unique<rhi::ShaderModule>(rhi::ShaderModule::from_file(
            *m_device, config.shader_output_dir + config.fragment_shader_filename, vk::ShaderStageFlagBits::eFragment));

        rhi::DescriptorSetLayout::Builder layout_builder;
        layout_builder.add_binding(0, vk::DescriptorType::eUniformBuffer,
                                   vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment);
        m_per_object_layout = std::make_unique<rhi::DescriptorSetLayout>(layout_builder.build(*m_device));

        rhi::DescriptorPool::Builder pool_builder;
        pool_builder.add_layout(*m_per_object_layout, kMaxRenderables * m_frame_count)
            .set_flags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
        m_descriptor_pool = std::make_unique<rhi::DescriptorPool>(pool_builder.build(*m_device));

        create_per_object_resources();

        std::array<vk::DescriptorSetLayout, 1> set_layouts{*m_per_object_layout->layout()};
        vk::PipelineLayoutCreateInfo           layout_info{};
        layout_info.setLayoutCount = static_cast<uint32_t>(set_layouts.size());
        layout_info.pSetLayouts    = set_layouts.data();
        m_pipeline_layout          = vk::raii::PipelineLayout{m_device->device(), layout_info};

        create_graphics_pipeline();

        m_forward_pass = std::make_unique<render::ForwardPass>(&m_pipeline_layout, &m_pipeline);
        m_present_pass = std::make_unique<render::PresentPass>();
    }

    ~ForwardPipeline() override {
        m_forward_pass.reset();
        m_object_sets.clear();
        m_descriptor_pool.reset();
    }

    void prepare_frame(const FramePrepareContext& ctx) override {
        auto* active_scene = ctx.world.active_scene();
        if (!active_scene)
            throw std::runtime_error("ForwardPipeline::prepare_frame: no active scene.");
        m_resource_manager = &ctx.resources;
        m_scene_view = build_forward_scene_view(*active_scene, ctx.resources);
    }

    void set_scene_viewport_extent(vk::Extent2D extent) override {
        if (extent.width == 0 || extent.height == 0)
            return;
        if (m_requested_scene_extent.width == extent.width && m_requested_scene_extent.height == extent.height)
            return;
        m_requested_scene_extent = extent;
        m_scene_extent_dirty     = true;
    }

    FrameColorSourceView frame_color_source_view(uint32_t frame_index) const override {
        if (frame_index >= m_color_images.size() || !m_color_images[frame_index])
            return {};
        return FrameColorSourceView{
            .image_view   = *m_color_images[frame_index]->image_view(),
            .image_layout = m_color_image_layouts[frame_index],
            .extent       = {m_color_images[frame_index]->width(), m_color_images[frame_index]->height()}};
    }

    void on_resize(int /*w*/, int /*h*/) override {}

    void handle_swapchain_state_change(const ActiveFrameScheduler::SwapchainState& /*state*/,
                                       const SwapchainChangeSummary& diff) override {
        if (diff.color_or_depth_changed())
            create_graphics_pipeline();
        if (diff.extent_or_depth_changed())
            m_scene_extent_dirty = true;
    }

    void render(FrameContext& ctx) override {
        // --- 1. Guard ---
        const auto extent = ctx.render_extent();
        if (extent.width == 0 || extent.height == 0)
            return;
        if (!m_resource_manager)
            throw std::runtime_error("ForwardPipeline: resource manager not set.");
        if (!m_scene_view)
            throw std::runtime_error("ForwardPipeline: scene view not set.");

        ensure_scene_targets(extent);
        const uint32_t frame_index = ctx.frame_index();
        check_frame_resources(frame_index);

        // --- 2. ForwardPass: render scene to offscreen color image ---
        m_forward_pass->execute(
            ctx,
            ForwardPass::RenderPassResources{.color_image  = m_color_images[frame_index].get(),
                                             .color_layout = &m_color_image_layouts[frame_index],
                                             .depth_image  = m_depth_images[frame_index].get(),
                                             .extent       = m_scene_target_extent,
                                             .draw_items   = build_draw_items(frame_index)});

        // --- 3. PresentPass: blit offscreen → swapchain ---
        m_present_pass->execute(
            ctx,
            PresentPass::RenderPassResources{.src_color_image  = m_color_images[frame_index].get(),
                                             .src_color_layout = &m_color_image_layouts[frame_index],
                                             .src_extent       = m_scene_target_extent});
    }

private:
    void check_frame_resources(uint32_t frame_index) const {
        if (frame_index >= m_object_uniform_buffers.size() || frame_index >= m_object_sets.size() ||
            frame_index >= m_depth_images.size() || frame_index >= m_color_images.size() ||
            frame_index >= m_color_image_layouts.size()) {
            throw std::runtime_error("ForwardPipeline frame resources not ready.");
        }
    }

    std::vector<ForwardPass::DrawItem> build_draw_items(uint32_t frame_index) {
        const auto& scene_view = m_scene_view.value();
        if (scene_view.renderables.size() > kMaxRenderables)
            throw std::runtime_error("Renderable count exceeds ForwardPipeline capacity.");

        auto& frame_ubos = m_object_uniform_buffers[frame_index];
        auto& frame_sets = m_object_sets[frame_index];

        std::vector<ForwardPass::DrawItem> items;
        items.reserve(scene_view.renderables.size());
        for (std::size_t i = 0; i < scene_view.renderables.size(); ++i) {
            const auto& r    = scene_view.renderables[i];
            auto&       mesh = require_mesh(r.mesh);

            UniformBufferObjectGpu ubo{};
            ubo.model      = pack_mat4_row_major(r.model);
            ubo.view       = pack_mat4_row_major(scene_view.camera.view);
            ubo.proj       = pack_mat4_row_major(scene_view.camera.proj);
            ubo.normal     = pack_mat4_row_major(r.normal);
            ubo.base_color = {static_cast<float>(r.base_color.x()), static_cast<float>(r.base_color.y()),
                              static_cast<float>(r.base_color.z()), static_cast<float>(r.base_color.w())};

            ubo.point_light_count = static_cast<uint32_t>(scene_view.point_lights.size());
            ubo.camera_world_pos  = {static_cast<float>(scene_view.camera.world_pos.x()),
                                     static_cast<float>(scene_view.camera.world_pos.y()),
                                     static_cast<float>(scene_view.camera.world_pos.z())};

            for (std::size_t j = 0; j < ubo.point_light_count; ++j) {
                const auto& pl                = scene_view.point_lights[j];
                ubo.point_lights[j].position  = {static_cast<float>(pl.position.x()),
                                                 static_cast<float>(pl.position.y()),
                                                 static_cast<float>(pl.position.z())};
                ubo.point_lights[j].intensity = pl.intensity;
                ubo.point_lights[j].color     = {static_cast<float>(pl.color.x()), static_cast<float>(pl.color.y()),
                                                 static_cast<float>(pl.color.z())};
                ubo.point_lights[j].range     = pl.range;
                ubo.point_lights[j].specular_strength = pl.specular_strength;
                ubo.point_lights[j].shininess         = pl.shininess;
            }

            std::memcpy(frame_ubos[i]->mapped_data(), &ubo, sizeof(ubo));

            items.push_back({.mesh = &mesh, .per_object_set = &frame_sets[i]});
        }
        return items;
    }

    rhi::Mesh& require_mesh(resource::MeshHandle handle) {
        if (!handle.is_valid())
            throw std::runtime_error("ForwardPipeline: invalid mesh handle.");
        return m_resource_manager->require_gpu<resource::MeshResourceKind>(handle, *m_device);
    }

    void ensure_scene_targets(vk::Extent2D fallback) {
        vk::Extent2D desired = fallback;
        if (m_requested_scene_extent.width > 0 && m_requested_scene_extent.height > 0)
            desired = m_requested_scene_extent;

        const bool need_recreate = m_scene_extent_dirty || m_scene_target_extent.width != desired.width ||
                                   m_scene_target_extent.height != desired.height || m_color_images.empty() ||
                                   m_depth_images.empty();
        if (!need_recreate)
            return;

        m_device->wait_idle();
        m_scene_target_extent = desired;
        create_color_images();
        create_depth_images();
        m_scene_extent_dirty = false;
    }

    void create_per_object_resources() {
        m_object_uniform_buffers.clear();
        m_object_uniform_buffers.resize(m_frame_count);
        m_object_sets.clear();
        m_object_sets.resize(m_frame_count);
        for (uint32_t f = 0; f < m_frame_count; ++f) {
            auto& bufs = m_object_uniform_buffers[f];
            bufs.reserve(kMaxRenderables);
            for (uint32_t s = 0; s < kMaxRenderables; ++s) {
                auto buf = std::make_unique<rhi::Buffer>(rhi::Buffer::create_host_visible_buffer(
                    *m_device, m_uniform_buffer_size, vk::BufferUsageFlagBits::eUniformBuffer));
                buf->map();
                bufs.push_back(std::move(buf));
            }
            m_object_sets[f] = m_descriptor_pool->allocate_multiple(*m_per_object_layout, kMaxRenderables);
            for (uint32_t s = 0; s < kMaxRenderables; ++s) {
                rhi::DescriptorWriter w;
                w.write_buffer(0, *m_object_uniform_buffers[f][s]->buffer(), 0, m_uniform_buffer_size);
                w.update(*m_device, *m_object_sets[f][s]);
            }
        }
    }

    void create_color_images() {
        m_color_images.clear();
        m_color_image_layouts.assign(m_frame_count, vk::ImageLayout::eUndefined);
        const vk::ImageUsageFlags usage = vk::ImageUsageFlagBits::eColorAttachment | vk::ImageUsageFlagBits::eSampled |
                                          vk::ImageUsageFlagBits::eTransferSrc;
        for (uint32_t i = 0; i < m_frame_count; ++i) {
            m_color_images.push_back(std::make_unique<rhi::Image>(
                rhi::Image(*m_device, m_scene_target_extent.width, m_scene_target_extent.height, m_color_format,
                           vk::ImageTiling::eOptimal, usage, vk::MemoryPropertyFlagBits::eDeviceLocal,
                           vk::ImageAspectFlagBits::eColor, false)));
        }
    }

    void create_depth_images() { m_depth_images = make_per_frame_depth_images(m_scene_target_extent, m_depth_format); }

    void create_graphics_pipeline() {
        std::vector<vk::PipelineShaderStageCreateInfo> stages = {m_vertex_shader_module->stage_create_info(),
                                                                 m_fragment_shader_module->stage_create_info()};

        auto                                   vi_state = rhi::Mesh::vertex_input_state();
        vk::PipelineVertexInputStateCreateInfo vi{};
        vi.vertexBindingDescriptionCount   = static_cast<uint32_t>(vi_state.bindings.size());
        vi.pVertexBindingDescriptions      = vi_state.bindings.data();
        vi.vertexAttributeDescriptionCount = static_cast<uint32_t>(vi_state.attributes.size());
        vi.pVertexAttributeDescriptions    = vi_state.attributes.data();

        vk::PipelineInputAssemblyStateCreateInfo ia{};
        ia.topology = vk::PrimitiveTopology::eTriangleList;

        vk::PipelineViewportStateCreateInfo vp{};
        vp.viewportCount = 1;
        vp.scissorCount  = 1;

        vk::PipelineRasterizationStateCreateInfo rs{};
        rs.polygonMode = vk::PolygonMode::eFill;
        rs.cullMode    = vk::CullModeFlagBits::eNone;
        rs.frontFace   = vk::FrontFace::eCounterClockwise;
        rs.lineWidth   = 1.0f;

        vk::PipelineMultisampleStateCreateInfo ms{};
        ms.rasterizationSamples = vk::SampleCountFlagBits::e1;

        vk::PipelineDepthStencilStateCreateInfo ds{};
        ds.depthTestEnable  = VK_TRUE;
        ds.depthWriteEnable = VK_TRUE;
        ds.depthCompareOp   = vk::CompareOp::eLess;

        vk::PipelineColorBlendAttachmentState cba{};
        cba.blendEnable    = VK_FALSE;
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
        info.pViewportState      = &vp;
        info.pRasterizationState = &rs;
        info.pMultisampleState   = &ms;
        info.pDepthStencilState  = &ds;
        info.pColorBlendState    = &cb;
        info.pDynamicState       = &dys;
        info.layout              = *m_pipeline_layout;
        info.renderPass          = VK_NULL_HANDLE;

        vk::PipelineRenderingCreateInfo ri{};
        vk::Format                      cfmt = m_color_format;
        ri.colorAttachmentCount              = 1;
        ri.pColorAttachmentFormats           = &cfmt;
        ri.depthAttachmentFormat             = m_depth_format;

        vk::StructureChain<vk::GraphicsPipelineCreateInfo, vk::PipelineRenderingCreateInfo> chain{info, ri};
        m_pipeline = vk::raii::Pipeline{m_device->device(), nullptr, chain.get<vk::GraphicsPipelineCreateInfo>()};
    }
};

}  // namespace rtr::system::render
