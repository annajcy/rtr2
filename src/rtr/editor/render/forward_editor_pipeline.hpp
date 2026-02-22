#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include "rtr/editor/core/editor_capture.hpp"
#include "rtr/editor/core/editor_host.hpp"
#include "rtr/editor/render/editor_imgui_pass.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/rhi/buffer.hpp"
#include "rtr/rhi/descriptor.hpp"
#include "rtr/rhi/mesh.hpp"
#include "rtr/rhi/shader_module.hpp"
#include "rtr/rhi/texture.hpp"
#include "rtr/system/render/frame_color_source.hpp"
#include "rtr/system/render/pipeline.hpp"
#include "rtr/system/render/pipeline/forward/forward_pass.hpp"
#include "rtr/system/render/pipeline/forward/forward_pipeline.hpp"  // GpuMat4, pack_mat4_row_major, ForwardSceneView types
#include "rtr/system/render/pipeline/forward/forward_scene_view.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view_builder.hpp"
#include "rtr/system/render/render_pass.hpp"
#include "rtr/utils/log.hpp"
#include "vulkan/vulkan.hpp"

// ============================================================================
// ForwardEditorPipeline (self-contained, composition-based)
//
// Sequence:
//   1. ForwardPass   — renders 3D scene to an offscreen color image.
//   2. Image barriers:
//        offscreen  eColorAttachmentOptimal → eShaderReadOnlyOptimal
//        swapchain  eUndefined              → eColorAttachmentOptimal
//   3. EditorImGuiPass — renders editor UI onto the swapchain;
//                        the scene view panel samples the offscreen image.
// ============================================================================

namespace rtr::editor::render {

class ForwardEditorPipeline final : public system::render::RenderPipelineBase,
                                    public system::render::IFramePreparePipeline,
                                    public system::render::IResourceAwarePipeline,
                                    public system::render::IFrameColorSource,
                                    public system::render::ISceneViewportSink,
                                    public IEditorInputCaptureSource {
    static constexpr uint32_t kMaxRenderables = 256;

    // ---- Vulkan pipeline resources ----
    vk::raii::PipelineLayout m_pipeline_layout{nullptr};
    vk::raii::Pipeline       m_pipeline{nullptr};

    std::unique_ptr<rhi::ShaderModule> m_vertex_shader_module{nullptr};
    std::unique_ptr<rhi::ShaderModule> m_fragment_shader_module{nullptr};

    std::unique_ptr<rhi::DescriptorSetLayout> m_per_object_layout{nullptr};
    std::unique_ptr<rhi::DescriptorPool>      m_descriptor_pool{nullptr};

    // ---- Per-frame GPU resource arrays ----
    vk::DeviceSize                                         m_uniform_buffer_size{0};
    std::vector<std::vector<std::unique_ptr<rhi::Buffer>>> m_object_uniform_buffers{};
    std::vector<std::vector<vk::raii::DescriptorSet>>      m_object_sets{};

    std::vector<std::unique_ptr<rhi::Image>> m_depth_images{};
    std::vector<std::unique_ptr<rhi::Image>> m_color_images{};
    std::vector<vk::ImageLayout>             m_color_image_layouts{};

    // ---- Scene target management ----
    vk::Extent2D m_scene_target_extent{};
    vk::Extent2D m_requested_scene_extent{};
    bool         m_scene_extent_dirty{false};

    std::optional<system::render::ForwardSceneView> m_scene_view{};
    resource::ResourceManager*                      m_resource_manager{};

    // ---- Passes ----
    std::unique_ptr<system::render::ForwardPass> m_forward_pass{nullptr};
    std::unique_ptr<EditorImGuiPass>             m_editor_pass{nullptr};

public:
    ForwardEditorPipeline(const system::render::PipelineRuntime& runtime, std::shared_ptr<EditorHost> editor_host)
        : system::render::RenderPipelineBase(runtime) {
        using namespace system::render;

        m_uniform_buffer_size = sizeof(UniformBufferObjectGpu);

        m_vertex_shader_module = std::make_unique<rhi::ShaderModule>(
            rhi::ShaderModule::from_file(m_device,
                                         "/Users/jinceyang/Desktop/codebase/graphics/rtr2/build/Debug/shaders/compiled/"
                                         "vert_buffer_vert.spv",
                                         vk::ShaderStageFlagBits::eVertex));
        m_fragment_shader_module = std::make_unique<rhi::ShaderModule>(
            rhi::ShaderModule::from_file(m_device,
                                         "/Users/jinceyang/Desktop/codebase/graphics/rtr2/build/Debug/shaders/compiled/"
                                         "vert_buffer_frag.spv",
                                         vk::ShaderStageFlagBits::eFragment));

        rhi::DescriptorSetLayout::Builder layout_builder;
        layout_builder.add_binding(0, vk::DescriptorType::eUniformBuffer,
                                   vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment);
        m_per_object_layout = std::make_unique<rhi::DescriptorSetLayout>(layout_builder.build(m_device));

        rhi::DescriptorPool::Builder pool_builder;
        pool_builder.add_layout(*m_per_object_layout, kMaxRenderables * m_frame_count)
            .set_flags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
        m_descriptor_pool = std::make_unique<rhi::DescriptorPool>(pool_builder.build(m_device));

        create_per_object_resources();

        std::array<vk::DescriptorSetLayout, 1> set_layouts{*m_per_object_layout->layout()};
        vk::PipelineLayoutCreateInfo           layout_info{};
        layout_info.setLayoutCount = static_cast<uint32_t>(set_layouts.size());
        layout_info.pSetLayouts    = set_layouts.data();
        m_pipeline_layout          = vk::raii::PipelineLayout{m_device->device(), layout_info};

        create_graphics_pipeline();

        m_forward_pass = std::make_unique<system::render::ForwardPass>(&m_pipeline_layout, &m_pipeline);
        m_editor_pass  = std::make_unique<EditorImGuiPass>(runtime, std::move(editor_host), this);
    }

    ~ForwardEditorPipeline() override {
        m_forward_pass.reset();
        m_editor_pass.reset();
        m_object_sets.clear();
        m_descriptor_pool.reset();
    }

    // -----------------------------------------------------------------------
    // IEditorInputCaptureSource
    // -----------------------------------------------------------------------
    bool wants_imgui_capture_mouse() const override { return m_editor_pass->wants_capture_mouse(); }
    bool wants_imgui_capture_keyboard() const override { return m_editor_pass->wants_capture_keyboard(); }

    // -----------------------------------------------------------------------
    // IFramePreparePipeline
    // -----------------------------------------------------------------------
    void prepare_frame(const system::render::FramePrepareContext& ctx) override {
        auto* scene = ctx.world.active_scene();
        if (!scene)
            throw std::runtime_error("ForwardEditorPipeline::prepare_frame: no active scene.");
        m_scene_view = system::render::build_forward_scene_view(*scene, ctx.resources);
    }

    // -----------------------------------------------------------------------
    // IResourceAwarePipeline
    // -----------------------------------------------------------------------
    void set_resource_manager(resource::ResourceManager* manager) override { m_resource_manager = manager; }

    // -----------------------------------------------------------------------
    // IFrameColorSource
    // -----------------------------------------------------------------------
    system::render::FrameColorSourceView frame_color_source_view(uint32_t frame_index) const override {
        if (frame_index >= m_color_images.size() || !m_color_images[frame_index])
            return {};
        return system::render::FrameColorSourceView{
            .image_view   = *m_color_images[frame_index]->image_view(),
            .image_layout = m_color_image_layouts[frame_index],
            .extent       = {m_color_images[frame_index]->width(), m_color_images[frame_index]->height()}};
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

    void handle_swapchain_state_change(const system::render::FrameScheduler::SwapchainState& state,
                                       const system::render::SwapchainChangeSummary&         diff) override {
        if (diff.color_or_depth_changed())
            create_graphics_pipeline();
        if (diff.extent_or_depth_changed())
            m_scene_extent_dirty = true;
        m_editor_pass->on_swapchain_recreated(state.image_count, state.color_format, state.depth_format);
    }

    // -----------------------------------------------------------------------
    // IRenderPipeline
    // -----------------------------------------------------------------------
    void render(system::render::FrameContext& ctx) override {
        using namespace system::render;

        // --- 1. Guard ---
        const auto extent = ctx.render_extent();
        if (extent.width == 0 || extent.height == 0)
            return;
        if (!m_resource_manager)
            throw std::runtime_error("ForwardEditorPipeline: resource manager not set.");
        if (!m_scene_view)
            throw std::runtime_error("ForwardEditorPipeline: scene view not set.");

        ensure_scene_targets(extent);
        const uint32_t frame_index = ctx.frame_index();

        // --- 2. ForwardPass: render scene to offscreen ---
        m_forward_pass->bind_render_pass_resources(
            ForwardPass::RenderPassResources{.color_image  = m_color_images[frame_index].get(),
                                             .color_layout = &m_color_image_layouts[frame_index],
                                             .depth_image  = m_depth_images[frame_index].get(),
                                             .extent       = m_scene_target_extent,
                                             .draw_items   = build_draw_items(frame_index)});
        m_forward_pass->execute(ctx);

        // --- 3. Image barriers ---
        auto& cmd = ctx.cmd().command_buffer();

        // offscreen: eColorAttachmentOptimal → eShaderReadOnlyOptimal
        vk::ImageMemoryBarrier2 offscreen_to_sampled{};
        offscreen_to_sampled.srcStageMask     = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        offscreen_to_sampled.dstStageMask     = vk::PipelineStageFlagBits2::eFragmentShader;
        offscreen_to_sampled.srcAccessMask    = vk::AccessFlagBits2::eColorAttachmentWrite;
        offscreen_to_sampled.dstAccessMask    = vk::AccessFlagBits2::eShaderRead;
        offscreen_to_sampled.oldLayout        = m_color_image_layouts[frame_index];
        offscreen_to_sampled.newLayout        = vk::ImageLayout::eShaderReadOnlyOptimal;
        offscreen_to_sampled.image            = *m_color_images[frame_index]->image();
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

        m_color_image_layouts[frame_index] = vk::ImageLayout::eShaderReadOnlyOptimal;

        // --- 4. EditorImGuiPass: render editor UI onto swapchain ---
        auto src = frame_color_source_view(frame_index);
        m_editor_pass->bind_render_pass_resources(EditorImGuiPass::RenderPassResources{
            .scene_image_view = src.image_view, .scene_image_layout = src.image_layout, .scene_extent = src.extent});
        m_editor_pass->execute(ctx);
    }

private:
    std::vector<system::render::ForwardPass::DrawItem> build_draw_items(uint32_t frame_index) {
        using namespace system::render;
        const auto& sv = m_scene_view.value();
        if (sv.renderables.size() > kMaxRenderables)
            throw std::runtime_error("Renderable count exceeds ForwardEditorPipeline capacity.");

        auto& ubos = m_object_uniform_buffers[frame_index];
        auto& sets = m_object_sets[frame_index];

        std::vector<ForwardPass::DrawItem> items;
        items.reserve(sv.renderables.size());
        for (std::size_t i = 0; i < sv.renderables.size(); ++i) {
            const auto& r    = sv.renderables[i];
            auto&       mesh = require_mesh(r.mesh);

            UniformBufferObjectGpu ubo{};
            ubo.model      = pack_mat4_row_major(r.model);
            ubo.view       = pack_mat4_row_major(sv.camera.view);
            ubo.proj       = pack_mat4_row_major(sv.camera.proj);
            ubo.normal     = pack_mat4_row_major(r.normal);
            ubo.base_color = {static_cast<float>(r.base_color.x()), static_cast<float>(r.base_color.y()),
                              static_cast<float>(r.base_color.z()), static_cast<float>(r.base_color.w())};

            ubo.point_light_count = static_cast<uint32_t>(sv.point_lights.size());
            ubo.camera_world_pos  = {static_cast<float>(sv.camera.world_pos.x()),
                                     static_cast<float>(sv.camera.world_pos.y()),
                                     static_cast<float>(sv.camera.world_pos.z())};

            for (std::size_t j = 0; j < ubo.point_light_count; ++j) {
                const auto& pl                = sv.point_lights[j];
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

            std::memcpy(ubos[i]->mapped_data(), &ubo, sizeof(ubo));
            items.push_back({.mesh = &mesh, .per_object_set = &sets[i]});
        }
        return items;
    }

    rhi::Mesh& require_mesh(resource::MeshHandle handle) {
        if (!handle.is_valid())
            throw std::runtime_error("ForwardEditorPipeline: invalid mesh handle.");
        return m_resource_manager->require_gpu<resource::MeshResourceKind>(handle, m_device);
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
                    m_device, m_uniform_buffer_size, vk::BufferUsageFlagBits::eUniformBuffer));
                buf->map();
                bufs.push_back(std::move(buf));
            }
            m_object_sets[f] = m_descriptor_pool->allocate_multiple(*m_per_object_layout, kMaxRenderables);
            for (uint32_t s = 0; s < kMaxRenderables; ++s) {
                rhi::DescriptorWriter w;
                w.write_buffer(0, *m_object_uniform_buffers[f][s]->buffer(), 0, m_uniform_buffer_size);
                w.update(m_device, *m_object_sets[f][s]);
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
                rhi::Image(m_device, m_scene_target_extent.width, m_scene_target_extent.height, m_color_format,
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
        info.pViewportState      = &vps;
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

}  // namespace rtr::editor::render
