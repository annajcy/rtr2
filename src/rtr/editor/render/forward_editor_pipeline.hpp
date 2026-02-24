#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
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
#include "rtr/system/render/pipeline/forward/forward_pass.hpp"
#include "rtr/system/render/pipeline/forward/forward_pipeline.hpp"  // GpuMat4, pack_mat4_row_major, ForwardSceneView types
#include "rtr/system/render/pipeline/forward/forward_scene_view.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view_builder.hpp"
#include "rtr/system/render/render_pipeline.hpp"
#include "rtr/system/render/render_resource_state.hpp"
#include "rtr/system/render/scene_target_controller.hpp"
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

struct ForwardEditorPipelineConfig {
    std::string vertex_shader_filename{"vert_buffer_vert.spv"};
    std::string fragment_shader_filename{"vert_buffer_frag.spv"};
};

class ForwardEditorPipeline final : public system::render::RenderPipeline,
                                    public IEditorInputCaptureSource {
    static constexpr uint32_t kMaxRenderables = 256;

    struct ForwardFrameTargets {
        std::array<system::render::FrameTrackedImage, rhi::kFramesInFlight> color_images;
        std::array<rhi::Image, rhi::kFramesInFlight> depth_images;

        ForwardFrameTargets(
            std::array<system::render::FrameTrackedImage, rhi::kFramesInFlight>&& color_images_in,
            std::array<rhi::Image, rhi::kFramesInFlight>&& depth_images_in
        )
            : color_images(std::move(color_images_in)),
              depth_images(std::move(depth_images_in)) {}
    };

    rhi::ShaderModule m_vertex_shader_module;
    rhi::ShaderModule m_fragment_shader_module;
    rhi::DescriptorSetLayout m_per_object_layout;
    rhi::DescriptorPool m_descriptor_pool;
    vk::raii::PipelineLayout m_pipeline_layout{nullptr};
    vk::raii::Pipeline m_pipeline{nullptr};

    vk::DeviceSize m_uniform_buffer_size{0};
    std::array<std::vector<rhi::Buffer>, rhi::kFramesInFlight> m_object_uniform_buffers{};
    std::array<std::vector<vk::raii::DescriptorSet>, rhi::kFramesInFlight> m_object_sets{};

    system::render::SceneTargetController<ForwardFrameTargets> m_scene_targets;
    std::optional<system::render::ForwardSceneView> m_scene_view{};

    system::render::ForwardPass m_forward_pass;
    EditorImGuiPass m_editor_pass;

public:
    ForwardEditorPipeline(
        const system::render::PipelineRuntime& runtime,
        std::shared_ptr<EditorHost> editor_host,
        const ForwardEditorPipelineConfig& config = {}
    )
        : system::render::RenderPipeline(runtime),
          m_vertex_shader_module(build_shader_module(
              m_device,
              resolve_shader_path(runtime, config.vertex_shader_filename),
              vk::ShaderStageFlagBits::eVertex
          )),
          m_fragment_shader_module(build_shader_module(
              m_device,
              resolve_shader_path(runtime, config.fragment_shader_filename),
              vk::ShaderStageFlagBits::eFragment
          )),
          m_per_object_layout(build_per_object_layout(m_device)),
          m_descriptor_pool(build_per_object_pool(m_device, m_per_object_layout,
                                                  static_cast<uint32_t>(rhi::kFramesInFlight), kMaxRenderables)),
          m_pipeline_layout(build_pipeline_layout(m_device, m_per_object_layout)),
          m_uniform_buffer_size(sizeof(system::render::UniformBufferObjectGpu)),
          m_scene_targets(*this, "ForwardEditorPipeline"),
          m_forward_pass(m_pipeline_layout, m_pipeline),
          m_editor_pass(runtime, std::move(editor_host), *this) {
        create_per_object_resources();
        create_graphics_pipeline();
    }

    ~ForwardEditorPipeline() override = default;

    bool wants_imgui_capture_mouse() const override { return m_editor_pass.wants_capture_mouse(); }
    bool wants_imgui_capture_keyboard() const override { return m_editor_pass.wants_capture_keyboard(); }

    void prepare_frame(const system::render::FramePrepareContext& ctx) override {
        auto* scene = ctx.world.active_scene();
        if (!scene)
            throw std::runtime_error("ForwardEditorPipeline::prepare_frame: no active scene.");
        m_scene_view = system::render::build_forward_scene_view(*scene, ctx.resources, m_device);
    }

    void on_resize(int /*w*/, int /*h*/) override {}

    void handle_swapchain_state_change(const system::render::FrameScheduler::SwapchainState& state,
                                       const system::render::SwapchainChangeSummary& diff) override {
        if (diff.color_or_depth_changed()) {
            create_graphics_pipeline();
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
        if (extent.width == 0 || extent.height == 0)
            return;
        if (!m_scene_view)
            throw std::runtime_error("ForwardEditorPipeline: scene view not set.");

        auto& frame_targets = m_scene_targets.ensure(
            ctx.frame_index(),
            extent,
            [this](vk::Extent2D desired_extent) { return create_frame_targets(desired_extent); },
            [](ForwardFrameTargets&) {}
        );

        const uint32_t frame_index = ctx.frame_index();
        auto& tracked_color = frame_targets.color_images[frame_index];

        m_forward_pass.execute(
            ctx,
            ForwardPass::RenderPassResources{
                .color = tracked_color.view(),
                .depth_image = frame_targets.depth_images[frame_index],
                .extent = m_scene_targets.scene_extent(),
                .draw_items = build_draw_items(frame_index)
            }
        );

        auto& cmd = ctx.cmd().command_buffer();

        vk::ImageMemoryBarrier2 offscreen_to_sampled{};
        offscreen_to_sampled.srcStageMask = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        offscreen_to_sampled.dstStageMask = vk::PipelineStageFlagBits2::eFragmentShader;
        offscreen_to_sampled.srcAccessMask = vk::AccessFlagBits2::eColorAttachmentWrite;
        offscreen_to_sampled.dstAccessMask = vk::AccessFlagBits2::eShaderRead;
        offscreen_to_sampled.oldLayout = tracked_color.layout;
        offscreen_to_sampled.newLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
        offscreen_to_sampled.image = *tracked_color.image.image();
        offscreen_to_sampled.subresourceRange = {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};

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

        tracked_color.layout = vk::ImageLayout::eShaderReadOnlyOptimal;

        m_editor_pass.execute(
            ctx,
            EditorImGuiPass::RenderPassResources{
                .scene_image_view = *tracked_color.image.image_view(),
                .scene_image_layout = tracked_color.layout,
                .scene_extent = {tracked_color.image.width(), tracked_color.image.height()}
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

    static rhi::DescriptorSetLayout build_per_object_layout(rhi::Device& device) {
        rhi::DescriptorSetLayout::Builder layout_builder;
        layout_builder.add_binding(
            0,
            vk::DescriptorType::eUniformBuffer,
            vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment
        );
        return layout_builder.build(device);
    }

    static rhi::DescriptorPool build_per_object_pool(
        rhi::Device& device,
        const rhi::DescriptorSetLayout& per_object_layout,
        uint32_t frame_count,
        uint32_t max_renderables
    ) {
        rhi::DescriptorPool::Builder pool_builder;
        pool_builder.add_layout(per_object_layout, max_renderables * frame_count)
            .set_flags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
        return pool_builder.build(device);
    }

    static vk::raii::PipelineLayout build_pipeline_layout(
        rhi::Device& device,
        const rhi::DescriptorSetLayout& per_object_layout
    ) {
        std::array<vk::DescriptorSetLayout, 1> set_layouts{*per_object_layout.layout()};
        vk::PipelineLayoutCreateInfo layout_info{};
        layout_info.setLayoutCount = static_cast<uint32_t>(set_layouts.size());
        layout_info.pSetLayouts = set_layouts.data();
        return vk::raii::PipelineLayout{device.device(), layout_info};
    }

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
            const auto& r = sv.renderables[i];

            UniformBufferObjectGpu ubo{};
            ubo.model = pack_mat4_row_major(r.model);
            ubo.view = pack_mat4_row_major(sv.camera.view);
            ubo.proj = pack_mat4_row_major(sv.camera.proj);
            ubo.normal = pack_mat4_row_major(r.normal);
            ubo.base_color = {static_cast<float>(r.base_color.x()), static_cast<float>(r.base_color.y()),
                              static_cast<float>(r.base_color.z()), static_cast<float>(r.base_color.w())};

            ubo.point_light_count = static_cast<uint32_t>(sv.point_lights.size());
            ubo.camera_world_pos = {static_cast<float>(sv.camera.world_pos.x()),
                                    static_cast<float>(sv.camera.world_pos.y()),
                                    static_cast<float>(sv.camera.world_pos.z())};

            for (std::size_t j = 0; j < ubo.point_light_count; ++j) {
                const auto& pl = sv.point_lights[j];
                ubo.point_lights[j].position = {static_cast<float>(pl.position.x()),
                                                static_cast<float>(pl.position.y()),
                                                static_cast<float>(pl.position.z())};
                ubo.point_lights[j].intensity = pl.intensity;
                ubo.point_lights[j].color = {static_cast<float>(pl.color.x()), static_cast<float>(pl.color.y()),
                                             static_cast<float>(pl.color.z())};
                ubo.point_lights[j].range = pl.range;
                ubo.point_lights[j].specular_strength = pl.specular_strength;
                ubo.point_lights[j].shininess = pl.shininess;
            }

            std::memcpy(ubos[i].mapped_data(), &ubo, sizeof(ubo));
            items.push_back({.mesh = r.mesh, .per_object_set = sets[i]});
        }
        return items;
    }

    void create_per_object_resources() {
        for (uint32_t f = 0; f < rhi::kFramesInFlight; ++f) {
            auto& bufs = m_object_uniform_buffers[f];
            bufs.clear();
            bufs.reserve(kMaxRenderables);
            for (uint32_t s = 0; s < kMaxRenderables; ++s) {
                auto buffer = rhi::Buffer::create_host_visible_buffer(
                    m_device, m_uniform_buffer_size, vk::BufferUsageFlagBits::eUniformBuffer
                );
                buffer.map();
                bufs.push_back(std::move(buffer));
            }
            m_object_sets[f] = m_descriptor_pool.allocate_multiple(m_per_object_layout, kMaxRenderables);
            for (uint32_t s = 0; s < kMaxRenderables; ++s) {
                rhi::DescriptorWriter w;
                w.write_buffer(0, *m_object_uniform_buffers[f][s].buffer(), 0, m_uniform_buffer_size);
                w.update(m_device, *m_object_sets[f][s]);
            }
        }
    }

    std::array<system::render::FrameTrackedImage, rhi::kFramesInFlight> create_color_images(vk::Extent2D scene_extent) const {
        const vk::ImageUsageFlags usage = vk::ImageUsageFlagBits::eColorAttachment | vk::ImageUsageFlagBits::eSampled |
                                          vk::ImageUsageFlagBits::eTransferSrc;
        return make_frame_array<system::render::FrameTrackedImage>([&](uint32_t) {
            return system::render::FrameTrackedImage{
                rhi::Image(
                    m_device,
                    scene_extent.width,
                    scene_extent.height,
                    m_color_format,
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

    ForwardFrameTargets create_frame_targets(vk::Extent2D scene_extent) const {
        return ForwardFrameTargets{
            create_color_images(scene_extent),
            make_per_frame_depth_images(scene_extent, m_depth_format)
        };
    }

    void create_graphics_pipeline() {
        std::vector<vk::PipelineShaderStageCreateInfo> stages = {
            m_vertex_shader_module.stage_create_info(),
            m_fragment_shader_module.stage_create_info()
        };

        auto vi_state = rhi::Mesh::vertex_input_state();
        vk::PipelineVertexInputStateCreateInfo vi{};
        vi.vertexBindingDescriptionCount = static_cast<uint32_t>(vi_state.bindings.size());
        vi.pVertexBindingDescriptions = vi_state.bindings.data();
        vi.vertexAttributeDescriptionCount = static_cast<uint32_t>(vi_state.attributes.size());
        vi.pVertexAttributeDescriptions = vi_state.attributes.data();

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
        ds.depthTestEnable = VK_TRUE;
        ds.depthWriteEnable = VK_TRUE;
        ds.depthCompareOp = vk::CompareOp::eLess;

        vk::PipelineColorBlendAttachmentState cba{};
        cba.blendEnable = VK_FALSE;
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
        info.layout = *m_pipeline_layout;
        info.renderPass = VK_NULL_HANDLE;

        vk::PipelineRenderingCreateInfo ri{};
        vk::Format cfmt = m_color_format;
        ri.colorAttachmentCount = 1;
        ri.pColorAttachmentFormats = &cfmt;
        ri.depthAttachmentFormat = m_depth_format;

        vk::StructureChain<vk::GraphicsPipelineCreateInfo, vk::PipelineRenderingCreateInfo> chain{info, ri};
        m_pipeline = vk::raii::Pipeline{m_device.device(), nullptr, chain.get<vk::GraphicsPipelineCreateInfo>()};
    }
};

}  // namespace rtr::editor::render
