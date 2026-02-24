#pragma once

#include <pbpt/math/math.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
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
#include "rtr/system/render/pass/present_pass.hpp"
#include "rtr/system/render/pipeline/forward/forward_pass.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view_builder.hpp"
#include "rtr/system/render/render_pipeline.hpp"
#include "rtr/system/render/render_resource_state.hpp"
#include "rtr/system/render/scene_target_controller.hpp"
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
    GpuMat4 out{};
    std::size_t idx = 0;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out.values[idx++] = static_cast<float>(m[r][c]);
    return out;
}

struct ForwardPipelineConfig {
    std::string vertex_shader_filename{"vert_buffer_vert.spv"};
    std::string fragment_shader_filename{"vert_buffer_frag.spv"};
};

// ---------------------------------------------------------------------------
// ForwardPipeline
// Sequence: ForwardPass (offscreen) → PresentPass (blit to swapchain).
// ---------------------------------------------------------------------------
class ForwardPipeline final : public RenderPipeline {
    static constexpr uint32_t kMaxRenderables = 256;

    struct ForwardFrameTargets {
        std::array<FrameTrackedImage, rhi::kFramesInFlight> color_images;
        std::array<rhi::Image, rhi::kFramesInFlight> depth_images;

        ForwardFrameTargets(
            std::array<FrameTrackedImage, rhi::kFramesInFlight>&& color_images_in,
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

    SceneTargetController<ForwardFrameTargets> m_scene_targets;
    std::optional<ForwardSceneView> m_scene_view{};

    render::ForwardPass m_forward_pass;
    render::PresentPass m_present_pass{};

public:
    ForwardPipeline(const render::PipelineRuntime& runtime, const ForwardPipelineConfig& config = {})
        : RenderPipeline(runtime),
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
          m_uniform_buffer_size(sizeof(UniformBufferObjectGpu)),
          m_scene_targets(*this, "ForwardPipeline"),
          m_forward_pass(m_pipeline_layout, m_pipeline) {
        create_per_object_resources();
        create_graphics_pipeline();
    }

    ~ForwardPipeline() override = default;

    void prepare_frame(const FramePrepareContext& ctx) override {
        auto* active_scene = ctx.world.active_scene();
        if (!active_scene)
            throw std::runtime_error("ForwardPipeline::prepare_frame: no active scene.");
        m_scene_view = build_forward_scene_view(*active_scene, ctx.resources, m_device);
    }

    void on_resize(int /*w*/, int /*h*/) override {}

    void handle_swapchain_state_change(const FrameScheduler::SwapchainState& /*state*/,
                                       const SwapchainChangeSummary& diff) override {
        if (diff.color_or_depth_changed()) {
            create_graphics_pipeline();
            m_scene_targets.request_recreate();
        }
        if (diff.extent_changed) {
            m_scene_targets.on_swapchain_extent_changed();
        }
    }

    void render(FrameContext& ctx) override {
        const auto extent = ctx.render_extent();
        if (extent.width == 0 || extent.height == 0)
            return;
        if (!m_scene_view)
            throw std::runtime_error("ForwardPipeline: scene view not set.");

        auto& frame_targets = m_scene_targets.ensure(
            extent,
            [this](vk::Extent2D desired_extent) { return create_frame_targets(desired_extent); },
            [](ForwardFrameTargets&) {}
        );

        const uint32_t frame_index = ctx.frame_index();
        check_frame_resources(frame_index);

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

        m_present_pass.execute(
            ctx,
            PresentPass::RenderPassResources{
                .src_color = tracked_color.view(),
                .src_extent = m_scene_targets.scene_extent()
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

    void check_frame_resources(uint32_t frame_index) const {
        if (frame_index >= rhi::kFramesInFlight) {
            throw std::runtime_error("ForwardPipeline frame index out of range.");
        }
        if (m_object_uniform_buffers[frame_index].empty() || m_object_sets[frame_index].empty()) {
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
            const auto& r = scene_view.renderables[i];

            UniformBufferObjectGpu ubo{};
            ubo.model = pack_mat4_row_major(r.model);
            ubo.view = pack_mat4_row_major(scene_view.camera.view);
            ubo.proj = pack_mat4_row_major(scene_view.camera.proj);
            ubo.normal = pack_mat4_row_major(r.normal);
            ubo.base_color = {static_cast<float>(r.base_color.x()), static_cast<float>(r.base_color.y()),
                              static_cast<float>(r.base_color.z()), static_cast<float>(r.base_color.w())};

            ubo.point_light_count = static_cast<uint32_t>(scene_view.point_lights.size());
            ubo.camera_world_pos = {static_cast<float>(scene_view.camera.world_pos.x()),
                                    static_cast<float>(scene_view.camera.world_pos.y()),
                                    static_cast<float>(scene_view.camera.world_pos.z())};

            for (std::size_t j = 0; j < ubo.point_light_count; ++j) {
                const auto& pl = scene_view.point_lights[j];
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

            std::memcpy(frame_ubos[i].mapped_data(), &ubo, sizeof(ubo));

            items.push_back({.mesh = r.mesh, .per_object_set = frame_sets[i]});
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

    std::array<FrameTrackedImage, rhi::kFramesInFlight> create_color_images(vk::Extent2D scene_extent) const {
        const vk::ImageUsageFlags usage = vk::ImageUsageFlagBits::eColorAttachment | vk::ImageUsageFlagBits::eSampled |
                                          vk::ImageUsageFlagBits::eTransferSrc;
        return make_frame_array<FrameTrackedImage>([&](uint32_t) {
            return FrameTrackedImage{
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

        vk::PipelineViewportStateCreateInfo vp{};
        vp.viewportCount = 1;
        vp.scissorCount = 1;

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
        info.pViewportState = &vp;
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

}  // namespace rtr::system::render
