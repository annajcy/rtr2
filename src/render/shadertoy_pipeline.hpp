#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "render/imgui_pass.hpp"
#include "render/pipeline.hpp"
#include "render/render_pass.hpp"
#include "rhi/buffer.hpp"
#include "rhi/descriptor.hpp"
#include "rhi/shader_module.hpp"
#include "rhi/texture.hpp"
#include "vulkan/vulkan.hpp"

namespace rtr::render {

struct ShaderToyPipelineConfig {
    std::string shader_output_dir{"/Users/jinceyang/Desktop/codebase/graphics/rtr2/build/Debug/shaders/compiled/"};
    std::string compute_shader_filename{"shadertoy_comp.spv"};
    std::string present_vertex_shader_filename{"shadertoy_vert.spv"};
    std::string present_fragment_shader_filename{"shadertoy_frag.spv"};
};

struct ShaderToyUniformBufferObject {
    alignas(16) std::array<float, 4> i_resolution{};
    alignas(16) std::array<float, 4> i_time{};
};

class ShaderToyComputePass final : public IRenderPass {
public:
    struct FrameResources {
        rhi::Buffer* uniform_buffer{};
        rhi::Image* offscreen_image{};
        vk::ImageLayout* offscreen_layout{};
        vk::raii::DescriptorSet* compute_set{};
    };

private:
    vk::raii::PipelineLayout* m_pipeline_layout{};
    vk::raii::Pipeline* m_compute_pipeline{};
    FrameResources m_frame_resources{};
    std::chrono::steady_clock::time_point m_start_time = std::chrono::steady_clock::now();
    std::vector<ResourceDependency> m_dependencies{
        {"shadertoy.uniform", ResourceAccess::eRead},
        {"shadertoy.compute", ResourceAccess::eRead},
        {"shadertoy.offscreen", ResourceAccess::eReadWrite}
    };

public:
    ShaderToyComputePass(
        vk::raii::PipelineLayout* pipeline_layout,
        vk::raii::Pipeline* compute_pipeline
    )
        : m_pipeline_layout(pipeline_layout),
          m_compute_pipeline(compute_pipeline) {}

    std::string_view name() const override { return "shadertoy.compute"; }

    const std::vector<ResourceDependency>& dependencies() const override { return m_dependencies; }

    void bind_frame_resources(const FrameResources& resources) {
        if (resources.uniform_buffer == nullptr ||
            resources.offscreen_image == nullptr ||
            resources.offscreen_layout == nullptr ||
            resources.compute_set == nullptr) {
            throw std::runtime_error("ShaderToyComputePass frame resources are incomplete.");
        }
        m_frame_resources = resources;
    }

    void execute(render::FrameContext& ctx) override {
        if (m_frame_resources.uniform_buffer == nullptr ||
            m_frame_resources.offscreen_image == nullptr ||
            m_frame_resources.offscreen_layout == nullptr ||
            m_frame_resources.compute_set == nullptr) {
            throw std::runtime_error("ShaderToyComputePass frame resources are not bound.");
        }

        update_uniform_buffer(*m_frame_resources.uniform_buffer, ctx.render_extent());

        auto& cmd = ctx.cmd().command_buffer();
        rhi::Image& offscreen = *m_frame_resources.offscreen_image;
        const vk::ImageLayout old_layout = *m_frame_resources.offscreen_layout;

        vk::PipelineStageFlags2 src_stage = vk::PipelineStageFlagBits2::eTopOfPipe;
        vk::AccessFlags2 src_access = vk::AccessFlagBits2::eNone;
        if (old_layout == vk::ImageLayout::eShaderReadOnlyOptimal) {
            src_stage = vk::PipelineStageFlagBits2::eFragmentShader;
            src_access = vk::AccessFlagBits2::eShaderSampledRead;
        } else if (old_layout == vk::ImageLayout::eGeneral) {
            src_stage = vk::PipelineStageFlagBits2::eComputeShader;
            src_access = vk::AccessFlagBits2::eShaderStorageWrite;
        }

        vk::ImageMemoryBarrier2 to_general{};
        to_general.srcStageMask = src_stage;
        to_general.dstStageMask = vk::PipelineStageFlagBits2::eComputeShader;
        to_general.srcAccessMask = src_access;
        to_general.dstAccessMask = vk::AccessFlagBits2::eShaderStorageWrite;
        to_general.oldLayout = old_layout;
        to_general.newLayout = vk::ImageLayout::eGeneral;
        to_general.image = *offscreen.image();
        to_general.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
        to_general.subresourceRange.baseMipLevel = 0;
        to_general.subresourceRange.levelCount = 1;
        to_general.subresourceRange.baseArrayLayer = 0;
        to_general.subresourceRange.layerCount = 1;

        vk::DependencyInfo to_general_dep{};
        to_general_dep.imageMemoryBarrierCount = 1;
        to_general_dep.pImageMemoryBarriers = &to_general;
        cmd.pipelineBarrier2(to_general_dep);

        *m_frame_resources.offscreen_layout = vk::ImageLayout::eGeneral;

        cmd.bindPipeline(vk::PipelineBindPoint::eCompute, **m_compute_pipeline);
        cmd.bindDescriptorSets(
            vk::PipelineBindPoint::eCompute,
            **m_pipeline_layout,
            0,
            **m_frame_resources.compute_set,
            {}
        );

        const vk::Extent2D extent = ctx.render_extent();
        const uint32_t group_count_x = (extent.width + 7) / 8;
        const uint32_t group_count_y = (extent.height + 7) / 8;
        cmd.dispatch(group_count_x, group_count_y, 1);
    }

private:
    void update_uniform_buffer(rhi::Buffer& uniform_buffer, const vk::Extent2D& extent) {
        const auto now = std::chrono::steady_clock::now();
        const float seconds = std::chrono::duration<float>(now - m_start_time).count();

        ShaderToyUniformBufferObject ubo{};
        ubo.i_resolution = {
            static_cast<float>(extent.width),
            static_cast<float>(extent.height),
            1.0f,
            0.0f
        };
        ubo.i_time = {seconds, 0.0f, 0.0f, 0.0f};

        std::memcpy(uniform_buffer.mapped_data(), &ubo, sizeof(ubo));
    }
};

class ShaderToyPresentPass final : public IRenderPass {
public:
    struct FrameResources {
        rhi::Image* offscreen_image{};
        vk::ImageLayout* offscreen_layout{};
        rhi::Image* depth_image{};
        vk::raii::DescriptorSet* present_set{};
    };

private:
    vk::raii::PipelineLayout* m_pipeline_layout{};
    vk::raii::Pipeline* m_present_pipeline{};
    FrameResources m_frame_resources{};
    std::vector<ResourceDependency> m_dependencies{
        {"shadertoy.present", ResourceAccess::eRead},
        {"shadertoy.offscreen", ResourceAccess::eRead},
        {"swapchain_color", ResourceAccess::eReadWrite},
        {"depth", ResourceAccess::eReadWrite}
    };

public:
    ShaderToyPresentPass(
        vk::raii::PipelineLayout* pipeline_layout,
        vk::raii::Pipeline* present_pipeline
    )
        : m_pipeline_layout(pipeline_layout),
          m_present_pipeline(present_pipeline) {}

    std::string_view name() const override { return "shadertoy.present"; }

    const std::vector<ResourceDependency>& dependencies() const override { return m_dependencies; }

    void bind_frame_resources(const FrameResources& resources) {
        if (resources.offscreen_image == nullptr ||
            resources.offscreen_layout == nullptr ||
            resources.depth_image == nullptr ||
            resources.present_set == nullptr) {
            throw std::runtime_error("ShaderToyPresentPass frame resources are incomplete.");
        }
        m_frame_resources = resources;
    }

    void execute(render::FrameContext& ctx) override {
        if (m_frame_resources.offscreen_image == nullptr ||
            m_frame_resources.offscreen_layout == nullptr ||
            m_frame_resources.depth_image == nullptr ||
            m_frame_resources.present_set == nullptr) {
            throw std::runtime_error("ShaderToyPresentPass frame resources are not bound.");
        }

        auto& cmd = ctx.cmd().command_buffer();
        rhi::Image& offscreen = *m_frame_resources.offscreen_image;
        rhi::Image& depth = *m_frame_resources.depth_image;

        vk::ImageMemoryBarrier2 to_sampled{};
        to_sampled.srcStageMask = vk::PipelineStageFlagBits2::eComputeShader;
        to_sampled.dstStageMask = vk::PipelineStageFlagBits2::eFragmentShader;
        to_sampled.srcAccessMask = vk::AccessFlagBits2::eShaderStorageWrite;
        to_sampled.dstAccessMask = vk::AccessFlagBits2::eShaderSampledRead;
        to_sampled.oldLayout = vk::ImageLayout::eGeneral;
        to_sampled.newLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
        to_sampled.image = *offscreen.image();
        to_sampled.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
        to_sampled.subresourceRange.baseMipLevel = 0;
        to_sampled.subresourceRange.levelCount = 1;
        to_sampled.subresourceRange.baseArrayLayer = 0;
        to_sampled.subresourceRange.layerCount = 1;

        vk::DependencyInfo to_sampled_dep{};
        to_sampled_dep.imageMemoryBarrierCount = 1;
        to_sampled_dep.pImageMemoryBarriers = &to_sampled;
        cmd.pipelineBarrier2(to_sampled_dep);
        *m_frame_resources.offscreen_layout = vk::ImageLayout::eShaderReadOnlyOptimal;

        vk::ImageMemoryBarrier2 to_color{};
        to_color.srcStageMask = vk::PipelineStageFlagBits2::eTopOfPipe;
        to_color.dstStageMask = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        to_color.srcAccessMask = vk::AccessFlagBits2::eNone;
        to_color.dstAccessMask = vk::AccessFlagBits2::eColorAttachmentWrite;
        to_color.oldLayout = vk::ImageLayout::eUndefined;
        to_color.newLayout = vk::ImageLayout::eColorAttachmentOptimal;
        to_color.image = ctx.swapchain_image();
        to_color.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
        to_color.subresourceRange.baseMipLevel = 0;
        to_color.subresourceRange.levelCount = 1;
        to_color.subresourceRange.baseArrayLayer = 0;
        to_color.subresourceRange.layerCount = 1;

        vk::ImageMemoryBarrier2 to_depth{};
        to_depth.srcStageMask = vk::PipelineStageFlagBits2::eTopOfPipe;
        to_depth.dstStageMask = vk::PipelineStageFlagBits2::eEarlyFragmentTests | vk::PipelineStageFlagBits2::eLateFragmentTests;
        to_depth.srcAccessMask = vk::AccessFlagBits2::eNone;
        to_depth.dstAccessMask = vk::AccessFlagBits2::eDepthStencilAttachmentRead | vk::AccessFlagBits2::eDepthStencilAttachmentWrite;
        to_depth.oldLayout = vk::ImageLayout::eUndefined;
        to_depth.newLayout = vk::ImageLayout::eDepthAttachmentOptimal;
        to_depth.image = *depth.image();
        to_depth.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eDepth;
        to_depth.subresourceRange.baseMipLevel = 0;
        to_depth.subresourceRange.levelCount = 1;
        to_depth.subresourceRange.baseArrayLayer = 0;
        to_depth.subresourceRange.layerCount = 1;

        std::array<vk::ImageMemoryBarrier2, 2> barriers = {to_color, to_depth};
        vk::DependencyInfo to_render_dep{};
        to_render_dep.imageMemoryBarrierCount = static_cast<uint32_t>(barriers.size());
        to_render_dep.pImageMemoryBarriers = barriers.data();
        cmd.pipelineBarrier2(to_render_dep);

        vk::ClearValue clear_value = vk::ClearValue{vk::ClearColorValue{0.0f, 0.0f, 0.0f, 1.0f}};
        vk::RenderingAttachmentInfo color_attachment_info{};
        color_attachment_info.imageView = *ctx.swapchain_image_view();
        color_attachment_info.imageLayout = vk::ImageLayout::eColorAttachmentOptimal;
        color_attachment_info.loadOp = vk::AttachmentLoadOp::eClear;
        color_attachment_info.storeOp = vk::AttachmentStoreOp::eStore;
        color_attachment_info.clearValue = clear_value;

        vk::ClearValue depth_clear{vk::ClearDepthStencilValue{1.0f, 0}};
        vk::RenderingAttachmentInfo depth_attachment_info{};
        depth_attachment_info.imageView = *depth.image_view();
        depth_attachment_info.imageLayout = vk::ImageLayout::eDepthAttachmentOptimal;
        depth_attachment_info.loadOp = vk::AttachmentLoadOp::eClear;
        depth_attachment_info.storeOp = vk::AttachmentStoreOp::eStore;
        depth_attachment_info.clearValue = depth_clear;

        vk::RenderingInfo rendering_info{};
        rendering_info.renderArea.offset = vk::Offset2D{0, 0};
        rendering_info.renderArea.extent = ctx.render_extent();
        rendering_info.layerCount = 1;
        rendering_info.colorAttachmentCount = 1;
        rendering_info.pColorAttachments = &color_attachment_info;
        rendering_info.pDepthAttachment = &depth_attachment_info;

        cmd.beginRendering(rendering_info);
        cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, **m_present_pipeline);
        cmd.bindDescriptorSets(
            vk::PipelineBindPoint::eGraphics,
            **m_pipeline_layout,
            1,
            **m_frame_resources.present_set,
            {}
        );

        vk::Viewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = static_cast<float>(ctx.render_extent().width);
        viewport.height = static_cast<float>(ctx.render_extent().height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        cmd.setViewport(0, viewport);

        vk::Rect2D scissor{};
        scissor.offset = vk::Offset2D{0, 0};
        scissor.extent = ctx.render_extent();
        cmd.setScissor(0, scissor);

        cmd.draw(3, 1, 0, 0);
        cmd.endRendering();
    }
};

class ShaderToyPipeline final : public RenderPipelineBase {
private:
    struct OffscreenFrameResources {
        std::unique_ptr<rhi::Image> image{};
        vk::ImageLayout layout{vk::ImageLayout::eUndefined};
    };

    vk::Format m_offscreen_format{vk::Format::eUndefined};

    std::unique_ptr<rhi::ShaderModule> m_compute_shader_module{nullptr};
    std::unique_ptr<rhi::ShaderModule> m_present_vertex_shader_module{nullptr};
    std::unique_ptr<rhi::ShaderModule> m_present_fragment_shader_module{nullptr};

    std::unique_ptr<rhi::DescriptorSystem> m_descriptor_system{nullptr};
    vk::raii::PipelineLayout m_pipeline_layout{nullptr};
    vk::raii::Pipeline m_compute_pipeline{nullptr};
    vk::raii::Pipeline m_present_pipeline{nullptr};

    vk::DeviceSize m_uniform_buffer_size{0};
    std::vector<std::unique_ptr<rhi::Buffer>> m_uniform_buffers{};
    std::vector<OffscreenFrameResources> m_offscreen_frame_resources{};
    std::vector<std::unique_ptr<rhi::Image>> m_depth_images{};
    std::unique_ptr<rhi::Sampler> m_offscreen_sampler{nullptr};

    std::unique_ptr<ShaderToyComputePass> m_compute_pass{nullptr};
    std::unique_ptr<ShaderToyPresentPass> m_present_pass{nullptr};
    std::unique_ptr<ImGUIPass> m_imgui_pass{nullptr};

public:
    ShaderToyPipeline(
        const render::PipelineRuntime& runtime,
        const ShaderToyPipelineConfig& config = {}
    )
        : RenderPipelineBase(runtime) {
        m_uniform_buffer_size = sizeof(ShaderToyUniformBufferObject);
        m_offscreen_format = pick_offscreen_format();

        m_compute_shader_module = std::make_unique<rhi::ShaderModule>(
            rhi::ShaderModule::from_file(
                m_device,
                config.shader_output_dir + config.compute_shader_filename,
                vk::ShaderStageFlagBits::eCompute
            )
        );
        m_present_vertex_shader_module = std::make_unique<rhi::ShaderModule>(
            rhi::ShaderModule::from_file(
                m_device,
                config.shader_output_dir + config.present_vertex_shader_filename,
                vk::ShaderStageFlagBits::eVertex
            )
        );
        m_present_fragment_shader_module = std::make_unique<rhi::ShaderModule>(
            rhi::ShaderModule::from_file(
                m_device,
                config.shader_output_dir + config.present_fragment_shader_filename,
                vk::ShaderStageFlagBits::eFragment
            )
        );

        m_uniform_buffers = make_per_frame_mapped_uniform_buffers(m_uniform_buffer_size);
        m_offscreen_sampler = std::make_unique<rhi::Sampler>(rhi::Sampler::create_default(m_device, 1));

        m_descriptor_system = std::make_unique<rhi::DescriptorSystem>(
            rhi::DescriptorSystem::Builder(m_device)
                .add_set("compute", 0, m_frame_count, [](rhi::DescriptorSetLayout::Builder& builder) {
                    builder.add_binding(0, vk::DescriptorType::eUniformBuffer, vk::ShaderStageFlagBits::eCompute);
                    builder.add_binding(1, vk::DescriptorType::eStorageImage, vk::ShaderStageFlagBits::eCompute);
                })
                .add_set("present", 1, m_frame_count, [](rhi::DescriptorSetLayout::Builder& builder) {
                    builder.add_binding(0, vk::DescriptorType::eCombinedImageSampler, vk::ShaderStageFlagBits::eFragment);
                })
                .build()
        );

        auto layout_info = rhi::DescriptorSystem::make_pipeline_layout_info(*m_descriptor_system);
        m_pipeline_layout = vk::raii::PipelineLayout{m_device->device(), layout_info.info};

        build_compute_pipeline();
        rebuild_present_graphics_pipeline();

        m_compute_pass = std::make_unique<ShaderToyComputePass>(
            &m_pipeline_layout,
            &m_compute_pipeline
        );
        m_present_pass = std::make_unique<ShaderToyPresentPass>(
            &m_pipeline_layout,
            &m_present_pipeline
        );
        m_imgui_pass = std::make_unique<ImGUIPass>(
            m_device,
            m_context,
            m_window,
            m_image_count,
            m_color_format,
            m_depth_format
        );
    }

    ~ShaderToyPipeline() override = default;

    ImGUIPass& imgui_pass() {
        if (!m_imgui_pass) {
            throw std::runtime_error("ShaderToyPipeline imgui pass is not initialized.");
        }
        return *m_imgui_pass;
    }

    const ImGUIPass& imgui_pass() const {
        if (!m_imgui_pass) {
            throw std::runtime_error("ShaderToyPipeline imgui pass is not initialized.");
        }
        return *m_imgui_pass;
    }

    void on_resize(int /*width*/, int /*height*/) override {}

    void handle_swapchain_state_change(
        const FrameScheduler::SwapchainState& /*state*/,
        const SwapchainChangeSummary& diff
    ) override {
        if (!has_valid_extent()) {
            return;
        }

        if (diff.extent_or_depth_changed() || m_offscreen_frame_resources.empty() || m_depth_images.empty()) {
            create_offscreen_images();
            create_depth_images();
            refresh_descriptor_sets();
        }

        if (diff.color_or_depth_changed() || m_present_pipeline == nullptr) {
            rebuild_present_graphics_pipeline();
        }

        if (m_imgui_pass) {
            m_imgui_pass->on_swapchain_recreated(m_image_count, m_color_format, m_depth_format);
        }
    }

    void render(FrameContext& ctx) override {
        const auto extent = ctx.render_extent();
        if (extent.width == 0 || extent.height == 0) {
            return;
        }
        const uint32_t frame_index = ctx.frame_index();
        if (frame_index >= m_uniform_buffers.size() ||
            frame_index >= m_offscreen_frame_resources.size() ||
            frame_index >= m_depth_images.size()) {
            throw std::runtime_error("ShaderToyPipeline frame resources are not ready.");
        }
        if (!m_compute_pass || !m_present_pass || !m_imgui_pass) {
            throw std::runtime_error("ShaderToyPipeline passes are not initialized.");
        }

        auto& offscreen_frame = m_offscreen_frame_resources[frame_index];

        m_compute_pass->bind_frame_resources(ShaderToyComputePass::FrameResources{
            .uniform_buffer = m_uniform_buffers[frame_index].get(),
            .offscreen_image = offscreen_frame.image.get(),
            .offscreen_layout = &offscreen_frame.layout,
            .compute_set = &m_descriptor_system->get_set("compute", frame_index)
        });
        m_compute_pass->execute(ctx);

        m_present_pass->bind_frame_resources(ShaderToyPresentPass::FrameResources{
            .offscreen_image = offscreen_frame.image.get(),
            .offscreen_layout = &offscreen_frame.layout,
            .depth_image = m_depth_images[frame_index].get(),
            .present_set = &m_descriptor_system->get_set("present", frame_index)
        });
        m_present_pass->execute(ctx);

        m_imgui_pass->bind_frame_resources(ImGUIPass::FrameResources{
            .depth_image = m_depth_images[frame_index].get()
        });
        m_imgui_pass->execute(ctx);
    }

private:
    vk::Format pick_offscreen_format() const {
        auto supports_storage_and_sampled = [this](vk::Format format) {
            const auto props = m_device->physical_device().getFormatProperties(format);
            const vk::FormatFeatureFlags features = props.optimalTilingFeatures;
            const bool supports_storage = (features & vk::FormatFeatureFlagBits::eStorageImage) == vk::FormatFeatureFlagBits::eStorageImage;
            const bool supports_sampled = (features & vk::FormatFeatureFlagBits::eSampledImage) == vk::FormatFeatureFlagBits::eSampledImage;
            return supports_storage && supports_sampled;
        };

        if (supports_storage_and_sampled(vk::Format::eR16G16B16A16Sfloat)) {
            return vk::Format::eR16G16B16A16Sfloat;
        }
        if (supports_storage_and_sampled(vk::Format::eR8G8B8A8Unorm)) {
            return vk::Format::eR8G8B8A8Unorm;
        }
        throw std::runtime_error("No supported offscreen format for storage+sampled image.");
    }

    void build_compute_pipeline() {
        vk::ComputePipelineCreateInfo compute_pipeline_create_info{};
        compute_pipeline_create_info.stage = m_compute_shader_module->stage_create_info();
        compute_pipeline_create_info.layout = *m_pipeline_layout;

        m_compute_pipeline = vk::raii::Pipeline{
            m_device->device(),
            nullptr,
            compute_pipeline_create_info
        };
    }

    void rebuild_present_graphics_pipeline() {
        std::vector<vk::PipelineShaderStageCreateInfo> shader_stage_infos = {
            m_present_vertex_shader_module->stage_create_info(),
            m_present_fragment_shader_module->stage_create_info()
        };

        vk::PipelineVertexInputStateCreateInfo vertex_input_info{};
        vertex_input_info.vertexBindingDescriptionCount = 0;
        vertex_input_info.vertexAttributeDescriptionCount = 0;

        vk::PipelineInputAssemblyStateCreateInfo input_assembly_info{};
        input_assembly_info.topology = vk::PrimitiveTopology::eTriangleList;

        vk::PipelineViewportStateCreateInfo viewport_info{};
        viewport_info.viewportCount = 1;
        viewport_info.scissorCount = 1;

        vk::PipelineRasterizationStateCreateInfo rasterization_info{};
        rasterization_info.depthClampEnable = VK_FALSE;
        rasterization_info.rasterizerDiscardEnable = VK_FALSE;
        rasterization_info.polygonMode = vk::PolygonMode::eFill;
        rasterization_info.cullMode = vk::CullModeFlagBits::eNone;
        rasterization_info.frontFace = vk::FrontFace::eCounterClockwise;
        rasterization_info.depthBiasEnable = VK_FALSE;
        rasterization_info.lineWidth = 1.0f;

        vk::PipelineMultisampleStateCreateInfo multisample_info{};
        multisample_info.rasterizationSamples = vk::SampleCountFlagBits::e1;

        vk::PipelineDepthStencilStateCreateInfo depth_info{};
        depth_info.depthTestEnable = VK_FALSE;
        depth_info.depthWriteEnable = VK_FALSE;
        depth_info.depthCompareOp = vk::CompareOp::eAlways;

        vk::PipelineColorBlendAttachmentState color_blend_attachment{};
        color_blend_attachment.blendEnable = VK_FALSE;
        color_blend_attachment.colorWriteMask =
            vk::ColorComponentFlagBits::eR |
            vk::ColorComponentFlagBits::eG |
            vk::ColorComponentFlagBits::eB |
            vk::ColorComponentFlagBits::eA;

        vk::PipelineColorBlendStateCreateInfo color_blend_state{};
        color_blend_state.logicOpEnable = VK_FALSE;
        color_blend_state.logicOp = vk::LogicOp::eCopy;
        color_blend_state.attachmentCount = 1;
        color_blend_state.pAttachments = &color_blend_attachment;

        std::vector<vk::DynamicState> dynamic_states = {
            vk::DynamicState::eViewport,
            vk::DynamicState::eScissor
        };
        vk::PipelineDynamicStateCreateInfo dynamic_state_info{};
        dynamic_state_info.dynamicStateCount = static_cast<uint32_t>(dynamic_states.size());
        dynamic_state_info.pDynamicStates = dynamic_states.data();

        vk::GraphicsPipelineCreateInfo graphics_pipeline_create_info{};
        graphics_pipeline_create_info.stageCount = static_cast<uint32_t>(shader_stage_infos.size());
        graphics_pipeline_create_info.pStages = shader_stage_infos.data();
        graphics_pipeline_create_info.pVertexInputState = &vertex_input_info;
        graphics_pipeline_create_info.pInputAssemblyState = &input_assembly_info;
        graphics_pipeline_create_info.pViewportState = &viewport_info;
        graphics_pipeline_create_info.pRasterizationState = &rasterization_info;
        graphics_pipeline_create_info.pMultisampleState = &multisample_info;
        graphics_pipeline_create_info.pDepthStencilState = &depth_info;
        graphics_pipeline_create_info.pColorBlendState = &color_blend_state;
        graphics_pipeline_create_info.pDynamicState = &dynamic_state_info;
        graphics_pipeline_create_info.layout = *m_pipeline_layout;
        graphics_pipeline_create_info.renderPass = VK_NULL_HANDLE;

        vk::PipelineRenderingCreateInfo pipeline_rendering_info{};
        vk::Format color_attachment_format = m_color_format;
        pipeline_rendering_info.colorAttachmentCount = 1;
        pipeline_rendering_info.pColorAttachmentFormats = &color_attachment_format;
        pipeline_rendering_info.depthAttachmentFormat = m_depth_format;

        vk::StructureChain<
            vk::GraphicsPipelineCreateInfo,
            vk::PipelineRenderingCreateInfo
        > pipeline_info_chain{
            graphics_pipeline_create_info,
            pipeline_rendering_info
        };

        m_present_pipeline = vk::raii::Pipeline{
            m_device->device(),
            nullptr,
            pipeline_info_chain.get<vk::GraphicsPipelineCreateInfo>()
        };
    }

    void create_offscreen_images() {
        m_offscreen_frame_resources.clear();
        m_offscreen_frame_resources.reserve(m_frame_count);

        const vk::ImageUsageFlags usage = vk::ImageUsageFlagBits::eStorage | vk::ImageUsageFlagBits::eSampled;
        for (uint32_t i = 0; i < m_frame_count; ++i) {
            OffscreenFrameResources resources{};
            resources.image = std::make_unique<rhi::Image>(
                rhi::Image(
                    m_device,
                    m_swapchain_extent.width,
                    m_swapchain_extent.height,
                    m_offscreen_format,
                    vk::ImageTiling::eOptimal,
                    usage,
                    vk::MemoryPropertyFlagBits::eDeviceLocal,
                    vk::ImageAspectFlagBits::eColor,
                    false
                )
            );
            resources.layout = vk::ImageLayout::eUndefined;
            m_offscreen_frame_resources.emplace_back(std::move(resources));
        }
    }

    void create_depth_images() {
        m_depth_images = make_per_frame_depth_images(m_swapchain_extent, m_depth_format);
    }

    void refresh_descriptor_sets() {
        if (m_offscreen_frame_resources.size() != m_frame_count) {
            throw std::runtime_error("Offscreen images are not ready for descriptor refresh.");
        }

        m_descriptor_system->update_set(
            "compute",
            [this](rhi::DescriptorWriter& writer, uint32_t index) {
                writer.write_buffer(0, *m_uniform_buffers[index]->buffer(), 0, m_uniform_buffer_size);
                writer.write_storage_image(
                    1,
                    *m_offscreen_frame_resources[index].image->image_view(),
                    vk::ImageLayout::eGeneral
                );
            }
        ).update_set(
            "present",
            [this](rhi::DescriptorWriter& writer, uint32_t index) {
                writer.write_combined_image(
                    0,
                    *m_offscreen_frame_resources[index].image->image_view(),
                    *m_offscreen_sampler->sampler(),
                    vk::ImageLayout::eShaderReadOnlyOptimal
                );
            }
        );
    }
};

} // namespace rtr::render
