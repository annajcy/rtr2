#pragma once

#include <array>
#include <stdexcept>

#include "rtr/rhi/descriptor.hpp"
#include "rtr/rhi/texture.hpp"
#include "rtr/system/render/frame_context.hpp"
#include "rtr/system/render/render_pass.hpp"
#include "rtr/system/render/render_resource_state.hpp"
#include "vulkan/vulkan.hpp"

namespace rtr::system::render {

struct PresentImagePassResources {
    TrackedImage offscreen;
    rhi::Image& depth_image;
    vk::raii::DescriptorSet& present_set;
};

class PresentImagePass final : public RenderPass<PresentImagePassResources> {
public:
    using RenderPassResources = PresentImagePassResources;

private:
    vk::raii::PipelineLayout& m_pipeline_layout;
    vk::raii::Pipeline& m_present_pipeline;

public:
    PresentImagePass(vk::raii::PipelineLayout& pipeline_layout, vk::raii::Pipeline& present_pipeline)
        : m_pipeline_layout(pipeline_layout), m_present_pipeline(present_pipeline) {}

protected:
    void validate(const RenderPassResources& resources) const override {
        require_valid_tracked_image(resources.offscreen, "PresentImagePass offscreen image is invalid.");
    }

    void do_execute(render::FrameContext& ctx, const RenderPassResources& resources) override {
        auto&       cmd       = ctx.cmd().command_buffer();
        rhi::Image& offscreen = resources.offscreen.image;
        rhi::Image& depth     = resources.depth_image;
        const vk::ImageLayout old_layout = resources.offscreen.layout;

        vk::PipelineStageFlags2 src_stage = vk::PipelineStageFlagBits2::eTopOfPipe;
        vk::AccessFlags2 src_access = vk::AccessFlagBits2::eNone;
        if (old_layout == vk::ImageLayout::eGeneral) {
            src_stage = vk::PipelineStageFlagBits2::eComputeShader;
            src_access = vk::AccessFlagBits2::eShaderStorageWrite;
        } else if (old_layout == vk::ImageLayout::eShaderReadOnlyOptimal) {
            src_stage = vk::PipelineStageFlagBits2::eFragmentShader;
            src_access = vk::AccessFlagBits2::eShaderRead;
        }

        vk::ImageMemoryBarrier2 to_sampled{};
        to_sampled.srcStageMask                    = src_stage;
        to_sampled.dstStageMask                    = vk::PipelineStageFlagBits2::eFragmentShader;
        to_sampled.srcAccessMask                   = src_access;
        to_sampled.dstAccessMask                   = vk::AccessFlagBits2::eShaderSampledRead;
        to_sampled.oldLayout                       = old_layout;
        to_sampled.newLayout                       = vk::ImageLayout::eShaderReadOnlyOptimal;
        to_sampled.image                           = *offscreen.image();
        to_sampled.subresourceRange.aspectMask     = vk::ImageAspectFlagBits::eColor;
        to_sampled.subresourceRange.baseMipLevel   = 0;
        to_sampled.subresourceRange.levelCount     = 1;
        to_sampled.subresourceRange.baseArrayLayer = 0;
        to_sampled.subresourceRange.layerCount     = 1;

        vk::DependencyInfo to_sampled_dep{};
        to_sampled_dep.imageMemoryBarrierCount = 1;
        to_sampled_dep.pImageMemoryBarriers    = &to_sampled;
        cmd.pipelineBarrier2(to_sampled_dep);
        resources.offscreen.layout = vk::ImageLayout::eShaderReadOnlyOptimal;

        vk::ImageMemoryBarrier2 to_color{};
        to_color.srcStageMask                    = vk::PipelineStageFlagBits2::eTopOfPipe;
        to_color.dstStageMask                    = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        to_color.srcAccessMask                   = vk::AccessFlagBits2::eNone;
        to_color.dstAccessMask                   = vk::AccessFlagBits2::eColorAttachmentWrite;
        to_color.oldLayout                       = vk::ImageLayout::eUndefined;
        to_color.newLayout                       = vk::ImageLayout::eColorAttachmentOptimal;
        to_color.image                           = ctx.swapchain_image();
        to_color.subresourceRange.aspectMask     = vk::ImageAspectFlagBits::eColor;
        to_color.subresourceRange.baseMipLevel   = 0;
        to_color.subresourceRange.levelCount     = 1;
        to_color.subresourceRange.baseArrayLayer = 0;
        to_color.subresourceRange.layerCount     = 1;

        vk::ImageMemoryBarrier2 to_depth{};
        to_depth.srcStageMask = vk::PipelineStageFlagBits2::eTopOfPipe;
        to_depth.dstStageMask =
            vk::PipelineStageFlagBits2::eEarlyFragmentTests | vk::PipelineStageFlagBits2::eLateFragmentTests;
        to_depth.srcAccessMask = vk::AccessFlagBits2::eNone;
        to_depth.dstAccessMask =
            vk::AccessFlagBits2::eDepthStencilAttachmentRead | vk::AccessFlagBits2::eDepthStencilAttachmentWrite;
        to_depth.oldLayout                       = vk::ImageLayout::eUndefined;
        to_depth.newLayout                       = vk::ImageLayout::eDepthAttachmentOptimal;
        to_depth.image                           = *depth.image();
        to_depth.subresourceRange.aspectMask     = vk::ImageAspectFlagBits::eDepth;
        to_depth.subresourceRange.baseMipLevel   = 0;
        to_depth.subresourceRange.levelCount     = 1;
        to_depth.subresourceRange.baseArrayLayer = 0;
        to_depth.subresourceRange.layerCount     = 1;

        std::array<vk::ImageMemoryBarrier2, 2> barriers = {to_color, to_depth};
        vk::DependencyInfo                     to_render_dep{};
        to_render_dep.imageMemoryBarrierCount = static_cast<uint32_t>(barriers.size());
        to_render_dep.pImageMemoryBarriers    = barriers.data();
        cmd.pipelineBarrier2(to_render_dep);

        vk::ClearValue              clear_value = vk::ClearValue{vk::ClearColorValue{0.0f, 0.0f, 0.0f, 1.0f}};
        vk::RenderingAttachmentInfo color_attachment_info{};
        color_attachment_info.imageView   = *ctx.swapchain_image_view();
        color_attachment_info.imageLayout = vk::ImageLayout::eColorAttachmentOptimal;
        color_attachment_info.loadOp      = vk::AttachmentLoadOp::eClear;
        color_attachment_info.storeOp     = vk::AttachmentStoreOp::eStore;
        color_attachment_info.clearValue  = clear_value;

        vk::ClearValue              depth_clear{vk::ClearDepthStencilValue{1.0f, 0}};
        vk::RenderingAttachmentInfo depth_attachment_info{};
        depth_attachment_info.imageView   = *depth.image_view();
        depth_attachment_info.imageLayout = vk::ImageLayout::eDepthAttachmentOptimal;
        depth_attachment_info.loadOp      = vk::AttachmentLoadOp::eClear;
        depth_attachment_info.storeOp     = vk::AttachmentStoreOp::eStore;
        depth_attachment_info.clearValue  = depth_clear;

        vk::RenderingInfo rendering_info{};
        rendering_info.renderArea.offset    = vk::Offset2D{0, 0};
        rendering_info.renderArea.extent    = ctx.render_extent();
        rendering_info.layerCount           = 1;
        rendering_info.colorAttachmentCount = 1;
        rendering_info.pColorAttachments    = &color_attachment_info;
        rendering_info.pDepthAttachment     = &depth_attachment_info;

        cmd.beginRendering(rendering_info);
        cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *m_present_pipeline);
        cmd.bindDescriptorSets(vk::PipelineBindPoint::eGraphics, *m_pipeline_layout, 1,
                               *resources.present_set, {});

        vk::Viewport viewport{};
        viewport.x        = 0.0f;
        viewport.y        = 0.0f;
        viewport.width    = static_cast<float>(ctx.render_extent().width);
        viewport.height   = static_cast<float>(ctx.render_extent().height);
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

}  // namespace rtr::system::render
