#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>

#include "rtr/rhi/texture.hpp"
#include "rtr/system/render/frame_context.hpp"
#include "rtr/system/render/render_pass.hpp"
#include "rtr/system/render/render_resource_state.hpp"
#include "vulkan/vulkan.hpp"

namespace rtr::system::render {

struct PresentPassResources {
    TrackedImage src_color;
    vk::Extent2D src_extent;
};

class PresentPass final : public RenderPass<PresentPassResources> {
public:
    using RenderPassResources = PresentPassResources;

public:
    PresentPass() = default;

protected:
    void validate(const RenderPassResources& resources) const override {
        require_valid_extent(resources.src_extent, "PresentPass frame resources are incomplete.");
        require_valid_tracked_image(resources.src_color, "PresentPass source color is invalid.");
    }

    void do_execute(render::FrameContext& ctx, const RenderPassResources& resources) override {
        auto& cmd          = ctx.cmd().command_buffer();
        auto& color_image  = resources.src_color.image;
        auto& color_layout = resources.src_color.layout;

        vk::ImageMemoryBarrier2 offscreen_to_src{};
        offscreen_to_src.srcStageMask                    = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        offscreen_to_src.dstStageMask                    = vk::PipelineStageFlagBits2::eTransfer;
        offscreen_to_src.srcAccessMask                   = vk::AccessFlagBits2::eColorAttachmentWrite;
        offscreen_to_src.dstAccessMask                   = vk::AccessFlagBits2::eTransferRead;
        offscreen_to_src.oldLayout                       = color_layout;
        offscreen_to_src.newLayout                       = vk::ImageLayout::eTransferSrcOptimal;
        offscreen_to_src.image                           = *color_image.image();
        offscreen_to_src.subresourceRange.aspectMask     = vk::ImageAspectFlagBits::eColor;
        offscreen_to_src.subresourceRange.baseMipLevel   = 0;
        offscreen_to_src.subresourceRange.levelCount     = 1;
        offscreen_to_src.subresourceRange.baseArrayLayer = 0;
        offscreen_to_src.subresourceRange.layerCount     = 1;

        vk::ImageMemoryBarrier2 swapchain_to_dst{};
        swapchain_to_dst.srcStageMask                    = vk::PipelineStageFlagBits2::eTopOfPipe;
        swapchain_to_dst.dstStageMask                    = vk::PipelineStageFlagBits2::eTransfer;
        swapchain_to_dst.srcAccessMask                   = vk::AccessFlagBits2::eNone;
        swapchain_to_dst.dstAccessMask                   = vk::AccessFlagBits2::eTransferWrite;
        swapchain_to_dst.oldLayout                       = vk::ImageLayout::eUndefined;
        swapchain_to_dst.newLayout                       = vk::ImageLayout::eTransferDstOptimal;
        swapchain_to_dst.image                           = ctx.swapchain_image();
        swapchain_to_dst.subresourceRange.aspectMask     = vk::ImageAspectFlagBits::eColor;
        swapchain_to_dst.subresourceRange.baseMipLevel   = 0;
        swapchain_to_dst.subresourceRange.levelCount     = 1;
        swapchain_to_dst.subresourceRange.baseArrayLayer = 0;
        swapchain_to_dst.subresourceRange.layerCount     = 1;

        std::array<vk::ImageMemoryBarrier2, 2> to_blit_barriers = {offscreen_to_src, swapchain_to_dst};
        vk::DependencyInfo                     to_blit_dep{};
        to_blit_dep.imageMemoryBarrierCount = static_cast<uint32_t>(to_blit_barriers.size());
        to_blit_dep.pImageMemoryBarriers    = to_blit_barriers.data();
        cmd.pipelineBarrier2(to_blit_dep);

        vk::ImageBlit2 blit{};
        blit.srcSubresource.aspectMask     = vk::ImageAspectFlagBits::eColor;
        blit.srcSubresource.mipLevel       = 0;
        blit.srcSubresource.baseArrayLayer = 0;
        blit.srcSubresource.layerCount     = 1;
        blit.srcOffsets[0]                 = vk::Offset3D{0, 0, 0};
        blit.srcOffsets[1]                 = vk::Offset3D{static_cast<int32_t>(resources.src_extent.width),
                                          static_cast<int32_t>(resources.src_extent.height), 1};
        blit.dstSubresource.aspectMask     = vk::ImageAspectFlagBits::eColor;
        blit.dstSubresource.mipLevel       = 0;
        blit.dstSubresource.baseArrayLayer = 0;
        blit.dstSubresource.layerCount     = 1;
        blit.dstOffsets[0]                 = vk::Offset3D{0, 0, 0};
        blit.dstOffsets[1]                 = vk::Offset3D{static_cast<int32_t>(ctx.render_extent().width),
                                          static_cast<int32_t>(ctx.render_extent().height), 1};

        vk::BlitImageInfo2 blit_info{};
        blit_info.srcImage       = *color_image.image();
        blit_info.srcImageLayout = vk::ImageLayout::eTransferSrcOptimal;
        blit_info.dstImage       = ctx.swapchain_image();
        blit_info.dstImageLayout = vk::ImageLayout::eTransferDstOptimal;
        blit_info.filter         = vk::Filter::eLinear;
        blit_info.regionCount    = 1;
        blit_info.pRegions       = &blit;
        cmd.blitImage2(blit_info);

        vk::ImageMemoryBarrier2 swapchain_to_color{};
        swapchain_to_color.srcStageMask                    = vk::PipelineStageFlagBits2::eTransfer;
        swapchain_to_color.dstStageMask                    = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        swapchain_to_color.srcAccessMask                   = vk::AccessFlagBits2::eTransferWrite;
        swapchain_to_color.dstAccessMask                   = vk::AccessFlagBits2::eColorAttachmentWrite;
        swapchain_to_color.oldLayout                       = vk::ImageLayout::eTransferDstOptimal;
        swapchain_to_color.newLayout                       = vk::ImageLayout::eColorAttachmentOptimal;
        swapchain_to_color.image                           = ctx.swapchain_image();
        swapchain_to_color.subresourceRange.aspectMask     = vk::ImageAspectFlagBits::eColor;
        swapchain_to_color.subresourceRange.baseMipLevel   = 0;
        swapchain_to_color.subresourceRange.levelCount     = 1;
        swapchain_to_color.subresourceRange.baseArrayLayer = 0;
        swapchain_to_color.subresourceRange.layerCount     = 1;

        vk::ImageMemoryBarrier2 offscreen_to_sampled{};
        offscreen_to_sampled.srcStageMask                    = vk::PipelineStageFlagBits2::eTransfer;
        offscreen_to_sampled.dstStageMask                    = vk::PipelineStageFlagBits2::eFragmentShader;
        offscreen_to_sampled.srcAccessMask                   = vk::AccessFlagBits2::eTransferRead;
        offscreen_to_sampled.dstAccessMask                   = vk::AccessFlagBits2::eShaderRead;
        offscreen_to_sampled.oldLayout                       = vk::ImageLayout::eTransferSrcOptimal;
        offscreen_to_sampled.newLayout                       = vk::ImageLayout::eShaderReadOnlyOptimal;
        offscreen_to_sampled.image                           = *color_image.image();
        offscreen_to_sampled.subresourceRange.aspectMask     = vk::ImageAspectFlagBits::eColor;
        offscreen_to_sampled.subresourceRange.baseMipLevel   = 0;
        offscreen_to_sampled.subresourceRange.levelCount     = 1;
        offscreen_to_sampled.subresourceRange.baseArrayLayer = 0;
        offscreen_to_sampled.subresourceRange.layerCount     = 1;

        std::array<vk::ImageMemoryBarrier2, 2> to_final_barriers = {swapchain_to_color, offscreen_to_sampled};
        vk::DependencyInfo                     to_final_dep{};
        to_final_dep.imageMemoryBarrierCount = static_cast<uint32_t>(to_final_barriers.size());
        to_final_dep.pImageMemoryBarriers    = to_final_barriers.data();
        cmd.pipelineBarrier2(to_final_dep);

        resources.src_color.layout = vk::ImageLayout::eShaderReadOnlyOptimal;
    }
};

}  // namespace rtr::system::render
