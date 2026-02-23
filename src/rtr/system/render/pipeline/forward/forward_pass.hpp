#pragma once

#include <array>
#include <string_view>
#include <vector>
#include <stdexcept>

#include "rtr/rhi/texture.hpp"
#include "rtr/rhi/mesh.hpp"
#include "rtr/rhi/descriptor.hpp"
#include "rtr/system/render/frame_context.hpp"
#include "rtr/system/render/render_pass.hpp"
#include "vulkan/vulkan.hpp"
#include "rtr/system/render/pipeline.hpp"

namespace rtr::system::render {

class ForwardPass final : public render::IRenderPass {
public:
    struct DrawItem {
        rhi::Mesh&               mesh;
        vk::raii::DescriptorSet& per_object_set;
    };

    struct RenderPassResources {
        rhi::Image&           color_image;
        vk::ImageLayout&      color_layout;
        rhi::Image&           depth_image;
        vk::Extent2D          extent;
        std::vector<DrawItem> draw_items{};
    };

private:
    vk::raii::PipelineLayout* m_pipeline_layout{};
    vk::raii::Pipeline*       m_pipeline{};

    std::vector<render::ResourceDependency> m_dependencies{{"forward.per_object", render::ResourceAccess::eRead},
                                                           {"offscreen_color", render::ResourceAccess::eReadWrite},
                                                           {"depth", render::ResourceAccess::eReadWrite}};

public:
    ForwardPass(vk::raii::PipelineLayout* pipeline_layout, vk::raii::Pipeline* pipeline)
        : m_pipeline_layout(pipeline_layout), m_pipeline(pipeline) {}

    std::string_view name() const override { return "forward_main"; }

    const std::vector<render::ResourceDependency>& dependencies() const override { return m_dependencies; }

    static void validate_resources(const RenderPassResources& resources) {
        if (resources.extent.width == 0 || resources.extent.height == 0) {
            throw std::runtime_error("ForwardPass frame resources are incomplete.");
        }
    }

    void execute(render::FrameContext& ctx, const RenderPassResources& resources) {
        validate_resources(resources);

        auto&       cmd         = ctx.cmd().command_buffer();
        rhi::Image& color_image = resources.color_image;
        rhi::Image& depth_image = resources.depth_image;

        vk::ImageMemoryBarrier2 to_color{};
        to_color.srcStageMask                  = (resources.color_layout == vk::ImageLayout::eUndefined)
                                                     ? vk::PipelineStageFlagBits2::eTopOfPipe
                                                     : vk::PipelineStageFlagBits2::eAllCommands;
        to_color.dstStageMask                  = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        to_color.srcAccessMask                 = (resources.color_layout == vk::ImageLayout::eUndefined)
                                                     ? vk::AccessFlagBits2::eNone
                                                     : vk::AccessFlagBits2::eMemoryRead | vk::AccessFlagBits2::eMemoryWrite;
        to_color.dstAccessMask                 = vk::AccessFlagBits2::eColorAttachmentWrite;
        to_color.oldLayout                     = resources.color_layout;
        to_color.newLayout                     = vk::ImageLayout::eColorAttachmentOptimal;
        to_color.image                         = *color_image.image();
        to_color.subresourceRange.aspectMask   = vk::ImageAspectFlagBits::eColor;
        to_color.subresourceRange.baseMipLevel = 0;
        to_color.subresourceRange.levelCount   = 1;
        to_color.subresourceRange.baseArrayLayer = 0;
        to_color.subresourceRange.layerCount     = 1;

        vk::ImageMemoryBarrier2 to_depth{};
        to_depth.srcStageMask = vk::PipelineStageFlagBits2::eTopOfPipe;
        to_depth.dstStageMask =
            vk::PipelineStageFlagBits2::eEarlyFragmentTests | vk::PipelineStageFlagBits2::eLateFragmentTests;
        to_depth.srcAccessMask                   = vk::AccessFlagBits2::eNone;
        to_depth.dstAccessMask                   = vk::AccessFlagBits2::eDepthStencilAttachmentWrite;
        to_depth.oldLayout                       = vk::ImageLayout::eUndefined;
        to_depth.newLayout                       = vk::ImageLayout::eDepthAttachmentOptimal;
        to_depth.image                           = *depth_image.image();
        to_depth.subresourceRange.aspectMask     = vk::ImageAspectFlagBits::eDepth;
        to_depth.subresourceRange.baseMipLevel   = 0;
        to_depth.subresourceRange.levelCount     = 1;
        to_depth.subresourceRange.baseArrayLayer = 0;
        to_depth.subresourceRange.layerCount     = 1;

        std::array<vk::ImageMemoryBarrier2, 2> to_render_barriers = {to_color, to_depth};
        vk::DependencyInfo                     to_render_dep{};
        to_render_dep.imageMemoryBarrierCount = static_cast<uint32_t>(to_render_barriers.size());
        to_render_dep.pImageMemoryBarriers    = to_render_barriers.data();
        cmd.pipelineBarrier2(to_render_dep);

        vk::ClearValue              clear_value = vk::ClearValue{vk::ClearColorValue{0.0f, 0.0f, 0.0f, 1.0f}};
        vk::RenderingAttachmentInfo color_attachment_info{};
        color_attachment_info.imageView   = *color_image.image_view();
        color_attachment_info.imageLayout = vk::ImageLayout::eColorAttachmentOptimal;
        color_attachment_info.loadOp      = vk::AttachmentLoadOp::eClear;
        color_attachment_info.storeOp     = vk::AttachmentStoreOp::eStore;
        color_attachment_info.clearValue  = clear_value;

        vk::ClearValue              depth_clear{vk::ClearDepthStencilValue{1.0f, 0}};
        vk::RenderingAttachmentInfo depth_attachment_info{};
        depth_attachment_info.imageView   = *depth_image.image_view();
        depth_attachment_info.imageLayout = vk::ImageLayout::eDepthAttachmentOptimal;
        depth_attachment_info.loadOp      = vk::AttachmentLoadOp::eClear;
        depth_attachment_info.storeOp     = vk::AttachmentStoreOp::eStore;
        depth_attachment_info.clearValue  = depth_clear;

        vk::RenderingInfo rendering_info{};
        rendering_info.renderArea.offset    = vk::Offset2D{0, 0};
        rendering_info.renderArea.extent    = resources.extent;
        rendering_info.layerCount           = 1;
        rendering_info.colorAttachmentCount = 1;
        rendering_info.pColorAttachments    = &color_attachment_info;
        rendering_info.pDepthAttachment     = &depth_attachment_info;

        cmd.beginRendering(rendering_info);
        cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *m_pipeline);

        vk::Viewport viewport{};
        viewport.x        = 0.0f;
        viewport.y        = 0.0f;
        viewport.width    = static_cast<float>(resources.extent.width);
        viewport.height   = static_cast<float>(resources.extent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        cmd.setViewport(0, viewport);

        vk::Rect2D scissor{};
        scissor.offset = vk::Offset2D{0, 0};
        scissor.extent = resources.extent;
        cmd.setScissor(0, scissor);

        for (const auto& item : resources.draw_items) {
            std::vector<vk::Buffer>     vertex_buffers = {item.mesh.vertex_buffer()};
            std::vector<vk::DeviceSize> offsets        = {0};
            cmd.bindVertexBuffers(0, vertex_buffers, offsets);
            cmd.bindIndexBuffer(item.mesh.index_buffer(), 0, vk::IndexType::eUint32);

            cmd.bindDescriptorSets(vk::PipelineBindPoint::eGraphics, **m_pipeline_layout, 0, *item.per_object_set, {});

            cmd.drawIndexed(item.mesh.index_count(), 1, 0, 0, 0);
        }

        cmd.endRendering();
        resources.color_layout = vk::ImageLayout::eColorAttachmentOptimal;
    }
};

}  // namespace rtr::system::render
