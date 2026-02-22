#pragma once

#include <memory>
#include <vector>

#include "rtr/editor/core/editor_capture.hpp"
#include "rtr/editor/core/editor_host.hpp"
#include "rtr/editor/render/editor_imgui_pass.hpp"
#include "rtr/system/render/frame_color_source.hpp"
#include "rtr/system/render/pipeline.hpp"
#include "rtr/system/render/pipeline/forward/forward_pipeline.hpp"
#include "rtr/system/render/render_pass.hpp"

namespace rtr::system::render {

class ForwardEditorPipeline final : public render::ForwardPipeline, public editor::IEditorInputCaptureSource {
private:
    std::unique_ptr<editor::render::EditorImGuiPass> m_editor_pass{};

public:
    ForwardEditorPipeline(const render::PipelineRuntime& runtime, std::shared_ptr<editor::EditorHost> editor_host)
        : render::ForwardPipeline(runtime) {
        m_editor_pass = std::make_unique<editor::render::EditorImGuiPass>(runtime, std::move(editor_host), this);
    }

    ~ForwardEditorPipeline() override = default;

    void handle_swapchain_state_change(const FrameScheduler::SwapchainState& state,
                                       const render::SwapchainChangeSummary& diff) override {
        render::ForwardPipeline::handle_swapchain_state_change(state, diff);
        m_editor_pass->on_swapchain_recreated(state.image_count, state.color_format, state.depth_format);
    }

    bool wants_imgui_capture_mouse() const override { return m_editor_pass->wants_capture_mouse(); }

    bool wants_imgui_capture_keyboard() const override { return m_editor_pass->wants_capture_keyboard(); }

    void render(render::FrameContext& ctx) override {
        // Run forward pipeline to render the 3D scene directly to the offscreen target.
        if (!render_forward_pass(ctx)) {
            return;
        }

        const uint32_t frame_index = ctx.frame_index();
        auto&          cmd         = ctx.cmd().command_buffer();

        vk::ImageMemoryBarrier2 swapchain_to_color{};
        swapchain_to_color.srcStageMask                    = vk::PipelineStageFlagBits2::eTopOfPipe;
        swapchain_to_color.dstStageMask                    = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        swapchain_to_color.srcAccessMask                   = vk::AccessFlagBits2::eNone;
        swapchain_to_color.dstAccessMask                   = vk::AccessFlagBits2::eColorAttachmentWrite;
        swapchain_to_color.oldLayout                       = vk::ImageLayout::eUndefined;
        swapchain_to_color.newLayout                       = vk::ImageLayout::eColorAttachmentOptimal;
        swapchain_to_color.image                           = ctx.swapchain_image();
        swapchain_to_color.subresourceRange.aspectMask     = vk::ImageAspectFlagBits::eColor;
        swapchain_to_color.subresourceRange.baseMipLevel   = 0;
        swapchain_to_color.subresourceRange.levelCount     = 1;
        swapchain_to_color.subresourceRange.baseArrayLayer = 0;
        swapchain_to_color.subresourceRange.layerCount     = 1;

        vk::ImageMemoryBarrier2 offscreen_to_sampled{};
        offscreen_to_sampled.srcStageMask                    = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        offscreen_to_sampled.dstStageMask                    = vk::PipelineStageFlagBits2::eFragmentShader;
        offscreen_to_sampled.srcAccessMask                   = vk::AccessFlagBits2::eColorAttachmentWrite;
        offscreen_to_sampled.dstAccessMask                   = vk::AccessFlagBits2::eShaderRead;
        offscreen_to_sampled.oldLayout                       = m_color_image_layouts[frame_index];
        offscreen_to_sampled.newLayout                       = vk::ImageLayout::eShaderReadOnlyOptimal;
        offscreen_to_sampled.image                           = *m_color_images[frame_index]->image();
        offscreen_to_sampled.subresourceRange.aspectMask     = vk::ImageAspectFlagBits::eColor;
        offscreen_to_sampled.subresourceRange.baseMipLevel   = 0;
        offscreen_to_sampled.subresourceRange.levelCount     = 1;
        offscreen_to_sampled.subresourceRange.baseArrayLayer = 0;
        offscreen_to_sampled.subresourceRange.layerCount     = 1;

        std::array<vk::ImageMemoryBarrier2, 2> barriers = {swapchain_to_color, offscreen_to_sampled};
        vk::DependencyInfo                     dep{};
        dep.imageMemoryBarrierCount = static_cast<uint32_t>(barriers.size());
        dep.pImageMemoryBarriers    = barriers.data();
        cmd.pipelineBarrier2(dep);

        m_color_image_layouts[frame_index] = vk::ImageLayout::eShaderReadOnlyOptimal;

        // Feed EditorImGuiPass the offscreen scene image from the ForwardPipeline.
        auto source_view = frame_color_source_view(frame_index);
        m_editor_pass->bind_render_pass_resources(
            editor::render::EditorImGuiPass::RenderPassResources{.scene_image_view   = source_view.image_view,
                                                                 .scene_image_layout = source_view.image_layout,
                                                                 .scene_extent       = source_view.extent});

        // Run Editor Pass which renders ImGui UI on top of Swapchain.
        m_editor_pass->execute(ctx);
    }
};

}  // namespace rtr::system::render
