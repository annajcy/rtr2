#pragma once

#include <memory>
#include <stdexcept>

#include "imgui.h"

#include "rtr/rhi/imgui_context.hpp"
#include "rtr/system/render/frame_context.hpp"

namespace rtr::editor::render {

class IImGuiOverlay {
public:
    virtual ~IImGuiOverlay() = default;
    virtual void draw_imgui() = 0;
};

class ImGuiOverlayPass final {
private:
    std::unique_ptr<rtr::rhi::ImGuiContext> m_imgui_context{};
    std::shared_ptr<IImGuiOverlay> m_overlay{};

public:
    ImGuiOverlayPass(
        rtr::rhi::Device* device,
        rtr::rhi::Context* context,
        rtr::rhi::Window* window,
        uint32_t image_count,
        vk::Format color_format,
        vk::Format depth_format
    )
        : m_imgui_context(std::make_unique<rtr::rhi::ImGuiContext>(
              device,
              context,
              window,
              image_count,
              color_format,
              depth_format
          )) {}

    void set_overlay(std::shared_ptr<IImGuiOverlay> overlay) {
        m_overlay = std::move(overlay);
    }

    void clear_overlay() {
        m_overlay.reset();
    }

    bool wants_capture_mouse() const {
        return m_imgui_context->wants_capture_mouse();
    }

    bool wants_capture_keyboard() const {
        return m_imgui_context->wants_capture_keyboard();
    }

    void on_swapchain_recreated(uint32_t image_count, vk::Format color_format, vk::Format depth_format) {
        m_imgui_context->on_swapchain_recreated(image_count, color_format, depth_format);
    }

    void render(rtr::system::render::FrameContext& ctx) {
        m_imgui_context->begin_frame();
        if (m_overlay) {
            m_overlay->draw_imgui();
        }

        ImDrawData* draw_data = m_imgui_context->prepare_draw_data();
        if (draw_data == nullptr) {
            return;
        }

        vk::RenderingAttachmentInfo color_attachment_info{};
        color_attachment_info.imageView = *ctx.swapchain_image_view();
        color_attachment_info.imageLayout = vk::ImageLayout::eColorAttachmentOptimal;
        color_attachment_info.loadOp = vk::AttachmentLoadOp::eLoad;
        color_attachment_info.storeOp = vk::AttachmentStoreOp::eStore;

        vk::RenderingInfo rendering_info{};
        rendering_info.renderArea.offset = vk::Offset2D{0, 0};
        rendering_info.renderArea.extent = ctx.render_extent();
        rendering_info.layerCount = 1;
        rendering_info.colorAttachmentCount = 1;
        rendering_info.pColorAttachments = &color_attachment_info;
        rendering_info.pDepthAttachment = nullptr;

        auto& command_buffer = ctx.cmd().command_buffer();
        command_buffer.beginRendering(rendering_info);
        m_imgui_context->render_draw_data(command_buffer, draw_data);
        command_buffer.endRendering();
    }
};

} // namespace rtr::editor::render
