#pragma once

#include <functional>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <vector>

#include "imgui.h"

#include "render/frame_context.hpp"
#include "render/render_pass.hpp"
#include "rhi/imgui_context.hpp"
#include "rhi/texture.hpp"

namespace rtr::render {

class ImGUIPass final : public IRenderPass {
public:
    using UiCallback = std::function<void()>;

private:
    std::unique_ptr<rhi::ImGuiContext> m_imgui_context{};
    std::vector<std::unique_ptr<rhi::Image>>* m_depth_images{};
    UiCallback m_ui_callback{};
    ImGuiDockNodeFlags m_dockspace_flags{ImGuiDockNodeFlags_PassthruCentralNode};
    std::vector<ResourceDependency> m_dependencies{
        {"swapchain_color", ResourceAccess::eReadWrite},
        {"depth", ResourceAccess::eRead}
    };

public:
    ImGUIPass(
        rhi::Device* device,
        rhi::Context* context,
        rhi::Window* window,
        uint32_t image_count,
        vk::Format color_format,
        vk::Format depth_format,
        std::vector<std::unique_ptr<rhi::Image>>* depth_images
    )
        : m_imgui_context(std::make_unique<rhi::ImGuiContext>(
              device,
              context,
              window,
              image_count,
              color_format,
              depth_format
          )),
          m_depth_images(depth_images) {
        if (m_depth_images == nullptr) {
            throw std::runtime_error("ImGUIPass requires depth image array.");
        }
    }

    std::string_view name() const override { return "imgui.overlay"; }

    const std::vector<ResourceDependency>& dependencies() const override {
        return m_dependencies;
    }

    void set_ui_callback(UiCallback cb) {
        m_ui_callback = std::move(cb);
    }

    void clear_ui_callback() {
        m_ui_callback = UiCallback{};
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

    void execute(render::FrameContext& ctx) override {
        if (ctx.frame_index() >= m_depth_images->size()) {
            throw std::runtime_error("ImGUIPass frame depth image is not ready.");
        }

        m_imgui_context->begin_frame();
        ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport(), m_dockspace_flags);
        if (m_ui_callback) {
            m_ui_callback();
        }

        ImDrawData* draw_data = m_imgui_context->prepare_draw_data();
        if (draw_data == nullptr) {
            return;
        }

        const rhi::Image& depth_image = *m_depth_images->at(ctx.frame_index());

        vk::RenderingAttachmentInfo color_attachment_info{};
        color_attachment_info.imageView = *ctx.swapchain_image_view();
        color_attachment_info.imageLayout = vk::ImageLayout::eColorAttachmentOptimal;
        color_attachment_info.loadOp = vk::AttachmentLoadOp::eLoad;
        color_attachment_info.storeOp = vk::AttachmentStoreOp::eStore;

        vk::RenderingAttachmentInfo depth_attachment_info{};
        depth_attachment_info.imageView = *depth_image.image_view();
        depth_attachment_info.imageLayout = vk::ImageLayout::eDepthAttachmentOptimal;
        depth_attachment_info.loadOp = vk::AttachmentLoadOp::eLoad;
        depth_attachment_info.storeOp = vk::AttachmentStoreOp::eStore;

        vk::RenderingInfo rendering_info{};
        rendering_info.renderArea.offset = vk::Offset2D{0, 0};
        rendering_info.renderArea.extent = ctx.render_extent();
        rendering_info.layerCount = 1;
        rendering_info.colorAttachmentCount = 1;
        rendering_info.pColorAttachments = &color_attachment_info;
        rendering_info.pDepthAttachment = &depth_attachment_info;

        auto& command_buffer = ctx.cmd().command_buffer();
        command_buffer.beginRendering(rendering_info);
        m_imgui_context->render_draw_data(command_buffer, draw_data);
        command_buffer.endRendering();
    }
};

} // namespace rtr::render
