#pragma once


#include <string_view>
#include <vector>

#include "render/frame_context.hpp"
#include "rhi/imgui_context.hpp"
#include "rhi/texture.hpp"

namespace rtr::render {

enum class ResourceAccess {
    eRead,
    eWrite,
    eReadWrite
};

struct ResourceDependency {
    std::string resource_name;
    ResourceAccess access{ResourceAccess::eRead};
};

class IRenderPass {
public:
    virtual ~IRenderPass() = default;
    virtual std::string_view name() const = 0;
    virtual const std::vector<ResourceDependency>& dependencies() const = 0;
    virtual void execute(render::FrameContext& ctx) = 0;
};

class ImGUIPass final : public IRenderPass {
public:
    using UiCallback = std::function<void()>;
    struct RenderPassResources {
        rhi::Image* depth_image{};
    };

private:
    std::unique_ptr<rhi::ImGuiContext> m_imgui_context{};
    RenderPassResources m_render_pass_resources{};
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
        vk::Format depth_format
    )
        : m_imgui_context(std::make_unique<rhi::ImGuiContext>(
              device,
              context,
              window,
              image_count,
              color_format,
              depth_format
          )) {}

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

    void bind_render_pass_resources(const RenderPassResources& resources) {
        if (resources.depth_image == nullptr) {
            throw std::runtime_error("ImGUIPass frame resources are incomplete.");
        }
        m_render_pass_resources = resources;
    }

    void execute(render::FrameContext& ctx) override {
        if (m_render_pass_resources.depth_image == nullptr) {
            throw std::runtime_error("ImGUIPass frame resources are not bound.");
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

        const rhi::Image& depth_image = *m_render_pass_resources.depth_image;

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
