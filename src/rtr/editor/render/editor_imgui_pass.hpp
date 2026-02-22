#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include "imgui.h"
#include "imgui_impl_vulkan.h"

#include "rtr/editor/core/editor_host.hpp"
#include "rtr/rhi/imgui_context.hpp"
#include "rtr/rhi/texture.hpp"
#include "rtr/system/render/frame_color_source.hpp"
#include "rtr/system/render/frame_context.hpp"
#include "rtr/system/render/render_pass.hpp"
#include "rtr/system/render/pipeline.hpp"
#include "vulkan/vulkan.hpp"

namespace rtr::editor::render {

class EditorImGuiPass final : public rtr::system::render::IRenderPass {
public:
    struct RenderPassResources {
        vk::ImageView   scene_image_view{};
        vk::ImageLayout scene_image_layout{vk::ImageLayout::eUndefined};
        vk::Extent2D    scene_extent{};
    };

private:
    class EditorHostOverlayAdapter final {
    private:
        std::shared_ptr<EditorHost> m_host{};

    public:
        explicit EditorHostOverlayAdapter(std::shared_ptr<EditorHost> host) : m_host(std::move(host)) {}

        void draw_imgui() {
            if (m_host) {
                m_host->draw_imgui();
            }
        }
    };

    struct SceneTextureEntry {
        vk::ImageView   image_view{};
        vk::ImageLayout image_layout{vk::ImageLayout::eUndefined};
        ImTextureID     texture_id{};
        ImVec2          texture_size{0.0f, 0.0f};
    };

    std::unique_ptr<rtr::rhi::ImGuiContext>   m_imgui_context{};
    std::shared_ptr<EditorHost>               m_editor_host{};
    std::shared_ptr<EditorHostOverlayAdapter> m_overlay_adapter{};

    std::unique_ptr<rtr::rhi::Sampler> m_scene_sampler{};
    std::vector<SceneTextureEntry>     m_scene_texture_entries{};
    ImTextureID                        m_current_scene_texture{};
    ImVec2                             m_current_scene_texture_size{0.0f, 0.0f};

    bool m_scene_hovered{false};
    bool m_scene_focused{false};

    std::vector<system::render::ResourceDependency> m_dependencies{
        {"offscreen_color", system::render::ResourceAccess::eRead},
        {"swapchain", system::render::ResourceAccess::eReadWrite}};

    RenderPassResources              m_resources{};
    system::render::IRenderPipeline* m_owner_pipeline{};

    static ImTextureID descriptor_set_to_texture_id(VkDescriptorSet descriptor_set) {
        if constexpr (std::is_pointer_v<ImTextureID>) {
            return reinterpret_cast<ImTextureID>(descriptor_set);
        } else {
            return static_cast<ImTextureID>(reinterpret_cast<std::uintptr_t>(descriptor_set));
        }
    }

    static VkDescriptorSet texture_id_to_descriptor_set(ImTextureID texture_id) {
        if constexpr (std::is_pointer_v<ImTextureID>) {
            return reinterpret_cast<VkDescriptorSet>(texture_id);
        } else {
            return reinterpret_cast<VkDescriptorSet>(static_cast<std::uintptr_t>(texture_id));
        }
    }

public:
    EditorImGuiPass(const system::render::PipelineRuntime& runtime, std::shared_ptr<EditorHost> editor_host,
                    system::render::IRenderPipeline* owner_pipeline)
        : m_imgui_context(std::make_unique<rtr::rhi::ImGuiContext>(runtime.device, runtime.context, runtime.window,
                                                                   runtime.image_count, runtime.color_format,
                                                                   runtime.depth_format)),
          m_editor_host(std::move(editor_host)),
          m_owner_pipeline(owner_pipeline) {
        if (!m_editor_host) {
            throw std::invalid_argument("EditorImGuiPass requires non-null editor host.");
        }

        m_overlay_adapter = std::make_shared<EditorHostOverlayAdapter>(m_editor_host);

        m_scene_sampler = std::make_unique<rtr::rhi::Sampler>(rtr::rhi::Sampler::create_default(runtime.device, 1));
        m_scene_texture_entries.resize(runtime.frame_count);
        bind_editor_services();
    }

    ~EditorImGuiPass() override {
        clear_editor_services();
        release_scene_textures();
    }

    std::string_view name() const override { return "editor_imgui_pass"; }

    const std::vector<system::render::ResourceDependency>& dependencies() const override { return m_dependencies; }

    void on_swapchain_recreated(uint32_t image_count, vk::Format color_format, vk::Format depth_format) {
        m_imgui_context->on_swapchain_recreated(image_count, color_format, depth_format);
        release_scene_textures();
    }

    bool wants_capture_mouse() const {
        if (m_scene_hovered) {
            return false;
        }
        return m_imgui_context->wants_capture_mouse();
    }

    bool wants_capture_keyboard() const {
        if (m_scene_focused) {
            return false;
        }
        return m_imgui_context->wants_capture_keyboard();
    }

    void bind_render_pass_resources(RenderPassResources resources) { m_resources = resources; }

    void execute(system::render::FrameContext& ctx) override {
        refresh_scene_texture(ctx.frame_index());

        m_imgui_context->begin_frame();
        if (m_overlay_adapter) {
            m_overlay_adapter->draw_imgui();
        }

        ImDrawData* draw_data = m_imgui_context->prepare_draw_data();
        if (draw_data == nullptr) {
            return;
        }

        vk::RenderingAttachmentInfo color_attachment_info{};
        color_attachment_info.imageView   = *ctx.swapchain_image_view();
        color_attachment_info.imageLayout = vk::ImageLayout::eColorAttachmentOptimal;
        color_attachment_info.loadOp      = vk::AttachmentLoadOp::eLoad;
        color_attachment_info.storeOp     = vk::AttachmentStoreOp::eStore;

        vk::RenderingInfo rendering_info{};
        rendering_info.renderArea.offset    = vk::Offset2D{0, 0};
        rendering_info.renderArea.extent    = ctx.render_extent();
        rendering_info.layerCount           = 1;
        rendering_info.colorAttachmentCount = 1;
        rendering_info.pColorAttachments    = &color_attachment_info;
        rendering_info.pDepthAttachment     = nullptr;

        auto& command_buffer = ctx.cmd().command_buffer();
        command_buffer.beginRendering(rendering_info);
        m_imgui_context->render_draw_data(command_buffer, draw_data);
        command_buffer.endRendering();
    }

private:
    void bind_editor_services() {
        auto& services                   = m_editor_host->context().services();
        services.get_scene_texture_id    = [this]() { return m_current_scene_texture; };
        services.get_scene_texture_size  = [this]() { return m_current_scene_texture_size; };
        services.set_scene_hovered       = [this](bool hovered) { m_scene_hovered = hovered; };
        services.set_scene_focused       = [this](bool focused) { m_scene_focused = focused; };
        services.set_scene_viewport_size = [this](std::uint32_t width, std::uint32_t height) {
            if (m_owner_pipeline) {
                auto* sink = dynamic_cast<system::render::ISceneViewportSink*>(m_owner_pipeline);
                if (sink != nullptr) {
                    sink->set_scene_viewport_extent(vk::Extent2D{width, height});
                }
            }
        };
    }

    void clear_editor_services() {
        if (!m_editor_host) {
            return;
        }
        auto& services                   = m_editor_host->context().services();
        services.get_scene_texture_id    = {};
        services.get_scene_texture_size  = {};
        services.set_scene_viewport_size = {};
        services.set_scene_hovered       = {};
        services.set_scene_focused       = {};
    }

    void release_scene_textures() {
        for (auto& entry : m_scene_texture_entries) {
            if (entry.texture_id != ImTextureID{}) {
                ImGui_ImplVulkan_RemoveTexture(texture_id_to_descriptor_set(entry.texture_id));
                entry.texture_id = ImTextureID{};
            }
            entry.image_view   = vk::ImageView{};
            entry.image_layout = vk::ImageLayout::eUndefined;
            entry.texture_size = ImVec2{0.0f, 0.0f};
        }
        m_current_scene_texture      = ImTextureID{};
        m_current_scene_texture_size = ImVec2{0.0f, 0.0f};
    }

    void refresh_scene_texture(std::uint32_t frame_index) {
        if (m_resources.scene_image_view == vk::ImageView{}) {
            m_current_scene_texture      = ImTextureID{};
            m_current_scene_texture_size = ImVec2{0.0f, 0.0f};
            return;
        }

        const vk::ImageView   view   = m_resources.scene_image_view;
        const vk::ImageLayout layout = m_resources.scene_image_layout;
        const vk::Extent2D    extent = m_resources.scene_extent;

        if (frame_index >= m_scene_texture_entries.size()) {
            m_scene_texture_entries.resize(frame_index + 1);
        }
        auto& entry = m_scene_texture_entries[frame_index];

        const bool descriptor_changed =
            entry.texture_id == ImTextureID{} || entry.image_view != view || entry.image_layout != layout;

        if (descriptor_changed) {
            if (entry.texture_id != ImTextureID{}) {
                ImGui_ImplVulkan_RemoveTexture(texture_id_to_descriptor_set(entry.texture_id));
            }

            const VkDescriptorSet descriptor_set =
                ImGui_ImplVulkan_AddTexture(static_cast<VkSampler>(*m_scene_sampler->sampler()),
                                            static_cast<VkImageView>(view), static_cast<VkImageLayout>(layout));
            entry.texture_id   = descriptor_set_to_texture_id(descriptor_set);
            entry.image_view   = view;
            entry.image_layout = layout;
        }

        entry.texture_size           = ImVec2{static_cast<float>(extent.width), static_cast<float>(extent.height)};
        m_current_scene_texture      = entry.texture_id;
        m_current_scene_texture_size = entry.texture_size;
    }
};

}  // namespace rtr::editor::render
