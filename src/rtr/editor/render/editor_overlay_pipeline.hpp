#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include "imgui.h"
#include "imgui_impl_vulkan.h"

#include "rtr/editor/editor_capture.hpp"
#include "rtr/editor/editor_host.hpp"
#include "rtr/editor/render/imgui_overlay_pass.hpp"
#include "rtr/rhi/texture.hpp"
#include "rtr/system/render/frame_color_source.hpp"
#include "rtr/system/render/pipeline.hpp"

namespace rtr::editor::render {

class EditorOverlayPipeline final : public system::render::IRenderPipeline,
                                    public system::render::IFramePreparePipeline,
                                    public system::render::IResourceAwarePipeline,
                                    public IEditorInputCaptureSource {
private:
    class EditorHostOverlayAdapter final : public IImGuiOverlay {
    private:
        std::shared_ptr<EditorHost> m_host{};

    public:
        explicit EditorHostOverlayAdapter(std::shared_ptr<EditorHost> host)
            : m_host(std::move(host)) {}

        void draw_imgui() override {
            if (m_host) {
                m_host->draw_imgui();
            }
        }
    };

    struct SceneTextureEntry {
        vk::ImageView image_view{};
        vk::ImageLayout image_layout{vk::ImageLayout::eUndefined};
        ImTextureID texture_id{};
        ImVec2 texture_size{0.0f, 0.0f};
    };

    std::unique_ptr<system::render::IRenderPipeline> m_runtime_pipeline{};
    std::shared_ptr<EditorHost> m_editor_host{};
    std::shared_ptr<EditorHostOverlayAdapter> m_overlay_adapter{};
    ImGuiOverlayPass m_imgui_pass;

    std::unique_ptr<rtr::rhi::Sampler> m_scene_sampler{};
    std::vector<SceneTextureEntry> m_scene_texture_entries{};
    ImTextureID m_current_scene_texture{};
    ImVec2 m_current_scene_texture_size{0.0f, 0.0f};
    bool m_scene_hovered{false};
    bool m_scene_focused{false};

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
    EditorOverlayPipeline(
        const system::render::PipelineRuntime& runtime,
        std::unique_ptr<system::render::IRenderPipeline> runtime_pipeline,
        std::shared_ptr<EditorHost> editor_host
    )
        : m_runtime_pipeline(std::move(runtime_pipeline)),
          m_editor_host(std::move(editor_host)),
          m_imgui_pass(
              runtime.device,
              runtime.context,
              runtime.window,
              runtime.image_count,
              runtime.color_format,
              runtime.depth_format
          ) {
        if (!runtime.is_valid()) {
            throw std::runtime_error("EditorOverlayPipeline requires valid pipeline runtime.");
        }
        if (!m_runtime_pipeline) {
            throw std::invalid_argument("EditorOverlayPipeline requires non-null runtime pipeline.");
        }
        if (!m_editor_host) {
            throw std::invalid_argument("EditorOverlayPipeline requires non-null editor host.");
        }

        m_overlay_adapter = std::make_shared<EditorHostOverlayAdapter>(m_editor_host);
        m_imgui_pass.set_overlay(m_overlay_adapter);
        m_scene_sampler = std::make_unique<rtr::rhi::Sampler>(
            rtr::rhi::Sampler::create_default(runtime.device, 1)
        );
        m_scene_texture_entries.resize(runtime.frame_count);
        bind_editor_services();
    }

    ~EditorOverlayPipeline() override {
        clear_editor_services();
        release_scene_textures();
    }

    system::render::IRenderPipeline& runtime_pipeline() {
        return *m_runtime_pipeline;
    }

    const system::render::IRenderPipeline& runtime_pipeline() const {
        return *m_runtime_pipeline;
    }

    void on_resize(int width, int height) override {
        m_runtime_pipeline->on_resize(width, height);
    }

    void on_swapchain_state_changed(const system::render::FrameScheduler::SwapchainState& state) override {
        m_runtime_pipeline->on_swapchain_state_changed(state);
        m_imgui_pass.on_swapchain_recreated(
            state.image_count,
            state.color_format,
            state.depth_format
        );
        release_scene_textures();
    }

    void render(system::render::FrameContext& ctx) override {
        m_runtime_pipeline->render(ctx);
        refresh_scene_texture(ctx.frame_index());
        m_imgui_pass.render(ctx);
    }

    void prepare_frame(const system::render::FramePrepareContext& ctx) override {
        auto* prepare = dynamic_cast<system::render::IFramePreparePipeline*>(m_runtime_pipeline.get());
        if (prepare != nullptr) {
            prepare->prepare_frame(ctx);
        }
    }

    void set_resource_manager(resource::ResourceManager* manager) override {
        auto* resource_aware = dynamic_cast<system::render::IResourceAwarePipeline*>(m_runtime_pipeline.get());
        if (resource_aware != nullptr) {
            resource_aware->set_resource_manager(manager);
        }
    }

    bool wants_imgui_capture_mouse() const override {
        if (m_scene_hovered) {
            return false;
        }
        return m_imgui_pass.wants_capture_mouse();
    }

    bool wants_imgui_capture_keyboard() const override {
        if (m_scene_focused) {
            return false;
        }
        return m_imgui_pass.wants_capture_keyboard();
    }

private:
    void bind_editor_services() {
        auto& services = m_editor_host->context().services();
        services.get_scene_texture_id = [this]() {
            return m_current_scene_texture;
        };
        services.get_scene_texture_size = [this]() {
            return m_current_scene_texture_size;
        };
        services.set_scene_hovered = [this](bool hovered) {
            m_scene_hovered = hovered;
        };
        services.set_scene_focused = [this](bool focused) {
            m_scene_focused = focused;
        };
        services.set_scene_viewport_size = [this](std::uint32_t width, std::uint32_t height) {
            auto* sink = dynamic_cast<system::render::ISceneViewportSink*>(m_runtime_pipeline.get());
            if (sink != nullptr) {
                sink->set_scene_viewport_extent(vk::Extent2D{
                    width,
                    height
                });
            }
        };
    }

    void clear_editor_services() {
        if (!m_editor_host) {
            return;
        }
        auto& services = m_editor_host->context().services();
        services.get_scene_texture_id = {};
        services.get_scene_texture_size = {};
        services.set_scene_viewport_size = {};
        services.set_scene_hovered = {};
        services.set_scene_focused = {};
    }

    void release_scene_textures() {
        for (auto& entry : m_scene_texture_entries) {
            if (entry.texture_id != ImTextureID{}) {
                ImGui_ImplVulkan_RemoveTexture(texture_id_to_descriptor_set(entry.texture_id));
                entry.texture_id = ImTextureID{};
            }
            entry.image_view = vk::ImageView{};
            entry.image_layout = vk::ImageLayout::eUndefined;
            entry.texture_size = ImVec2{0.0f, 0.0f};
        }
        m_current_scene_texture = ImTextureID{};
        m_current_scene_texture_size = ImVec2{0.0f, 0.0f};
    }

    void refresh_scene_texture(std::uint32_t frame_index) {
        auto* color_source = dynamic_cast<const system::render::IFrameColorSource*>(m_runtime_pipeline.get());
        if (color_source == nullptr) {
            m_current_scene_texture = ImTextureID{};
            m_current_scene_texture_size = ImVec2{0.0f, 0.0f};
            return;
        }

        const auto view = color_source->frame_color_source_view(frame_index);
        if (!view.valid()) {
            m_current_scene_texture = ImTextureID{};
            m_current_scene_texture_size = ImVec2{0.0f, 0.0f};
            return;
        }

        if (frame_index >= m_scene_texture_entries.size()) {
            m_scene_texture_entries.resize(frame_index + 1);
        }
        auto& entry = m_scene_texture_entries[frame_index];

        const bool descriptor_changed =
            entry.texture_id == ImTextureID{} ||
            entry.image_view != view.image_view ||
            entry.image_layout != view.image_layout;
        if (descriptor_changed) {
            if (entry.texture_id != ImTextureID{}) {
                ImGui_ImplVulkan_RemoveTexture(texture_id_to_descriptor_set(entry.texture_id));
            }

            const VkDescriptorSet descriptor_set = ImGui_ImplVulkan_AddTexture(
                static_cast<VkSampler>(*m_scene_sampler->sampler()),
                static_cast<VkImageView>(view.image_view),
                static_cast<VkImageLayout>(view.image_layout)
            );
            entry.texture_id = descriptor_set_to_texture_id(descriptor_set);
            entry.image_view = view.image_view;
            entry.image_layout = view.image_layout;
        }

        entry.texture_size = ImVec2{
            static_cast<float>(view.extent.width),
            static_cast<float>(view.extent.height)
        };
        m_current_scene_texture = entry.texture_id;
        m_current_scene_texture_size = entry.texture_size;
    }
};

} // namespace rtr::editor::render
