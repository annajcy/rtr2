#pragma once

#include <algorithm>
#include <cstdint>
#include <cmath>

#include "imgui.h"

#include "rtr/editor/core/editor_panel.hpp"
#include "rtr/framework/core/camera.hpp"

namespace rtr::editor {

class SceneViewPanel final : public IEditorPanel {
private:
    enum class AspectMode {
        Preset16x9,
        Preset4x3,
        FollowCamera
    };

    bool m_visible{true};
    ImVec2 m_last_content_size{0.0f, 0.0f};
    ImVec2 m_last_requested_size{0.0f, 0.0f};
    float m_target_aspect_ratio{16.0f / 9.0f};
    AspectMode m_aspect_mode{AspectMode::FollowCamera};

    static const char* aspect_mode_label(AspectMode mode) {
        switch (mode) {
            case AspectMode::Preset16x9:
                return "16:9";
            case AspectMode::Preset4x3:
                return "4:3";
            case AspectMode::FollowCamera:
                return "Follow Camera";
        }
        return "Follow Camera";
    }

    static ImVec2 fit_size_to_aspect(ImVec2 content_size, float aspect_ratio) {
        if (content_size.x <= 0.0f || content_size.y <= 0.0f) {
            return ImVec2{1.0f, 1.0f};
        }

        if (aspect_ratio <= 0.0f) {
            return ImVec2{
                std::max(1.0f, content_size.x),
                std::max(1.0f, content_size.y)
            };
        }

        float width = content_size.x;
        float height = width / aspect_ratio;
        if (height > content_size.y) {
            height = content_size.y;
            width = height * aspect_ratio;
        }

        return ImVec2{
            std::max(1.0f, width),
            std::max(1.0f, height)
        };
    }

    float resolve_target_aspect(EditorContext& ctx, ImVec2 texture_size) const {
        if (m_aspect_mode == AspectMode::Preset16x9) {
            return 16.0f / 9.0f;
        }
        if (m_aspect_mode == AspectMode::Preset4x3) {
            return 4.0f / 3.0f;
        }

        const auto* active_scene = ctx.world().active_scene();
        if (active_scene != nullptr) {
            const auto* active_camera = active_scene->active_camera();
            if (active_camera != nullptr) {
                if (active_camera->camera_type() == framework::core::CameraType::Perspective) {
                    const auto* perspective = dynamic_cast<const framework::core::PerspectiveCamera*>(active_camera);
                    if (perspective != nullptr && perspective->aspect_ratio() > 0.0f) {
                        return perspective->aspect_ratio();
                    }
                } else if (active_camera->camera_type() == framework::core::CameraType::Orthographic) {
                    const auto* orthographic = dynamic_cast<const framework::core::OrthographicCamera*>(active_camera);
                    if (orthographic != nullptr) {
                        const float width = orthographic->right_bound() - orthographic->left_bound();
                        const float height = orthographic->top_bound() - orthographic->bottom_bound();
                        if (width > 0.0f && height > 0.0f) {
                            return width / height;
                        }
                    }
                }
            }
        }

        if (texture_size.x > 0.0f && texture_size.y > 0.0f) {
            return texture_size.x / texture_size.y;
        }
        return m_target_aspect_ratio;
    }

public:
    std::string_view id() const override {
        return "scene_view";
    }

    int order() const override {
        return 5;
    }

    bool visible() const override {
        return m_visible;
    }

    void set_visible(bool visible) override {
        m_visible = visible;
    }

    void on_imgui(EditorContext& ctx) override {
        if (!m_visible) {
            return;
        }

        bool hovered = false;
        bool focused = false;
        if (!ImGui::Begin("Scene", &m_visible)) {
            if (ctx.services().set_scene_hovered) {
                ctx.services().set_scene_hovered(false);
            }
            if (ctx.services().set_scene_focused) {
                ctx.services().set_scene_focused(false);
            }
            ImGui::End();
            return;
        }

        const ImVec2 content_size = ImGui::GetContentRegionAvail();
        ImTextureID texture_id{};
        ImVec2 texture_size{0.0f, 0.0f};
        if (ctx.services().get_scene_texture_id) {
            texture_id = ctx.services().get_scene_texture_id();
        }
        if (ctx.services().get_scene_texture_size) {
            texture_size = ctx.services().get_scene_texture_size();
        }

        bool mode_changed = false;
        if (ImGui::BeginCombo("Aspect", aspect_mode_label(m_aspect_mode))) {
            const auto selectable_mode = [&](AspectMode mode) {
                const bool selected = m_aspect_mode == mode;
                if (ImGui::Selectable(aspect_mode_label(mode), selected)) {
                    m_aspect_mode = mode;
                    mode_changed = true;
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            };
            selectable_mode(AspectMode::Preset16x9);
            selectable_mode(AspectMode::Preset4x3);
            selectable_mode(AspectMode::FollowCamera);
            ImGui::EndCombo();
        }

        m_target_aspect_ratio = resolve_target_aspect(ctx, texture_size);

        if (ctx.services().set_scene_viewport_size) {
            const ImVec2 requested_size = fit_size_to_aspect(content_size, m_target_aspect_ratio);
            const bool content_changed =
                std::fabs(content_size.x - m_last_content_size.x) > 0.5f ||
                std::fabs(content_size.y - m_last_content_size.y) > 0.5f;
            const bool requested_changed =
                std::fabs(requested_size.x - m_last_requested_size.x) > 0.5f ||
                std::fabs(requested_size.y - m_last_requested_size.y) > 0.5f;

            if (content_changed || requested_changed || mode_changed) {
                const std::uint32_t w = static_cast<std::uint32_t>(requested_size.x);
                const std::uint32_t h = static_cast<std::uint32_t>(requested_size.y);
                ctx.services().set_scene_viewport_size(w, h);
                m_last_content_size = content_size;
                m_last_requested_size = requested_size;
            }
        }

        if (texture_id != ImTextureID{} && texture_size.x > 0.0f && texture_size.y > 0.0f &&
            content_size.x > 0.0f && content_size.y > 0.0f) {
            const float sx = content_size.x / texture_size.x;
            const float sy = content_size.y / texture_size.y;
            const float scale = std::min(sx, sy);
            ImVec2 draw_size{
                texture_size.x * scale,
                texture_size.y * scale
            };
            draw_size.x = std::max(1.0f, draw_size.x);
            draw_size.y = std::max(1.0f, draw_size.y);

            const ImVec2 cursor = ImGui::GetCursorPos();
            ImGui::SetCursorPos(ImVec2{
                cursor.x + (content_size.x - draw_size.x) * 0.5f,
                cursor.y + (content_size.y - draw_size.y) * 0.5f
            });
            ImGui::Image(texture_id, draw_size, ImVec2{0.0f, 1.0f}, ImVec2{1.0f, 0.0f});
            hovered = ImGui::IsItemHovered();
        } else {
            ImGui::TextDisabled("Scene texture is unavailable.");
        }

        focused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);

        if (ctx.services().set_scene_hovered) {
            ctx.services().set_scene_hovered(hovered);
        }
        if (ctx.services().set_scene_focused) {
            ctx.services().set_scene_focused(focused);
        }

        ImGui::End();
    }
};

} // namespace rtr::editor
