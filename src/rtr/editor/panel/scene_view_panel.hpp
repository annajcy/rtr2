#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

#include "ImGuizmo.h"
#include "imgui.h"

#include "rtr/editor/core/editor_panel.hpp"
#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/framework/component/camera/orthographic_camera.hpp"
#include "rtr/framework/component/camera/perspective_camera.hpp"
#include "rtr/framework/core/scene.hpp"

namespace rtr::editor {

class SceneViewPanel final : public IEditorPanel {
private:
    enum class AspectMode {
        Preset16x9,
        Preset4x3,
        FollowCamera
    };

    struct ActiveCameraInfo {
        const framework::component::Camera* camera{nullptr};
        pbpt::math::Mat4 view{1.0f};
        pbpt::math::Mat4 proj{1.0f};
        float aspect_ratio{0.0f};
        bool orthographic{false};

        bool valid() const {
            return camera != nullptr;
        }
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

    static const char* gizmo_operation_label(EditorGizmoOperation operation) {
        switch (operation) {
            case EditorGizmoOperation::Translate:
                return "Translate";
            case EditorGizmoOperation::Rotate:
                return "Rotate";
            case EditorGizmoOperation::Scale:
                return "Scale";
        }
        return "Translate";
    }

    static const char* gizmo_mode_label(EditorGizmoMode mode) {
        switch (mode) {
            case EditorGizmoMode::Local:
                return "Local";
            case EditorGizmoMode::World:
                return "World";
        }
        return "Local";
    }

    static ImGuizmo::OPERATION to_imguizmo_operation(EditorGizmoOperation operation) {
        switch (operation) {
            case EditorGizmoOperation::Translate:
                return ImGuizmo::TRANSLATE;
            case EditorGizmoOperation::Rotate:
                return ImGuizmo::ROTATE;
            case EditorGizmoOperation::Scale:
                return ImGuizmo::SCALE;
        }
        return ImGuizmo::TRANSLATE;
    }

    static ImGuizmo::MODE to_imguizmo_mode(EditorGizmoMode mode) {
        switch (mode) {
            case EditorGizmoMode::Local:
                return ImGuizmo::LOCAL;
            case EditorGizmoMode::World:
                return ImGuizmo::WORLD;
        }
        return ImGuizmo::LOCAL;
    }

    static std::array<float, 16> to_imguizmo_matrix(const pbpt::math::Mat4& matrix) {
        std::array<float, 16> values{};
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                values[col * 4 + row] = static_cast<float>(matrix[row][col]);
            }
        }
        return values;
    }

    static pbpt::math::Mat4 from_imguizmo_matrix(const float* values) {
        pbpt::math::Mat4 matrix{0.0f};
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                matrix[row][col] = values[col * 4 + row];
            }
        }
        return matrix;
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

    static std::optional<ActiveCameraInfo> find_active_camera(EditorContext& ctx) {
        auto* active_scene = ctx.world().active_scene();
        if (active_scene == nullptr) {
            return std::nullopt;
        }

        std::optional<ActiveCameraInfo> camera_info{};
        for (const auto node_id : active_scene->scene_graph().active_nodes()) {
            const auto* go = active_scene->find_game_object(node_id);
            if (go == nullptr || !go->enabled()) {
                continue;
            }

            const auto* camera = go->get_component<framework::component::Camera>();
            if (camera == nullptr || !camera->enabled() || !camera->active()) {
                continue;
            }

            if (camera_info.has_value()) {
                return std::nullopt;
            }

            ActiveCameraInfo resolved{};
            resolved.camera = camera;
            resolved.view = camera->view_matrix();
            resolved.proj = camera->projection_matrix();

            if (const auto* perspective =
                    dynamic_cast<const framework::component::PerspectiveCamera*>(camera);
                perspective != nullptr) {
                resolved.aspect_ratio = perspective->aspect_ratio();
                resolved.orthographic = false;
            } else if (const auto* orthographic =
                           dynamic_cast<const framework::component::OrthographicCamera*>(camera);
                       orthographic != nullptr) {
                const float width = orthographic->right_bound() - orthographic->left_bound();
                const float height = orthographic->top_bound() - orthographic->bottom_bound();
                if (width > 0.0f && height > 0.0f) {
                    resolved.aspect_ratio = width / height;
                }
                resolved.orthographic = true;
            }

            camera_info = resolved;
        }

        return camera_info;
    }

    float resolve_target_aspect(EditorContext& ctx, ImVec2 texture_size) const {
        if (m_aspect_mode == AspectMode::Preset16x9) {
            return 16.0f / 9.0f;
        }
        if (m_aspect_mode == AspectMode::Preset4x3) {
            return 4.0f / 3.0f;
        }

        if (const auto active_camera = find_active_camera(ctx); active_camera.has_value()) {
            if (active_camera->aspect_ratio > 0.0f) {
                return active_camera->aspect_ratio;
            }
        }

        if (texture_size.x > 0.0f && texture_size.y > 0.0f) {
            return texture_size.x / texture_size.y;
        }
        return m_target_aspect_ratio;
    }

    static void draw_gizmo_toolbar(EditorContext& ctx) {
        auto& gizmo = ctx.gizmo_state();
        const bool has_selection = ctx.selection().has_game_object();

        ImGui::BeginDisabled(!has_selection);
        if (ImGui::RadioButton("Translate", gizmo.operation == EditorGizmoOperation::Translate)) {
            gizmo.operation = EditorGizmoOperation::Translate;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Rotate", gizmo.operation == EditorGizmoOperation::Rotate)) {
            gizmo.operation = EditorGizmoOperation::Rotate;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Scale", gizmo.operation == EditorGizmoOperation::Scale)) {
            gizmo.operation = EditorGizmoOperation::Scale;
        }
        ImGui::SameLine();
        if (ImGui::Button(gizmo_mode_label(gizmo.mode))) {
            gizmo.mode =
                gizmo.mode == EditorGizmoMode::Local ? EditorGizmoMode::World : EditorGizmoMode::Local;
        }
        ImGui::EndDisabled();

        if (has_selection) {
            ImGui::SameLine();
            ImGui::TextDisabled("[%s | %s]", gizmo_operation_label(gizmo.operation), gizmo_mode_label(gizmo.mode));
        }
    }

    static void handle_gizmo_shortcuts(EditorContext& ctx, bool scene_focused) {
        if (!scene_focused || !ctx.selection().has_game_object()) {
            return;
        }

        auto& gizmo = ctx.gizmo_state();
        if (ImGui::IsKeyPressed(ImGuiKey_W, false)) {
            gizmo.operation = EditorGizmoOperation::Translate;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_E, false)) {
            gizmo.operation = EditorGizmoOperation::Rotate;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_R, false)) {
            gizmo.operation = EditorGizmoOperation::Scale;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Q, false)) {
            gizmo.mode = gizmo.mode == EditorGizmoMode::Local ? EditorGizmoMode::World : EditorGizmoMode::Local;
        }
    }

    static bool apply_gizmo_to_selection(
        EditorContext& ctx,
        const ActiveCameraInfo& active_camera,
        const EditorViewportRect& viewport_rect
    ) {
        auto* active_scene = ctx.world().active_scene();
        const auto& selection = ctx.selection();
        if (active_scene == nullptr ||
            !selection.has_game_object() ||
            selection.scene_id != active_scene->id() ||
            !viewport_rect.valid()) {
            return false;
        }

        auto* game_object = active_scene->find_game_object(selection.game_object_id);
        if (game_object == nullptr) {
            return false;
        }

        auto node = game_object->node();
        if (!node.valid()) {
            return false;
        }

        auto view = to_imguizmo_matrix(active_camera.view);
        auto proj = to_imguizmo_matrix(active_camera.proj);
        auto world = to_imguizmo_matrix(node.world_matrix());

        ImGuizmo::SetOrthographic(active_camera.orthographic);
        ImGuizmo::SetDrawlist();
        ImGuizmo::SetRect(viewport_rect.x, viewport_rect.y, viewport_rect.width, viewport_rect.height);
        ImGuizmo::Manipulate(
            view.data(),
            proj.data(),
            to_imguizmo_operation(ctx.gizmo_state().operation),
            to_imguizmo_mode(ctx.gizmo_state().mode),
            world.data()
        );

        const bool gizmo_using =
            ImGuizmo::IsUsing() ||
            (ImGuizmo::IsOver() && ImGui::IsMouseDown(ImGuiMouseButton_Left));

        if (ImGuizmo::IsUsing()) {
            const pbpt::math::Mat4 manipulated_world = from_imguizmo_matrix(world.data());
            if (node.parent_id() == framework::core::SceneGraph::kVirtualRootId) {
                node.set_local_model_matrix(manipulated_world);
            } else {
                const pbpt::math::Mat4 parent_world = active_scene->scene_graph().node(node.parent_id()).world_matrix();
                const pbpt::math::Mat4 local = pbpt::math::inverse(parent_world) * manipulated_world;
                node.set_local_model_matrix(local);
            }
        }

        return gizmo_using;
    }

    void publish_viewport_state(EditorContext& ctx, const EditorViewportRect& rect, bool hovered, bool focused, bool gizmo_using) const {
        ctx.gizmo_state().viewport_rect = rect;
        ctx.gizmo_state().enabled = ctx.selection().has_game_object();
        ctx.gizmo_state().using_gizmo = gizmo_using;

        if (ctx.services().set_scene_viewport_rect) {
            ctx.services().set_scene_viewport_rect(rect);
        }
        if (ctx.services().set_scene_hovered) {
            ctx.services().set_scene_hovered(hovered);
        }
        if (ctx.services().set_scene_focused) {
            ctx.services().set_scene_focused(focused);
        }
        if (ctx.services().set_scene_gizmo_using) {
            ctx.services().set_scene_gizmo_using(gizmo_using);
        }
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

        if (!ImGui::Begin("Scene", &m_visible)) {
            publish_viewport_state(ctx, EditorViewportRect{}, false, false, false);
            ImGui::End();
            return;
        }

        ImGuizmo::BeginFrame();

        draw_gizmo_toolbar(ctx);
        const bool scene_focused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
        handle_gizmo_shortcuts(ctx, scene_focused);

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

        bool hovered = false;
        bool gizmo_using = false;
        EditorViewportRect viewport_rect{};

        if (texture_id != ImTextureID{} && texture_size.x > 0.0f && texture_size.y > 0.0f &&
            content_size.x > 0.0f && content_size.y > 0.0f) {
            const float sx = content_size.x / texture_size.x;
            const float sy = content_size.y / texture_size.y;
            const float scale = std::min(sx, sy);
            ImVec2 draw_size{
                std::max(1.0f, texture_size.x * scale),
                std::max(1.0f, texture_size.y * scale)
            };

            const ImVec2 cursor = ImGui::GetCursorPos();
            ImGui::SetCursorPos(ImVec2{
                cursor.x + (content_size.x - draw_size.x) * 0.5f,
                cursor.y + (content_size.y - draw_size.y) * 0.5f
            });

            const ImVec2 image_min = ImGui::GetCursorScreenPos();
            ImGui::Image(texture_id, draw_size, ImVec2{0.0f, 1.0f}, ImVec2{1.0f, 0.0f});
            hovered = ImGui::IsItemHovered();

            viewport_rect = EditorViewportRect{
                .x = image_min.x,
                .y = image_min.y,
                .width = draw_size.x,
                .height = draw_size.y
            };

            if (const auto active_camera = find_active_camera(ctx); active_camera.has_value()) {
                gizmo_using = apply_gizmo_to_selection(ctx, *active_camera, viewport_rect);
            }
        } else {
            ImGui::TextDisabled("Scene texture is unavailable.");
        }

        publish_viewport_state(ctx, viewport_rect, hovered, scene_focused, gizmo_using);

        ImGui::End();
    }
};

} // namespace rtr::editor
