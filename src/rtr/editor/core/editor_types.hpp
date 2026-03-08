#pragma once

#include <cstdint>
#include <functional>

#include "imgui.h"

#include "rtr/framework/core/types.hpp"

namespace rtr::editor {

struct EditorViewportRect {
    float x{0.0f};
    float y{0.0f};
    float width{0.0f};
    float height{0.0f};

    bool valid() const {
        return width > 0.0f && height > 0.0f;
    }
};

enum class EditorGizmoOperation {
    Translate,
    Rotate,
    Scale,
};

enum class EditorGizmoMode {
    Local,
    World,
};

struct EditorGizmoState {
    EditorGizmoOperation operation{EditorGizmoOperation::Translate};
    EditorGizmoMode mode{EditorGizmoMode::Local};
    EditorViewportRect viewport_rect{};
    bool enabled{false};
    bool using_gizmo{false};
};

struct EditorSelection {
    framework::core::SceneId scene_id{framework::core::kInvalidSceneId};
    framework::core::GameObjectId game_object_id{framework::core::kInvalidGameObjectId};

    bool has_game_object() const {
        return scene_id != framework::core::kInvalidSceneId &&
            game_object_id != framework::core::kInvalidGameObjectId;
    }

    void clear() {
        scene_id = framework::core::kInvalidSceneId;
        game_object_id = framework::core::kInvalidGameObjectId;
    }
};

struct EditorFrameData {
    std::uint64_t frame_serial{0};
    double delta_seconds{0.0};
    bool paused{false};
};

struct EditorServices {
    std::function<void(framework::core::GameObjectId)> request_focus_game_object{};
    std::function<ImTextureID()> get_scene_texture_id{};
    std::function<ImVec2()> get_scene_texture_size{};
    std::function<void(std::uint32_t, std::uint32_t)> set_scene_viewport_size{};
    std::function<void(const EditorViewportRect&)> set_scene_viewport_rect{};
    std::function<void(bool)> set_scene_hovered{};
    std::function<void(bool)> set_scene_focused{};
    std::function<void(bool)> set_scene_gizmo_using{};
};

} // namespace rtr::editor
