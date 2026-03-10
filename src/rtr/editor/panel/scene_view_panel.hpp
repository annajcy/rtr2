#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

#include "imgui.h"
#include "ImGuizmo.h"

#include "rtr/editor/core/editor_panel.hpp"
#include "rtr/editor/core/scene_picking.hpp"
#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/framework/component/camera/orthographic_camera.hpp"
#include "rtr/framework/component/camera/perspective_camera.hpp"
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/physics/box_collider.hpp"
#include "rtr/framework/component/physics/sphere_collider.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/utils/obj_types.hpp"

namespace rtr::editor {

namespace scene_view_detail {

struct MeshEdge {
    std::uint32_t a{0};
    std::uint32_t b{0};

    bool operator==(const MeshEdge& other) const = default;
};

inline std::uint64_t mesh_edge_key(std::uint32_t a, std::uint32_t b) {
    const auto [min_index, max_index] = std::minmax(a, b);
    return (static_cast<std::uint64_t>(min_index) << 32u) | static_cast<std::uint64_t>(max_index);
}

inline std::vector<MeshEdge> build_unique_mesh_edges(const utils::ObjMeshData& mesh) {
    std::vector<MeshEdge> edges{};
    if (mesh.indices.size() < 3u) {
        return edges;
    }

    std::unordered_map<std::uint64_t, MeshEdge> unique_edges{};
    unique_edges.reserve(mesh.indices.size());

    const auto add_edge = [&](std::uint32_t a, std::uint32_t b) {
        const auto [min_index, max_index] = std::minmax(a, b);
        unique_edges.try_emplace(mesh_edge_key(min_index, max_index), MeshEdge{.a = min_index, .b = max_index});
    };

    for (std::size_t i = 0; i + 2u < mesh.indices.size(); i += 3u) {
        const std::uint32_t i0 = mesh.indices[i + 0u];
        const std::uint32_t i1 = mesh.indices[i + 1u];
        const std::uint32_t i2 = mesh.indices[i + 2u];
        add_edge(i0, i1);
        add_edge(i1, i2);
        add_edge(i2, i0);
    }

    edges.reserve(unique_edges.size());
    for (const auto& [_, edge] : unique_edges) {
        edges.push_back(edge);
    }
    std::sort(edges.begin(), edges.end(), [](const MeshEdge& lhs, const MeshEdge& rhs) {
        return lhs.a == rhs.a ? lhs.b < rhs.b : lhs.a < rhs.a;
    });
    return edges;
}

inline const std::vector<MeshEdge>& cached_mesh_edges(resource::ResourceManager& resources, resource::MeshHandle mesh_handle) {
    static std::unordered_map<std::uint64_t, std::vector<MeshEdge>> cache{};

    const auto it = cache.find(mesh_handle.value);
    if (it != cache.end()) {
        return it->second;
    }

    const auto& mesh = resources.cpu<resource::MeshResourceKind>(mesh_handle);
    const auto [inserted_it, _] = cache.emplace(mesh_handle.value, build_unique_mesh_edges(mesh));
    return inserted_it->second;
}

inline std::optional<ImVec2> project_world_to_viewport(
    const pbpt::math::Vec3& world_position,
    const pbpt::math::Mat4& view,
    const pbpt::math::Mat4& proj,
    const EditorViewportRect& viewport_rect
) {
    constexpr float kClipEpsilon = 1e-6f;
    const pbpt::math::Vec4 clip = proj * (view * pbpt::math::Vec4(world_position, 1.0f));

    if (!std::isfinite(clip.x()) || !std::isfinite(clip.y()) || !std::isfinite(clip.z()) || !std::isfinite(clip.w()) ||
        clip.w() <= kClipEpsilon) {
        return std::nullopt;
    }

    const float inv_w = 1.0f / clip.w();
    const float ndc_x = clip.x() * inv_w;
    const float ndc_y = clip.y() * inv_w;

    if (!std::isfinite(ndc_x) || !std::isfinite(ndc_y)) {
        return std::nullopt;
    }

    const float screen_x = viewport_rect.x + ((ndc_x * 0.5f) + 0.5f) * viewport_rect.width;
    const float screen_y = viewport_rect.y + ((-ndc_y * 0.5f) + 0.5f) * viewport_rect.height;
    if (!std::isfinite(screen_x) || !std::isfinite(screen_y)) {
        return std::nullopt;
    }

    return ImVec2{screen_x, screen_y};
}

inline pbpt::math::Float sphere_world_radius(
    pbpt::math::Float base_radius,
    const pbpt::math::Vec3& local_scale,
    const pbpt::math::Vec3& world_scale
) {
    return base_radius * (local_scale * world_scale).abs().max();
}

inline pbpt::math::Vec3 box_world_half_extents(
    const pbpt::math::Vec3& base_half_extents,
    const pbpt::math::Vec3& local_scale,
    const pbpt::math::Vec3& world_scale
) {
    return base_half_extents * local_scale * world_scale;
}

inline std::vector<pbpt::math::Vec3> build_sphere_ring_points(
    const pbpt::math::Vec3& center,
    pbpt::math::Float radius,
    const pbpt::math::Vec3& axis_a,
    const pbpt::math::Vec3& axis_b,
    std::size_t segments
) {
    constexpr float kTau = 6.28318530717958647692f;

    std::vector<pbpt::math::Vec3> points{};
    if (segments == 0u || radius <= 0.0f) {
        return points;
    }

    const auto tangent_a = pbpt::math::normalize(axis_a);
    const auto tangent_b = pbpt::math::normalize(axis_b);

    points.reserve(segments);
    for (std::size_t i = 0; i < segments; ++i) {
        const float angle = kTau * static_cast<float>(i) / static_cast<float>(segments);
        points.push_back(center + radius * (std::cos(angle) * tangent_a + std::sin(angle) * tangent_b));
    }
    return points;
}

inline std::array<pbpt::math::Vec3, 8> build_box_corners(
    const pbpt::math::Vec3& center,
    const pbpt::math::Quat& rotation,
    const pbpt::math::Vec3& half_extents
) {
    return {
        center + rotation * pbpt::math::Vec3{-half_extents.x(), -half_extents.y(), -half_extents.z()},
        center + rotation * pbpt::math::Vec3{ half_extents.x(), -half_extents.y(), -half_extents.z()},
        center + rotation * pbpt::math::Vec3{ half_extents.x(),  half_extents.y(), -half_extents.z()},
        center + rotation * pbpt::math::Vec3{-half_extents.x(),  half_extents.y(), -half_extents.z()},
        center + rotation * pbpt::math::Vec3{-half_extents.x(), -half_extents.y(),  half_extents.z()},
        center + rotation * pbpt::math::Vec3{ half_extents.x(), -half_extents.y(),  half_extents.z()},
        center + rotation * pbpt::math::Vec3{ half_extents.x(),  half_extents.y(),  half_extents.z()},
        center + rotation * pbpt::math::Vec3{-half_extents.x(),  half_extents.y(),  half_extents.z()},
    };
}

}  // namespace scene_view_detail

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

    static void sanitize_snap_settings(EditorGizmoState& gizmo) {
        gizmo.translation_snap.x() = std::max(0.001f, gizmo.translation_snap.x());
        gizmo.translation_snap.y() = std::max(0.001f, gizmo.translation_snap.y());
        gizmo.translation_snap.z() = std::max(0.001f, gizmo.translation_snap.z());
        gizmo.rotation_snap_degrees = std::max(0.1f, gizmo.rotation_snap_degrees);
        gizmo.scale_snap = std::max(0.001f, gizmo.scale_snap);
    }

    static std::array<float, 3> current_snap_values(const EditorGizmoState& gizmo) {
        switch (gizmo.operation) {
            case EditorGizmoOperation::Translate:
                return {gizmo.translation_snap.x(), gizmo.translation_snap.y(), gizmo.translation_snap.z()};
            case EditorGizmoOperation::Rotate:
                return {gizmo.rotation_snap_degrees, 0.0f, 0.0f};
            case EditorGizmoOperation::Scale:
                return {gizmo.scale_snap, 0.0f, 0.0f};
        }
        return {1.0f, 1.0f, 1.0f};
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

    static EditorGizmoOperation resolved_gizmo_operation(const EditorGizmoState& gizmo) {
        return gizmo.operation;
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

    static pbpt::math::Vec3 safe_divide_by_scale(
        const pbpt::math::Vec3& value,
        const pbpt::math::Vec3& scale,
        const pbpt::math::Vec3& fallback
    ) {
        constexpr float kScaleEpsilon = 1e-6f;
        pbpt::math::Vec3 result = fallback;
        for (int axis = 0; axis < 3; ++axis) {
            if (std::abs(scale[axis]) > kScaleEpsilon) {
                result[axis] = value[axis] / scale[axis];
            }
        }
        return result;
    }

    static pbpt::math::Vec3 clamp_positive_scale(
        const pbpt::math::Vec3& scale,
        const pbpt::math::Vec3& fallback
    ) {
        constexpr float kMinScale = 0.001f;
        pbpt::math::Vec3 result = scale;
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(result[axis])) {
                result[axis] = fallback[axis];
            }
            result[axis] = std::max(result[axis], kMinScale);
        }
        return result;
    }

    static pbpt::math::Vec3 collider_world_position(const auto& node, const pbpt::math::Vec3& local_position) {
        return node.world_position() + node.world_rotation() * (local_position * node.world_scale());
    }

    static pbpt::math::Vec3 local_position_from_world_position(
        const auto& node,
        const pbpt::math::Vec3& world_position,
        const pbpt::math::Vec3& fallback
    ) {
        const auto unscaled_local_position =
            node.world_rotation().inversed() * (world_position - node.world_position());
        return safe_divide_by_scale(unscaled_local_position, node.world_scale(), fallback);
    }

    static pbpt::math::Vec3 collider_world_scale(const auto& node, const pbpt::math::Vec3& local_scale) {
        return (node.world_scale() * local_scale).abs();
    }

    static pbpt::math::Vec3 local_scale_from_world_scale(
        const auto& node,
        const pbpt::math::Vec3& world_scale,
        const pbpt::math::Vec3& fallback
    ) {
        return clamp_positive_scale(
            safe_divide_by_scale(world_scale, node.world_scale().abs(), fallback),
            fallback
        );
    }

    static void draw_projected_segment(
        ImDrawList* draw_list,
        const pbpt::math::Vec3& world_a,
        const pbpt::math::Vec3& world_b,
        const ActiveCameraInfo& active_camera,
        const EditorViewportRect& viewport_rect,
        ImU32 color,
        float thickness
    ) {
        const auto screen_a =
            scene_view_detail::project_world_to_viewport(world_a, active_camera.view, active_camera.proj, viewport_rect);
        const auto screen_b =
            scene_view_detail::project_world_to_viewport(world_b, active_camera.view, active_camera.proj, viewport_rect);
        if (!screen_a.has_value() || !screen_b.has_value()) {
            return;
        }

        draw_list->AddLine(*screen_a, *screen_b, color, thickness);
    }

    static void draw_mesh_outline(
        ImDrawList* draw_list,
        EditorContext& ctx,
        const auto& node,
        const framework::component::MeshRenderer& mesh_renderer,
        const ActiveCameraInfo& active_camera,
        const EditorViewportRect& viewport_rect
    ) {
        constexpr ImU32 kMeshOutlineColor = IM_COL32(64, 220, 255, 255);
        constexpr float kMeshOutlineThickness = 1.5f;

        const auto mesh_handle = mesh_renderer.mesh_handle();
        if (!mesh_handle.is_valid() || !ctx.resources().alive<resource::MeshResourceKind>(mesh_handle)) {
            return;
        }

        const auto& cpu_mesh = ctx.resources().cpu<resource::MeshResourceKind>(mesh_handle);
        const auto& edges = scene_view_detail::cached_mesh_edges(ctx.resources(), mesh_handle);
        const auto& world_matrix = node.world_matrix();

        for (const auto& edge : edges) {
            if (edge.a >= cpu_mesh.vertices.size() || edge.b >= cpu_mesh.vertices.size()) {
                continue;
            }

            const pbpt::math::Vec4 clip_a = world_matrix * pbpt::math::Vec4(cpu_mesh.vertices[edge.a].position, 1.0f);
            const pbpt::math::Vec4 clip_b = world_matrix * pbpt::math::Vec4(cpu_mesh.vertices[edge.b].position, 1.0f);
            const pbpt::math::Vec3 world_a{clip_a.x(), clip_a.y(), clip_a.z()};
            const pbpt::math::Vec3 world_b{clip_b.x(), clip_b.y(), clip_b.z()};
            draw_projected_segment(
                draw_list,
                world_a,
                world_b,
                active_camera,
                viewport_rect,
                kMeshOutlineColor,
                kMeshOutlineThickness
            );
        }
    }

    static void draw_sphere_collider_outline(
        ImDrawList* draw_list,
        const auto& node,
        const framework::component::SphereCollider& sphere_collider,
        const ActiveCameraInfo& active_camera,
        const EditorViewportRect& viewport_rect
    ) {
        constexpr ImU32 kColliderOutlineColor = IM_COL32(255, 170, 64, 255);
        constexpr float kColliderOutlineThickness = 2.0f;
        constexpr std::size_t kSphereSegments = 48u;

        const pbpt::math::Vec3 center = collider_world_position(node, sphere_collider.local_position());
        const float radius = scene_view_detail::sphere_world_radius(
            sphere_collider.radius(),
            sphere_collider.local_scale(),
            node.world_scale()
        );
        if (radius <= 0.0f) {
            return;
        }

        const auto ring_xy = scene_view_detail::build_sphere_ring_points(
            center, radius, pbpt::math::Vec3{1.0f, 0.0f, 0.0f}, pbpt::math::Vec3{0.0f, 1.0f, 0.0f}, kSphereSegments);
        const auto ring_yz = scene_view_detail::build_sphere_ring_points(
            center, radius, pbpt::math::Vec3{0.0f, 1.0f, 0.0f}, pbpt::math::Vec3{0.0f, 0.0f, 1.0f}, kSphereSegments);
        const auto ring_xz = scene_view_detail::build_sphere_ring_points(
            center, radius, pbpt::math::Vec3{1.0f, 0.0f, 0.0f}, pbpt::math::Vec3{0.0f, 0.0f, 1.0f}, kSphereSegments);

        const auto draw_ring = [&](const std::vector<pbpt::math::Vec3>& ring) {
            for (std::size_t i = 0; i < ring.size(); ++i) {
                const pbpt::math::Vec3& world_a = ring[i];
                const pbpt::math::Vec3& world_b = ring[(i + 1u) % ring.size()];
                draw_projected_segment(
                    draw_list,
                    world_a,
                    world_b,
                    active_camera,
                    viewport_rect,
                    kColliderOutlineColor,
                    kColliderOutlineThickness
                );
            }
        };

        draw_ring(ring_xy);
        draw_ring(ring_yz);
        draw_ring(ring_xz);
    }

    static void draw_box_collider_outline(
        ImDrawList* draw_list,
        const auto& node,
        const framework::component::BoxCollider& box_collider,
        const ActiveCameraInfo& active_camera,
        const EditorViewportRect& viewport_rect
    ) {
        constexpr ImU32 kColliderOutlineColor = IM_COL32(255, 170, 64, 255);
        constexpr float kColliderOutlineThickness = 2.0f;
        constexpr std::array<std::pair<int, int>, 12> kBoxEdges{{
            {0, 1}, {1, 2}, {2, 3}, {3, 0},
            {4, 5}, {5, 6}, {6, 7}, {7, 4},
            {0, 4}, {1, 5}, {2, 6}, {3, 7},
        }};

        const pbpt::math::Vec3 center = collider_world_position(node, box_collider.local_position());
        const pbpt::math::Quat rotation = pbpt::math::normalize(node.world_rotation() * box_collider.local_rotation());
        const pbpt::math::Vec3 half_extents = scene_view_detail::box_world_half_extents(
            box_collider.half_extents(),
            box_collider.local_scale(),
            node.world_scale()
        );
        const auto corners = scene_view_detail::build_box_corners(center, rotation, half_extents);

        for (const auto [a, b] : kBoxEdges) {
            draw_projected_segment(
                draw_list,
                corners[static_cast<std::size_t>(a)],
                corners[static_cast<std::size_t>(b)],
                active_camera,
                viewport_rect,
                kColliderOutlineColor,
                kColliderOutlineThickness
            );
        }
    }

    static void draw_selection_overlay(
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
            return;
        }

        auto* game_object = active_scene->find_game_object(selection.game_object_id);
        if (game_object == nullptr || !game_object->enabled()) {
            return;
        }

        const auto node = game_object->node();
        if (!node.is_valid()) {
            return;
        }

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        if (draw_list == nullptr) {
            return;
        }

        draw_list->PushClipRect(
            ImVec2{viewport_rect.x, viewport_rect.y},
            ImVec2{viewport_rect.x + viewport_rect.width, viewport_rect.y + viewport_rect.height},
            true
        );

        switch (ctx.gizmo_state().target) {
            case EditorGizmoTarget::GameObjectTransform:
                if (const auto* mesh_renderer = game_object->get_component<framework::component::MeshRenderer>();
                    mesh_renderer != nullptr && mesh_renderer->enabled()) {
                    draw_mesh_outline(draw_list, ctx, node, *mesh_renderer, active_camera, viewport_rect);
                }
                break;
            case EditorGizmoTarget::SphereColliderLocal:
                if (const auto* sphere_collider = game_object->get_component<framework::component::SphereCollider>();
                    sphere_collider != nullptr && sphere_collider->enabled()) {
                    draw_sphere_collider_outline(draw_list, node, *sphere_collider, active_camera, viewport_rect);
                }
                break;
            case EditorGizmoTarget::BoxColliderLocal:
                if (const auto* box_collider = game_object->get_component<framework::component::BoxCollider>();
                    box_collider != nullptr && box_collider->enabled()) {
                    draw_box_collider_outline(draw_list, node, *box_collider, active_camera, viewport_rect);
                }
                break;
        }

        draw_list->PopClipRect();
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
        sanitize_snap_settings(gizmo);

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
            ImGui::TextDisabled(
                "[T Translate | R Rotate | Y Scale | F Mode] [%s | %s]",
                gizmo_operation_label(gizmo.operation),
                gizmo_mode_label(gizmo.mode)
            );
        }

        ImGui::SameLine();
        ImGui::Checkbox("Snap", &gizmo.snap_enabled);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140.0f);
        switch (gizmo.operation) {
            case EditorGizmoOperation::Translate:
                ImGui::InputFloat3("Snap##translate", &gizmo.translation_snap.x());
                break;
            case EditorGizmoOperation::Rotate:
                ImGui::InputFloat("Angle Snap##rotate", &gizmo.rotation_snap_degrees);
                break;
            case EditorGizmoOperation::Scale:
                ImGui::InputFloat("Scale Snap##scale", &gizmo.scale_snap);
                break;
        }
        sanitize_snap_settings(gizmo);
    }

    static void handle_gizmo_shortcuts(EditorContext& ctx, bool scene_focused) {
        if (!scene_focused || !ctx.selection().has_game_object()) {
            return;
        }

        auto& gizmo = ctx.gizmo_state();
        if (ImGui::IsKeyPressed(ImGuiKey_T, false)) {
            gizmo.operation = EditorGizmoOperation::Translate;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_R, false)) {
            gizmo.operation = EditorGizmoOperation::Rotate;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Y, false)) {
            gizmo.operation = EditorGizmoOperation::Scale;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_F, false)) {
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
        if (!node.is_valid()) {
            return false;
        }

        auto gizmo_target = ctx.gizmo_state().target;
        auto* sphere_collider = game_object->get_component<framework::component::SphereCollider>();
        auto* box_collider = game_object->get_component<framework::component::BoxCollider>();
        if (gizmo_target == EditorGizmoTarget::SphereColliderLocal &&
            (sphere_collider == nullptr || !sphere_collider->enabled())) {
            ctx.reset_gizmo_target();
            gizmo_target = ctx.gizmo_state().target;
        }
        if (gizmo_target == EditorGizmoTarget::BoxColliderLocal &&
            (box_collider == nullptr || !box_collider->enabled())) {
            ctx.reset_gizmo_target();
            gizmo_target = ctx.gizmo_state().target;
        }

        auto view = to_imguizmo_matrix(active_camera.view);
        auto proj = to_imguizmo_matrix(active_camera.proj);
        pbpt::math::Mat4 gizmo_world_matrix = node.world_matrix();
        switch (gizmo_target) {
            case EditorGizmoTarget::SphereColliderLocal:
                gizmo_world_matrix = pbpt::math::compose_trs(
                    collider_world_position(node, sphere_collider->local_position()),
                    pbpt::math::normalize(node.world_rotation() * sphere_collider->local_rotation()),
                    collider_world_scale(node, sphere_collider->local_scale())
                );
                break;
            case EditorGizmoTarget::BoxColliderLocal:
                gizmo_world_matrix = pbpt::math::compose_trs(
                    collider_world_position(node, box_collider->local_position()),
                    pbpt::math::normalize(node.world_rotation() * box_collider->local_rotation()),
                    collider_world_scale(node, box_collider->local_scale())
                );
                break;
            case EditorGizmoTarget::GameObjectTransform:
                break;
        }

        auto world = to_imguizmo_matrix(gizmo_world_matrix);
        const bool snap_active = ctx.gizmo_state().snap_enabled || ImGui::IsKeyDown(ImGuiKey_ModShift);
        auto resolved_operation = resolved_gizmo_operation(ctx.gizmo_state());
        const auto snap = current_snap_values(EditorGizmoState{
            .operation = resolved_operation,
            .mode = ctx.gizmo_state().mode,
            .target = ctx.gizmo_state().target,
            .viewport_rect = ctx.gizmo_state().viewport_rect,
            .translation_snap = ctx.gizmo_state().translation_snap,
            .rotation_snap_degrees = ctx.gizmo_state().rotation_snap_degrees,
            .scale_snap = ctx.gizmo_state().scale_snap,
            .snap_enabled = ctx.gizmo_state().snap_enabled,
            .enabled = ctx.gizmo_state().enabled,
            .using_gizmo = ctx.gizmo_state().using_gizmo,
        });

        ImGuizmo::SetOrthographic(active_camera.orthographic);
        ImGuizmo::SetDrawlist();
        ImGuizmo::SetRect(viewport_rect.x, viewport_rect.y, viewport_rect.width, viewport_rect.height);
        ImGuizmo::Manipulate(
            view.data(),
            proj.data(),
            to_imguizmo_operation(resolved_operation),
            to_imguizmo_mode(ctx.gizmo_state().mode),
            world.data(),
            nullptr,
            snap_active ? snap.data() : nullptr
        );

        const bool gizmo_using =
            ImGuizmo::IsUsing() ||
            (ImGuizmo::IsOver() && ImGui::IsMouseDown(ImGuiMouseButton_Left));

        if (ImGuizmo::IsUsing()) {
            const pbpt::math::Mat4 manipulated_world = from_imguizmo_matrix(world.data());
            switch (gizmo_target) {
                case EditorGizmoTarget::SphereColliderLocal: {
                    const auto world_position = pbpt::math::extract_translation(manipulated_world);
                    sphere_collider->set_local_position(
                        local_position_from_world_position(node, world_position, sphere_collider->local_position())
                    );
                    sphere_collider->set_local_rotation(pbpt::math::normalize(
                        node.world_rotation().inversed() * pbpt::math::extract_rotation(manipulated_world)
                    ));
                    sphere_collider->set_local_scale(
                        local_scale_from_world_scale(node, pbpt::math::extract_scale(manipulated_world),
                                                     sphere_collider->local_scale())
                    );
                    break;
                }
                case EditorGizmoTarget::BoxColliderLocal: {
                    const auto world_position = pbpt::math::extract_translation(manipulated_world);
                    box_collider->set_local_position(
                        local_position_from_world_position(node, world_position, box_collider->local_position())
                    );
                    box_collider->set_local_rotation(pbpt::math::normalize(
                        node.world_rotation().inversed() * pbpt::math::extract_rotation(manipulated_world)
                    ));
                    box_collider->set_local_scale(
                        local_scale_from_world_scale(node, pbpt::math::extract_scale(manipulated_world),
                                                     box_collider->local_scale())
                    );
                    break;
                }
                case EditorGizmoTarget::GameObjectTransform:
                    if (node.parent_id() == framework::core::SceneGraph::kVirtualRootId) {
                        node.set_local_model_matrix(manipulated_world);
                    } else {
                        const pbpt::math::Mat4 parent_world =
                            active_scene->scene_graph().node(node.parent_id()).world_matrix();
                        const pbpt::math::Mat4 local = pbpt::math::inverse(parent_world) * manipulated_world;
                        node.set_local_model_matrix(local);
                    }
                    break;
            }
        }

        return gizmo_using;
    }

    static void apply_scene_picking(
        EditorContext& ctx,
        const ActiveCameraInfo& active_camera,
        const EditorViewportRect& viewport_rect
    ) {
        auto* active_scene = ctx.world().active_scene();
        if (active_scene == nullptr) {
            return;
        }

        const auto pick_ray =
            make_scene_pick_ray(viewport_rect, ImGui::GetMousePos(), active_camera.view, active_camera.proj);
        if (!pick_ray.has_value()) {
            return;
        }

        const auto result = pick_scene_game_object(ctx.resources(), *active_scene, *pick_ray);
        if (!result.has_value()) {
            ctx.clear_selection();
            return;
        }

        ctx.set_selection(result->scene_id, result->game_object_id);
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
        bool scene_pick_requested = false;

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
            scene_pick_requested = hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left);

            viewport_rect = EditorViewportRect{
                .x = image_min.x,
                .y = image_min.y,
                .width = draw_size.x,
                .height = draw_size.y
            };

            if (const auto active_camera = find_active_camera(ctx); active_camera.has_value()) {
                gizmo_using = apply_gizmo_to_selection(ctx, *active_camera, viewport_rect);
                draw_selection_overlay(ctx, *active_camera, viewport_rect);
                if (scene_pick_requested && !ImGuizmo::IsOver() && !ImGuizmo::IsUsingAny()) {
                    apply_scene_picking(ctx, *active_camera, viewport_rect);
                }
            }
        } else {
            ImGui::TextDisabled("Scene texture is unavailable.");
        }

        publish_viewport_state(ctx, viewport_rect, hovered, scene_focused, gizmo_using);

        ImGui::End();
    }
};

} // namespace rtr::editor
