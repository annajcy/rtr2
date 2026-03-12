#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_map>

#include <pbpt/geometry/ray.hpp>
#include <pbpt/math/math.h>

#include "imgui.h"

#include "rtr/editor/core/editor_types.hpp"
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/physics/rigid_body/box_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/sphere_collider.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/resource/resource_kinds.hpp"
#include "rtr/resource/resource_manager.hpp"

namespace rtr::editor {

enum class ScenePickHitSource {
    SphereCollider,
    BoxCollider,
    MeshBounds,
};

struct ScenePickResult {
    framework::core::SceneId scene_id{framework::core::kInvalidSceneId};
    framework::core::GameObjectId game_object_id{framework::core::kInvalidGameObjectId};
    float distance{0.0f};
    ScenePickHitSource source{ScenePickHitSource::MeshBounds};
};

struct ScenePickRay {
    pbpt::math::Pt3 origin{0.0f, 0.0f, 0.0f};
    pbpt::math::Vec3 direction{0.0f, 0.0f, -1.0f};
};

struct LocalMeshBounds {
    pbpt::math::Vec3 min{0.0f, 0.0f, 0.0f};
    pbpt::math::Vec3 max{0.0f, 0.0f, 0.0f};
    bool valid{false};
};

namespace detail {

inline std::optional<float> intersect_ray_sphere(
    const ScenePickRay& ray,
    const pbpt::math::Vec3& center,
    float radius
) {
    const pbpt::math::Vec3 to_center = ray.origin - pbpt::math::Pt3(center.x(), center.y(), center.z());
    const float b = 2.0f * pbpt::math::dot(ray.direction, to_center);
    const float c = pbpt::math::dot(to_center, to_center) - radius * radius;
    const float discriminant = b * b - 4.0f * c;
    if (discriminant < 0.0f) {
        return std::nullopt;
    }

    const float sqrt_d = std::sqrt(discriminant);
    const float t0 = (-b - sqrt_d) * 0.5f;
    const float t1 = (-b + sqrt_d) * 0.5f;
    if (t0 >= 0.0f) {
        return t0;
    }
    if (t1 >= 0.0f) {
        return t1;
    }
    return std::nullopt;
}

inline std::optional<float> intersect_ray_aabb(
    const pbpt::math::Pt3& origin,
    const pbpt::math::Vec3& direction,
    const pbpt::math::Vec3& min,
    const pbpt::math::Vec3& max
) {
    constexpr float kEpsilon = 1e-6f;
    float t_min = 0.0f;
    float t_max = std::numeric_limits<float>::infinity();

    for (int axis = 0; axis < 3; ++axis) {
        const float dir = direction[axis];
        const float org = origin[axis];
        if (std::abs(dir) < kEpsilon) {
            if (org < min[axis] || org > max[axis]) {
                return std::nullopt;
            }
            continue;
        }

        const float inv_dir = 1.0f / dir;
        float t0 = (min[axis] - org) * inv_dir;
        float t1 = (max[axis] - org) * inv_dir;
        if (t0 > t1) {
            std::swap(t0, t1);
        }
        t_min = std::max(t_min, t0);
        t_max = std::min(t_max, t1);
        if (t_min > t_max) {
            return std::nullopt;
        }
    }

    return t_min;
}

inline LocalMeshBounds compute_local_mesh_bounds(
    resource::ResourceManager& resources,
    resource::MeshHandle mesh_handle
) {
    static std::unordered_map<std::uint64_t, LocalMeshBounds> cache{};

    if (!mesh_handle.is_valid()) {
        return {};
    }

    if (const auto it = cache.find(mesh_handle.value); it != cache.end()) {
        return it->second;
    }

    const auto& mesh = resources.cpu<resource::MeshResourceKind>(mesh_handle);
    if (mesh.vertices.empty()) {
        return {};
    }

    LocalMeshBounds bounds{};
    bounds.min = mesh.vertices.front().position;
    bounds.max = mesh.vertices.front().position;
    for (const auto& vertex : mesh.vertices) {
        bounds.min.x() = std::min(bounds.min.x(), vertex.position.x());
        bounds.min.y() = std::min(bounds.min.y(), vertex.position.y());
        bounds.min.z() = std::min(bounds.min.z(), vertex.position.z());
        bounds.max.x() = std::max(bounds.max.x(), vertex.position.x());
        bounds.max.y() = std::max(bounds.max.y(), vertex.position.y());
        bounds.max.z() = std::max(bounds.max.z(), vertex.position.z());
    }
    bounds.valid = true;
    cache.emplace(mesh_handle.value, bounds);
    return bounds;
}

inline std::optional<float> intersect_world_box(
    const ScenePickRay& ray,
    const pbpt::math::Vec3& center,
    const pbpt::math::Quat& rotation,
    const pbpt::math::Vec3& half_extents
) {
    const pbpt::math::Quat inv_rotation = rotation.inversed();
    const pbpt::math::Pt3 local_origin = pbpt::math::Pt3(inv_rotation * (ray.origin - pbpt::math::Pt3(center.x(), center.y(), center.z())));
    const pbpt::math::Vec3 local_direction = inv_rotation * ray.direction;
    return intersect_ray_aabb(local_origin, local_direction, -half_extents, half_extents);
}

inline std::optional<float> intersect_mesh_bounds(
    const ScenePickRay& ray,
    const pbpt::math::Mat4& world_matrix,
    const LocalMeshBounds& bounds
) {
    if (!bounds.valid) {
        return std::nullopt;
    }

    const pbpt::math::Mat4 inv_world = pbpt::math::inverse(world_matrix);

    const pbpt::math::Vec4 origin_h(ray.origin.x(), ray.origin.y(), ray.origin.z(), 1.0f);
    const pbpt::math::Vec4 target_h(
        ray.origin.x() + ray.direction.x(),
        ray.origin.y() + ray.direction.y(),
        ray.origin.z() + ray.direction.z(),
        1.0f
    );
    const pbpt::math::Vec4 local_origin_h = inv_world * origin_h;
    const pbpt::math::Vec4 local_target_h = inv_world * target_h;

    const pbpt::math::Pt3 local_origin(local_origin_h.x(), local_origin_h.y(), local_origin_h.z());
    const pbpt::math::Pt3 local_target(local_target_h.x(), local_target_h.y(), local_target_h.z());
    const pbpt::math::Vec3 local_direction = (local_target - local_origin).normalized();

    const auto local_t = intersect_ray_aabb(local_origin, local_direction, bounds.min, bounds.max);
    if (!local_t.has_value()) {
        return std::nullopt;
    }

    const pbpt::math::Pt3 local_hit = local_origin + local_direction * *local_t;
    const pbpt::math::Vec4 local_hit_h(local_hit.x(), local_hit.y(), local_hit.z(), 1.0f);
    const pbpt::math::Vec4 world_hit_h = world_matrix * local_hit_h;
    const pbpt::math::Pt3 world_hit(world_hit_h.x(), world_hit_h.y(), world_hit_h.z());
    return pbpt::math::length(world_hit - ray.origin);
}

} // namespace detail

inline std::optional<ScenePickRay> make_scene_pick_ray(
    const EditorViewportRect& viewport_rect,
    const ImVec2& mouse_pos,
    const pbpt::math::Mat4& view,
    const pbpt::math::Mat4& proj
) {
    if (!viewport_rect.valid()) {
        return std::nullopt;
    }

    const float u = (mouse_pos.x - viewport_rect.x) / viewport_rect.width;
    const float v = (mouse_pos.y - viewport_rect.y) / viewport_rect.height;
    if (u < 0.0f || u > 1.0f || v < 0.0f || v > 1.0f) {
        return std::nullopt;
    }

    const float ndc_x = u * 2.0f - 1.0f;
    const float ndc_y = v * 2.0f - 1.0f;

    const pbpt::math::Mat4 inv_view_proj = pbpt::math::inverse(proj * view);
    pbpt::math::Vec4 near_h = inv_view_proj * pbpt::math::Vec4(ndc_x, ndc_y, 0.0f, 1.0f);
    pbpt::math::Vec4 far_h = inv_view_proj * pbpt::math::Vec4(ndc_x, ndc_y, 1.0f, 1.0f);

    if (std::abs(near_h.w()) < 1e-6f || std::abs(far_h.w()) < 1e-6f) {
        return std::nullopt;
    }

    near_h /= near_h.w();
    far_h /= far_h.w();

    const pbpt::math::Pt3 near_point(near_h.x(), near_h.y(), near_h.z());
    const pbpt::math::Pt3 far_point(far_h.x(), far_h.y(), far_h.z());
    const pbpt::math::Vec3 direction = (far_point - near_point).normalized();

    return ScenePickRay{
        .origin = near_point,
        .direction = direction
    };
}

inline std::optional<ScenePickResult> pick_scene_game_object(
    resource::ResourceManager& resources,
    framework::core::Scene& scene,
    const ScenePickRay& ray
) {
    std::optional<ScenePickResult> best{};
    constexpr float kDistanceTieEpsilon = 1e-4f;

    const auto prefer_candidate = [&](float distance, ScenePickHitSource source) {
        if (!best.has_value()) {
            return true;
        }
        if (distance + kDistanceTieEpsilon < best->distance) {
            return true;
        }
        if (std::abs(distance - best->distance) <= kDistanceTieEpsilon) {
            const bool best_is_mesh = best->source == ScenePickHitSource::MeshBounds;
            const bool source_is_mesh = source == ScenePickHitSource::MeshBounds;
            if (best_is_mesh && !source_is_mesh) {
                return true;
            }
        }
        return false;
    };

    for (const auto node_id : scene.scene_graph().active_nodes()) {
        auto* game_object = scene.find_game_object(node_id);
        if (game_object == nullptr || !game_object->enabled()) {
            continue;
        }

        std::optional<float> hit_distance{};
        ScenePickHitSource hit_source = ScenePickHitSource::MeshBounds;

        if (const auto* sphere = game_object->get_component<framework::component::SphereCollider>();
            sphere != nullptr && sphere->enabled()) {
            const auto node = game_object->node();
            const pbpt::math::Vec3 scale = sphere->local_scale() * node.world_scale();
            const pbpt::math::Vec3 center =
                node.world_position() + node.world_rotation() * (sphere->local_position() * node.world_scale());
            const float radius = sphere->radius() * scale.abs().max();
            hit_distance = detail::intersect_ray_sphere(ray, center, radius);
            hit_source = ScenePickHitSource::SphereCollider;
        } else if (const auto* box = game_object->get_component<framework::component::BoxCollider>();
                   box != nullptr && box->enabled()) {
            const auto node = game_object->node();
            const pbpt::math::Vec3 scale = (box->local_scale() * node.world_scale()).abs();
            const pbpt::math::Vec3 center =
                node.world_position() + node.world_rotation() * (box->local_position() * node.world_scale());
            const pbpt::math::Quat rotation = pbpt::math::normalize(node.world_rotation() * box->local_rotation());
            const pbpt::math::Vec3 half_extents = box->half_extents() * scale;
            hit_distance = detail::intersect_world_box(ray, center, rotation, half_extents);
            hit_source = ScenePickHitSource::BoxCollider;
        } else if (const auto* mesh_renderer = game_object->get_component<framework::component::MeshRenderer>();
                   mesh_renderer != nullptr && mesh_renderer->enabled() && mesh_renderer->mesh_handle().is_valid()) {
            const LocalMeshBounds bounds = detail::compute_local_mesh_bounds(resources, mesh_renderer->mesh_handle());
            hit_distance = detail::intersect_mesh_bounds(ray, game_object->node().world_matrix(), bounds);
            hit_source = ScenePickHitSource::MeshBounds;
        }

        if (!hit_distance.has_value() || *hit_distance < 0.0f) {
            continue;
        }

        if (!prefer_candidate(*hit_distance, hit_source)) {
            continue;
        }

        best = ScenePickResult{
            .scene_id = scene.id(),
            .game_object_id = game_object->id(),
            .distance = *hit_distance,
            .source = hit_source
        };
    }

    return best;
}

} // namespace rtr::editor
