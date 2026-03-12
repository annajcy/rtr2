#pragma once

#include <pbpt/math/math.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

#include "imgui.h"

#include "rtr/editor/core/editor_panel.hpp"
#include "rtr/framework/component/camera_control/free_look_camera_controller.hpp"
#include "rtr/framework/component/camera_control/trackball_camera_controller.hpp"
#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/framework/component/physics/rigid_body/plane_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/box_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/mesh_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/reset_position.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/component/physics/rigid_body/sphere_collider.hpp"
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/light/point_light.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/types.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::editor {

class InspectorPanel final : public IEditorPanel {
private:
    static std::shared_ptr<spdlog::logger> logger() { return utils::get_logger("editor.panel.inspector"); }

    bool m_visible{true};
    int  m_order{200};

    static void draw_transform_editor(framework::core::GameObject& game_object) {
        auto node = game_object.node();

        pbpt::math::Vec3 local_position = node.local_position();
        if (ImGui::DragFloat3("Position", &local_position.x(), 0.05f)) {
            node.set_local_position(local_position);
            logger()->debug("Transform position updated (game_object_id={}, value=[{:.4f}, {:.4f}, {:.4f}]).",
                            game_object.id(), local_position.x(), local_position.y(), local_position.z());
        }

        pbpt::math::Vec3 local_euler = node.rotation_euler();
        if (ImGui::DragFloat3("Rotation (deg)", &local_euler.x(), 0.5f)) {
            node.set_local_rotation(pbpt::math::Quat(pbpt::math::radians(local_euler)));
            logger()->debug("Transform rotation updated (game_object_id={}, euler_deg=[{:.3f}, {:.3f}, {:.3f}]).",
                            game_object.id(), local_euler.x(), local_euler.y(), local_euler.z());
        }

        pbpt::math::Vec3 local_scale = node.local_scale();
        if (ImGui::DragFloat3("Scale", &local_scale.x(), 0.02f)) {
            local_scale.x() = std::max(local_scale.x(), 0.0001f);
            local_scale.y() = std::max(local_scale.y(), 0.0001f);
            local_scale.z() = std::max(local_scale.z(), 0.0001f);
            node.set_local_scale(local_scale);
            logger()->debug("Transform scale updated (game_object_id={}, value=[{:.4f}, {:.4f}, {:.4f}]).",
                            game_object.id(), local_scale.x(), local_scale.y(), local_scale.z());
        }
    }

    static void draw_camera_editor(framework::core::Scene& scene, framework::core::GameObject& game_object) {
        auto* camera = game_object.get_component<framework::component::Camera>();
        if (camera == nullptr) {
            return;
        }

        if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool enabled = camera->enabled();
            if (ImGui::Checkbox("Enabled##camera", &enabled)) {
                game_object.set_component_enabled<framework::component::Camera>(enabled);
                logger()->debug("Camera enabled updated (game_object_id={}, enabled={}).", game_object.id(), enabled);
            }

            bool active = camera->active();
            if (ImGui::Checkbox("Active", &active)) {
                if (active) {
                    for (const auto& go : scene.game_objects()) {
                        if (go == nullptr || go->id() == game_object.id()) {
                            continue;
                        }
                        if (auto* other_camera = go->get_component<framework::component::Camera>();
                            other_camera != nullptr) {
                            other_camera->set_active(false);
                        }
                    }
                }
                camera->set_active(active);
                logger()->debug("Camera active updated (game_object_id={}, active={}).", game_object.id(), active);
            }

            float near_bound = camera->near_bound();
            float far_bound  = camera->far_bound();
            if (ImGui::DragFloat("Near", &near_bound, 0.01f, 0.0001f, far_bound - 0.0001f)) {
                camera->near_bound() = near_bound;
                logger()->debug("Camera near updated (game_object_id={}, near={:.4f}).", game_object.id(), near_bound);
            }
            if (ImGui::DragFloat("Far", &far_bound, 0.05f, near_bound + 0.0001f, 5000.0f)) {
                camera->far_bound() = far_bound;
                logger()->debug("Camera far updated (game_object_id={}, far={:.4f}).", game_object.id(), far_bound);
            }

            if (auto* perspective = dynamic_cast<framework::component::PerspectiveCamera*>(camera);
                perspective != nullptr) {
                ImGui::Text("Type: Perspective");
                float fov = perspective->fov_degrees();
                if (ImGui::DragFloat("FOV (deg)", &fov, 0.1f, 1.0f, 179.0f)) {
                    perspective->fov_degrees() = fov;
                    logger()->debug("Perspective FOV updated (game_object_id={}, fov_deg={:.3f}).", game_object.id(),
                                    fov);
                }

                float aspect = perspective->aspect_ratio();
                if (ImGui::DragFloat("Aspect", &aspect, 0.01f, 0.1f, 10.0f)) {
                    perspective->aspect_ratio() = aspect;
                    logger()->debug("Perspective aspect updated (game_object_id={}, aspect={:.4f}).", game_object.id(),
                                    aspect);
                }
            } else if (auto* orthographic = dynamic_cast<framework::component::OrthographicCamera*>(camera);
                       orthographic != nullptr) {
                ImGui::Text("Type: Orthographic");
                float left   = orthographic->left_bound();
                float right  = orthographic->right_bound();
                float bottom = orthographic->bottom_bound();
                float top    = orthographic->top_bound();

                if (ImGui::DragFloat("Left", &left, 0.05f)) {
                    orthographic->left_bound() = left;
                    logger()->debug("Orthographic left updated (game_object_id={}, left={:.4f}).", game_object.id(),
                                    left);
                }
                if (ImGui::DragFloat("Right", &right, 0.05f)) {
                    orthographic->right_bound() = right;
                    logger()->debug("Orthographic right updated (game_object_id={}, right={:.4f}).", game_object.id(),
                                    right);
                }
                if (ImGui::DragFloat("Bottom", &bottom, 0.05f)) {
                    orthographic->bottom_bound() = bottom;
                    logger()->debug("Orthographic bottom updated (game_object_id={}, bottom={:.4f}).", game_object.id(),
                                    bottom);
                }
                if (ImGui::DragFloat("Top", &top, 0.05f)) {
                    orthographic->top_bound() = top;
                    logger()->debug("Orthographic top updated (game_object_id={}, top={:.4f}).", game_object.id(), top);
                }
            } else {
                ImGui::TextDisabled("Unknown camera type.");
            }
        }
    }

    static void draw_mesh_renderer_editor(framework::core::GameObject& game_object) {
        auto* mesh_renderer = game_object.get_component<framework::component::MeshRenderer>();
        if (mesh_renderer == nullptr) {
            return;
        }

        if (ImGui::CollapsingHeader("MeshRenderer", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool enabled = mesh_renderer->enabled();
            if (ImGui::Checkbox("Enabled##mesh_renderer", &enabled)) {
                game_object.set_component_enabled<framework::component::MeshRenderer>(enabled);
                logger()->debug("MeshRenderer enabled updated (game_object_id={}, enabled={}).", game_object.id(),
                                enabled);
            }

            pbpt::math::Vec4 base_color = mesh_renderer->base_color();
            if (ImGui::ColorEdit4("Base Color", &base_color.x())) {
                mesh_renderer->set_base_color(base_color);
                logger()->debug(
                    "MeshRenderer base_color updated (game_object_id={}, rgba=[{:.3f}, {:.3f}, {:.3f}, {:.3f}]).",
                    game_object.id(), base_color.x(), base_color.y(), base_color.z(), base_color.w());
            }

            ImGui::Text("Mesh Handle: %llu", static_cast<unsigned long long>(mesh_renderer->mesh_handle().value));
        }
    }

    static void draw_point_light_editor(framework::core::GameObject& game_object) {
        auto* point_light = game_object.get_component<framework::component::light::PointLight>();
        if (point_light == nullptr) {
            return;
        }

        if (ImGui::CollapsingHeader("PointLight", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool enabled = point_light->enabled();
            if (ImGui::Checkbox("Enabled##point_light", &enabled)) {
                game_object.set_component_enabled<framework::component::light::PointLight>(enabled);
                logger()->debug("PointLight enabled updated (game_object_id={}, enabled={}).", game_object.id(),
                                enabled);
            }

            pbpt::math::Vec3 color = point_light->color;
            if (ImGui::ColorEdit3("Color", &color.x())) {
                point_light->set_color(color);
            }

            float intensity = point_light->intensity;
            if (ImGui::DragFloat("Intensity", &intensity, 0.1f, 0.0f, 1000.0f)) {
                point_light->set_intensity(std::max(0.0f, intensity));
            }

            float range = point_light->range;
            if (ImGui::DragFloat("Range", &range, 0.5f, 0.1f, 1000.0f)) {
                point_light->set_range(std::max(0.1f, range));
            }

            float spec_strength = point_light->specular_strength;
            if (ImGui::DragFloat("Specular Strength", &spec_strength, 0.05f, 0.0f, 10.0f)) {
                point_light->set_specular_strength(std::max(0.0f, spec_strength));
            }

            float shininess = point_light->shininess;
            if (ImGui::DragFloat("Shininess", &shininess, 1.0f, 1.0f, 256.0f)) {
                point_light->set_shininess(std::max(1.0f, shininess));
            }
        }
    }

    static void draw_rigid_body_editor(framework::core::GameObject& game_object) {
        auto* rigid_body = game_object.get_component<framework::component::RigidBody>();
        if (rigid_body == nullptr) {
            return;
        }

        if (ImGui::CollapsingHeader("RigidBody", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool enabled = rigid_body->enabled();
            if (ImGui::Checkbox("Enabled##rigid_body", &enabled)) {
                game_object.set_component_enabled<framework::component::RigidBody>(enabled);
                logger()->debug("RigidBody enabled updated (game_object_id={}, enabled={}).", game_object.id(),
                                enabled);
            }

            ImGui::Text("RigidBody ID: %llu", static_cast<unsigned long long>(rigid_body->rigid_body_id()));
            if (!rigid_body->has_rigid_body()) {
                ImGui::TextDisabled("Physics body is not registered.");
                return;
            }

            pbpt::math::Vec3 position = rigid_body->position();
            if (ImGui::DragFloat3("Position##rigid_body", &position.x(), 0.05f)) {
                rigid_body->set_position(position);
                logger()->debug("RigidBody position updated (game_object_id={}, value=[{:.4f}, {:.4f}, {:.4f}]).",
                                game_object.id(), position.x(), position.y(), position.z());
            }

            bool use_gravity = rigid_body->use_gravity();
            if (ImGui::Checkbox("Use Gravity", &use_gravity)) {
                rigid_body->set_use_gravity(use_gravity);
                logger()->debug("RigidBody use_gravity updated (game_object_id={}, use_gravity={}).", game_object.id(),
                                use_gravity);
            }

            float mass = rigid_body->mass();
            if (ImGui::DragFloat("Mass", &mass, 0.01f, 0.001f, 1000.0f, "%.3f")) {
                mass = std::max(0.001f, mass);
                rigid_body->set_mass(mass);
                logger()->debug("RigidBody mass updated (game_object_id={}, mass={:.4f}).", game_object.id(), mass);
            }

            float restitution = rigid_body->restitution();
            if (ImGui::DragFloat("Restitution", &restitution, 0.01f, 0.0f, 1.0f)) {
                restitution = pbpt::math::clamp(restitution, 0.0f, 1.0f);
                rigid_body->set_restitution(restitution);
                logger()->debug("RigidBody restitution updated (game_object_id={}, restitution={:.4f}).",
                                game_object.id(), restitution);
            }

            float friction = rigid_body->friction();
            if (ImGui::DragFloat("Friction", &friction, 0.01f, 0.0f, 1000.0f)) {
                friction = std::max(0.0f, friction);
                rigid_body->set_friction(friction);
                logger()->debug("RigidBody friction updated (game_object_id={}, friction={:.4f}).",
                                game_object.id(), friction);
            }

            float linear_decay = rigid_body->linear_decay();
            if (ImGui::DragFloat("Linear Decay", &linear_decay, 0.001f, 0.0f, 1.0f)) {
                linear_decay = pbpt::math::clamp(linear_decay, 0.0f, 1.0f);
                rigid_body->set_linear_decay(linear_decay);
                logger()->debug("RigidBody linear_decay updated (game_object_id={}, linear_decay={:.4f}).",
                                game_object.id(), linear_decay);
            }

            float angular_decay = rigid_body->angular_decay();
            if (ImGui::DragFloat("Angular Decay", &angular_decay, 0.001f, 0.0f, 1.0f)) {
                angular_decay = pbpt::math::clamp(angular_decay, 0.0f, 1.0f);
                rigid_body->set_angular_decay(angular_decay);
                logger()->debug("RigidBody angular_decay updated (game_object_id={}, angular_decay={:.4f}).",
                                game_object.id(), angular_decay);
            }

            pbpt::math::Vec3 linear_velocity = rigid_body->linear_velocity();
            ImGui::Text("Linear Velocity: [%.4f, %.4f, %.4f]", linear_velocity.x(), linear_velocity.y(),
                        linear_velocity.z());

            const pbpt::math::Vec3 orientation_euler = pbpt::math::degrees(pbpt::math::euler_angles(rigid_body->orientation()));
            ImGui::Text("Orientation (deg): [%.3f, %.3f, %.3f]", orientation_euler.x(), orientation_euler.y(),
                        orientation_euler.z());

            const pbpt::math::Vec3 angular_velocity = rigid_body->angular_velocity();
            ImGui::Text("Angular Velocity: [%.4f, %.4f, %.4f]", angular_velocity.x(), angular_velocity.y(),
                        angular_velocity.z());

            pbpt::math::Mat3 inverse_inertia_tensor_ref = rigid_body->inverse_inertia_tensor_ref();
            pbpt::math::Vec3 inverse_inertia_diagonal = {
                inverse_inertia_tensor_ref[0][0],
                inverse_inertia_tensor_ref[1][1],
                inverse_inertia_tensor_ref[2][2],
            };
            if (ImGui::DragFloat3("Inv Inertia Diag", &inverse_inertia_diagonal.x(), 0.01f, 0.0f, 1000.0f)) {
                pbpt::math::Mat3 updated = pbpt::math::Mat3::zeros();
                updated[0][0] = std::max(0.0f, inverse_inertia_diagonal.x());
                updated[1][1] = std::max(0.0f, inverse_inertia_diagonal.y());
                updated[2][2] = std::max(0.0f, inverse_inertia_diagonal.z());
                rigid_body->set_inverse_inertia_tensor_ref(updated);
                logger()->debug(
                    "RigidBody inverse inertia diagonal updated (game_object_id={}, value=[{:.4f}, {:.4f}, {:.4f}]).",
                    game_object.id(), updated[0][0], updated[1][1], updated[2][2]);
            }
        }
    }

    static void draw_sphere_collider_gizmo_controls(EditorContext& ctx, framework::core::GameObject& game_object) {
        auto& gizmo = ctx.gizmo_state();
        const bool editing = gizmo.target == EditorGizmoTarget::SphereColliderLocal;
        if (editing) {
            ImGui::TextDisabled("Editing: SphereCollider local transform");
            if (ImGui::Button("Stop Editing Collider##sphere_collider")) {
                gizmo.target = EditorGizmoTarget::GameObjectTransform;
                logger()->debug("SphereCollider gizmo editing disabled (game_object_id={}).", game_object.id());
            }
        } else if (ImGui::Button("Edit With Gizmo##sphere_collider")) {
            gizmo.target = EditorGizmoTarget::SphereColliderLocal;
            logger()->debug("SphereCollider gizmo editing enabled (game_object_id={}).", game_object.id());
        }
    }

    static void draw_sphere_collider_editor(EditorContext& ctx, framework::core::GameObject& game_object) {
        auto* sphere = game_object.get_component<framework::component::SphereCollider>();
        if (sphere == nullptr) {
            return;
        }

        if (ImGui::CollapsingHeader("SphereCollider", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool enabled = sphere->enabled();
            if (ImGui::Checkbox("Enabled##sphere_collider", &enabled)) {
                game_object.set_component_enabled<framework::component::SphereCollider>(enabled);
                logger()->debug("SphereCollider enabled updated (game_object_id={}, enabled={}).", game_object.id(),
                                enabled);
            }

            float radius = sphere->radius();
            if (ImGui::DragFloat("Radius", &radius, 0.01f, 0.01f, 1000.0f)) {
                sphere->set_radius(std::max(0.01f, radius));
                logger()->debug("SphereCollider radius updated (game_object_id={}, radius={:.4f}).", game_object.id(),
                                sphere->radius());
            }

            draw_sphere_collider_gizmo_controls(ctx, game_object);

            pbpt::math::Vec3 local_position = sphere->local_position();
            if (ImGui::DragFloat3("Local Position##sphere_collider", &local_position.x(), 0.05f)) {
                sphere->set_local_position(local_position);
                logger()->debug(
                    "SphereCollider local_position updated (game_object_id={}, value=[{:.4f}, {:.4f}, {:.4f}]).",
                    game_object.id(), local_position.x(), local_position.y(), local_position.z());
            }

            pbpt::math::Vec3 local_euler = pbpt::math::degrees(pbpt::math::euler_angles(sphere->local_rotation()));
            if (ImGui::DragFloat3("Local Rotation (deg)##sphere_collider", &local_euler.x(), 0.5f)) {
                sphere->set_local_rotation(pbpt::math::Quat(pbpt::math::radians(local_euler)));
                logger()->debug(
                    "SphereCollider local_rotation updated (game_object_id={}, euler_deg=[{:.3f}, {:.3f}, {:.3f}]).",
                    game_object.id(), local_euler.x(), local_euler.y(), local_euler.z());
            }

            pbpt::math::Vec3 local_scale = sphere->local_scale();
            if (ImGui::DragFloat3("Local Scale##sphere_collider", &local_scale.x(), 0.02f, 0.001f, 1000.0f)) {
                local_scale.x() = std::max(local_scale.x(), 0.001f);
                local_scale.y() = std::max(local_scale.y(), 0.001f);
                local_scale.z() = std::max(local_scale.z(), 0.001f);
                sphere->set_local_scale(local_scale);
                logger()->debug(
                    "SphereCollider local_scale updated (game_object_id={}, value=[{:.4f}, {:.4f}, {:.4f}]).",
                    game_object.id(), local_scale.x(), local_scale.y(), local_scale.z());
            }
        }
    }

    static void draw_box_collider_gizmo_controls(EditorContext& ctx, framework::core::GameObject& game_object) {
        auto& gizmo = ctx.gizmo_state();
        const bool editing = gizmo.target == EditorGizmoTarget::BoxColliderLocal;
        if (editing) {
            ImGui::TextDisabled("Editing: BoxCollider local transform");
            if (ImGui::Button("Stop Editing Collider##box_collider")) {
                gizmo.target = EditorGizmoTarget::GameObjectTransform;
                logger()->debug("BoxCollider gizmo editing disabled (game_object_id={}).", game_object.id());
            }
        } else if (ImGui::Button("Edit With Gizmo##box_collider")) {
            gizmo.target = EditorGizmoTarget::BoxColliderLocal;
            logger()->debug("BoxCollider gizmo editing enabled (game_object_id={}).", game_object.id());
        }
    }

    static void draw_box_collider_editor(EditorContext& ctx, framework::core::GameObject& game_object) {
        auto* box = game_object.get_component<framework::component::BoxCollider>();
        if (box == nullptr) {
            return;
        }

        if (ImGui::CollapsingHeader("BoxCollider", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool enabled = box->enabled();
            if (ImGui::Checkbox("Enabled##box_collider", &enabled)) {
                game_object.set_component_enabled<framework::component::BoxCollider>(enabled);
                logger()->debug("BoxCollider enabled updated (game_object_id={}, enabled={}).", game_object.id(),
                                enabled);
            }

            pbpt::math::Vec3 half_extents = box->half_extents();
            if (ImGui::DragFloat3("Half Extents", &half_extents.x(), 0.05f, 0.01f, 1000.0f)) {
                half_extents.x() = std::max(0.01f, half_extents.x());
                half_extents.y() = std::max(0.01f, half_extents.y());
                half_extents.z() = std::max(0.01f, half_extents.z());
                box->set_half_extents(half_extents);
                logger()->debug(
                    "BoxCollider half_extents updated (game_object_id={}, value=[{:.4f}, {:.4f}, {:.4f}]).",
                    game_object.id(), half_extents.x(), half_extents.y(), half_extents.z());
            }

            draw_box_collider_gizmo_controls(ctx, game_object);

            pbpt::math::Vec3 local_position = box->local_position();
            if (ImGui::DragFloat3("Local Position##box_collider", &local_position.x(), 0.05f)) {
                box->set_local_position(local_position);
                logger()->debug(
                    "BoxCollider local_position updated (game_object_id={}, value=[{:.4f}, {:.4f}, {:.4f}]).",
                    game_object.id(), local_position.x(), local_position.y(), local_position.z());
            }

            pbpt::math::Vec3 local_euler = pbpt::math::degrees(pbpt::math::euler_angles(box->local_rotation()));
            if (ImGui::DragFloat3("Local Rotation (deg)", &local_euler.x(), 0.5f)) {
                box->set_local_rotation(pbpt::math::Quat(pbpt::math::radians(local_euler)));
                logger()->debug(
                    "BoxCollider local_rotation updated (game_object_id={}, euler_deg=[{:.3f}, {:.3f}, {:.3f}]).",
                    game_object.id(), local_euler.x(), local_euler.y(), local_euler.z());
            }

            pbpt::math::Vec3 local_scale = box->local_scale();
            if (ImGui::DragFloat3("Local Scale##box_collider", &local_scale.x(), 0.02f, 0.001f, 1000.0f)) {
                local_scale.x() = std::max(local_scale.x(), 0.001f);
                local_scale.y() = std::max(local_scale.y(), 0.001f);
                local_scale.z() = std::max(local_scale.z(), 0.001f);
                box->set_local_scale(local_scale);
                logger()->debug(
                    "BoxCollider local_scale updated (game_object_id={}, value=[{:.4f}, {:.4f}, {:.4f}]).",
                    game_object.id(), local_scale.x(), local_scale.y(), local_scale.z());
            }
        }
    }

    static void draw_plane_collider_editor(framework::core::GameObject& game_object) {
        auto* plane = game_object.get_component<framework::component::PlaneCollider>();
        if (plane == nullptr) {
            return;
        }

        if (ImGui::CollapsingHeader("PlaneCollider", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool enabled = plane->enabled();
            if (ImGui::Checkbox("Enabled##plane_collider", &enabled)) {
                game_object.set_component_enabled<framework::component::PlaneCollider>(enabled);
                logger()->debug("PlaneCollider enabled updated (game_object_id={}, enabled={}).", game_object.id(),
                                enabled);
            }

            pbpt::math::Vec3 normal_local = plane->normal_local();
            if (ImGui::DragFloat3("Normal##plane_collider", &normal_local.x(), 0.01f, -1.0f, 1.0f)) {
                const bool finite = std::isfinite(normal_local.x()) && std::isfinite(normal_local.y()) &&
                                    std::isfinite(normal_local.z());
                if (finite && pbpt::math::dot(normal_local, normal_local) > 1e-12f) {
                    plane->set_normal_local(normal_local);
                    normal_local = plane->normal_local();
                    logger()->debug(
                        "PlaneCollider normal_local updated (game_object_id={}, value=[{:.4f}, {:.4f}, {:.4f}]).",
                        game_object.id(), normal_local.x(), normal_local.y(), normal_local.z());
                }
            }

            pbpt::math::Vec3 local_position = plane->local_position();
            if (ImGui::DragFloat3("Local Position##plane_collider", &local_position.x(), 0.05f)) {
                plane->set_local_position(local_position);
                logger()->debug(
                    "PlaneCollider local_position updated (game_object_id={}, value=[{:.4f}, {:.4f}, {:.4f}]).",
                    game_object.id(), local_position.x(), local_position.y(), local_position.z());
            }

            pbpt::math::Vec3 local_euler = pbpt::math::degrees(pbpt::math::euler_angles(plane->local_rotation()));
            if (ImGui::DragFloat3("Local Rotation (deg)##plane_collider", &local_euler.x(), 0.5f)) {
                plane->set_local_rotation(pbpt::math::Quat(pbpt::math::radians(local_euler)));
                logger()->debug(
                    "PlaneCollider local_rotation updated (game_object_id={}, euler_deg=[{:.3f}, {:.3f}, {:.3f}]).",
                    game_object.id(), local_euler.x(), local_euler.y(), local_euler.z());
            }
        }
    }

    static void draw_mesh_collider_editor(framework::core::GameObject& game_object) {
        auto* mesh = game_object.get_component<framework::component::MeshCollider>();
        if (mesh == nullptr) {
            return;
        }

        if (ImGui::CollapsingHeader("MeshCollider", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool enabled = mesh->enabled();
            if (ImGui::Checkbox("Enabled##mesh_collider", &enabled)) {
                game_object.set_component_enabled<framework::component::MeshCollider>(enabled);
                logger()->debug("MeshCollider enabled updated (game_object_id={}, enabled={}).", game_object.id(),
                                enabled);
            }

            if (const auto* mesh_renderer = game_object.get_component<framework::component::MeshRenderer>();
                mesh_renderer != nullptr) {
                ImGui::Text("Mesh Handle: %llu",
                            static_cast<unsigned long long>(mesh_renderer->mesh_handle().value));
            } else {
                ImGui::TextDisabled("MeshRenderer is required on the same GameObject.");
            }

            pbpt::math::Vec3 local_position = mesh->local_position();
            if (ImGui::DragFloat3("Local Position##mesh_collider", &local_position.x(), 0.05f)) {
                mesh->set_local_position(local_position);
                logger()->debug(
                    "MeshCollider local_position updated (game_object_id={}, value=[{:.4f}, {:.4f}, {:.4f}]).",
                    game_object.id(), local_position.x(), local_position.y(), local_position.z());
            }

            pbpt::math::Vec3 local_euler = pbpt::math::degrees(pbpt::math::euler_angles(mesh->local_rotation()));
            if (ImGui::DragFloat3("Local Rotation (deg)##mesh_collider", &local_euler.x(), 0.5f)) {
                mesh->set_local_rotation(pbpt::math::Quat(pbpt::math::radians(local_euler)));
                logger()->debug(
                    "MeshCollider local_rotation updated (game_object_id={}, euler_deg=[{:.3f}, {:.3f}, {:.3f}]).",
                    game_object.id(), local_euler.x(), local_euler.y(), local_euler.z());
            }

            pbpt::math::Vec3 local_scale = mesh->local_scale();
            if (ImGui::DragFloat3("Local Scale##mesh_collider", &local_scale.x(), 0.02f, 0.001f, 1000.0f)) {
                local_scale.x() = std::max(0.001f, local_scale.x());
                local_scale.y() = std::max(0.001f, local_scale.y());
                local_scale.z() = std::max(0.001f, local_scale.z());
                mesh->set_local_scale(local_scale);
                logger()->debug(
                    "MeshCollider local_scale updated (game_object_id={}, value=[{:.4f}, {:.4f}, {:.4f}]).",
                    game_object.id(), local_scale.x(), local_scale.y(), local_scale.z());
            }
        }
    }

    static void draw_reset_position_editor(framework::core::GameObject& game_object) {
        auto* reset_position = game_object.get_component<framework::component::ResetPosition>();
        if (reset_position == nullptr) {
            return;
        }

        if (ImGui::CollapsingHeader("ResetPosition", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool enabled = reset_position->enabled();
            if (ImGui::Checkbox("Enabled##reset_position", &enabled)) {
                game_object.set_component_enabled<framework::component::ResetPosition>(enabled);
                logger()->debug("ResetPosition enabled updated (game_object_id={}, enabled={}).", game_object.id(),
                                enabled);
            }

            float threshold_y = reset_position->threshold_y();
            if (ImGui::DragFloat("Threshold Y", &threshold_y, 0.05f)) {
                reset_position->set_threshold_y(threshold_y);
                logger()->debug("ResetPosition threshold_y updated (game_object_id={}, threshold_y={:.4f}).",
                                game_object.id(), reset_position->threshold_y());
            }

            pbpt::math::Vec3 reset_target = reset_position->reset_position();
            if (ImGui::DragFloat3("Reset Position", &reset_target.x(), 0.05f)) {
                reset_position->set_reset_position(reset_target);
                logger()->debug(
                    "ResetPosition target updated (game_object_id={}, value=[{:.4f}, {:.4f}, {:.4f}]).",
                    game_object.id(), reset_target.x(), reset_target.y(), reset_target.z());
            }
        }
    }

    static void draw_free_look_editor(framework::core::GameObject& game_object) {
        auto* free_look = game_object.get_component<framework::component::FreeLookCameraController>();
        if (free_look == nullptr) {
            return;
        }

        if (ImGui::CollapsingHeader("FreeLook Controller", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool enabled = free_look->enabled();
            if (ImGui::Checkbox("Enabled##free_look", &enabled)) {
                game_object.set_component_enabled<framework::component::FreeLookCameraController>(enabled);
                logger()->debug("FreeLook enabled updated (game_object_id={}, enabled={}).", game_object.id(), enabled);
            }

            auto config = free_look->config();
            bool dirty  = false;
            dirty |= ImGui::DragFloat("Move Speed", &config.move_speed, 0.01f, 0.01f, 100.0f);
            dirty |= ImGui::DragFloat("Sprint Multiplier", &config.sprint_multiplier, 0.01f, 0.1f, 50.0f);
            dirty |= ImGui::DragFloat("Mouse Sensitivity", &config.mouse_sensitivity, 0.001f, 0.001f, 5.0f);
            dirty |= ImGui::DragFloat("Zoom Speed", &config.zoom_speed, 0.01f, 0.01f, 10.0f);
            dirty |= ImGui::DragFloat("Pitch Min", &config.pitch_min_degrees, 0.1f, -179.0f, 179.0f);
            dirty |= ImGui::DragFloat("Pitch Max", &config.pitch_max_degrees, 0.1f, -179.0f, 179.0f);

            if (config.pitch_min_degrees > config.pitch_max_degrees) {
                const float old_min = config.pitch_min_degrees;
                const float old_max = config.pitch_max_degrees;
                std::swap(config.pitch_min_degrees, config.pitch_max_degrees);
                dirty = true;
                logger()->debug(
                    "FreeLook pitch bounds corrected (game_object_id={}, old_min={:.3f}, old_max={:.3f}, "
                    "new_min={:.3f}, new_max={:.3f}).",
                    game_object.id(), old_min, old_max, config.pitch_min_degrees, config.pitch_max_degrees);
            }

            if (dirty) {
                free_look->set_config(config);
                logger()->debug(
                    "FreeLook config updated (game_object_id={}, move_speed={:.3f}, sprint_multiplier={:.3f}, "
                    "mouse_sensitivity={:.4f}, zoom_speed={:.3f}, pitch_min={:.3f}, pitch_max={:.3f}).",
                    game_object.id(), config.move_speed, config.sprint_multiplier, config.mouse_sensitivity,
                    config.zoom_speed, config.pitch_min_degrees, config.pitch_max_degrees);
            }
        }
    }

    static void draw_trackball_editor(framework::core::GameObject& game_object) {
        auto* trackball = game_object.get_component<framework::component::TrackBallCameraController>();
        if (trackball == nullptr) {
            return;
        }

        if (ImGui::CollapsingHeader("TrackBall Controller", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool enabled = trackball->enabled();
            if (ImGui::Checkbox("Enabled##trackball", &enabled)) {
                game_object.set_component_enabled<framework::component::TrackBallCameraController>(enabled);
                logger()->debug("TrackBall enabled updated (game_object_id={}, enabled={}).", game_object.id(),
                                enabled);
            }

            auto config = trackball->config();
            bool dirty  = false;
            dirty |= ImGui::DragFloat("Rotate Speed", &config.rotate_speed, 0.001f, 0.001f, 10.0f);
            dirty |= ImGui::DragFloat("Pan Speed", &config.pan_speed, 0.0001f, 0.0001f, 1.0f);
            dirty |= ImGui::DragFloat("Zoom Speed##trackball", &config.zoom_speed, 0.01f, 0.01f, 20.0f);
            dirty |= ImGui::DragFloat("Pitch Min##trackball", &config.pitch_min_degrees, 0.1f, -179.0f, 179.0f);
            dirty |= ImGui::DragFloat("Pitch Max##trackball", &config.pitch_max_degrees, 0.1f, -179.0f, 179.0f);
            dirty |= ImGui::DragFloat3("World Up", &config.world_up.x(), 0.01f, -1.0f, 1.0f);

            if (config.pitch_min_degrees > config.pitch_max_degrees) {
                const float old_min = config.pitch_min_degrees;
                const float old_max = config.pitch_max_degrees;
                std::swap(config.pitch_min_degrees, config.pitch_max_degrees);
                dirty = true;
                logger()->debug(
                    "TrackBall pitch bounds corrected (game_object_id={}, old_min={:.3f}, old_max={:.3f}, "
                    "new_min={:.3f}, new_max={:.3f}).",
                    game_object.id(), old_min, old_max, config.pitch_min_degrees, config.pitch_max_degrees);
            }
            if (pbpt::math::length(config.world_up) <= 1e-6f) {
                const pbpt::math::Vec3 old_world_up = config.world_up;
                config.world_up                     = pbpt::math::Vec3{0.0f, 1.0f, 0.0f};
                dirty                               = true;
                logger()->debug(
                    "TrackBall world_up corrected (game_object_id={}, old=[{:.3f}, {:.3f}, {:.3f}], new=[{:.3f}, "
                    "{:.3f}, {:.3f}]).",
                    game_object.id(), old_world_up.x(), old_world_up.y(), old_world_up.z(), config.world_up.x(),
                    config.world_up.y(), config.world_up.z());
            }

            if (dirty) {
                trackball->set_config(config);
                logger()->debug(
                    "TrackBall config updated (game_object_id={}, rotate_speed={:.4f}, pan_speed={:.5f}, "
                    "zoom_speed={:.3f}, pitch_min={:.3f}, pitch_max={:.3f}, world_up=[{:.3f}, {:.3f}, {:.3f}]).",
                    game_object.id(), config.rotate_speed, config.pan_speed, config.zoom_speed,
                    config.pitch_min_degrees, config.pitch_max_degrees, config.world_up.x(), config.world_up.y(),
                    config.world_up.z());
            }

            pbpt::math::Vec3 target = trackball->target();
            if (ImGui::DragFloat3("Target", &target.x(), 0.05f)) {
                trackball->set_target(target);
                logger()->debug("TrackBall target updated (game_object_id={}, target=[{:.4f}, {:.4f}, {:.4f}]).",
                                game_object.id(), target.x(), target.y(), target.z());
            }
        }
    }

public:
    std::string_view id() const override { return "inspector"; }

    int order() const override { return m_order; }

    bool visible() const override { return m_visible; }

    void set_visible(bool visible) override { m_visible = visible; }

    void on_imgui(EditorContext& ctx) override {
        if (!m_visible) {
            return;
        }

        if (!ImGui::Begin("Inspector", &m_visible)) {
            ImGui::End();
            return;
        }

        const auto& selection = ctx.selection();
        if (!selection.has_game_object()) {
            ImGui::TextDisabled("No GameObject selected.");
            ImGui::End();
            return;
        }

        auto* scene = ctx.world().find_scene(selection.scene_id);
        if (scene == nullptr) {
            ImGui::TextDisabled("Selected scene no longer exists.");
            ImGui::End();
            return;
        }

        auto* game_object = scene->find_game_object(selection.game_object_id);
        if (game_object == nullptr) {
            ImGui::TextDisabled("Selected GameObject no longer exists.");
            ImGui::End();
            return;
        }

        ImGui::Text("GameObject #%llu", static_cast<unsigned long long>(game_object->id()));

        const std::string current_name = std::string(scene->game_object_name(game_object->id()).value_or("GameObject"));
        std::array<char, 256> name_buffer{};
        std::snprintf(name_buffer.data(), name_buffer.size(), "%s", current_name.c_str());
        if (ImGui::InputText("Name", name_buffer.data(), name_buffer.size())) {
            if (scene->rename_game_object(game_object->id(), name_buffer.data())) {
                const std::string updated_name =
                    std::string(scene->game_object_name(game_object->id()).value_or("GameObject"));
                logger()->debug("GameObject name updated (game_object_id={}, name='{}').", game_object->id(),
                                updated_name);
            }
        }

        bool enabled = game_object->enabled();
        if (ImGui::Checkbox("Enabled##game_object", &enabled)) {
            game_object->set_enabled(enabled);
            logger()->debug("GameObject enabled updated (game_object_id={}, enabled={}).", game_object->id(), enabled);
        }

        if (ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen)) {
            draw_transform_editor(*game_object);
        }

        draw_camera_editor(*scene, *game_object);
        draw_mesh_renderer_editor(*game_object);
        draw_point_light_editor(*game_object);
        draw_rigid_body_editor(*game_object);
        draw_sphere_collider_editor(ctx, *game_object);
        draw_box_collider_editor(ctx, *game_object);
        draw_plane_collider_editor(*game_object);
        draw_mesh_collider_editor(*game_object);
        draw_reset_position_editor(*game_object);
        draw_free_look_editor(*game_object);
        draw_trackball_editor(*game_object);

        ImGui::End();
    }
};

}  // namespace rtr::editor
