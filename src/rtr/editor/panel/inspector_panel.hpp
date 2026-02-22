#pragma once

#include <pbpt/math/math.h>

#include <algorithm>
#include <array>
#include <cstdio>


#include "imgui.h"

#include "rtr/editor/core/editor_panel.hpp"
#include "rtr/framework/component/camera_control/free_look_camera_controller.hpp"
#include "rtr/framework/component/camera_control/trackball_camera_controller.hpp"
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/core/camera.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/types.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::editor {

class InspectorPanel final : public IEditorPanel {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("editor.panel.inspector");
    }

    bool m_visible{true};
    int m_order{200};

    static void draw_transform_editor(framework::core::GameObject& game_object) {
        auto node = game_object.node();

        pbpt::math::vec3 local_position = node.local_position();
        if (ImGui::DragFloat3("Position", &local_position.x(), 0.05f)) {
            node.set_local_position(local_position);
            logger()->debug(
                "Transform position updated (game_object_id={}, value=[{:.4f}, {:.4f}, {:.4f}]).",
                game_object.id(),
                local_position.x(),
                local_position.y(),
                local_position.z()
            );
        }

        pbpt::math::vec3 local_euler = node.rotation_euler();
        if (ImGui::DragFloat3("Rotation (deg)", &local_euler.x(), 0.5f)) {
            node.set_local_rotation(pbpt::math::quat(pbpt::math::radians(local_euler)));
            logger()->debug(
                "Transform rotation updated (game_object_id={}, euler_deg=[{:.3f}, {:.3f}, {:.3f}]).",
                game_object.id(),
                local_euler.x(),
                local_euler.y(),
                local_euler.z()
            );
        }

        pbpt::math::vec3 local_scale = node.local_scale();
        if (ImGui::DragFloat3("Scale", &local_scale.x(), 0.02f)) {
            local_scale.x() = std::max(local_scale.x(), 0.0001f);
            local_scale.y() = std::max(local_scale.y(), 0.0001f);
            local_scale.z() = std::max(local_scale.z(), 0.0001f);
            node.set_local_scale(local_scale);
            logger()->debug(
                "Transform scale updated (game_object_id={}, value=[{:.4f}, {:.4f}, {:.4f}]).",
                game_object.id(),
                local_scale.x(),
                local_scale.y(),
                local_scale.z()
            );
        }
    }

    static void draw_camera_editor(framework::core::Scene& scene, framework::core::GameObject& game_object) {
        auto* camera = scene.camera_manager().camera(game_object.id());
        if (camera == nullptr) {
            return;
        }

        if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("Type: %s", camera->camera_type() == framework::core::CameraType::Perspective ? "Perspective" : "Orthographic");

            float near_bound = camera->near_bound();
            float far_bound = camera->far_bound();
            if (ImGui::DragFloat("Near", &near_bound, 0.01f, 0.0001f, far_bound - 0.0001f)) {
                camera->near_bound() = near_bound;
                logger()->debug(
                    "Camera near updated (game_object_id={}, near={:.4f}).",
                    game_object.id(),
                    near_bound
                );
            }
            if (ImGui::DragFloat("Far", &far_bound, 0.05f, near_bound + 0.0001f, 5000.0f)) {
                camera->far_bound() = far_bound;
                logger()->debug(
                    "Camera far updated (game_object_id={}, far={:.4f}).",
                    game_object.id(),
                    far_bound
                );
            }

            if (auto* perspective = dynamic_cast<framework::core::PerspectiveCamera*>(camera);
                perspective != nullptr) {
                float fov = perspective->fov_degrees();
                if (ImGui::DragFloat("FOV (deg)", &fov, 0.1f, 1.0f, 179.0f)) {
                    perspective->fov_degrees() = fov;
                    logger()->debug(
                        "Perspective FOV updated (game_object_id={}, fov_deg={:.3f}).",
                        game_object.id(),
                        fov
                    );
                }

                float aspect = perspective->aspect_ratio();
                if (ImGui::DragFloat("Aspect", &aspect, 0.01f, 0.1f, 10.0f)) {
                    perspective->set_aspect_ratio(aspect);
                    logger()->debug(
                        "Perspective aspect updated (game_object_id={}, aspect={:.4f}).",
                        game_object.id(),
                        aspect
                    );
                }
            } else if (auto* orthographic = dynamic_cast<framework::core::OrthographicCamera*>(camera);
                       orthographic != nullptr) {
                float left = orthographic->left_bound();
                float right = orthographic->right_bound();
                float bottom = orthographic->bottom_bound();
                float top = orthographic->top_bound();

                if (ImGui::DragFloat("Left", &left, 0.05f)) {
                    orthographic->left_bound() = left;
                    logger()->debug(
                        "Orthographic left updated (game_object_id={}, left={:.4f}).",
                        game_object.id(),
                        left
                    );
                }
                if (ImGui::DragFloat("Right", &right, 0.05f)) {
                    orthographic->right_bound() = right;
                    logger()->debug(
                        "Orthographic right updated (game_object_id={}, right={:.4f}).",
                        game_object.id(),
                        right
                    );
                }
                if (ImGui::DragFloat("Bottom", &bottom, 0.05f)) {
                    orthographic->bottom_bound() = bottom;
                    logger()->debug(
                        "Orthographic bottom updated (game_object_id={}, bottom={:.4f}).",
                        game_object.id(),
                        bottom
                    );
                }
                if (ImGui::DragFloat("Top", &top, 0.05f)) {
                    orthographic->top_bound() = top;
                    logger()->debug(
                        "Orthographic top updated (game_object_id={}, top={:.4f}).",
                        game_object.id(),
                        top
                    );
                }
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
                mesh_renderer->set_enabled(enabled);
                logger()->debug(
                    "MeshRenderer enabled updated (game_object_id={}, enabled={}).",
                    game_object.id(),
                    enabled
                );
            }

            pbpt::math::vec4 base_color = mesh_renderer->base_color();
            if (ImGui::ColorEdit4("Base Color", &base_color.x())) {
                mesh_renderer->set_base_color(base_color);
                logger()->debug(
                    "MeshRenderer base_color updated (game_object_id={}, rgba=[{:.3f}, {:.3f}, {:.3f}, {:.3f}]).",
                    game_object.id(),
                    base_color.x(),
                    base_color.y(),
                    base_color.z(),
                    base_color.w()
                );
            }

            ImGui::Text(
                "Mesh Handle: %llu",
                static_cast<unsigned long long>(mesh_renderer->mesh_handle().value)
            );
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
                free_look->set_enabled(enabled);
                logger()->debug(
                    "FreeLook enabled updated (game_object_id={}, enabled={}).",
                    game_object.id(),
                    enabled
                );
            }

            auto config = free_look->config();
            bool dirty = false;
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
                    "FreeLook pitch bounds corrected (game_object_id={}, old_min={:.3f}, old_max={:.3f}, new_min={:.3f}, new_max={:.3f}).",
                    game_object.id(),
                    old_min,
                    old_max,
                    config.pitch_min_degrees,
                    config.pitch_max_degrees
                );
            }

            if (dirty) {
                free_look->set_config(config);
                logger()->debug(
                    "FreeLook config updated (game_object_id={}, move_speed={:.3f}, sprint_multiplier={:.3f}, mouse_sensitivity={:.4f}, zoom_speed={:.3f}, pitch_min={:.3f}, pitch_max={:.3f}).",
                    game_object.id(),
                    config.move_speed,
                    config.sprint_multiplier,
                    config.mouse_sensitivity,
                    config.zoom_speed,
                    config.pitch_min_degrees,
                    config.pitch_max_degrees
                );
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
                trackball->set_enabled(enabled);
                logger()->debug(
                    "TrackBall enabled updated (game_object_id={}, enabled={}).",
                    game_object.id(),
                    enabled
                );
            }

            auto config = trackball->config();
            bool dirty = false;
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
                    "TrackBall pitch bounds corrected (game_object_id={}, old_min={:.3f}, old_max={:.3f}, new_min={:.3f}, new_max={:.3f}).",
                    game_object.id(),
                    old_min,
                    old_max,
                    config.pitch_min_degrees,
                    config.pitch_max_degrees
                );
            }
            if (pbpt::math::length(config.world_up) <= 1e-6f) {
                const pbpt::math::vec3 old_world_up = config.world_up;
                config.world_up = pbpt::math::vec3{0.0f, 1.0f, 0.0f};
                dirty = true;
                logger()->debug(
                    "TrackBall world_up corrected (game_object_id={}, old=[{:.3f}, {:.3f}, {:.3f}], new=[{:.3f}, {:.3f}, {:.3f}]).",
                    game_object.id(),
                    old_world_up.x(),
                    old_world_up.y(),
                    old_world_up.z(),
                    config.world_up.x(),
                    config.world_up.y(),
                    config.world_up.z()
                );
            }

            if (dirty) {
                trackball->set_config(config);
                logger()->debug(
                    "TrackBall config updated (game_object_id={}, rotate_speed={:.4f}, pan_speed={:.5f}, zoom_speed={:.3f}, pitch_min={:.3f}, pitch_max={:.3f}, world_up=[{:.3f}, {:.3f}, {:.3f}]).",
                    game_object.id(),
                    config.rotate_speed,
                    config.pan_speed,
                    config.zoom_speed,
                    config.pitch_min_degrees,
                    config.pitch_max_degrees,
                    config.world_up.x(),
                    config.world_up.y(),
                    config.world_up.z()
                );
            }

            pbpt::math::vec3 target = trackball->target();
            if (ImGui::DragFloat3("Target", &target.x(), 0.05f)) {
                trackball->set_target(target);
                logger()->debug(
                    "TrackBall target updated (game_object_id={}, target=[{:.4f}, {:.4f}, {:.4f}]).",
                    game_object.id(),
                    target.x(),
                    target.y(),
                    target.z()
                );
            }
        }
    }

public:
    std::string_view id() const override {
        return "inspector";
    }

    int order() const override {
        return m_order;
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

        ImGui::Text(
            "GameObject #%llu",
            static_cast<unsigned long long>(game_object->id())
        );

        std::array<char, 256> name_buffer{};
        std::snprintf(name_buffer.data(), name_buffer.size(), "%s", game_object->name().c_str());
        if (ImGui::InputText("Name", name_buffer.data(), name_buffer.size())) {
            game_object->set_name(name_buffer.data());
            logger()->debug(
                "GameObject name updated (game_object_id={}, name='{}').",
                game_object->id(),
                game_object->name()
            );
        }

        bool enabled = game_object->enabled();
        if (ImGui::Checkbox("Enabled##game_object", &enabled)) {
            game_object->set_enabled(enabled);
            logger()->debug(
                "GameObject enabled updated (game_object_id={}, enabled={}).",
                game_object->id(),
                enabled
            );
        }

        if (ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen)) {
            draw_transform_editor(*game_object);
        }

        draw_camera_editor(*scene, *game_object);
        draw_mesh_renderer_editor(*game_object);
        draw_free_look_editor(*game_object);
        draw_trackball_editor(*game_object);

        ImGui::End();
    }
};

} // namespace rtr::editor
