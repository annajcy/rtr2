#pragma once

#include <algorithm>
#include <array>
#include <cstdio>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/trigonometric.hpp>

#include "imgui.h"

#include "rtr/editor/editor_panel.hpp"
#include "rtr/framework/component/camera_control/free_look_camera_controller.hpp"
#include "rtr/framework/component/camera_control/trackball_camera_controller.hpp"
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/core/camera.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/types.hpp"

namespace rtr::editor {

class InspectorPanel final : public IEditorPanel {
private:
    bool m_visible{true};
    int m_order{200};

    static void draw_transform_editor(framework::core::GameObject& game_object) {
        auto node = game_object.node();

        glm::vec3 local_position = node.local_position();
        if (ImGui::DragFloat3("Position", &local_position.x, 0.05f)) {
            node.set_local_position(local_position);
        }

        glm::vec3 local_euler = node.rotation_euler();
        if (ImGui::DragFloat3("Rotation (deg)", &local_euler.x, 0.5f)) {
            node.set_local_rotation(glm::quat(glm::radians(local_euler)));
        }

        glm::vec3 local_scale = node.local_scale();
        if (ImGui::DragFloat3("Scale", &local_scale.x, 0.02f)) {
            local_scale.x = std::max(local_scale.x, 0.0001f);
            local_scale.y = std::max(local_scale.y, 0.0001f);
            local_scale.z = std::max(local_scale.z, 0.0001f);
            node.set_local_scale(local_scale);
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
            }
            if (ImGui::DragFloat("Far", &far_bound, 0.05f, near_bound + 0.0001f, 5000.0f)) {
                camera->far_bound() = far_bound;
            }

            if (auto* perspective = dynamic_cast<framework::core::PerspectiveCamera*>(camera);
                perspective != nullptr) {
                float fov = perspective->fov_degrees();
                if (ImGui::DragFloat("FOV (deg)", &fov, 0.1f, 1.0f, 179.0f)) {
                    perspective->fov_degrees() = fov;
                }

                float aspect = perspective->aspect_ratio();
                if (ImGui::DragFloat("Aspect", &aspect, 0.01f, 0.1f, 10.0f)) {
                    perspective->set_aspect_ratio(aspect);
                }
            } else if (auto* orthographic = dynamic_cast<framework::core::OrthographicCamera*>(camera);
                       orthographic != nullptr) {
                float left = orthographic->left_bound();
                float right = orthographic->right_bound();
                float bottom = orthographic->bottom_bound();
                float top = orthographic->top_bound();

                if (ImGui::DragFloat("Left", &left, 0.05f)) {
                    orthographic->left_bound() = left;
                }
                if (ImGui::DragFloat("Right", &right, 0.05f)) {
                    orthographic->right_bound() = right;
                }
                if (ImGui::DragFloat("Bottom", &bottom, 0.05f)) {
                    orthographic->bottom_bound() = bottom;
                }
                if (ImGui::DragFloat("Top", &top, 0.05f)) {
                    orthographic->top_bound() = top;
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
            }

            glm::vec4 base_color = mesh_renderer->base_color();
            if (ImGui::ColorEdit4("Base Color", &base_color.x)) {
                mesh_renderer->set_base_color(base_color);
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
                std::swap(config.pitch_min_degrees, config.pitch_max_degrees);
                dirty = true;
            }

            if (dirty) {
                free_look->set_config(config);
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
            }

            auto config = trackball->config();
            bool dirty = false;
            dirty |= ImGui::DragFloat("Rotate Speed", &config.rotate_speed, 0.001f, 0.001f, 10.0f);
            dirty |= ImGui::DragFloat("Pan Speed", &config.pan_speed, 0.0001f, 0.0001f, 1.0f);
            dirty |= ImGui::DragFloat("Zoom Speed##trackball", &config.zoom_speed, 0.01f, 0.01f, 20.0f);
            dirty |= ImGui::DragFloat("Pitch Min##trackball", &config.pitch_min_degrees, 0.1f, -179.0f, 179.0f);
            dirty |= ImGui::DragFloat("Pitch Max##trackball", &config.pitch_max_degrees, 0.1f, -179.0f, 179.0f);
            dirty |= ImGui::DragFloat3("World Up", &config.world_up.x, 0.01f, -1.0f, 1.0f);

            if (config.pitch_min_degrees > config.pitch_max_degrees) {
                std::swap(config.pitch_min_degrees, config.pitch_max_degrees);
                dirty = true;
            }
            if (glm::length(config.world_up) <= 1e-6f) {
                config.world_up = glm::vec3{0.0f, 1.0f, 0.0f};
                dirty = true;
            }

            if (dirty) {
                trackball->set_config(config);
            }

            glm::vec3 target = trackball->target();
            if (ImGui::DragFloat3("Target", &target.x, 0.05f)) {
                trackball->set_target(target);
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
        }

        bool enabled = game_object->enabled();
        if (ImGui::Checkbox("Enabled##game_object", &enabled)) {
            game_object->set_enabled(enabled);
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

