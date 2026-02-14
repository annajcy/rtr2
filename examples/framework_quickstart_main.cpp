#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "imgui.h"
#include <glm/vec4.hpp>

#include "rtr/app/app_runtime.hpp"
#include "rtr/framework/component/camera_control/free_look_camera_controller.hpp"
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/core/camera.hpp"
#include "rtr/system/input/input_types.hpp"
#include "rtr/system/render/pipeline/forward/forward_pipeline.hpp"

int main() {
    constexpr uint32_t kWidth = 1280;
    constexpr uint32_t kHeight = 720;
    constexpr uint32_t kMaxFramesInFlight = 2;

    try {
        rtr::app::AppRuntime runtime(rtr::app::AppRuntimeConfig{
            .window_width = kWidth,
            .window_height = kHeight,
            .window_title = "RTR Framework Quickstart",
            .max_frames_in_flight = kMaxFramesInFlight
        });

        auto pipeline = std::make_unique<rtr::system::render::ForwardPipeline>(
            runtime.renderer().build_pipeline_runtime(),
            rtr::system::render::ForwardPipelineConfig{}
        );
        auto* forward_pipeline = pipeline.get();

        forward_pipeline->imgui_pass().set_ui_callback([]() {
            ImGui::Begin("Quickstart controls");
            ImGui::Text("Right Mouse: Look");
            ImGui::Text("WASD + Q/E: Move");
            ImGui::Text("Shift: Sprint");
            ImGui::Text("Mouse Wheel: Zoom");
            ImGui::Text("ESC: Quit");
            ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
            ImGui::End();
        });

        runtime.input_system().set_is_intercept_capture([forward_pipeline](bool is_mouse) {
            if (is_mouse) {
                return forward_pipeline->imgui_pass().wants_capture_mouse();
            }
            return forward_pipeline->imgui_pass().wants_capture_keyboard();
        });

        runtime.set_pipeline(std::move(pipeline));

        auto& scene = runtime.world().create_scene("main_scene");

        auto& camera_go = scene.create_game_object("main_camera");
        auto& camera = scene.camera_manager().create_perspective_camera(camera_go.id());
        camera.set_aspect_ratio(static_cast<float>(kWidth) / static_cast<float>(kHeight));
        camera_go.node().set_local_position({0.0f, 1.0f, -6.0f});
        camera_go.add_component<rtr::framework::component::FreeLookCameraController>(
            &runtime.input_system().state(),
            &scene.camera_manager()
        );
        (void)scene.set_active_camera(camera_go.id());

        auto add_mesh_renderer = [&](rtr::framework::core::GameObject& go,
                                     const std::string& mesh_path,
                                     const glm::vec4& base_color) {
            const auto mesh_handle =
                runtime.resource_manager().create_mesh_from_obj_relative_path(mesh_path);
            (void)go.add_component<rtr::framework::component::MeshRenderer>(
                mesh_handle,
                base_color
            );
        };

        auto& go_a = scene.create_game_object("mesh_a");
        add_mesh_renderer(go_a, "models/spot.obj", glm::vec4{0.2f, 0.7f, 0.9f, 1.0f});
        go_a.node().set_local_position({-2.5f, 0.0f, 0.0f});

        auto& go_b = scene.create_game_object("mesh_b");
        add_mesh_renderer(go_b, "models/stanford_bunny.obj", glm::vec4{0.9f, 0.85f, 0.75f, 1.0f});
        go_b.node().set_local_position({0.0f, 0.0f, 0.0f});

        auto& go_c = scene.create_game_object("mesh_c");
        add_mesh_renderer(go_c, "models/colored_quad.obj", glm::vec4{0.9f, 0.25f, 0.25f, 1.0f});
        go_c.node().set_local_position({2.5f, 0.0f, 0.0f});

        runtime.set_callbacks(rtr::app::RuntimeCallbacks{
            .on_pre_render = [](rtr::app::RuntimeContext& ctx) {
                auto* active_scene = ctx.world.active_scene();
                if (active_scene == nullptr) {
                    throw std::runtime_error("No active scene.");
                }

                auto* active_camera = active_scene->active_camera();
                if (active_camera == nullptr) {
                    throw std::runtime_error("Active scene has no active camera.");
                }

                const auto [fb_w, fb_h] = ctx.renderer.window().framebuffer_size();
                if (fb_w > 0 && fb_h > 0) {
                    active_camera->set_aspect_ratio(
                        static_cast<float>(fb_w) / static_cast<float>(fb_h)
                    );
                }

                if (ctx.input.state().key_down(rtr::system::input::KeyCode::ESCAPE)) {
                    ctx.renderer.window().close();
                }
            }
        });

        const auto result = runtime.run();
        if (!result.ok) {
            throw std::runtime_error(result.error_message);
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
