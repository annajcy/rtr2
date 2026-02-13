#include <cstdlib>
#include <iostream>
#include <memory>

#include "imgui.h"

#include "rtr/framework/component/mesh_renderer.hpp"
#include "rtr/framework/component/trackball_camera_controller.hpp"
#include "rtr/framework/core/camera.hpp"
#include "rtr/framework/core/engine.hpp"
#include "rtr/framework/integration/forward_scene_view_builder.hpp"
#include "rtr/system/input/input_system.hpp"
#include "rtr/system/input/input_types.hpp"
#include "rtr/system/render/forward_pipeline.hpp"
#include "rtr/system/render/renderer.hpp"

int main() {
    constexpr uint32_t kWidth = 1280;
    constexpr uint32_t kHeight = 720;
    constexpr uint32_t kMaxFramesInFlight = 2;

    try {
        auto renderer = std::make_unique<rtr::system::render::Renderer>(
            static_cast<int>(kWidth),
            static_cast<int>(kHeight),
            "RTR Framework TrackBall Scene",
            kMaxFramesInFlight
        );

        auto pipeline = std::make_unique<rtr::system::render::ForwardPipeline>(
            renderer->build_pipeline_runtime(),
            rtr::system::render::ForwardPipelineConfig{}
        );
        auto* forward_pipeline = pipeline.get();

        forward_pipeline->imgui_pass().set_ui_callback([]() {
            ImGui::Begin("Framework TrackBall Scene");
            ImGui::Text("Left Mouse: Orbit");
            ImGui::Text("Middle Mouse: Pan");
            ImGui::Text("Mouse Wheel: Zoom");
            ImGui::Text("ESC: Quit");
            ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
            ImGui::End();
        });

        auto input_system = std::make_unique<rtr::system::input::InputSystem>(&renderer->window());
        input_system->set_is_intercept_capture([forward_pipeline](bool is_mouse) {
            if (is_mouse) {
                return forward_pipeline->imgui_pass().wants_capture_mouse();
            }
            return forward_pipeline->imgui_pass().wants_capture_keyboard();
        });

        renderer->set_pipeline(std::move(pipeline));

        rtr::framework::core::Engine engine(rtr::framework::core::EngineConfig{
            .window_width = kWidth,
            .window_height = kHeight,
            .window_title = "RTR Framework TrackBall Scene",
            .max_frames_in_flight = kMaxFramesInFlight
        });

        auto& scene = engine.world().create_scene("main_scene");

        auto& camera_go = scene.create_game_object("main_camera");
        auto& camera = scene.camera_manager().create_perspective_camera(camera_go.id());
        camera.set_aspect_ratio(static_cast<float>(kWidth) / static_cast<float>(kHeight));
        camera_go.node().set_local_position({0.0f, 2.0f, -8.0f});
        auto& trackball = camera_go.add_component<rtr::framework::component::TrackBallCameraController>(
            &input_system->state(),
            &scene.camera_manager()
        );
        trackball.set_target({0.0f, 0.0f, 0.0f});
        (void)scene.set_active_camera(camera_go.id());

        auto& go_a = scene.create_game_object("mesh_a");
        go_a.add_component<rtr::framework::component::MeshRenderer>(
            "assets/models/spot.obj",
            "assets/textures/spot_texture.png"
        );
        go_a.node().set_local_position({-2.5f, 0.0f, 0.0f});

        auto& go_b = scene.create_game_object("mesh_b");
        go_b.add_component<rtr::framework::component::MeshRenderer>(
            "assets/models/stanford_bunny.obj",
            "assets/textures/viking_room.png"
        );
        go_b.node().set_local_position({0.0f, 0.0f, 0.0f});

        auto& go_c = scene.create_game_object("mesh_c");
        go_c.add_component<rtr::framework::component::MeshRenderer>(
            "assets/models/colored_quad.obj"
        );
        go_c.node().set_local_position({2.5f, 0.0f, 0.0f});

        engine.set_loop_hooks(rtr::framework::core::Engine::LoopHooks{
            .input_begin = [&]() { input_system->begin_frame(); },
            .input_poll = [&]() { renderer->window().poll_events(); },
            .input_end = [&]() { input_system->end_frame(); },
            .render = [&]() {
                auto* active_scene = engine.world().active_scene();
                if (active_scene == nullptr) {
                    throw std::runtime_error("No active scene.");
                }

                auto* active_camera = active_scene->active_camera();
                if (active_camera == nullptr) {
                    throw std::runtime_error("Active scene has no active camera.");
                }

                const auto [fb_w, fb_h] = renderer->window().framebuffer_size();
                if (fb_w > 0 && fb_h > 0) {
                    active_camera->set_aspect_ratio(
                        static_cast<float>(fb_w) / static_cast<float>(fb_h)
                    );
                }

                forward_pipeline->set_scene_view(
                    rtr::framework::integration::build_forward_scene_view(*active_scene)
                );
                renderer->draw_frame();

                if (input_system->state().key_down(rtr::system::input::KeyCode::ESCAPE)) {
                    renderer->window().close();
                }
            },
            .should_close = [&]() { return renderer->window().is_should_close(); }
        });

        engine.run();
        renderer->device().wait_idle();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
