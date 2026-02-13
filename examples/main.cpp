#include <cstdlib>
#include <iostream>
#include <memory>

#include "imgui.h"
#include "rtr/system/input/input_system.hpp"
#include "rtr/system/input/input_types.hpp"
#include "rtr/system/render/forward_pipeline.hpp"
#include "rtr/system/render/renderer.hpp"

int main() {
    try {
        constexpr uint32_t kWidth = 800;
        constexpr uint32_t kHeight = 600;
        constexpr uint32_t kMaxFramesInFlight = 2;

        auto renderer = std::make_unique<rtr::system::render::Renderer>(
            static_cast<int>(kWidth),
            static_cast<int>(kHeight),
            "RTR Application",
            kMaxFramesInFlight
        );

        auto pipeline = std::make_unique<rtr::system::render::ForwardPipeline>(
            renderer->build_pipeline_runtime(),
            rtr::system::render::ForwardPipelineConfig{}
        );
        auto* forward_pipeline = pipeline.get();

        forward_pipeline->imgui_pass().set_ui_callback([]() {
            ImGui::Begin("RTR2");
            ImGui::Text("ImGui overlay active");
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

        while (!renderer->window().is_should_close()) {
            input_system->begin_frame();
            renderer->window().poll_events();
            renderer->draw_frame();
            if (input_system->state().key_down(rtr::system::input::KeyCode::Q)) {
                renderer->window().close();
            }
            input_system->end_frame();
        }

        renderer->device().wait_idle();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
