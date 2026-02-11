#include <cstdlib>
#include <iostream>
#include <memory>

#include "imgui.h"

#include "system/input/input_system.hpp"
#include "system/input/input_types.hpp"
#include "system/render/renderer.hpp"
#include "system/render/shadertoy_pipeline.hpp"

int main() {
    constexpr uint32_t kWidth = 800;
    constexpr uint32_t kHeight = 600;
    constexpr uint32_t kMaxFramesInFlight = 2;

    try {
        auto renderer = std::make_unique<rtr::system::render::Renderer>(
            static_cast<int>(kWidth),
            static_cast<int>(kHeight),
            "RTR ShaderToy",
            kMaxFramesInFlight
        );

        auto pipeline = std::make_unique<rtr::system::render::ShaderToyPipeline>(
            renderer->build_pipeline_runtime(),
            rtr::system::render::ShaderToyPipelineConfig{}
        );
        auto* shadertoy_pipeline = pipeline.get();

        shadertoy_pipeline->imgui_pass().set_ui_callback([]() {
            ImGui::Begin("ShaderToyPipeline");
            ImGui::Text("Compute -> Present pipeline active");
            ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
            ImGui::Text("Press Q to quit");
            ImGui::End();
        });

        auto input_system = std::make_unique<rtr::system::input::InputSystem>(&renderer->window());
        input_system->set_is_intercept_capture([shadertoy_pipeline](bool is_mouse) {
            if (is_mouse) {
                return shadertoy_pipeline->imgui_pass().wants_capture_mouse();
            }
            return shadertoy_pipeline->imgui_pass().wants_capture_keyboard();
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
