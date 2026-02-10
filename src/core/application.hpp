#pragma once

#include <cstdint>
#include <memory>

#include "imgui.h"

#include "input/input_system.hpp"
#include "input/input_types.hpp"
#include "render/forward_pipeline.hpp"
#include "render/renderer.hpp"

namespace rtr::core {

constexpr uint32_t WIDTH = 800;
constexpr uint32_t HEIGHT = 600;
constexpr uint32_t MAX_FRAMES_IN_FLIGHT = 2;

class Application {
private:
    std::unique_ptr<render::Renderer> m_renderer{};
    std::unique_ptr<input::InputSystem> m_input_system{};

public:
    Application() {
        m_renderer = std::make_unique<render::Renderer>(
            static_cast<int>(WIDTH),
            static_cast<int>(HEIGHT),
            "RTR Application",
            MAX_FRAMES_IN_FLIGHT
        );

        m_input_system = std::make_unique<input::InputSystem>(&m_renderer->window());
        m_input_system->set_is_intercept_capture([this](bool is_mouse) {
            if (is_mouse) {
                return m_renderer->imgui_wants_capture_mouse();
            }
            return m_renderer->imgui_wants_capture_keyboard();
        });

        auto pipeline = std::make_unique<render::ForwardPipeline>(
            m_renderer->build_pipeline_runtime(),
            render::ForwardPipelineConfig{}
        );

        m_renderer->set_pipeline(std::move(pipeline));
        m_renderer->set_ui_callback([]() {
            ImGui::Begin("RTR2");
            ImGui::Text("ImGui overlay active");
            ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
            ImGui::End();
        });
    }

    void run() { loop(); }

    ~Application() = default;

    void loop() {
        while (!m_renderer->window().is_should_close()) {
            m_input_system->begin_frame();
            m_renderer->window().poll_events();
            m_renderer->draw_frame();
            if (m_input_system->state().key_down(input::KeyCode::Q)) {
                m_renderer->window().close();
            }
            m_input_system->end_frame();
        }
        m_renderer->device().wait_idle();
    }
};

} // namespace rtr::core
