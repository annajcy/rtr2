#pragma once

#include <cstdint>
#include <memory>

#include "imgui.h"

#include "render/forward_pipeline.hpp"
#include "render/renderer.hpp"

namespace rtr::core {

constexpr uint32_t WIDTH = 800;
constexpr uint32_t HEIGHT = 600;
constexpr uint32_t MAX_FRAMES_IN_FLIGHT = 2;

class Application {
private:
    std::unique_ptr<render::Renderer> m_renderer{};

public:
    Application() {
        m_renderer = std::make_unique<render::Renderer>(
            static_cast<int>(WIDTH),
            static_cast<int>(HEIGHT),
            "RTR Application",
            MAX_FRAMES_IN_FLIGHT
        );
       
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
            m_renderer->window().poll_events();
            m_renderer->draw_frame();
        }

        m_renderer->device().wait_idle();
    }
};

} // namespace rtr::core
