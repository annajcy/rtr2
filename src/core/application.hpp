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
    std::unique_ptr<render::ForwardPipeline> m_render_pipeline{};

public:
    Application() {
        m_renderer = std::make_unique<render::Renderer>(
            static_cast<int>(WIDTH),
            static_cast<int>(HEIGHT),
            "RTR Application",
            MAX_FRAMES_IN_FLIGHT
        );

        m_render_pipeline = std::make_unique<render::ForwardPipeline>(m_renderer.get());
        m_render_pipeline->set_imgui_ui_builder([]() {
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
            m_renderer->draw_frame([this](render::FrameContext& ctx) {
                m_render_pipeline->execute_frame(ctx);
            });
        }

        m_renderer->device().wait_idle();
    }
};

} // namespace rtr::core
