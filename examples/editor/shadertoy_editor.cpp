#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <memory>

#include "rtr/editor/core/editor_capture.hpp"
#include "rtr/editor/core/editor_host.hpp"
#include "rtr/editor/panel/hierarchy_panel.hpp"
#include "rtr/editor/panel/inspector_panel.hpp"
#include "rtr/editor/panel/logger_panel.hpp"
#include "rtr/editor/panel/scene_view_panel.hpp"
#include "rtr/editor/panel/stats_panel.hpp"
#include "rtr/editor/panel/shadertoy_settings_panel.hpp"
#include "rtr/editor/render/shadertoy_editor_pipeline.hpp"
#include "rtr/framework/core/engine.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/input/input_system.hpp"
#include "rtr/system/render/renderer.hpp"

namespace {
constexpr uint32_t kMaxFramesInFlight = 2;
}

int main() {
    try {
        rtr::resource::ResourceManager resource_manager(kMaxFramesInFlight);

        auto renderer =
            std::make_unique<rtr::system::render::Renderer>(1280, 720, "RTR ShaderToy Editor", kMaxFramesInFlight);
        auto input_system = std::make_unique<rtr::system::input::InputSystem>(&renderer->window());

        rtr::framework::core::Engine engine(
            rtr::framework::core::EngineConfig{.window_width         = 1280,
                                               .window_height        = 720,
                                               .window_title         = "RTR ShaderToy Editor",
                                               .max_frames_in_flight = kMaxFramesInFlight});
        engine.world().set_resource_manager(&resource_manager);

        // Editor Setup
        auto editor_host = std::make_shared<rtr::editor::EditorHost>();
        editor_host->bind_runtime(&engine.world(), &resource_manager, renderer.get(), input_system.get());

        auto editor_pipeline = std::make_unique<rtr::editor::render::ShaderToyEditorPipeline>(
            renderer->build_pipeline_runtime(), editor_host);

        editor_host->register_panel(std::make_unique<rtr::editor::SceneViewPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::HierarchyPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::InspectorPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::StatsPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::LoggerPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::ShaderToySettingsPanel>(editor_pipeline.get()));

        // Hide panels by default as requested
        editor_host->set_panel_visible("hierarchy", false);
        editor_host->set_panel_visible("inspector", false);

        rtr::editor::bind_input_capture_to_editor(*input_system, *editor_pipeline);

        auto* active_editor_pipeline = editor_pipeline.get();
        renderer->set_pipeline(std::move(editor_pipeline));

        engine.set_loop_hooks(rtr::framework::core::Engine::LoopHooks{
            .input_begin = [&]() { input_system->begin_frame(); },
            .input_poll  = [&]() { renderer->window().poll_events(); },
            .input_end   = [&]() { input_system->end_frame(); },
            .render =
                [&]() {
                    static std::uint64_t frame_serial = 0;

                    editor_host->begin_frame(rtr::editor::EditorFrameData{
                        .frame_serial  = frame_serial,
                        .delta_seconds = 0.0,
                        .paused        = engine.paused(),
                    });

                    renderer->draw_frame();
                    resource_manager.tick(frame_serial++);

                    if (input_system->state().key_down(rtr::system::input::KeyCode::ESCAPE)) {
                        renderer->window().close();
                    }
                },
            .should_close = [&]() { return renderer->window().is_should_close(); }});

        engine.run();

        renderer->device().wait_idle();
        resource_manager.flush_after_wait_idle();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
