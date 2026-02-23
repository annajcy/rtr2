#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "rtr/app/app_runtime.hpp"
#include "rtr/editor/core/editor_capture.hpp"
#include "rtr/editor/core/editor_host.hpp"
#include "rtr/editor/panel/hierarchy_panel.hpp"
#include "rtr/editor/panel/inspector_panel.hpp"
#include "rtr/editor/panel/logger_panel.hpp"
#include "rtr/editor/panel/scene_view_panel.hpp"
#include "rtr/editor/panel/shadertoy_settings_panel.hpp"
#include "rtr/editor/panel/stats_panel.hpp"
#include "rtr/editor/render/shadertoy_editor_pipeline.hpp"
#include "rtr/system/input/input_types.hpp"

namespace {
constexpr uint32_t kMaxFramesInFlight = 2;
}

int main() {
    try {
        rtr::app::AppRuntime runtime(rtr::app::AppRuntimeConfig{
            .window_width         = 1280,
            .window_height        = 720,
            .window_title         = "RTR ShaderToy Editor",
            .max_frames_in_flight = kMaxFramesInFlight,
        });

        auto editor_host = std::make_shared<rtr::editor::EditorHost>(runtime);

        auto editor_pipeline = std::make_unique<rtr::editor::render::ShaderToyEditorPipeline>(
            runtime.renderer().build_pipeline_runtime(), editor_host);

        editor_host->register_panel(std::make_unique<rtr::editor::SceneViewPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::HierarchyPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::InspectorPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::StatsPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::LoggerPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::ShaderToySettingsPanel>(editor_pipeline.get()));

        editor_host->set_panel_visible("hierarchy", false);
        editor_host->set_panel_visible("inspector", false);

        rtr::editor::bind_input_capture_to_editor(runtime.input_system(), *editor_pipeline);
        runtime.set_pipeline(std::move(editor_pipeline));

        runtime.set_callbacks(rtr::app::RuntimeCallbacks{
            .on_post_update =
                [editor_host](rtr::app::RuntimeContext& ctx) {
                    editor_host->begin_frame(rtr::editor::EditorFrameData{
                        .frame_serial  = ctx.frame_serial,
                        .delta_seconds = ctx.delta_seconds,
                        .paused        = ctx.paused,
                    });
                },
            .on_pre_render =
                [](rtr::app::RuntimeContext& ctx) {
                    if (ctx.input.state().key_down(rtr::system::input::KeyCode::ESCAPE)) {
                        ctx.renderer.window().close();
                    }
                },
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
