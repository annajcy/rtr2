#include <cstdlib>
#include <iostream>
#include <memory>

#include "rtr/editor/editor_attach.hpp"
#include "rtr/editor/editor_host.hpp"
#include "rtr/editor/hierarchy_panel.hpp"
#include "rtr/editor/inspector_panel.hpp"
#include "rtr/editor/stats_panel.hpp"
#include "rtr/framework/core/world.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/input/input_system.hpp"
#include "rtr/system/input/input_types.hpp"
#include "rtr/system/render/renderer.hpp"
#include "rtr/system/render/pipeline/shadertoy/shadertoy_pipeline.hpp"

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

        auto input_system = std::make_unique<rtr::system::input::InputSystem>(&renderer->window());

        auto world = std::make_unique<rtr::framework::core::World>();
        auto resources = std::make_unique<rtr::resource::ResourceManager>(kMaxFramesInFlight);
        world->set_resource_manager(resources.get());
        (void)world->create_scene("editor_scene");

        auto editor_host = std::make_shared<rtr::editor::EditorHost>();
        editor_host->bind_runtime(
            world.get(),
            resources.get(),
            renderer.get(),
            input_system.get()
        );
        editor_host->register_panel(std::make_unique<rtr::editor::HierarchyPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::InspectorPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::StatsPanel>());
        rtr::editor::attach_editor_host(*shadertoy_pipeline, editor_host);
        rtr::editor::bind_input_capture_to_pipeline(*input_system, *shadertoy_pipeline);

        renderer->set_pipeline(std::move(pipeline));

        std::uint64_t frame_serial = 0;
        while (!renderer->window().is_should_close()) {
            input_system->begin_frame();
            renderer->window().poll_events();

            editor_host->begin_frame(rtr::editor::EditorFrameData{
                .frame_serial = frame_serial,
                .delta_seconds = 0.0,
                .paused = false,
            });

            renderer->draw_frame();
            if (input_system->state().key_down(rtr::system::input::KeyCode::Q)) {
                renderer->window().close();
            }
            input_system->end_frame();
            ++frame_serial;
        }

        renderer->device().wait_idle();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
