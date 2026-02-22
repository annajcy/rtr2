#include <cstdlib>
#include <iostream>
#include <memory>

#include "rtr/framework/core/world.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/input/input_system.hpp"
#include "rtr/system/input/input_types.hpp"
#include "rtr/system/render/renderer.hpp"
#include "rtr/system/render/pipeline/shadertoy/shadertoy_pipeline.hpp"

int main() {
    constexpr uint32_t kWidth             = 800;
    constexpr uint32_t kHeight            = 600;
    constexpr uint32_t kMaxFramesInFlight = 2;

    try {
        auto renderer = std::make_unique<rtr::system::render::Renderer>(
            static_cast<int>(kWidth), static_cast<int>(kHeight), "RTR ShaderToy", kMaxFramesInFlight);

        auto runtime_pipeline = std::make_unique<rtr::system::render::ShaderToyPipeline>(
            renderer->build_pipeline_runtime(), rtr::system::render::ShaderToyPipelineConfig{});

        auto input_system = std::make_unique<rtr::system::input::InputSystem>(&renderer->window());

        auto world     = std::make_unique<rtr::framework::core::World>();
        auto resources = std::make_unique<rtr::resource::ResourceManager>(kMaxFramesInFlight);
        world->set_resource_manager(resources.get());
        (void)world->create_scene("editor_scene");

        renderer->set_pipeline(std::move(runtime_pipeline));

        std::uint64_t frame_serial = 0;
        while (!renderer->window().is_should_close()) {
            input_system->begin_frame();
            renderer->window().poll_events();
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
