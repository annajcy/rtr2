#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "rtr/app/app_runtime.hpp"
#include "rtr/system/input/input_types.hpp"
#include "rtr/system/render/pipeline/shadertoy/shadertoy_pipeline.hpp"

int main() {
    constexpr uint32_t kWidth             = 800;
    constexpr uint32_t kHeight            = 600;
    constexpr uint32_t kMaxFramesInFlight = 2;

    try {
        rtr::app::AppRuntime runtime(rtr::app::AppRuntimeConfig{
            .window_width         = kWidth,
            .window_height        = kHeight,
            .window_title         = "RTR ShaderToy",
            .max_frames_in_flight = kMaxFramesInFlight,
        });

        auto runtime_pipeline = std::make_unique<rtr::system::render::ShaderToyPipeline>(
            runtime.renderer().build_pipeline_runtime(), rtr::system::render::ShaderToyPipelineConfig{});
        runtime.set_pipeline(std::move(runtime_pipeline));

        (void)runtime.world().create_scene("runtime_scene");

        runtime.set_callbacks(rtr::app::RuntimeCallbacks{
            .on_pre_render =
                [](rtr::app::RuntimeContext& ctx) {
                    if (ctx.input.state().key_down(rtr::system::input::KeyCode::Q)) {
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
