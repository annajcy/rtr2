#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "rtr/app/offline_runtime.hpp"
#include "rtr/system/render/pipeline/forward/forward_pipeline.hpp"
#include "ipc_offline_scene.hpp"

int main() {
    constexpr uint32_t kWidth = 1280;
    constexpr uint32_t kHeight = 720;

    try {
        rtr::app::OfflineRuntime runtime(rtr::app::OfflineRuntimeConfig{
            .width = kWidth,
            .height = kHeight,
            .sim_dt = 0.01,
            .steps_per_output_frame = 1,
            .total_output_frames = 120,
            .output_fps = 100.0,
            .export_frames = false,
            .output_dir = {},
            .filename_pattern = "frame_{:06d}.png",
            .create_non_interactive_window = true,
        });

        auto pipeline = std::make_unique<rtr::system::render::ForwardPipeline>(
            runtime.renderer().build_pipeline_runtime()
        );
        runtime.set_pipeline(std::move(pipeline));
        rtr::examples::headless::setup_ipc_fixed_end_scene(runtime, kWidth, kHeight);

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
