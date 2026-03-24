#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "rtr/app/offline_runtime.hpp"
#include "rtr/system/render/pipeline/forward/forward_pipeline.hpp"
#include "ipc_offline_scene.hpp"

int main() {
    constexpr uint32_t kWidth = 1280;
    constexpr uint32_t kHeight = 720;
    const std::filesystem::path output_dir = "output/headless/ipc_fixed_end_block";

    try {
        rtr::app::OfflineRuntime runtime(rtr::app::OfflineRuntimeConfig{
            .width = kWidth,
            .height = kHeight,
            .sim_dt = 0.01,
            .steps_per_output_frame = 1,
            .total_output_frames = 120,
            .output_fps = 100.0,
            .export_frames = true,
            .output_dir = output_dir,
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

        std::cout << "Exported PNG frames to " << output_dir << "\n";
        std::cout << "Create a video with:\n";
        std::cout << "  python3 examples/headless/make_video.py "
                  << output_dir.string() << " "
                  << (output_dir / "offline.mp4").string() << " 100\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
