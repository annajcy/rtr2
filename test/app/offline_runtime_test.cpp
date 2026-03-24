#include <gtest/gtest.h>

#include <stdexcept>

#include "rtr/app/offline_runtime.hpp"

namespace rtr::app {
namespace {

TEST(OfflineRuntimeConfigTest, MatchingFpsPassesValidation) {
    const OfflineRuntimeConfig config{
        .width = 640,
        .height = 480,
        .sim_dt = 0.01,
        .steps_per_output_frame = 2,
        .total_output_frames = 10,
        .output_fps = 50.0,
        .output_dir = "output/test_frames",
    };

    EXPECT_NO_THROW(validate_offline_runtime_config(config));
}

TEST(OfflineRuntimeConfigTest, MismatchedFpsCanFailValidation) {
    const OfflineRuntimeConfig config{
        .width = 640,
        .height = 480,
        .sim_dt = 0.01,
        .steps_per_output_frame = 2,
        .total_output_frames = 10,
        .output_fps = 60.0,
        .output_dir = "output/test_frames",
        .warn_on_fps_mismatch = false,
        .fail_on_fps_mismatch = true,
    };

    EXPECT_THROW(validate_offline_runtime_config(config), std::invalid_argument);
}

TEST(OfflineRuntimeConfigTest, RejectsZeroExtent) {
    OfflineRuntimeConfig config{};
    config.width = 0;
    EXPECT_THROW(validate_offline_runtime_config(config), std::invalid_argument);
}

TEST(OfflineRuntimeConfigTest, RejectsZeroTotalOutputFrames) {
    OfflineRuntimeConfig config{};
    config.total_output_frames = 0;
    EXPECT_THROW(validate_offline_runtime_config(config), std::invalid_argument);
}

TEST(OfflineRuntimeConfigTest, AllowsEmptyOutputDirWhenExportDisabled) {
    OfflineRuntimeConfig config{};
    config.export_frames = false;
    config.output_dir.clear();
    EXPECT_NO_THROW(validate_offline_runtime_config(config));
}

}  // namespace
}  // namespace rtr::app
