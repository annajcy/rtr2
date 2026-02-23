#include <cstdlib>
#include <optional>
#include <string>

#include "gtest/gtest.h"

#include "rtr/rhi/context.hpp"
#include "rtr/rhi/device.hpp"
#include "rtr/rhi/frame_constants.hpp"
#include "rtr/rhi/window.hpp"
#include "rtr/system/render/frame_scheduler.hpp"

namespace rtr::system::render::test {

namespace {

bool gpu_tests_enabled() {
    const char* value = std::getenv("RTR_RUN_GPU_TESTS");
    return value != nullptr && std::string(value) == "1";
}

void require_gpu_tests_enabled() {
    if (!gpu_tests_enabled()) {
        GTEST_SKIP() << "Set RTR_RUN_GPU_TESTS=1 to run integration GPU tests.";
    }
}

rhi::ContextCreateInfo make_context_create_info(rhi::Window& window) {
    rhi::ContextCreateInfo info{};
    info.app_name = "FrameSchedulerIntegrationTest";
    info.instance_extensions = window.required_extensions();
    info.surface_creator = [&window](const vk::raii::Instance& instance) {
        return window.create_vk_surface(instance);
    };
    info.enable_validation_layers = false;
    return info;
}

struct Harness {
    rhi::Window window;
    rhi::Context context;
    rhi::Device device;
    FrameScheduler<rhi::kFramesInFlight> scheduler;

    Harness()
        : window(640, 480, "rtr_frame_scheduler_integration"),
          context(make_context_create_info(window)),
          device(context),
          scheduler(window, context, device) {}

    ~Harness() {
        try {
            device.wait_idle();
        } catch (...) {
        }
    }
};

bool try_submit_single_empty_frame(Harness& harness) {
    auto ticket = harness.scheduler.begin_frame();
    if (!ticket.has_value()) {
        return false;
    }

    ticket->command_buffer->reset();
    ticket->command_buffer->record(
        [](rtr::rhi::CommandBuffer&) {},
        vk::CommandBufferUsageFlagBits::eOneTimeSubmit
    );
    harness.scheduler.submit_and_present(*ticket);
    return true;
}

} // namespace

TEST(FrameSchedulerIntegrationTest, CanSubmitAndPresentAtLeastOneFrame) {
    require_gpu_tests_enabled();

    Harness harness;
    bool submitted = false;
    for (int i = 0; i < 32 && !submitted; ++i) {
        harness.window.poll_events();
        submitted = try_submit_single_empty_frame(harness);
    }

    harness.device.wait_idle();
    EXPECT_TRUE(submitted);
}

TEST(FrameSchedulerIntegrationTest, ResizeTriggersSwapchainGenerationChange) {
    require_gpu_tests_enabled();

    Harness harness;
    const auto generation_before = harness.scheduler.swapchain_state().generation;

    harness.scheduler.on_window_resized(1, 1);

    bool generation_changed = false;
    for (int i = 0; i < 64 && !generation_changed; ++i) {
        harness.window.poll_events();
        (void)try_submit_single_empty_frame(harness);
        generation_changed = harness.scheduler.swapchain_state().generation > generation_before;
    }

    harness.device.wait_idle();
    EXPECT_TRUE(generation_changed);
}

} // namespace rtr::system::render::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
