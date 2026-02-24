#include <cstdint>

#include "gtest/gtest.h"

#include "rtr/system/render/render_pipeline.hpp"

namespace rtr::system::render::test {

namespace {

class ProbePipeline final : public RenderPipeline {
public:
    explicit ProbePipeline(const PipelineRuntime& runtime)
        : RenderPipeline(runtime) {}

    void render(FrameContext&) override {}

    const SwapchainChangeSummary& last_diff() const { return m_last_diff; }
    const FrameScheduler::SwapchainState& last_state() const { return m_last_state; }

private:
    SwapchainChangeSummary m_last_diff{};
    FrameScheduler::SwapchainState m_last_state{};

    void handle_swapchain_state_change(
        const FrameScheduler::SwapchainState& state,
        const SwapchainChangeSummary& diff
    ) override {
        m_last_state = state;
        m_last_diff = diff;
    }
};

PipelineRuntime make_runtime_stub() {
    auto& device = *reinterpret_cast<rhi::Device*>(0x1);
    auto& context = *reinterpret_cast<rhi::Context*>(0x1);
    auto& window = *reinterpret_cast<rhi::Window*>(0x1);
    return PipelineRuntime{
        .device = device,
        .context = context,
        .window = window,
        .image_count = 3,
        .color_format = vk::Format::eB8G8R8A8Unorm,
        .depth_format = vk::Format::eD32Sfloat,
        .shader_root_dir = {}
    };
}

FrameScheduler::SwapchainState make_state() {
    return FrameScheduler::SwapchainState{
        .generation = 1,
        .extent = vk::Extent2D{640, 480},
        .image_count = 3,
        .color_format = vk::Format::eB8G8R8A8Unorm,
        .depth_format = vk::Format::eD32Sfloat
    };
}

} // namespace

TEST(PipelineBaseTest, TracksEachSwapchainFieldChange) {
    ProbePipeline pipeline(make_runtime_stub());

    auto baseline = make_state();
    pipeline.on_swapchain_state_changed(baseline);

    pipeline.on_swapchain_state_changed(baseline);
    auto diff = pipeline.last_diff();
    EXPECT_FALSE(diff.extent_changed);
    EXPECT_FALSE(diff.image_count_changed);
    EXPECT_FALSE(diff.color_format_changed);
    EXPECT_FALSE(diff.depth_format_changed);

    auto extent_changed = baseline;
    extent_changed.extent = vk::Extent2D{800, 600};
    pipeline.on_swapchain_state_changed(extent_changed);
    diff = pipeline.last_diff();
    EXPECT_TRUE(diff.extent_changed);
    EXPECT_FALSE(diff.image_count_changed);
    EXPECT_FALSE(diff.color_format_changed);
    EXPECT_FALSE(diff.depth_format_changed);

    auto image_count_changed = extent_changed;
    image_count_changed.image_count = 4;
    pipeline.on_swapchain_state_changed(image_count_changed);
    diff = pipeline.last_diff();
    EXPECT_FALSE(diff.extent_changed);
    EXPECT_TRUE(diff.image_count_changed);
    EXPECT_FALSE(diff.color_format_changed);
    EXPECT_FALSE(diff.depth_format_changed);

    auto color_changed = image_count_changed;
    color_changed.color_format = vk::Format::eR8G8B8A8Unorm;
    pipeline.on_swapchain_state_changed(color_changed);
    diff = pipeline.last_diff();
    EXPECT_FALSE(diff.extent_changed);
    EXPECT_FALSE(diff.image_count_changed);
    EXPECT_TRUE(diff.color_format_changed);
    EXPECT_FALSE(diff.depth_format_changed);

    auto depth_changed = color_changed;
    depth_changed.depth_format = vk::Format::eD24UnormS8Uint;
    pipeline.on_swapchain_state_changed(depth_changed);
    diff = pipeline.last_diff();
    EXPECT_FALSE(diff.extent_changed);
    EXPECT_FALSE(diff.image_count_changed);
    EXPECT_FALSE(diff.color_format_changed);
    EXPECT_TRUE(diff.depth_format_changed);
}

TEST(PipelineBaseTest, SummaryHelpersReflectFlags) {
    SwapchainChangeSummary summary{};
    EXPECT_FALSE(summary.extent_or_depth_changed());
    EXPECT_FALSE(summary.color_or_depth_changed());

    summary.extent_changed = true;
    EXPECT_TRUE(summary.extent_or_depth_changed());
    EXPECT_FALSE(summary.color_or_depth_changed());

    summary = {};
    summary.color_format_changed = true;
    EXPECT_FALSE(summary.extent_or_depth_changed());
    EXPECT_TRUE(summary.color_or_depth_changed());

    summary = {};
    summary.depth_format_changed = true;
    EXPECT_TRUE(summary.extent_or_depth_changed());
    EXPECT_TRUE(summary.color_or_depth_changed());
}

TEST(PipelineBaseTest, TypedEventSubscriptionAndPublish) {
    ProbePipeline pipeline(make_runtime_stub());

    uint32_t observed_width = 0;
    uint32_t observed_height = 0;
    uint32_t publish_count = 0;

    auto token = pipeline.subscribe_event<SceneViewportResizeEvent>(
        [&](const SceneViewportResizeEvent& event) {
            ++publish_count;
            observed_width = event.width;
            observed_height = event.height;
        }
    );

    pipeline.publish_event(SceneViewportResizeEvent{.width = 320, .height = 180});
    EXPECT_EQ(publish_count, 1u);
    EXPECT_EQ(observed_width, 320u);
    EXPECT_EQ(observed_height, 180u);

    token.reset();
    pipeline.publish_event(SceneViewportResizeEvent{.width = 640, .height = 360});
    EXPECT_EQ(publish_count, 1u);
}

} // namespace rtr::system::render::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
