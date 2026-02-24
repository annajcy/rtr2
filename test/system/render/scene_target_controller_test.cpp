#include <cstdint>

#include "gtest/gtest.h"

#include "rtr/system/render/render_pipeline.hpp"
#include "rtr/system/render/scene_target_controller.hpp"

namespace rtr::system::render::test {

namespace {

class ProbePipeline final : public RenderPipeline {
public:
    explicit ProbePipeline(const PipelineRuntime& runtime)
        : RenderPipeline(runtime) {}

    void render(FrameContext&) override {}

    std::uint32_t wait_call_count{0};

    void wait_for_scene_target_rebuild() override {
        ++wait_call_count;
    }

private:
    void handle_swapchain_state_change(const FrameScheduler::SwapchainState&, const SwapchainChangeSummary&) override {}
};

struct TargetsStub {
    vk::Extent2D extent{};
    std::uint32_t generation{0};
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

}  // namespace

TEST(SceneTargetControllerTest, IgnoresZeroViewportResizeAndUsesFallbackExtent) {
    ProbePipeline pipeline(make_runtime_stub());
    SceneTargetController<TargetsStub> controller(pipeline, "probe");

    std::uint32_t create_count = 0;
    auto& targets = controller.ensure(
        vk::Extent2D{640, 480},
        [&](vk::Extent2D extent) {
            ++create_count;
            return TargetsStub{.extent = extent, .generation = create_count};
        },
        [](TargetsStub&) {}
    );

    EXPECT_EQ(create_count, 1u);
    EXPECT_EQ(targets.extent.width, 640u);
    EXPECT_EQ(targets.extent.height, 480u);
    EXPECT_EQ(pipeline.wait_call_count, 1u);

    pipeline.publish_event(SceneViewportResizeEvent{.width = 0, .height = 0});
    (void)controller.ensure(
        vk::Extent2D{640, 480},
        [&](vk::Extent2D extent) {
            ++create_count;
            return TargetsStub{.extent = extent, .generation = create_count};
        },
        [](TargetsStub&) {}
    );

    EXPECT_EQ(create_count, 1u);
    EXPECT_EQ(pipeline.wait_call_count, 1u);
}

TEST(SceneTargetControllerTest, RecreatesOnceWhenViewportOrSwapchainMarksDirty) {
    ProbePipeline pipeline(make_runtime_stub());
    SceneTargetController<TargetsStub> controller(pipeline, "probe");

    std::uint32_t create_count = 0;
    auto create = [&](vk::Extent2D extent) {
        ++create_count;
        return TargetsStub{.extent = extent, .generation = create_count};
    };

    (void)controller.ensure(vk::Extent2D{640, 480}, create, [](TargetsStub&) {});
    EXPECT_EQ(create_count, 1u);

    pipeline.publish_event(SceneViewportResizeEvent{.width = 1024, .height = 768});
    controller.on_swapchain_extent_changed();

    auto& recreated = controller.ensure(vk::Extent2D{1280, 720}, create, [](TargetsStub&) {});
    EXPECT_EQ(create_count, 2u);
    EXPECT_EQ(recreated.extent.width, 1024u);
    EXPECT_EQ(recreated.extent.height, 768u);

    (void)controller.ensure(vk::Extent2D{1280, 720}, create, [](TargetsStub&) {});
    EXPECT_EQ(create_count, 2u);
}

}  // namespace rtr::system::render::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
