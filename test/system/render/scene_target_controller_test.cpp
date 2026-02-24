#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "rtr/rhi/frame_constants.hpp"
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
        0,
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
    EXPECT_EQ(controller.active_generation(), 1u);
    EXPECT_TRUE(controller.recreated_this_frame());
    EXPECT_EQ(pipeline.wait_call_count, 0u);

    pipeline.publish_event(SceneViewportResizeEvent{.width = 0, .height = 0});
    (void)controller.ensure(
        0,
        vk::Extent2D{640, 480},
        [&](vk::Extent2D extent) {
            ++create_count;
            return TargetsStub{.extent = extent, .generation = create_count};
        },
        [](TargetsStub&) {}
    );

    EXPECT_EQ(create_count, 1u);
    EXPECT_FALSE(controller.recreated_this_frame());
    EXPECT_EQ(pipeline.wait_call_count, 0u);
}

TEST(SceneTargetControllerTest, RecreatesOnceWhenViewportOrSwapchainMarksDirty) {
    ProbePipeline pipeline(make_runtime_stub());
    SceneTargetController<TargetsStub> controller(pipeline, "probe");

    std::uint32_t create_count = 0;
    auto create = [&](vk::Extent2D extent) {
        ++create_count;
        return TargetsStub{.extent = extent, .generation = create_count};
    };

    (void)controller.ensure(0, vk::Extent2D{640, 480}, create, [](TargetsStub&) {});
    EXPECT_EQ(create_count, 1u);

    pipeline.publish_event(SceneViewportResizeEvent{.width = 1024, .height = 768});
    controller.on_swapchain_extent_changed();

    auto& recreated = controller.ensure(0, vk::Extent2D{1280, 720}, create, [](TargetsStub&) {});
    EXPECT_EQ(create_count, 2u);
    EXPECT_EQ(recreated.extent.width, 1024u);
    EXPECT_EQ(recreated.extent.height, 768u);
    EXPECT_EQ(controller.active_generation(), 2u);
    EXPECT_TRUE(controller.recreated_this_frame());

    (void)controller.ensure(0, vk::Extent2D{1280, 720}, create, [](TargetsStub&) {});
    EXPECT_EQ(create_count, 2u);
    EXPECT_FALSE(controller.recreated_this_frame());
}

TEST(SceneTargetControllerTest, RejectsInvalidFrameIndex) {
    ProbePipeline pipeline(make_runtime_stub());
    SceneTargetController<TargetsStub> controller(pipeline, "probe");

    EXPECT_THROW(
        (void)controller.ensure(
            rhi::kFramesInFlight,
            vk::Extent2D{640, 480},
            [](vk::Extent2D extent) { return TargetsStub{.extent = extent, .generation = 1}; },
            [](TargetsStub&) {}
        ),
        std::out_of_range
    );
}

TEST(SceneTargetControllerTest, RetiresOldTargetsAfterAllFrameSlotsBecomeFenceSafe) {
    struct LifetimeTrackedTargets {
        vk::Extent2D extent{};
        std::uint32_t generation{0};
        std::shared_ptr<int> lifetime{};
    };

    ProbePipeline pipeline(make_runtime_stub());
    SceneTargetController<LifetimeTrackedTargets> controller(pipeline, "probe");

    std::vector<std::weak_ptr<int>> weak_lifetimes;
    std::uint32_t create_count = 0;
    auto create = [&](vk::Extent2D extent) {
        ++create_count;
        auto lifetime = std::make_shared<int>(static_cast<int>(create_count));
        weak_lifetimes.push_back(lifetime);
        return LifetimeTrackedTargets{
            .extent = extent,
            .generation = create_count,
            .lifetime = std::move(lifetime)
        };
    };

    (void)controller.ensure(0, vk::Extent2D{640, 480}, create, [](LifetimeTrackedTargets&) {});
    EXPECT_EQ(create_count, 1u);
    ASSERT_EQ(weak_lifetimes.size(), 1u);
    EXPECT_FALSE(weak_lifetimes[0].expired());

    pipeline.publish_event(SceneViewportResizeEvent{.width = 800, .height = 600});
    (void)controller.ensure(0, vk::Extent2D{640, 480}, create, [](LifetimeTrackedTargets&) {});
    EXPECT_EQ(create_count, 2u);
    ASSERT_EQ(weak_lifetimes.size(), 2u);
    EXPECT_FALSE(weak_lifetimes[0].expired());

    (void)controller.ensure(0, vk::Extent2D{640, 480}, create, [](LifetimeTrackedTargets&) {});
    EXPECT_FALSE(weak_lifetimes[0].expired());

    (void)controller.ensure(1, vk::Extent2D{640, 480}, create, [](LifetimeTrackedTargets&) {});
    EXPECT_TRUE(weak_lifetimes[0].expired());
    EXPECT_FALSE(weak_lifetimes[1].expired());
}

}  // namespace rtr::system::render::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
