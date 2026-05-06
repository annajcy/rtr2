#include <array>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "rtr/system/render/render_resource_state.hpp"
#include "rtr/system/render/renderer.hpp"

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

class NoopPipeline final : public RenderPipeline {
private:
    std::array<FrameTrackedImage, rhi::kFramesInFlight> m_color_images;
    vk::Extent2D m_extent{1, 1};

public:
    explicit NoopPipeline(const PipelineRuntime& runtime)
        : RenderPipeline(runtime),
          m_color_images(create_color_images(vk::Extent2D{1, 1})) {}

    PipelineFinalOutput final_output(uint32_t frame_index) override {
        return PipelineFinalOutput{
            .color = m_color_images[frame_index].view(),
            .extent = m_extent
        };
    }

    void render(FrameContext& ctx) override {
        if (m_extent.width != ctx.render_extent().width || m_extent.height != ctx.render_extent().height) {
            m_extent = ctx.render_extent();
            m_color_images = create_color_images(m_extent);
        }

        auto& cmd = ctx.cmd().command_buffer();
        auto& tracked_color = m_color_images[ctx.frame_index()];

        vk::ImageMemoryBarrier2 to_color{};
        to_color.srcStageMask                    = vk::PipelineStageFlagBits2::eTopOfPipe;
        to_color.dstStageMask                    = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        to_color.srcAccessMask                   = vk::AccessFlagBits2::eNone;
        to_color.dstAccessMask                   = vk::AccessFlagBits2::eColorAttachmentWrite;
        to_color.oldLayout                       = tracked_color.layout;
        to_color.newLayout                       = vk::ImageLayout::eColorAttachmentOptimal;
        to_color.image                           = *tracked_color.image.image();
        to_color.subresourceRange.aspectMask     = vk::ImageAspectFlagBits::eColor;
        to_color.subresourceRange.baseMipLevel   = 0;
        to_color.subresourceRange.levelCount     = 1;
        to_color.subresourceRange.baseArrayLayer = 0;
        to_color.subresourceRange.layerCount     = 1;

        vk::DependencyInfo dep{};
        dep.imageMemoryBarrierCount = 1;
        dep.pImageMemoryBarriers    = &to_color;
        cmd.pipelineBarrier2(dep);

        vk::RenderingAttachmentInfo color_attachment{};
        color_attachment.imageView   = *tracked_color.image.image_view();
        color_attachment.imageLayout = vk::ImageLayout::eColorAttachmentOptimal;
        color_attachment.loadOp      = vk::AttachmentLoadOp::eClear;
        color_attachment.storeOp     = vk::AttachmentStoreOp::eStore;
        color_attachment.clearValue  = vk::ClearColorValue(std::array<float, 4>{0.0f, 0.0f, 0.0f, 1.0f});

        vk::RenderingInfo rendering_info{};
        rendering_info.renderArea.offset    = vk::Offset2D{0, 0};
        rendering_info.renderArea.extent    = ctx.render_extent();
        rendering_info.layerCount           = 1;
        rendering_info.colorAttachmentCount = 1;
        rendering_info.pColorAttachments    = &color_attachment;
        cmd.beginRendering(rendering_info);
        cmd.endRendering();
        tracked_color.layout = vk::ImageLayout::eColorAttachmentOptimal;
    }

private:
    void handle_swapchain_state_change(const FrameScheduler::SwapchainState& state, const SwapchainChangeSummary&) override {
        m_extent = state.extent;
        if (m_extent.width > 0 && m_extent.height > 0) {
            m_color_images = create_color_images(m_extent);
        }
    }

    std::array<FrameTrackedImage, rhi::kFramesInFlight> create_color_images(vk::Extent2D extent) {
        return make_frame_array<FrameTrackedImage>([&](uint32_t) {
            return FrameTrackedImage{
                rhi::Image(
                    m_device,
                    extent.width,
                    extent.height,
                    m_color_format,
                    vk::ImageTiling::eOptimal,
                    vk::ImageUsageFlagBits::eColorAttachment |
                        vk::ImageUsageFlagBits::eSampled |
                        vk::ImageUsageFlagBits::eTransferSrc,
                    vk::MemoryPropertyFlagBits::eDeviceLocal,
                    vk::ImageAspectFlagBits::eColor,
                    false
                ),
                vk::ImageLayout::eUndefined
            };
        });
    }
};

}  // namespace

TEST(RendererIntegrationTest, DrawFrameThrowsWithoutPipeline) {
    require_gpu_tests_enabled();

    Renderer renderer(640, 480, "rtr_renderer_integration");
    EXPECT_THROW((void)renderer.draw_frame(), std::runtime_error);
}

TEST(RendererIntegrationTest, SetPipelineRejectsNullAndSecondAssignment) {
    require_gpu_tests_enabled();

    Renderer renderer(640, 480, "rtr_renderer_pipeline_guard");
    EXPECT_THROW(renderer.set_pipeline(std::unique_ptr<RenderPipeline>{}), std::runtime_error);

    renderer.set_pipeline(std::make_unique<NoopPipeline>(renderer.build_pipeline_runtime()));
    EXPECT_THROW(renderer.set_pipeline(std::make_unique<NoopPipeline>(renderer.build_pipeline_runtime())),
                 std::runtime_error);
}

TEST(RendererIntegrationTest, DrawFrameWithNoopPipelineCanAdvanceFrameIndex) {
    require_gpu_tests_enabled();

    Renderer renderer(640, 480, "rtr_renderer_noop_draw");
    renderer.set_pipeline(std::make_unique<NoopPipeline>(renderer.build_pipeline_runtime()));

    const auto initial_frame_index = renderer.frame_scheduler().current_frame_index();
    bool       advanced            = false;
    for (int i = 0; i < 16 && !advanced; ++i) {
        renderer.window().poll_events();
        renderer.draw_frame();
        advanced = renderer.frame_scheduler().current_frame_index() != initial_frame_index;
    }

    renderer.device().wait_idle();
    EXPECT_TRUE(advanced);
}

TEST(RendererIntegrationTest, ComputeAsyncWithEmptyRecordCompletes) {
    require_gpu_tests_enabled();

    Renderer renderer(640, 480, "rtr_renderer_compute_async");
    auto         job = renderer.compute_async([](rtr::rhi::CommandBuffer&) {});

    EXPECT_TRUE(job.valid());
    EXPECT_NO_THROW(job.wait());
    renderer.device().wait_idle();
}

}  // namespace rtr::system::render::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
