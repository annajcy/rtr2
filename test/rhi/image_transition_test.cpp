#include <stdexcept>

#include "gtest/gtest.h"

#include "rtr/rhi/texture.hpp"

namespace rtr::rhi::test {

TEST(ImageTransitionTest, ReturnsExpectedConfigForSupportedTransition) {
    const auto config = Image::make_transition_config(
        vk::ImageLayout::eUndefined,
        vk::ImageLayout::eTransferDstOptimal,
        vk::ImageAspectFlagBits::eColor
    );

    EXPECT_EQ(config.old_layout, vk::ImageLayout::eUndefined);
    EXPECT_EQ(config.new_layout, vk::ImageLayout::eTransferDstOptimal);
    EXPECT_EQ(config.src_stage, vk::PipelineStageFlagBits::eTopOfPipe);
    EXPECT_EQ(config.dst_stage, vk::PipelineStageFlagBits::eTransfer);
    EXPECT_EQ(config.src_access, vk::AccessFlags{});
    EXPECT_EQ(config.dst_access, vk::AccessFlagBits::eTransferWrite);
    EXPECT_EQ(config.aspect_mask, vk::ImageAspectFlagBits::eColor);
}

TEST(ImageTransitionTest, PreservesAspectMaskAcrossTransitionConfig) {
    const auto config = Image::make_transition_config(
        vk::ImageLayout::eUndefined,
        vk::ImageLayout::eDepthAttachmentOptimal,
        vk::ImageAspectFlagBits::eDepth
    );

    EXPECT_EQ(config.aspect_mask, vk::ImageAspectFlagBits::eDepth);
    EXPECT_EQ(config.dst_stage, vk::PipelineStageFlagBits::eEarlyFragmentTests);
    EXPECT_EQ(
        config.dst_access,
        vk::AccessFlagBits::eDepthStencilAttachmentWrite | vk::AccessFlagBits::eDepthStencilAttachmentRead
    );
}

TEST(ImageTransitionTest, ThrowsForUnsupportedTransition) {
    EXPECT_THROW(
        (void)Image::make_transition_config(
            vk::ImageLayout::ePresentSrcKHR,
            vk::ImageLayout::eTransferDstOptimal,
            vk::ImageAspectFlagBits::eColor
        ),
        std::invalid_argument
    );
}

} // namespace rtr::rhi::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
