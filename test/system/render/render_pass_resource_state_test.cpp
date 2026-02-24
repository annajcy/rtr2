#include <type_traits>
#include <utility>

#include "gtest/gtest.h"

#include "rtr/system/render/pass/present_image_pass.hpp"
#include "rtr/system/render/pass/present_pass.hpp"
#include "rtr/system/render/pipeline/forward/forward_pass.hpp"
#include "rtr/system/render/pipeline/shadertoy/shadertoy_compute_pass.hpp"
#include "rtr/system/render/render_resource_state.hpp"

namespace rtr::system::render::test {

TEST(RenderPassResourceStateTest, PassResourcesUseTrackedImage) {
    EXPECT_TRUE((std::is_same_v<decltype(std::declval<ForwardPassResources>().color), TrackedImage>));
    EXPECT_TRUE((std::is_same_v<decltype(std::declval<PresentPassResources>().src_color), TrackedImage>));
    EXPECT_TRUE((std::is_same_v<decltype(std::declval<ComputePassResources>().offscreen), TrackedImage>));
    EXPECT_TRUE((std::is_same_v<decltype(std::declval<PresentImagePassResources>().offscreen), TrackedImage>));
}

TEST(RenderPassResourceStateTest, TrackedImageLayoutIsReferenceView) {
    auto& fake_image = *reinterpret_cast<rhi::Image*>(0x1);
    vk::ImageLayout layout = vk::ImageLayout::eUndefined;

    TrackedImage tracked{.image = fake_image, .layout = layout};
    tracked.layout = vk::ImageLayout::eGeneral;

    EXPECT_EQ(layout, vk::ImageLayout::eGeneral);
}

}  // namespace rtr::system::render::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
