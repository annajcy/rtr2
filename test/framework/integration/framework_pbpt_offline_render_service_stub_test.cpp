#include <gtest/gtest.h>

#include <string>

#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/pbpt/pbpt_offline_render_service.hpp"

namespace rtr::framework::integration::testing {

TEST(PbptOfflineRenderServiceStubTest, StartFailsWithDisabledMessage) {
    core::Scene scene(1, "stub_scene");
    PbptOfflineRenderService service;

    EXPECT_FALSE(service.start(scene, OfflineRenderConfig{
        .scene_xml_path = "unused_scene.xml",
        .output_exr_path = "unused_output.exr",
        .spp = 1
    }));
    EXPECT_EQ(service.state(), OfflineRenderState::Failed);
    EXPECT_FALSE(service.is_running());
    EXPECT_NE(service.last_message().find("disabled"), std::string::npos);
}

} // namespace rtr::framework::integration::testing
