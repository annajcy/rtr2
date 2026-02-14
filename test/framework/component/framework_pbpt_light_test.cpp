#include <stdexcept>
#include <cstddef>

#include "gtest/gtest.h"

#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/pbpt/pbpt_light.hpp"
#include "rtr/framework/core/scene.hpp"

namespace rtr::framework::component::test {

TEST(FrameworkPbptLightTest, ThrowsWhenMeshRendererIsMissing) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("light");

    EXPECT_THROW(
        (void)go.add_component<PbptLight>(),
        std::runtime_error
    );
}

TEST(FrameworkPbptLightTest, CanAttachWhenMeshRendererExists) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("light");
    auto& renderer = go.add_component<MeshRenderer>("assets/models/spot.obj", "");
    auto& pbpt_light = go.add_component<PbptLight>();

    EXPECT_EQ(&pbpt_light.mesh_renderer(), &renderer);
    EXPECT_EQ(pbpt_light.mesh_path(), "assets/models/spot.obj");
}

TEST(FrameworkPbptLightTest, RadianceSpectrumValidationThrowsForInvalidData) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("light");
    (void)go.add_component<MeshRenderer>("assets/models/spot.obj", "");
    auto& pbpt_light = go.add_component<PbptLight>();

    EXPECT_THROW(
        (void)pbpt_light.set_radiance_spectrum({}),
        std::invalid_argument
    );
    EXPECT_THROW(
        (void)pbpt_light.set_radiance_spectrum({
            {450.0f, 1.0f},
            {420.0f, 1.0f},
        }),
        std::invalid_argument
    );
    EXPECT_THROW(
        (void)pbpt_light.set_radiance_spectrum({
            {450.0f, -0.1f},
            {500.0f, 1.0f},
        }),
        std::invalid_argument
    );
}

TEST(FrameworkPbptLightTest, RadianceSpectrumSetAndReadBack) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("light");
    (void)go.add_component<MeshRenderer>("assets/models/spot.obj", "");
    auto& pbpt_light = go.add_component<PbptLight>();

    const PbptSpectrum spectrum{
        PbptSpectrumPoint{410.0f, 0.0f},
        PbptSpectrumPoint{500.0f, 8.0f},
        PbptSpectrumPoint{600.0f, 15.6f},
        PbptSpectrumPoint{700.0f, 18.4f},
    };
    pbpt_light.set_radiance_spectrum(spectrum);

    const auto& out = pbpt_light.area_emitter().radiance_spectrum;
    ASSERT_EQ(out.size(), spectrum.size());
    for (std::size_t i = 0; i < out.size(); ++i) {
        EXPECT_FLOAT_EQ(out[i].lambda_nm, spectrum[i].lambda_nm);
        EXPECT_FLOAT_EQ(out[i].value, spectrum[i].value);
    }
}

} // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
