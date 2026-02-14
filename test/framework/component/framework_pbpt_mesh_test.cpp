#include <stdexcept>
#include <cstddef>

#include "gtest/gtest.h"

#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/pbpt/pbpt_mesh.hpp"
#include "rtr/framework/core/scene.hpp"

namespace rtr::framework::component::test {

TEST(FrameworkPbptMeshTest, ThrowsWhenMeshRendererIsMissing) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");

    EXPECT_THROW(
        (void)go.add_component<PbptMesh>(),
        std::runtime_error
    );
}

TEST(FrameworkPbptMeshTest, CanAttachWhenMeshRendererExists) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    auto& renderer = go.add_component<MeshRenderer>(resource::MeshHandle{10});
    auto& pbpt_mesh = go.add_component<PbptMesh>();

    EXPECT_EQ(&pbpt_mesh.mesh_renderer(), &renderer);
    EXPECT_EQ(pbpt_mesh.mesh_handle(), resource::MeshHandle{10});
    EXPECT_TRUE(pbpt_mesh.is_reflectance_spectrum());
    EXPECT_FALSE(pbpt_mesh.is_reflectance_rgb());
}

TEST(FrameworkPbptMeshTest, MeshHandleTracksMeshRendererUpdates) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    auto& renderer = go.add_component<MeshRenderer>(resource::MeshHandle{10});
    auto& pbpt_mesh = go.add_component<PbptMesh>();

    EXPECT_EQ(pbpt_mesh.mesh_handle(), resource::MeshHandle{10});
    renderer.set_mesh_handle(resource::MeshHandle{30});
    EXPECT_EQ(pbpt_mesh.mesh_handle(), resource::MeshHandle{30});
}

TEST(FrameworkPbptMeshTest, ReflectanceSpectrumSetAndReadBack) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    (void)go.add_component<MeshRenderer>(resource::MeshHandle{10});
    auto& pbpt_mesh = go.add_component<PbptMesh>();

    const PbptSpectrum spectrum{
        PbptSpectrumPoint{410.0f, 0.1f},
        PbptSpectrumPoint{500.0f, 0.2f},
        PbptSpectrumPoint{620.0f, 0.3f},
    };
    pbpt_mesh.set_reflectance_spectrum(spectrum);

    const auto& out = pbpt_mesh.reflectance_spectrum();
    ASSERT_EQ(out.size(), spectrum.size());
    for (std::size_t i = 0; i < out.size(); ++i) {
        EXPECT_FLOAT_EQ(out[i].lambda_nm, spectrum[i].lambda_nm);
        EXPECT_FLOAT_EQ(out[i].value, spectrum[i].value);
    }
}

TEST(FrameworkPbptMeshTest, ReflectanceSpectrumValidationThrowsForInvalidData) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    (void)go.add_component<MeshRenderer>(resource::MeshHandle{10});
    auto& pbpt_mesh = go.add_component<PbptMesh>();

    EXPECT_THROW(
        (void)pbpt_mesh.set_reflectance_spectrum({}),
        std::invalid_argument
    );

    EXPECT_THROW(
        (void)pbpt_mesh.set_reflectance_spectrum({
            {500.0f, 0.2f},
            {450.0f, 0.3f},
        }),
        std::invalid_argument
    );

    EXPECT_THROW(
        (void)pbpt_mesh.set_reflectance_spectrum({
            {500.0f, -0.1f},
            {600.0f, 0.2f},
        }),
        std::invalid_argument
    );
}

TEST(FrameworkPbptMeshTest, ReflectanceRgbSetAndReadBack) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    (void)go.add_component<MeshRenderer>(resource::MeshHandle{10});
    auto& pbpt_mesh = go.add_component<PbptMesh>();

    pbpt_mesh.set_reflectance_rgb(PbptRgb{0.1f, 0.2f, 0.3f});
    EXPECT_TRUE(pbpt_mesh.is_reflectance_rgb());
    EXPECT_FALSE(pbpt_mesh.is_reflectance_spectrum());

    const auto& rgb = pbpt_mesh.reflectance_rgb();
    EXPECT_FLOAT_EQ(rgb.r, 0.1f);
    EXPECT_FLOAT_EQ(rgb.g, 0.2f);
    EXPECT_FLOAT_EQ(rgb.b, 0.3f);
}

TEST(FrameworkPbptMeshTest, ReflectanceRgbValidationThrowsForInvalidData) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    (void)go.add_component<MeshRenderer>(resource::MeshHandle{10});
    auto& pbpt_mesh = go.add_component<PbptMesh>();

    EXPECT_THROW((void)pbpt_mesh.set_reflectance_rgb(PbptRgb{-0.1f, 0.2f, 0.3f}), std::invalid_argument);
    EXPECT_THROW((void)pbpt_mesh.set_reflectance_rgb(PbptRgb{0.1f, 1.2f, 0.3f}), std::invalid_argument);
}

TEST(FrameworkPbptMeshTest, WrongReflectanceAccessorThrows) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    (void)go.add_component<MeshRenderer>(resource::MeshHandle{10});
    auto& pbpt_mesh = go.add_component<PbptMesh>();

    EXPECT_THROW((void)pbpt_mesh.reflectance_rgb(), std::logic_error);

    pbpt_mesh.set_reflectance_rgb(PbptRgb{0.3f, 0.4f, 0.5f});
    EXPECT_THROW((void)pbpt_mesh.reflectance_spectrum(), std::logic_error);
}

} // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
