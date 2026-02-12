#include <stdexcept>

#include "gtest/gtest.h"

#include <glm/vec3.hpp>

#include "framework/component/mesh_renderer.hpp"
#include "framework/component/pbpt_mesh.hpp"
#include "framework/core/scene.hpp"

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
    auto& renderer = go.add_component<MeshRenderer>("assets/models/spot.obj", "");
    auto& pbpt_mesh = go.add_component<PbptMesh>();

    EXPECT_EQ(&pbpt_mesh.mesh_renderer(), &renderer);
    EXPECT_EQ(pbpt_mesh.mesh_path(), "assets/models/spot.obj");
}

TEST(FrameworkPbptMeshTest, MeshPathTracksMeshRendererUpdates) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    auto& renderer = go.add_component<MeshRenderer>("assets/models/spot.obj", "");
    auto& pbpt_mesh = go.add_component<PbptMesh>();

    EXPECT_EQ(pbpt_mesh.mesh_path(), "assets/models/spot.obj");
    renderer.set_mesh_path("assets/models/stanford_bunny.obj");
    EXPECT_EQ(pbpt_mesh.mesh_path(), "assets/models/stanford_bunny.obj");
}

TEST(FrameworkPbptMeshTest, ReflectanceIsClampedToZeroOne) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    (void)go.add_component<MeshRenderer>("assets/models/spot.obj", "");
    auto& pbpt_mesh = go.add_component<PbptMesh>();

    pbpt_mesh.set_reflectance_rgb(glm::vec3{-0.3f, 0.4f, 1.2f});
    EXPECT_FLOAT_EQ(pbpt_mesh.diffuse_bsdf().reflectance_rgb.x, 0.0f);
    EXPECT_FLOAT_EQ(pbpt_mesh.diffuse_bsdf().reflectance_rgb.y, 0.4f);
    EXPECT_FLOAT_EQ(pbpt_mesh.diffuse_bsdf().reflectance_rgb.z, 1.0f);

    pbpt_mesh.set_reflectance_rgb(2.0f, -1.0f, 0.6f);
    EXPECT_FLOAT_EQ(pbpt_mesh.diffuse_bsdf().reflectance_rgb.x, 1.0f);
    EXPECT_FLOAT_EQ(pbpt_mesh.diffuse_bsdf().reflectance_rgb.y, 0.0f);
    EXPECT_FLOAT_EQ(pbpt_mesh.diffuse_bsdf().reflectance_rgb.z, 0.6f);
}

} // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
