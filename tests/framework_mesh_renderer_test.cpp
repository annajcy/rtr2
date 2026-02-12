#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "framework/component/mesh_renderer.hpp"
#include "framework/core/scene.hpp"

namespace rtr::framework::component::test {

TEST(FrameworkMeshRendererTest, ConstructEmptyAlbedoFallsBackToCheckerboard) {
    MeshRenderer renderer("assets/models/spot.obj", "");
    EXPECT_EQ(
        renderer.albedo_texture_path(),
        std::string(MeshRenderer::kDefaultAlbedoCheckerboardPath)
    );
}

TEST(FrameworkMeshRendererTest, SetAlbedoEmptyFallsBackToCheckerboard) {
    MeshRenderer renderer(
        "assets/models/spot.obj",
        "assets/textures/spot_texture.png"
    );
    renderer.set_albedo_texture_path("");
    EXPECT_EQ(
        renderer.albedo_texture_path(),
        std::string(MeshRenderer::kDefaultAlbedoCheckerboardPath)
    );
}

TEST(FrameworkMeshRendererTest, ConstructCustomAlbedoUsesCustomPath) {
    MeshRenderer renderer(
        "assets/models/spot.obj",
        "assets/textures/spot_texture.png"
    );
    EXPECT_EQ(renderer.albedo_texture_path(), "assets/textures/spot_texture.png");
}

TEST(FrameworkMeshRendererTest, SetAlbedoCustomOverridesDefault) {
    MeshRenderer renderer("assets/models/spot.obj", "");
    renderer.set_albedo_texture_path("assets/textures/viking_room.png");
    EXPECT_EQ(renderer.albedo_texture_path(), "assets/textures/viking_room.png");
}

TEST(FrameworkMeshRendererTest, MeshPathEmptyThrows) {
    EXPECT_THROW((void)MeshRenderer("", ""), std::invalid_argument);

    MeshRenderer renderer("assets/models/spot.obj", "");
    EXPECT_THROW((void)renderer.set_mesh_path(""), std::invalid_argument);
}

TEST(FrameworkMeshRendererTest, GameObjectCanAddAndQueryMeshRenderer) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    auto& renderer = go.add_component<MeshRenderer>("assets/models/spot.obj", "");

    EXPECT_TRUE(go.has_component<MeshRenderer>());
    EXPECT_EQ(go.get_component<MeshRenderer>(), &renderer);
    EXPECT_EQ(
        renderer.albedo_texture_path(),
        std::string(MeshRenderer::kDefaultAlbedoCheckerboardPath)
    );
}

TEST(FrameworkMeshRendererTest, GameObjectEnforcesUniqueMeshRendererType) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    (void)go.add_component<MeshRenderer>("assets/models/spot.obj", "");

    EXPECT_THROW(
        (void)go.add_component<MeshRenderer>("assets/models/viking_room.obj", ""),
        std::runtime_error
    );
}

} // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
