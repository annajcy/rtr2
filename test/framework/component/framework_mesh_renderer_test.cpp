#include <stdexcept>

#include "gtest/gtest.h"

#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/core/scene.hpp"

namespace rtr::framework::component::test {

TEST(FrameworkMeshRendererTest, ConstructWithValidHandles) {
    MeshRenderer renderer(resource::MeshHandle{1}, resource::TextureHandle{2});
    EXPECT_EQ(renderer.mesh_handle(), resource::MeshHandle{1});
    EXPECT_EQ(renderer.albedo_texture_handle(), resource::TextureHandle{2});
}

TEST(FrameworkMeshRendererTest, InvalidHandleThrows) {
    EXPECT_THROW(
        (void)MeshRenderer(resource::MeshHandle{}, resource::TextureHandle{2}),
        std::invalid_argument
    );
    EXPECT_THROW(
        (void)MeshRenderer(resource::MeshHandle{1}, resource::TextureHandle{}),
        std::invalid_argument
    );

    MeshRenderer renderer(resource::MeshHandle{1}, resource::TextureHandle{2});
    EXPECT_THROW((void)renderer.set_mesh_handle(resource::MeshHandle{}), std::invalid_argument);
    EXPECT_THROW((void)renderer.set_albedo_texture_handle(resource::TextureHandle{}), std::invalid_argument);
}

TEST(FrameworkMeshRendererTest, SetHandlesUpdatesState) {
    MeshRenderer renderer(resource::MeshHandle{1}, resource::TextureHandle{2});

    renderer.set_mesh_handle(resource::MeshHandle{3});
    renderer.set_albedo_texture_handle(resource::TextureHandle{4});

    EXPECT_EQ(renderer.mesh_handle(), resource::MeshHandle{3});
    EXPECT_EQ(renderer.albedo_texture_handle(), resource::TextureHandle{4});
}

TEST(FrameworkMeshRendererTest, GameObjectCanAddAndQueryMeshRenderer) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    auto& renderer = go.add_component<MeshRenderer>(resource::MeshHandle{11}, resource::TextureHandle{12});

    EXPECT_TRUE(go.has_component<MeshRenderer>());
    EXPECT_EQ(go.get_component<MeshRenderer>(), &renderer);
    EXPECT_EQ(renderer.mesh_handle(), resource::MeshHandle{11});
    EXPECT_EQ(renderer.albedo_texture_handle(), resource::TextureHandle{12});
}

TEST(FrameworkMeshRendererTest, GameObjectEnforcesUniqueMeshRendererType) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    (void)go.add_component<MeshRenderer>(resource::MeshHandle{1}, resource::TextureHandle{2});

    EXPECT_THROW(
        (void)go.add_component<MeshRenderer>(resource::MeshHandle{3}, resource::TextureHandle{4}),
        std::runtime_error
    );
}

} // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
