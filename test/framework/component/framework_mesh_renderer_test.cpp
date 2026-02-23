#include <pbpt/math/math.h>
#include <stdexcept>

#include "gtest/gtest.h"


#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/core/scene.hpp"

namespace rtr::framework::component::test {

TEST(FrameworkMeshRendererTest, ConstructWithValidHandles) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    auto& renderer = go.add_component<MeshRenderer>(resource::MeshHandle{1});
    EXPECT_EQ(renderer.mesh_handle(), resource::MeshHandle{1});
    EXPECT_EQ(renderer.base_color(), pbpt::math::vec4(1.0f));
}

TEST(FrameworkMeshRendererTest, InvalidMeshHandleThrows) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    EXPECT_THROW(
        (void)go.add_component<MeshRenderer>(resource::MeshHandle{}),
        std::invalid_argument
    );

    auto& renderer = go.add_component<MeshRenderer>(resource::MeshHandle{1});
    EXPECT_THROW((void)renderer.set_mesh_handle(resource::MeshHandle{}), std::invalid_argument);
}

TEST(FrameworkMeshRendererTest, AllowsCustomBaseColor) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    auto& renderer = go.add_component<MeshRenderer>(resource::MeshHandle{1}, pbpt::math::vec4{0.2f, 0.3f, 0.4f, 1.0f});
    EXPECT_EQ(renderer.base_color(), pbpt::math::vec4(0.2f, 0.3f, 0.4f, 1.0f));
}

TEST(FrameworkMeshRendererTest, SettersUpdateState) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    auto& renderer = go.add_component<MeshRenderer>(resource::MeshHandle{1});

    renderer.set_mesh_handle(resource::MeshHandle{3});
    renderer.set_base_color(pbpt::math::vec4{0.1f, 0.2f, 0.3f, 1.0f});

    EXPECT_EQ(renderer.mesh_handle(), resource::MeshHandle{3});
    EXPECT_EQ(renderer.base_color(), pbpt::math::vec4(0.1f, 0.2f, 0.3f, 1.0f));
}

TEST(FrameworkMeshRendererTest, GameObjectCanAddAndQueryMeshRenderer) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    auto& renderer = go.add_component<MeshRenderer>(resource::MeshHandle{11});

    EXPECT_TRUE(go.has_component<MeshRenderer>());
    EXPECT_EQ(go.get_component<MeshRenderer>(), &renderer);
    EXPECT_EQ(renderer.mesh_handle(), resource::MeshHandle{11});
}

TEST(FrameworkMeshRendererTest, GameObjectEnforcesUniqueMeshRendererType) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    (void)go.add_component<MeshRenderer>(resource::MeshHandle{1});

    EXPECT_THROW(
        (void)go.add_component<MeshRenderer>(resource::MeshHandle{3}),
        std::runtime_error
    );
}

} // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
