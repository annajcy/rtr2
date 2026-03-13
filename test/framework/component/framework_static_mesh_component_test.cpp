#include <pbpt/math/math.h>
#include <stdexcept>

#include "gtest/gtest.h"


#include "rtr/framework/component/material/static_mesh_component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/resource/resource_manager.hpp"

namespace rtr::framework::component::test {

TEST(FrameworkStaticMeshComponentTest, ConstructWithValidHandles) {
    core::Scene scene(1);
    resource::ResourceManager resources{};
    auto& go = scene.create_game_object("mesh");
    auto& renderer = go.add_component<StaticMeshComponent>(resources, resource::MeshHandle{1});
    EXPECT_EQ(renderer.mesh_handle(), resource::MeshHandle{1});
    EXPECT_EQ(renderer.base_color(), pbpt::math::Vec4(1.0f));
}

TEST(FrameworkStaticMeshComponentTest, InvalidMeshHandleThrows) {
    core::Scene scene(1);
    resource::ResourceManager resources{};
    auto& go = scene.create_game_object("mesh");
    EXPECT_THROW(
        (void)go.add_component<StaticMeshComponent>(resources, resource::MeshHandle{}),
        std::invalid_argument
    );

    auto& renderer = go.add_component<StaticMeshComponent>(resources, resource::MeshHandle{1});
    EXPECT_THROW((void)renderer.set_mesh_handle(resource::MeshHandle{}), std::invalid_argument);
}

TEST(FrameworkStaticMeshComponentTest, AllowsCustomBaseColor) {
    core::Scene scene(1);
    resource::ResourceManager resources{};
    auto& go = scene.create_game_object("mesh");
    auto& renderer = go.add_component<StaticMeshComponent>(
        resources, resource::MeshHandle{1}, pbpt::math::Vec4{0.2f, 0.3f, 0.4f, 1.0f});
    EXPECT_EQ(renderer.base_color(), pbpt::math::Vec4(0.2f, 0.3f, 0.4f, 1.0f));
}

TEST(FrameworkStaticMeshComponentTest, SettersUpdateState) {
    core::Scene scene(1);
    resource::ResourceManager resources{};
    auto& go = scene.create_game_object("mesh");
    auto& renderer = go.add_component<StaticMeshComponent>(resources, resource::MeshHandle{1});

    renderer.set_mesh_handle(resource::MeshHandle{3});
    renderer.set_base_color(pbpt::math::Vec4{0.1f, 0.2f, 0.3f, 1.0f});

    EXPECT_EQ(renderer.mesh_handle(), resource::MeshHandle{3});
    EXPECT_EQ(renderer.base_color(), pbpt::math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));
}

TEST(FrameworkStaticMeshComponentTest, GameObjectCanAddAndQueryStaticMeshComponent) {
    core::Scene scene(1);
    resource::ResourceManager resources{};
    auto& go = scene.create_game_object("mesh");
    auto& renderer = go.add_component<StaticMeshComponent>(resources, resource::MeshHandle{11});

    EXPECT_TRUE(go.has_component<StaticMeshComponent>());
    EXPECT_EQ(go.get_component<StaticMeshComponent>(), &renderer);
    EXPECT_EQ(renderer.mesh_handle(), resource::MeshHandle{11});
}

TEST(FrameworkStaticMeshComponentTest, GameObjectEnforcesUniqueStaticMeshComponentType) {
    core::Scene scene(1);
    resource::ResourceManager resources{};
    auto& go = scene.create_game_object("mesh");
    (void)go.add_component<StaticMeshComponent>(resources, resource::MeshHandle{1});

    EXPECT_THROW(
        (void)go.add_component<StaticMeshComponent>(resources, resource::MeshHandle{3}),
        std::runtime_error
    );
}

} // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
