#include <stdexcept>

#include "gtest/gtest.h"

#include <glm/vec4.hpp>

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
    EXPECT_EQ(renderer.base_color(), glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
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

TEST(FrameworkPbptMeshTest, MeshRendererAccessorTracksBaseColorUpdates) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    auto& renderer = go.add_component<MeshRenderer>(resource::MeshHandle{10});
    auto& pbpt_mesh = go.add_component<PbptMesh>();

    renderer.set_base_color(glm::vec4(0.2f, 0.4f, 0.6f, 1.0f));
    const auto& exposed_renderer = pbpt_mesh.mesh_renderer();
    EXPECT_EQ(exposed_renderer.base_color(), glm::vec4(0.2f, 0.4f, 0.6f, 1.0f));
}

} // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
