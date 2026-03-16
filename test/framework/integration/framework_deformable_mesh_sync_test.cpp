#include <gtest/gtest.h>

#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/physics/deformable_mesh_sync.hpp"
#include "rtr/resource/resource_manager.hpp"

namespace {

rtr::utils::ObjMeshData make_sync_mesh() {
    rtr::utils::ObjMeshData mesh{};
    mesh.vertices = {
        {{0.0f, 0.0f, 0.0f}, {0.1f, 0.2f}, {0.0f, 1.0f, 0.0f}},
        {{1.0f, 0.0f, 0.0f}, {0.3f, 0.4f}, {0.0f, 1.0f, 0.0f}},
        {{0.0f, 1.0f, 0.0f}, {0.5f, 0.6f}, {0.0f, 1.0f, 0.0f}},
    };
    mesh.indices = {0, 1, 2};
    return mesh;
}

} // namespace

TEST(FrameworkDeformableMeshSyncTest, SyncUpdatesCpuVisiblePositionsAndPreservesUvs) {
    rtr::resource::ResourceManager resources{};
    const auto handle = resources.create<rtr::resource::DeformableMeshResourceKind>(make_sync_mesh());

    rtr::framework::core::Scene scene(1);
    auto& go = scene.create_game_object("mesh");
    auto& renderer = go.add_component<rtr::framework::component::DeformableMeshComponent>(resources, handle);

    rtr::system::physics::DeformableMeshState state{};
    state.positions = {
        {0.0f, 0.5f, 0.0f},
        {1.0f, 0.0f, 0.2f},
        {0.0f, 1.2f, 0.0f},
    };
    state.indices = {0, 1, 2};

    rtr::framework::integration::physics::sync_deformable_mesh_to_renderer(state, renderer);

    const auto local_vertices = renderer.local_vertices();
    ASSERT_EQ(local_vertices.size(), state.positions.size());
    for (std::size_t i = 0; i < state.positions.size(); ++i) {
        EXPECT_FLOAT_EQ(local_vertices[i].x(), state.positions[i].x());
        EXPECT_FLOAT_EQ(local_vertices[i].y(), state.positions[i].y());
        EXPECT_FLOAT_EQ(local_vertices[i].z(), state.positions[i].z());
    }

    const auto& cpu_mesh = resources.cpu<rtr::resource::DeformableMeshResourceKind>(handle);
    EXPECT_FLOAT_EQ(cpu_mesh.vertices[0].uv.x(), 0.1f);
    EXPECT_FLOAT_EQ(cpu_mesh.vertices[0].uv.y(), 0.2f);
    EXPECT_FLOAT_EQ(cpu_mesh.vertices[1].uv.x(), 0.3f);
    EXPECT_FLOAT_EQ(cpu_mesh.vertices[1].uv.y(), 0.4f);
    EXPECT_FLOAT_EQ(cpu_mesh.vertices[2].uv.x(), 0.5f);
    EXPECT_FLOAT_EQ(cpu_mesh.vertices[2].uv.y(), 0.6f);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
