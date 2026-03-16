#include <gtest/gtest.h>

#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/render/renderer.hpp"

using namespace rtr::framework::component;
using namespace rtr::rhi;

namespace {

rtr::utils::ObjMeshData make_deformable_triangle() {
    rtr::utils::ObjMeshData data{};
    data.vertices = {
        {{0.0f, 0.0f, 0.0f}, {0.2f, 0.3f}, {0.0f, 1.0f, 0.0f}},
        {{1.0f, 0.0f, 0.0f}, {0.4f, 0.5f}, {0.0f, 1.0f, 0.0f}},
        {{0.0f, 1.0f, 0.0f}, {0.6f, 0.7f}, {0.0f, 1.0f, 0.0f}},
    };
    data.indices = {0, 1, 2};
    return data;
}

} // namespace

TEST(DeformableMeshComponentTest, BasicUsage) {
    const char* env_str = std::getenv("RTR_RUN_GPU_TESTS");
    if (env_str == nullptr || std::string(env_str) != "1") {
        GTEST_SKIP() << "Skipping GPU test. Set RTR_RUN_GPU_TESTS=1 to run.";
        return;
    }

    rtr::system::render::Renderer renderer_engine(100, 100, "DefRendererTest");
    Device& device = renderer_engine.device();

    rtr::resource::ResourceManager resources{};
    auto handle = resources.create<rtr::resource::DeformableMeshResourceKind>(make_deformable_triangle());
    EXPECT_TRUE(handle.is_valid());

    rtr::framework::core::Scene scene(1);
    auto& go = scene.create_game_object("test");

    DeformableMeshComponent renderer(go, resources, handle);
    EXPECT_EQ(renderer.mesh_handle(), handle);

    auto mv = renderer.mesh_view(device);
    EXPECT_EQ(mv.index_count, 3);
    EXPECT_TRUE(static_cast<vk::Buffer>(mv.vertex_buffer) != vk::Buffer{});
}

TEST(DeformableMeshComponentTest, ApplyDeformedSurfaceUpdatesCpuStateAndPreservesUvs) {
    rtr::resource::ResourceManager resources{};
    auto handle = resources.create<rtr::resource::DeformableMeshResourceKind>(make_deformable_triangle());

    rtr::framework::core::Scene scene(1);
    auto& go = scene.create_game_object("test");

    DeformableMeshComponent renderer(go, resources, handle);

    const std::vector<pbpt::math::Vec3> positions = {
        {0.0f, 0.2f, 0.0f},
        {1.0f, 0.3f, 0.0f},
        {0.0f, 1.4f, 0.0f},
    };
    const std::vector<pbpt::math::Vec3> normals = {
        {0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, 1.0f},
    };

    renderer.apply_deformed_surface(positions, normals);

    const auto local_vertices = renderer.local_vertices();
    ASSERT_EQ(local_vertices.size(), positions.size());
    for (std::size_t i = 0; i < positions.size(); ++i) {
        EXPECT_FLOAT_EQ(local_vertices[i].x(), positions[i].x());
        EXPECT_FLOAT_EQ(local_vertices[i].y(), positions[i].y());
        EXPECT_FLOAT_EQ(local_vertices[i].z(), positions[i].z());
    }

    const auto& cpu_mesh = resources.cpu<rtr::resource::DeformableMeshResourceKind>(handle);
    ASSERT_EQ(cpu_mesh.vertices.size(), 3u);
    EXPECT_FLOAT_EQ(cpu_mesh.vertices[0].uv.x(), 0.2f);
    EXPECT_FLOAT_EQ(cpu_mesh.vertices[0].uv.y(), 0.3f);
    EXPECT_FLOAT_EQ(cpu_mesh.vertices[1].uv.x(), 0.4f);
    EXPECT_FLOAT_EQ(cpu_mesh.vertices[1].uv.y(), 0.5f);
    EXPECT_FLOAT_EQ(cpu_mesh.vertices[2].uv.x(), 0.6f);
    EXPECT_FLOAT_EQ(cpu_mesh.vertices[2].uv.y(), 0.7f);

    const char* env_str = std::getenv("RTR_RUN_GPU_TESTS");
    if (env_str != nullptr && std::string(env_str) == "1") {
        rtr::system::render::Renderer renderer_engine(100, 100, "DefRendererUpdateTest");
        Device& device = renderer_engine.device();
        auto mv = renderer.mesh_view(device);
        EXPECT_EQ(mv.index_count, 3);
        EXPECT_TRUE(static_cast<vk::Buffer>(mv.vertex_buffer) != vk::Buffer{});
    }
}
