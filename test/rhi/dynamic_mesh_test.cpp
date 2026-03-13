#include <gtest/gtest.h>
#include "rtr/rhi/dynamic_mesh.hpp"
#include "rtr/system/render/renderer.hpp"

using namespace rtr::rhi;

// Testing GPU requires RTR_RUN_GPU_TESTS=1
TEST(DynamicMeshTest, DeviceLocalAndHostVisible) {
    const char* env_str = std::getenv("RTR_RUN_GPU_TESTS");
    if (env_str == nullptr || std::string(env_str) != "1") {
        GTEST_SKIP() << "Skipping GPU test. Set RTR_RUN_GPU_TESTS=1 to run.";
        return;
    }

    rtr::system::render::Renderer renderer(100, 100, "DynamicMeshTest");
    Device& device = renderer.device();

    rtr::utils::ObjMeshData data;
    data.vertices.push_back(rtr::utils::ObjVertex{ {0,0,0}, {0,0}, {0,1,0} });
    data.vertices.push_back(rtr::utils::ObjVertex{ {1,0,0}, {1,0}, {0,1,0} });
    data.vertices.push_back(rtr::utils::ObjVertex{ {0,1,0}, {0,1}, {0,1,0} });
    data.indices = {0, 1, 2};

    DynamicMesh mesh(device, data);
    EXPECT_EQ(mesh.vertex_count(), 3);
    EXPECT_EQ(mesh.index_count(), 3);
    EXPECT_TRUE(static_cast<vk::Buffer>(mesh.vertex_buffer()) != vk::Buffer{});
    EXPECT_TRUE(static_cast<vk::Buffer>(mesh.index_buffer())  != vk::Buffer{});

    // Test update
    rtr::utils::ObjVertex v0 = data.vertices[0];
    v0.position.x() = 5.0f;
    std::vector<rtr::utils::ObjVertex> new_verts = { v0, data.vertices[1], data.vertices[2] };
    mesh.update_vertices(new_verts);
}
