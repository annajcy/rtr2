#include <gtest/gtest.h>
#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/render/renderer.hpp"

using namespace rtr::framework::component;
using namespace rtr::rhi;

TEST(DeformableMeshComponentTest, BasicUsage) {
    const char* env_str = std::getenv("RTR_RUN_GPU_TESTS");
    if (env_str == nullptr || std::string(env_str) != "1") {
        GTEST_SKIP() << "Skipping GPU test. Set RTR_RUN_GPU_TESTS=1 to run.";
        return;
    }

    rtr::system::render::Renderer renderer_engine(100, 100, "DefRendererTest");
    Device& device = renderer_engine.device();

    rtr::resource::ResourceManager resources{};

    rtr::utils::ObjMeshData data;
    data.vertices.push_back(rtr::utils::ObjVertex{ {0,0,0}, {0,0}, {0,1,0} });
    data.vertices.push_back(rtr::utils::ObjVertex{ {1,0,0}, {1,0}, {0,1,0} });
    data.vertices.push_back(rtr::utils::ObjVertex{ {0,1,0}, {0,1}, {0,1,0} });
    data.indices = {0, 1, 2};

    auto handle = resources.create<rtr::resource::DeformableMeshResourceKind>(data);
    EXPECT_TRUE(handle.is_valid());

    rtr::framework::core::Scene scene(1);
    auto& go = scene.create_game_object("test");

    DeformableMeshComponent renderer(go, resources, handle);
    EXPECT_EQ(renderer.mesh_handle(), handle);

    auto mv = renderer.mesh_view(device);
    EXPECT_EQ(mv.index_count, 3);
    EXPECT_TRUE(static_cast<vk::Buffer>(mv.vertex_buffer) != vk::Buffer{});
}
