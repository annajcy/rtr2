#include <pbpt/math/math.h>

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/framework/component/light/point_light.hpp"
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/rhi/context.hpp"
#include "rtr/rhi/device.hpp"
#include "rtr/rhi/window.hpp"
#include "rtr/system/render/pipeline/forward/forward_pipeline.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view_builder.hpp"

namespace rtr::framework::integration::test {

namespace {

bool gpu_tests_enabled() {
    const char* value = std::getenv("RTR_RUN_GPU_TESTS");
    return value != nullptr && std::string(value) == "1";
}

rhi::ContextCreateInfo make_context_create_info(rhi::Window& window) {
    rhi::ContextCreateInfo info{};
    info.app_name = "FrameworkForwardSceneViewBuilderTest";
    info.instance_extensions = window.required_extensions();
    info.surface_creator = [&window](const vk::raii::Instance& instance) {
        return window.create_vk_surface(instance);
    };
    info.enable_validation_layers = false;
    return info;
}

struct GpuHarness {
    rhi::Window window;
    rhi::Context context;
    rhi::Device device;

    GpuHarness()
        : window(640, 480, "framework_forward_scene_view_builder_test"),
          context(make_context_create_info(window)),
          device(context) {}

    ~GpuHarness() {
        try {
            device.wait_idle();
        } catch (...) {
        }
    }
};

resource::MeshHandle create_test_mesh(resource::ResourceManager& resources) {
    utils::ObjMeshData mesh{};
    mesh.vertices = {
        {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{1.0f, 0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}},
    };
    mesh.indices = {0, 1, 2};
    return resources.create<rtr::resource::MeshResourceKind>(std::move(mesh));
}

void add_renderer(framework::core::GameObject& go, resource::ResourceManager& resources) {
    (void)go.add_component<component::MeshRenderer>(create_test_mesh(resources));
}

void add_renderer_with_color(framework::core::GameObject& go, resource::ResourceManager& resources) {
    (void)go.add_component<component::MeshRenderer>(create_test_mesh(resources),
                                                    pbpt::math::vec4{0.3f, 0.4f, 0.5f, 1.0f});
}

void add_active_camera(core::Scene& scene, const std::string& name = "camera") {
    auto& camera_go = scene.create_game_object(name);
    auto& camera    = camera_go.add_component<component::PerspectiveCamera>();
    camera.set_active(true);
}

void expect_mat4_near(const pbpt::math::mat4& lhs, const pbpt::math::mat4& rhs, float eps = 1e-5f) {
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            EXPECT_NEAR(lhs[c][r], rhs[c][r], eps);
        }
    }
}

pbpt::math::vec4 multiply_packed(const system::render::GpuMat4& matrix, const pbpt::math::vec4& vector) {
    pbpt::math::vec4 result{0.0f, 0.0f, 0.0f, 0.0f};
    for (int row = 0; row < 4; ++row) {
        float value = 0.0f;
        for (int col = 0; col < 4; ++col) {
            value += matrix.values[static_cast<std::size_t>(row * 4 + col)] * vector[col];
        }
        result[row] = value;
    }
    return result;
}

}  // namespace

TEST(FrameworkForwardSceneViewBuilderTest, CollectsPointLightsAndCameraPos) {
    if (!gpu_tests_enabled()) {
        GTEST_SKIP() << "Set RTR_RUN_GPU_TESTS=1 to run integration GPU tests.";
    }
    GpuHarness               harness;
    core::Scene               scene(1);
    resource::ResourceManager resources{};
    auto&                     camera_go = scene.create_game_object("camera");
    auto&                     camera    = camera_go.add_component<component::PerspectiveCamera>();
    camera.set_active(true);
    camera_go.node().set_world_position({10.0f, 20.0f, 30.0f});

    auto& light_go1 = scene.create_game_object("light1");
    light_go1.node().set_world_position({1.0f, 0.0f, 0.0f});
    light_go1.add_component<component::light::PointLight>().set_intensity(10.0f);

    auto& light_go_disabled = scene.create_game_object("light_disabled");
    auto& disabled_pl       = light_go_disabled.add_component<component::light::PointLight>();
    disabled_pl.set_enabled(false);

    auto& light_go2 = scene.create_game_object("light2");
    light_go2.node().set_world_position({2.0f, 0.0f, 0.0f});
    light_go2.add_component<component::light::PointLight>().set_intensity(20.0f);

    auto& light_go3 = scene.create_game_object("light3");
    light_go3.add_component<component::light::PointLight>();
    auto& light_go4 = scene.create_game_object("light4");
    light_go4.add_component<component::light::PointLight>();

    auto& light_go5 = scene.create_game_object("light5");
    light_go5.add_component<component::light::PointLight>().set_intensity(50.0f);

    const auto view = system::render::build_forward_scene_view(scene, resources, harness.device);

    EXPECT_FLOAT_EQ(view.camera.world_pos.x(), 10.0f);
    EXPECT_FLOAT_EQ(view.camera.world_pos.y(), 20.0f);
    EXPECT_FLOAT_EQ(view.camera.world_pos.z(), 30.0f);

    ASSERT_EQ(view.point_lights.size(), 4u);
    EXPECT_FLOAT_EQ(view.point_lights[0].intensity, 10.0f);
    EXPECT_FLOAT_EQ(view.point_lights[0].position.x(), 1.0f);
    EXPECT_FLOAT_EQ(view.point_lights[1].intensity, 20.0f);
    EXPECT_FLOAT_EQ(view.point_lights[1].position.x(), 2.0f);
}

TEST(FrameworkForwardSceneViewBuilderTest, ReturnsBlackFrameWhenNoActiveCamera) {
    if (!gpu_tests_enabled()) {
        GTEST_SKIP() << "Set RTR_RUN_GPU_TESTS=1 to run integration GPU tests.";
    }
    GpuHarness               harness;
    core::Scene               scene(1);
    resource::ResourceManager resources{};
    auto&                     go = scene.create_game_object("mesh");
    add_renderer(go, resources);

    const auto view = system::render::build_forward_scene_view(scene, resources, harness.device);
    EXPECT_TRUE(view.renderables.empty());
    EXPECT_TRUE(view.point_lights.empty());
}

TEST(FrameworkForwardSceneViewBuilderTest, ReturnsBlackFrameWhenMultipleActiveCameras) {
    if (!gpu_tests_enabled()) {
        GTEST_SKIP() << "Set RTR_RUN_GPU_TESTS=1 to run integration GPU tests.";
    }
    GpuHarness               harness;
    core::Scene               scene(1);
    resource::ResourceManager resources{};
    add_active_camera(scene, "camera_a");
    add_active_camera(scene, "camera_b");
    auto& mesh_go = scene.create_game_object("mesh");
    add_renderer(mesh_go, resources);
    auto& light_go = scene.create_game_object("light");
    (void)light_go.add_component<component::light::PointLight>();

    const auto view = system::render::build_forward_scene_view(scene, resources, harness.device);
    EXPECT_TRUE(view.renderables.empty());
    EXPECT_TRUE(view.point_lights.empty());
}

TEST(FrameworkForwardSceneViewBuilderTest, ExtractsOnlyActiveNodesWithMeshRenderer) {
    if (!gpu_tests_enabled()) {
        GTEST_SKIP() << "Set RTR_RUN_GPU_TESTS=1 to run integration GPU tests.";
    }
    GpuHarness               harness;
    core::Scene               scene(1);
    resource::ResourceManager resources{};
    add_active_camera(scene);

    auto& parent  = scene.create_game_object("parent");
    auto& child   = scene.create_game_object("child");
    auto& alive   = scene.create_game_object("alive");
    auto& no_mesh = scene.create_game_object("no_mesh");
    (void)no_mesh;

    ASSERT_TRUE(scene.scene_graph().set_parent(child.id(), parent.id(), false));

    add_renderer(parent, resources);
    add_renderer(child, resources);
    add_renderer(alive, resources);

    parent.set_enabled(false);

    const auto                 view = system::render::build_forward_scene_view(scene, resources, harness.device);
    std::vector<std::uint64_t> ids{};
    ids.reserve(view.renderables.size());
    for (const auto& renderable : view.renderables) {
        ids.emplace_back(renderable.instance_id);
    }

    EXPECT_TRUE(std::find(ids.begin(), ids.end(), static_cast<std::uint64_t>(alive.id())) != ids.end());
    EXPECT_FALSE(std::find(ids.begin(), ids.end(), static_cast<std::uint64_t>(parent.id())) != ids.end());
    EXPECT_FALSE(std::find(ids.begin(), ids.end(), static_cast<std::uint64_t>(child.id())) != ids.end());
}

TEST(FrameworkForwardSceneViewBuilderTest, DisabledMeshRendererIsExcludedFromRenderables) {
    if (!gpu_tests_enabled()) {
        GTEST_SKIP() << "Set RTR_RUN_GPU_TESTS=1 to run integration GPU tests.";
    }
    GpuHarness               harness;
    core::Scene               scene(1);
    resource::ResourceManager resources{};
    add_active_camera(scene);

    auto& obj1 = scene.create_game_object("obj1");
    add_renderer(obj1, resources);
    auto& obj2 = scene.create_game_object("obj2");
    add_renderer(obj2, resources);

    auto* renderer2 = obj2.get_component<component::MeshRenderer>();
    ASSERT_NE(renderer2, nullptr);
    renderer2->set_enabled(false);

    const auto view = system::render::build_forward_scene_view(scene, resources, harness.device);
    ASSERT_EQ(view.renderables.size(), 1u);
    EXPECT_EQ(view.renderables[0].instance_id, static_cast<std::uint64_t>(obj1.id()));
}

TEST(FrameworkForwardSceneViewBuilderTest, ComputesModelAndNormalFromWorldTransform) {
    if (!gpu_tests_enabled()) {
        GTEST_SKIP() << "Set RTR_RUN_GPU_TESTS=1 to run integration GPU tests.";
    }
    GpuHarness               harness;
    core::Scene               scene(1);
    resource::ResourceManager resources{};
    add_active_camera(scene);

    auto& mesh_go = scene.create_game_object("mesh");
    add_renderer(mesh_go, resources);

    auto node = mesh_go.node();
    node.set_local_position({1.0f, 2.0f, 3.0f});
    node.set_local_rotation(pbpt::math::angleAxis(pbpt::math::radians(35.0f), pbpt::math::vec3(0.0f, 1.0f, 0.0f)));
    node.set_local_scale({2.0f, 1.5f, 0.5f});

    const auto view = system::render::build_forward_scene_view(scene, resources, harness.device);
    auto       it   = std::find_if(view.renderables.begin(), view.renderables.end(),
                                   [&](const system::render::ForwardSceneRenderable& renderable) {
                                       return renderable.instance_id == static_cast<std::uint64_t>(mesh_go.id());
                                   });
    ASSERT_TRUE(it != view.renderables.end());

    const pbpt::math::mat4 expected_model  = scene.scene_graph().node(mesh_go.id()).world_matrix();
    const pbpt::math::mat4 expected_normal = pbpt::math::transpose(pbpt::math::inverse(expected_model));
    expect_mat4_near(it->model, expected_model);
    expect_mat4_near(it->normal, expected_normal);
}

TEST(FrameworkForwardSceneViewBuilderTest, SupportsBaseColorPath) {
    if (!gpu_tests_enabled()) {
        GTEST_SKIP() << "Set RTR_RUN_GPU_TESTS=1 to run integration GPU tests.";
    }
    GpuHarness               harness;
    core::Scene               scene(1);
    resource::ResourceManager resources{};
    add_active_camera(scene);

    auto& mesh_go = scene.create_game_object("mesh");
    add_renderer_with_color(mesh_go, resources);

    const auto view = system::render::build_forward_scene_view(scene, resources, harness.device);
    ASSERT_EQ(view.renderables.size(), 1u);
    EXPECT_EQ(view.renderables[0].base_color, pbpt::math::vec4(0.3f, 0.4f, 0.5f, 1.0f));
}

TEST(FrameworkForwardSceneViewBuilderTest, ForwardGpuPackingUsesStableRowMajorOrder) {
    pbpt::math::mat4 matrix{1.0f};
    float            value = 1.0f;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            matrix[row][col] = value++;
        }
    }

    const system::render::GpuMat4 packed = system::render::pack_mat4_row_major(matrix);
    ASSERT_EQ(packed.values.size(), 16u);

    for (std::size_t i = 0; i < packed.values.size(); ++i) {
        EXPECT_NEAR(packed.values[i], static_cast<float>(i + 1), 1e-5f);
    }
}

TEST(FrameworkForwardSceneViewBuilderTest, PackedMatrixChainMatchesCpuClipComputation) {
    pbpt::math::mat4 model = pbpt::math::translate(pbpt::math::mat4{1.0f}, pbpt::math::vec3{1.5f, -0.25f, 2.0f});
    model                  = model * pbpt::math::mat4_cast(pbpt::math::angleAxis(
                        pbpt::math::radians(23.0f), pbpt::math::normalize(pbpt::math::vec3{0.2f, 1.0f, 0.4f})));
    model                  = pbpt::math::scale(model, pbpt::math::vec3{1.2f, 0.8f, 1.5f});

    pbpt::math::mat4 view = pbpt::math::lookAt(pbpt::math::vec3{4.0f, 3.0f, -7.0f}, pbpt::math::vec3{0.0f, 0.0f, 0.0f},
                                               pbpt::math::vec3{0.0f, 1.0f, 0.0f});
    pbpt::math::mat4 proj = pbpt::math::perspective(pbpt::math::radians(45.0f), 1.3f, 0.1f, 100.0f);
    proj[1][1] *= -1.0f;

    const pbpt::math::vec4 position{0.3f, -0.7f, 1.1f, 1.0f};

    const pbpt::math::vec4 cpu_clip = proj * (view * (model * position));

    const system::render::GpuMat4 model_gpu = system::render::pack_mat4_row_major(model);
    const system::render::GpuMat4 view_gpu  = system::render::pack_mat4_row_major(view);
    const system::render::GpuMat4 proj_gpu  = system::render::pack_mat4_row_major(proj);
    const pbpt::math::vec4        gpu_clip =
        multiply_packed(proj_gpu, multiply_packed(view_gpu, multiply_packed(model_gpu, position)));

    for (int i = 0; i < 4; ++i) {
        EXPECT_NEAR(cpu_clip[i], gpu_clip[i], 1e-4f);
    }
}

}  // namespace rtr::framework::integration::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
