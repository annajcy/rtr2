#include <algorithm>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include <glm/gtc/quaternion.hpp>

#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view_builder.hpp"

namespace rtr::framework::integration::test {

namespace {

resource::MeshHandle create_test_mesh(resource::ResourceManager& resources) {
    utils::ObjMeshData mesh{};
    mesh.vertices = {
        {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{1.0f, 0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}},
    };
    mesh.indices = {0, 1, 2};
    return resources.create_mesh(std::move(mesh));
}

resource::TextureHandle create_test_texture(resource::ResourceManager& resources) {
    utils::ImageData tex{};
    tex.width = 1;
    tex.height = 1;
    tex.channels = 4;
    tex.pixels = {255, 255, 255, 255};
    return resources.create_texture(std::move(tex), true);
}

void add_renderer(
    framework::core::GameObject& go,
    resource::ResourceManager& resources
) {
    (void)go.add_component<component::MeshRenderer>(
        create_test_mesh(resources),
        create_test_texture(resources)
    );
}

void expect_mat4_near(const glm::mat4& lhs, const glm::mat4& rhs, float eps = 1e-5f) {
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            EXPECT_NEAR(lhs[c][r], rhs[c][r], eps);
        }
    }
}

} // namespace

TEST(FrameworkForwardSceneViewBuilderTest, ThrowsWhenNoActiveCamera) {
    core::Scene scene(1, "scene");
    resource::ResourceManager resources{};
    auto& go = scene.create_game_object("mesh");
    add_renderer(go, resources);

    EXPECT_THROW(
        (void)system::render::build_forward_scene_view(scene, resources),
        std::runtime_error
    );
}

TEST(FrameworkForwardSceneViewBuilderTest, ExtractsOnlyActiveNodesWithMeshRenderer) {
    core::Scene scene(1, "scene");
    resource::ResourceManager resources{};
    auto& camera_go = scene.create_game_object("camera");
    (void)scene.camera_manager().create_perspective_camera(camera_go.id());
    ASSERT_TRUE(scene.set_active_camera(camera_go.id()));

    auto& parent = scene.create_game_object("parent");
    auto& child = scene.create_game_object("child");
    auto& alive = scene.create_game_object("alive");
    auto& no_mesh = scene.create_game_object("no_mesh");
    (void)no_mesh;

    ASSERT_TRUE(scene.scene_graph().set_parent(child.id(), parent.id(), false));

    add_renderer(parent, resources);
    add_renderer(child, resources);
    add_renderer(alive, resources);

    parent.set_enabled(false);

    const auto view = system::render::build_forward_scene_view(scene, resources);
    std::vector<std::uint64_t> ids{};
    ids.reserve(view.renderables.size());
    for (const auto& renderable : view.renderables) {
        ids.emplace_back(renderable.instance_id);
    }

    EXPECT_TRUE(std::find(ids.begin(), ids.end(), static_cast<std::uint64_t>(alive.id())) != ids.end());
    EXPECT_FALSE(std::find(ids.begin(), ids.end(), static_cast<std::uint64_t>(parent.id())) != ids.end());
    EXPECT_FALSE(std::find(ids.begin(), ids.end(), static_cast<std::uint64_t>(child.id())) != ids.end());
}

TEST(FrameworkForwardSceneViewBuilderTest, ComputesModelAndNormalFromWorldTransform) {
    core::Scene scene(1, "scene");
    resource::ResourceManager resources{};
    auto& camera_go = scene.create_game_object("camera");
    (void)scene.camera_manager().create_perspective_camera(camera_go.id());
    ASSERT_TRUE(scene.set_active_camera(camera_go.id()));

    auto& mesh_go = scene.create_game_object("mesh");
    add_renderer(mesh_go, resources);

    auto node = mesh_go.node();
    node.set_local_position({1.0f, 2.0f, 3.0f});
    node.set_local_rotation(glm::angleAxis(glm::radians(35.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
    node.set_local_scale({2.0f, 1.5f, 0.5f});
    scene.scene_graph().update_world_transforms();

    const auto view = system::render::build_forward_scene_view(scene, resources);
    auto it = std::find_if(
        view.renderables.begin(),
        view.renderables.end(),
        [&](const system::render::ForwardSceneRenderable& renderable) {
            return renderable.instance_id == static_cast<std::uint64_t>(mesh_go.id());
        }
    );
    ASSERT_TRUE(it != view.renderables.end());

    const glm::mat4 expected_model = scene.scene_graph().node(mesh_go.id()).world_matrix();
    const glm::mat4 expected_normal = glm::transpose(glm::inverse(expected_model));
    expect_mat4_near(it->model, expected_model);
    expect_mat4_near(it->normal, expected_normal);
}

} // namespace rtr::framework::integration::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
