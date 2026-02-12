#include <algorithm>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include <glm/gtc/quaternion.hpp>

#include "framework/component/mesh_renderer.hpp"
#include "framework/core/scene.hpp"
#include "framework/integration/forward_scene_view_builder.hpp"

namespace rtr::framework::integration::test {

static void expect_mat4_near(const glm::mat4& lhs, const glm::mat4& rhs, float eps = 1e-5f) {
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            EXPECT_NEAR(lhs[c][r], rhs[c][r], eps);
        }
    }
}

TEST(FrameworkForwardSceneViewBuilderTest, ThrowsWhenNoActiveCamera) {
    core::Scene scene(1, "scene");
    auto& go = scene.create_game_object("mesh");
    (void)go.add_component<component::MeshRenderer>("assets/models/spot.obj", "");

    EXPECT_THROW(
        (void)build_forward_scene_view(scene),
        std::runtime_error
    );
}

TEST(FrameworkForwardSceneViewBuilderTest, ExtractsOnlyActiveNodesWithMeshRenderer) {
    core::Scene scene(1, "scene");
    auto& camera_go = scene.create_game_object("camera");
    (void)scene.camera_manager().create_perspective_camera(camera_go.id());
    ASSERT_TRUE(scene.set_active_camera(camera_go.id()));

    auto& parent = scene.create_game_object("parent");
    auto& child = scene.create_game_object("child");
    auto& alive = scene.create_game_object("alive");
    auto& no_mesh = scene.create_game_object("no_mesh");
    (void)no_mesh;

    ASSERT_TRUE(scene.scene_graph().set_parent(child.id(), parent.id(), false));

    (void)parent.add_component<component::MeshRenderer>("assets/models/spot.obj", "");
    (void)child.add_component<component::MeshRenderer>("assets/models/stanford_bunny.obj", "");
    (void)alive.add_component<component::MeshRenderer>("assets/models/colored_quad.obj", "");

    parent.set_enabled(false);

    const auto view = build_forward_scene_view(scene);
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
    auto& camera_go = scene.create_game_object("camera");
    (void)scene.camera_manager().create_perspective_camera(camera_go.id());
    ASSERT_TRUE(scene.set_active_camera(camera_go.id()));

    auto& mesh_go = scene.create_game_object("mesh");
    (void)mesh_go.add_component<component::MeshRenderer>("assets/models/spot.obj", "");

    auto node = mesh_go.node();
    node.set_local_position({1.0f, 2.0f, 3.0f});
    node.set_local_rotation(glm::angleAxis(glm::radians(35.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
    node.set_local_scale({2.0f, 1.5f, 0.5f});
    scene.scene_graph().update_world_transforms();

    const auto view = build_forward_scene_view(scene);
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
