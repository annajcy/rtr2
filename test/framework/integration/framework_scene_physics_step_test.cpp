#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/component/physics/cloth/cloth_component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/physics/scene_physics_step.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/physics/physics_system.hpp"

namespace rtr::framework::integration::physics::test {

namespace {

rtr::utils::ObjMeshData make_quad_mesh() {
    rtr::utils::ObjMeshData data{};
    data.vertices = {
        {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{1.0f, 0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{1.0f, 1.0f, 0.0f}, {1.0f, 1.0f}, {0.0f, 0.0f, 1.0f}},
        {{0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}},
    };
    data.indices = {0, 1, 2, 0, 2, 3};
    return data;
}

}  // namespace

TEST(ScenePhysicsStepTest, HelperRunsPhysicsSystemAndSyncsClothStateToRenderer) {
    rtr::resource::ResourceManager resources{};
    rtr::system::physics::PhysicsSystem physics_system{};
    const auto handle = resources.create<rtr::resource::DeformableMeshResourceKind>(make_quad_mesh());

    rtr::framework::core::Scene scene(1);
    auto& go = scene.create_game_object("cloth_quad");
    go.node().set_local_position(pbpt::math::Vec3{5.0f, 0.0f, 0.0f});

    auto& renderer = go.add_component<rtr::framework::component::DeformableMeshComponent>(resources, handle);
    auto& cloth = go.add_component<rtr::framework::component::ClothComponent>(
        physics_system.cloth_world(),
        std::vector<rtr::system::physics::VertexID>{0},
        rtr::system::physics::ClothParams{
            .default_vertex_mass = 1.0f,
            .gravity = pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
            .edge_stiffness = 0.0f,
            .bend_stiffness = 0.0f,
            .spring_damping = 0.0f,
            .velocity_damping = 0.0f,
            .substeps = 1u,
        }
    );

    auto& instance = cloth.cloth();
    instance.state.position(0) = instance.topology.rest_position(0);
    instance.state.position(1) = pbpt::math::Vec3{1.0f, 0.5f, 0.0f};
    instance.state.position(2) = pbpt::math::Vec3{1.0f, 1.0f, 0.0f};
    instance.state.position(3) = pbpt::math::Vec3{0.0f, 1.0f, 0.0f};

    step_scene_physics(scene, physics_system, 1.0f / 60.0f);

    const auto local_vertices = renderer.local_vertices();
    ASSERT_EQ(local_vertices.size(), 4u);
    EXPECT_EQ(local_vertices[0], pbpt::math::Vec3(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(local_vertices[1], pbpt::math::Vec3(1.0f, 0.5f, 0.0f));
    EXPECT_EQ(local_vertices[2], pbpt::math::Vec3(1.0f, 1.0f, 0.0f));
}

}  // namespace rtr::framework::integration::physics::test
