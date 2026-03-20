#include <stdexcept>

#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/component/physics/cloth/cloth_component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/physics/cloth_scene_sync.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/physics/cloth/cloth_world.hpp"

namespace rtr::framework::component::test {

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

TEST(FrameworkClothComponentTest, RegistersTopologyAndPinnedVerticesIntoClothWorld) {
    resource::ResourceManager resources{};
    system::physics::ClothWorld cloth_world{};
    auto handle = resources.create<resource::DeformableMeshResourceKind>(make_quad_mesh());

    core::Scene scene(1);
    auto& go = scene.create_game_object("cloth");
    go.node().set_local_position(pbpt::math::Vec3{3.0f, 4.0f, 5.0f});
    go.node().set_local_scale(pbpt::math::Vec3{2.0f, 1.0f, 1.0f});

    auto& renderer = go.add_component<DeformableMeshComponent>(resources, handle);
    auto& cloth = go.add_component<ClothComponent>(
        cloth_world,
        std::vector<system::physics::VertexID>{0, 3},
        system::physics::ClothParams{.default_vertex_mass = 2.0f}
    );

    ASSERT_TRUE(renderer.has_valid_mesh());
    ASSERT_TRUE(cloth.has_cloth());
    ASSERT_EQ(cloth_world.cloth_count(), 1u);

    const auto& instance = cloth.cloth();
    EXPECT_EQ(instance.topology.vertex_count, 4);
    EXPECT_EQ(instance.state.vertex_count(), 4u);
    EXPECT_EQ(instance.spring_network.spring_count(system::physics::ClothSpringKind::Edge), 5u);
    EXPECT_EQ(instance.spring_network.spring_count(system::physics::ClothSpringKind::Bend), 1u);
    EXPECT_EQ(instance.topology.rest_position(0), pbpt::math::Vec3(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(instance.topology.rest_position(1), pbpt::math::Vec3(1.0f, 0.0f, 0.0f));
    EXPECT_TRUE(instance.state.pinned(0));
    EXPECT_TRUE(instance.state.pinned(3));
    EXPECT_FALSE(instance.state.pinned(1));
    EXPECT_FLOAT_EQ(instance.state.mass(2), 2.0f);
    EXPECT_FLOAT_EQ(cloth.params().default_vertex_mass, 2.0f);
}

TEST(FrameworkClothComponentTest, DisableUnregistersAndEnableReRegistersCloth) {
    resource::ResourceManager resources{};
    system::physics::ClothWorld cloth_world{};
    auto handle = resources.create<resource::DeformableMeshResourceKind>(make_quad_mesh());

    core::Scene scene(1);
    auto& go = scene.create_game_object("cloth");
    (void)go.add_component<DeformableMeshComponent>(resources, handle);
    auto& cloth = go.add_component<ClothComponent>(cloth_world, std::vector<system::physics::VertexID>{1});

    ASSERT_TRUE(cloth.has_cloth());
    const auto original_id = cloth.cloth_id();
    go.set_component_enabled<ClothComponent>(false);
    EXPECT_FALSE(cloth.has_cloth());
    EXPECT_EQ(cloth_world.cloth_count(), 0u);

    go.set_component_enabled<ClothComponent>(true);
    EXPECT_TRUE(cloth.has_cloth());
    EXPECT_EQ(cloth_world.cloth_count(), 1u);
    EXPECT_NE(cloth.cloth_id(), original_id);
}

TEST(FrameworkClothComponentTest, ClothSceneSyncWritesLocalPositionsToRendererDirectly) {
    resource::ResourceManager resources{};
    system::physics::ClothWorld cloth_world{};
    auto handle = resources.create<resource::DeformableMeshResourceKind>(make_quad_mesh());

    core::Scene scene(1);
    auto& go = scene.create_game_object("cloth");
    auto& renderer = go.add_component<DeformableMeshComponent>(resources, handle);
    auto& cloth = go.add_component<ClothComponent>(cloth_world);

    auto& instance = cloth.cloth();
    instance.state.position(0) = pbpt::math::Vec3{0.0f, 2.0f, 0.0f};
    instance.state.position(1) = pbpt::math::Vec3{1.0f, 0.0f, 0.0f};
    instance.state.position(2) = pbpt::math::Vec3{1.0f, 1.0f, 0.0f};
    instance.state.position(3) = pbpt::math::Vec3{0.0f, 1.0f, 0.0f};

    integration::physics::sync_cloth_to_scene(scene, cloth_world);

    const auto local_vertices = renderer.local_vertices();
    ASSERT_EQ(local_vertices.size(), 4u);
    EXPECT_EQ(local_vertices[0], pbpt::math::Vec3(0.0f, 2.0f, 0.0f));
    EXPECT_EQ(local_vertices[1], pbpt::math::Vec3(1.0f, 0.0f, 0.0f));
}

TEST(FrameworkClothComponentTest, ClothSceneSyncSkipsRendererWriteWhenMeshResourceIsUnavailable) {
    resource::ResourceManager resources{};
    system::physics::ClothWorld cloth_world{};
    auto handle = resources.create<resource::DeformableMeshResourceKind>(make_quad_mesh());

    core::Scene scene(1);
    auto& go = scene.create_game_object("cloth");
    auto& renderer = go.add_component<DeformableMeshComponent>(resources, handle);
    auto& cloth = go.add_component<ClothComponent>(cloth_world);

    auto baseline_vertices = renderer.local_vertices();
    resources.unload<resource::DeformableMeshResourceKind>(handle);

    auto& instance = cloth.cloth();
    instance.state.position(0) = pbpt::math::Vec3{4.0f, 0.0f, 0.0f};

    EXPECT_NO_THROW(integration::physics::sync_cloth_to_scene(scene, cloth_world));
    EXPECT_FALSE(renderer.has_valid_mesh());
    EXPECT_EQ(cloth.cloth().state.vertex_count(), 4u);
    EXPECT_EQ(baseline_vertices.size(), 4u);
}

TEST(FrameworkClothComponentTest, InvalidPinnedVertexThrowsDuringRegistration) {
    resource::ResourceManager resources{};
    system::physics::ClothWorld cloth_world{};
    auto handle = resources.create<resource::DeformableMeshResourceKind>(make_quad_mesh());

    core::Scene scene(1);
    auto& go = scene.create_game_object("cloth");
    (void)go.add_component<DeformableMeshComponent>(resources, handle);

    EXPECT_THROW(
        (void)go.add_component<ClothComponent>(cloth_world, std::vector<system::physics::VertexID>{99}),
        std::out_of_range
    );
}

}  // namespace rtr::framework::component::test
