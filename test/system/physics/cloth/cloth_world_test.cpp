#include <cmath>

#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/system/physics/cloth/cloth_mesh_topology_builder.hpp"
#include "rtr/system/physics/cloth/cloth_world.hpp"

namespace rtr::system::physics::test {

namespace {

std::vector<pbpt::math::Vec3> make_quad_positions() {
    return {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {1.0f, 1.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
    };
}

std::vector<std::uint32_t> make_quad_indices() {
    return {0, 1, 2, 0, 2, 3};
}

ClothInstance make_quad_instance(ClothParams params = {}) {
    auto topology = build_cloth_topology(make_quad_positions(), make_quad_indices());
    auto state = ClothState::from_topology(topology, params.default_vertex_mass);
    return ClothInstance{
        .topology = std::move(topology),
        .state = std::move(state),
        .params = params,
    };
}

pbpt::math::Float edge_length(const ClothInstance& instance, VertexID a, VertexID b) {
    return pbpt::math::length(instance.state.position(b) - instance.state.position(a));
}

pbpt::math::Vec3 centroid(const ClothInstance& instance) {
    pbpt::math::Vec3 sum{0.0f, 0.0f, 0.0f};
    for (const auto& position : instance.state.positions) {
        sum += position;
    }
    return sum / static_cast<float>(instance.state.positions.size());
}

}  // namespace

TEST(ClothWorldTest, PinnedVerticesRemainAtRestWithZeroVelocity) {
    ClothWorld cloth_world{};
    auto instance = make_quad_instance(ClothParams{
        .default_vertex_mass = 1.0f,
        .gravity = pbpt::math::Vec3{0.0f, -9.81f, 0.0f},
        .edge_stiffness = 800.0f,
        .bend_stiffness = 80.0f,
        .spring_damping = 8.0f,
        .velocity_damping = 0.02f,
        .substeps = 8u,
    });
    instance.state.set_pinned(0, true);
    instance.state.position(0) = pbpt::math::Vec3{5.0f, 5.0f, 5.0f};
    instance.state.velocity(0) = pbpt::math::Vec3{1.0f, 2.0f, 3.0f};

    const auto cloth_id = cloth_world.create_cloth(std::move(instance));
    cloth_world.step(1.0f / 30.0f);
    cloth_world.step(1.0f / 30.0f);

    const auto& cloth = cloth_world.get_cloth(cloth_id);
    EXPECT_EQ(cloth.state.position(0), cloth.topology.rest_position(0));
    EXPECT_EQ(cloth.state.velocity(0), pbpt::math::Vec3(0.0f, 0.0f, 0.0f));
}

TEST(ClothWorldTest, GravityMovesCentroidDownWithoutIntroducingNonFiniteValues) {
    ClothWorld cloth_world{};
    auto instance = make_quad_instance(ClothParams{
        .default_vertex_mass = 1.0f,
        .gravity = pbpt::math::Vec3{0.0f, -9.81f, 0.0f},
        .edge_stiffness = 800.0f,
        .bend_stiffness = 80.0f,
        .spring_damping = 8.0f,
        .velocity_damping = 0.02f,
        .substeps = 8u,
    });
    instance.state.set_pinned(0, true);
    instance.state.set_pinned(3, true);

    const auto before_centroid_y = centroid(instance).y();
    const auto cloth_id = cloth_world.create_cloth(std::move(instance));
    for (int i = 0; i < 30; ++i) {
        cloth_world.step(1.0f / 60.0f);
    }

    const auto& cloth = cloth_world.get_cloth(cloth_id);
    EXPECT_LT(centroid(cloth).y(), before_centroid_y);

    for (const auto& position : cloth.state.positions) {
        EXPECT_TRUE(std::isfinite(position.x()));
        EXPECT_TRUE(std::isfinite(position.y()));
        EXPECT_TRUE(std::isfinite(position.z()));
    }
    for (const auto& velocity : cloth.state.velocities) {
        EXPECT_TRUE(std::isfinite(velocity.x()));
        EXPECT_TRUE(std::isfinite(velocity.y()));
        EXPECT_TRUE(std::isfinite(velocity.z()));
    }
}

TEST(ClothWorldTest, StretchedEdgeMovesTowardRestLength) {
    ClothWorld cloth_world{};
    auto instance = make_quad_instance(ClothParams{
        .default_vertex_mass = 1.0f,
        .gravity = pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        .edge_stiffness = 1200.0f,
        .bend_stiffness = 80.0f,
        .spring_damping = 12.0f,
        .velocity_damping = 0.05f,
        .substeps = 8u,
    });
    instance.state.set_pinned(0, true);
    instance.state.set_pinned(3, true);
    instance.state.position(1) = pbpt::math::Vec3{1.8f, 0.0f, 0.0f};
    instance.state.position(2) = pbpt::math::Vec3{1.8f, 1.0f, 0.0f};

    const auto initial_error = std::abs(edge_length(instance, 0, 1) - instance.topology.edge_rest_length(0));
    const auto cloth_id = cloth_world.create_cloth(std::move(instance));
    for (int i = 0; i < 60; ++i) {
        cloth_world.step(1.0f / 60.0f);
    }

    const auto& cloth = cloth_world.get_cloth(cloth_id);
    const auto final_error = std::abs(edge_length(cloth, 0, 1) - cloth.topology.edge_rest_length(0));
    EXPECT_LT(final_error, initial_error);
}

}  // namespace rtr::system::physics::test
