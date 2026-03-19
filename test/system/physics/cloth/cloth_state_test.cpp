#include <cmath>
#include <stdexcept>

#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/system/physics/cloth/cloth_state.hpp"

namespace rtr::system::physics::test {

namespace {

ClothTopology make_topology() {
    ClothTopology topology{};
    topology.vertex_count = 3;
    topology.triangles = {
        ClothTriangle{.vertices = {0, 1, 2}, .edges = {0, 1, 2}},
    };
    topology.edges = {
        ClothEdge{.vertices = {0, 1}, .adjacent_triangles = {0, kInvalidTriangleID}},
        ClothEdge{.vertices = {1, 2}, .adjacent_triangles = {0, kInvalidTriangleID}},
        ClothEdge{.vertices = {0, 2}, .adjacent_triangles = {0, kInvalidTriangleID}},
    };
    topology.vertex_edge_adjacency = {{0, 2}, {0, 1}, {1, 2}};
    topology.vertex_triangle_adjacency = {{0}, {0}, {0}};
    topology.rest_positions = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
    };
    topology.edge_rest_lengths = {1.0f, std::sqrt(2.0f), 1.0f};
    topology.render_triangle_indices = {0, 1, 2};
    return topology;
}

}  // namespace

TEST(ClothStateTest, InitializesFromTopologyAndSupportsQueries) {
    const auto topology = make_topology();
    auto state = ClothState::from_topology(topology, 2.5f);

    ASSERT_EQ(state.vertex_count(), 3u);
    EXPECT_EQ(state.position(1), pbpt::math::Vec3(1.0f, 0.0f, 0.0f));
    EXPECT_EQ(state.velocity(2), pbpt::math::Vec3(0.0f, 0.0f, 0.0f));
    EXPECT_FLOAT_EQ(state.mass(0), 2.5f);
    EXPECT_FALSE(state.pinned(2));

    state.position(1) = pbpt::math::Vec3{2.0f, 0.0f, 0.0f};
    state.velocity(2) = pbpt::math::Vec3{0.0f, 3.0f, 0.0f};
    state.set_pinned(0, true);

    EXPECT_TRUE(state.pinned(0));
    EXPECT_EQ(state.edge_positions(topology, 0), (std::array<pbpt::math::Vec3, 2>{
        pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        pbpt::math::Vec3{2.0f, 0.0f, 0.0f},
    }));
    EXPECT_EQ(state.triangle_velocities(topology, 0), (std::array<pbpt::math::Vec3, 3>{
        pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        pbpt::math::Vec3{0.0f, 3.0f, 0.0f},
    }));
}

TEST(ClothStateTest, InvalidQueriesThrow) {
    const auto topology = make_topology();
    auto state = ClothState::from_topology(topology);

    EXPECT_THROW((void)state.position(kInvalidVertexID), std::out_of_range);
    EXPECT_THROW((void)state.mass(10), std::out_of_range);
    EXPECT_THROW((void)state.edge_positions(topology, 10), std::out_of_range);
    EXPECT_THROW((void)state.triangle_positions(topology, 10), std::out_of_range);
}

}  // namespace rtr::system::physics::test
