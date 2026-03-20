#include <algorithm>
#include <cmath>

#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/system/physics/cloth/cloth_mesh_topology_builder.hpp"
#include "rtr/system/physics/cloth/cloth_spring.hpp"

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

const ClothSpring* find_spring(const ClothSpringNetwork& network, ClothSpringKind kind, VertexID a, VertexID b) {
    const auto [v0, v1] = std::minmax(a, b);
    for (const auto& spring : network.springs) {
        const auto [s0, s1] = std::minmax(spring.vertices[0], spring.vertices[1]);
        if (spring.kind == kind && s0 == v0 && s1 == v1) {
            return &spring;
        }
    }
    return nullptr;
}

}  // namespace

TEST(ClothSpringTest, BuildsEdgeAndBendSpringsForQuadMesh) {
    const auto topology = build_cloth_topology(make_quad_positions(), make_quad_indices());
    const auto spring_network = build_cloth_spring_network(topology);

    EXPECT_EQ(spring_network.spring_count(ClothSpringKind::Edge), 5u);
    EXPECT_EQ(spring_network.spring_count(ClothSpringKind::Bend), 1u);

    const auto* edge_spring = find_spring(spring_network, ClothSpringKind::Edge, 0, 1);
    ASSERT_NE(edge_spring, nullptr);
    EXPECT_FLOAT_EQ(edge_spring->rest_length, 1.0f);

    const auto* bend_spring = find_spring(spring_network, ClothSpringKind::Bend, 1, 3);
    ASSERT_NE(bend_spring, nullptr);
    EXPECT_FLOAT_EQ(bend_spring->rest_length, std::sqrt(2.0f));
}

}  // namespace rtr::system::physics::test
