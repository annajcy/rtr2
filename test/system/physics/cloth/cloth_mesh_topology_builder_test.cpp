#include <stdexcept>

#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/system/physics/cloth/cloth_mesh_topology_builder.hpp"

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

EdgeID find_edge_id(const ClothTopology& topology, VertexID a, VertexID b) {
    const auto [v0, v1] = std::minmax(a, b);
    for (std::size_t i = 0; i < topology.edges.size(); ++i) {
        if (topology.edges[i].vertices == std::array<VertexID, 2>{v0, v1}) {
            return static_cast<EdgeID>(i);
        }
    }
    return kInvalidEdgeID;
}

}  // namespace

TEST(ClothMeshTopologyBuilderTest, BuildsUniqueEdgesAndAdjacencyForQuadMesh) {
    const auto topology = build_cloth_topology(make_quad_positions(), make_quad_indices());

    EXPECT_EQ(topology.vertex_count, 4);
    EXPECT_EQ(topology.triangles.size(), 2u);
    EXPECT_EQ(topology.edges.size(), 5u);
    EXPECT_EQ(topology.render_triangle_indices, make_quad_indices());

    const auto diagonal = find_edge_id(topology, 0, 2);
    ASSERT_NE(diagonal, kInvalidEdgeID);
    EXPECT_FALSE(topology.edge_is_boundary(diagonal));
    EXPECT_EQ(topology.edge_adjacent_triangles(diagonal), (std::array<TriangleID, 2>{0, 1}));

    const auto boundary = find_edge_id(topology, 0, 1);
    ASSERT_NE(boundary, kInvalidEdgeID);
    EXPECT_TRUE(topology.edge_is_boundary(boundary));
    EXPECT_EQ(topology.edge_adjacent_triangles(boundary), (std::array<TriangleID, 2>{0, kInvalidTriangleID}));

    EXPECT_EQ(topology.vertex_incident_edges(0).size(), 3u);
    EXPECT_EQ(topology.vertex_incident_triangles(0).size(), 2u);
    EXPECT_EQ(topology.triangle_edges(0).size(), 3u);
}

TEST(ClothMeshTopologyBuilderTest, StoresRestPositionsAndLengthsInLocalSpace) {
    const auto topology = build_cloth_topology(make_quad_positions(), make_quad_indices());

    EXPECT_EQ(topology.rest_position(0), pbpt::math::Vec3(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(topology.rest_position(2), pbpt::math::Vec3(1.0f, 1.0f, 0.0f));

    const auto horizontal = find_edge_id(topology, 0, 1);
    const auto vertical = find_edge_id(topology, 0, 3);
    ASSERT_NE(horizontal, kInvalidEdgeID);
    ASSERT_NE(vertical, kInvalidEdgeID);
    EXPECT_FLOAT_EQ(topology.edge_rest_length(horizontal), 1.0f);
    EXPECT_FLOAT_EQ(topology.edge_rest_length(vertical), 1.0f);
}

TEST(ClothMeshTopologyBuilderTest, RejectsDegenerateTrianglesAndNonManifoldEdges) {
    EXPECT_THROW(
        (void)build_cloth_topology(
            std::vector<pbpt::math::Vec3>{{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}},
            std::vector<std::uint32_t>{0, 0, 1}
        ),
        std::invalid_argument
    );

    EXPECT_THROW(
        (void)build_cloth_topology(
            std::vector<pbpt::math::Vec3>{
                {0.0f, 0.0f, 0.0f},
                {1.0f, 0.0f, 0.0f},
                {0.0f, 1.0f, 0.0f},
                {0.0f, 0.0f, 1.0f},
                {1.0f, 1.0f, 0.0f},
            },
            std::vector<std::uint32_t>{0, 1, 2, 1, 0, 3, 0, 1, 4}
        ),
        std::invalid_argument
    );
}

}  // namespace rtr::system::physics::test
