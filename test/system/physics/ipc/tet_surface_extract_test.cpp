#include <gtest/gtest.h>
#include "rtr/system/physics/ipc/model/tet_surface_extract.hpp"
#include <array>
#include <vector>

using namespace rtr::system::physics::ipc;

TEST(TetSurfaceExtract, SingleTetrahedron) {
    uint32_t vertex_count = 4;
    std::vector<std::array<uint32_t, 4>> tets = {
        {0, 1, 2, 3}
    };

    auto result = extract_tet_surface(vertex_count, tets);

    EXPECT_EQ(result.surface_indices.size(), 12); // 4 faces * 3 vertices
    EXPECT_EQ(result.surface_vertex_ids.size(), 4);
    EXPECT_EQ(result.surface_vertex_ids[0], 0);
    EXPECT_EQ(result.surface_vertex_ids[3], 3);
}

TEST(TetSurfaceExtract, TwoTetrahedronsSharingFace) {
    uint32_t vertex_count = 5;
    std::vector<std::array<uint32_t, 4>> tets = {
        {0, 1, 2, 3}, // base: 0,1,2, top: 3. Face 0,1,2 shared
        {0, 2, 1, 4}  // base: 0,2,1, top: 4 (note: opposite winding on base avoids normal flip)
    };

    auto result = extract_tet_surface(vertex_count, tets);

    // 2 tets = 8 faces total, 2 faces shared (1 from each), so 6 surface faces * 3 = 18 indices
    EXPECT_EQ(result.surface_indices.size(), 18);
    EXPECT_EQ(result.surface_vertex_ids.size(), 5);
}
