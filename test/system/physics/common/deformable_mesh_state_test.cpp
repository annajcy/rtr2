#include <gtest/gtest.h>
#include "rtr/system/physics/common/deformable_mesh_state.hpp"

using namespace rtr::system::physics;

TEST(DeformableMeshState, BasicUsage) {
    DeformableMeshState state;
    state.positions.push_back({1.0f, 2.0f, 3.0f});
    state.velocities.push_back({0.0f, 0.0f, 0.0f});
    state.masses.push_back(1.0f);
    state.indices.push_back(0);

    EXPECT_EQ(state.vertex_count(), 1);
}
