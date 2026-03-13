#include <gtest/gtest.h>
#include "rtr/system/physics/common/normal_recompute.hpp"

using namespace rtr::system::physics;

TEST(NormalRecompute, SingleQuad) {
    std::vector<pbpt::math::Vec3> positions = {
        {-1.0f, 0.0f, -1.0f},
        { 1.0f, 0.0f, -1.0f},
        { 1.0f, 0.0f,  1.0f},
        {-1.0f, 0.0f,  1.0f}
    };

    std::vector<uint32_t> indices = {
        0, 1, 2, // triangle 1
        2, 3, 0  // triangle 2
    };

    auto normals = recompute_vertex_normals(positions, indices);

    ASSERT_EQ(normals.size(), 4);
    for (const auto& n : normals) {
        EXPECT_NEAR(n.x(), 0.0f, 1e-6f);
        EXPECT_NEAR(n.y(), -1.0f, 1e-6f); // Should point DOWN based on winding
        EXPECT_NEAR(n.z(), 0.0f, 1e-6f);
    }
}
