#include <algorithm>
#include <numeric>

#include <gtest/gtest.h>

#include "rtr/system/physics/ipc/model/tet_body.hpp"

namespace rtr::system::physics::ipc::test {

TEST(TetBodyTest, SingleTetPrecompute) {
    TetBody body{};
    body.geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
    };
    body.geometry.tets = {{{0, 1, 2, 3}}};
    body.density = 12.0;

    body.precompute();

    ASSERT_EQ(body.geometry.Dm_inv.size(), 1u);
    ASSERT_EQ(body.geometry.rest_volumes.size(), 1u);
    ASSERT_EQ(body.vertex_masses.size(), 4u);
    EXPECT_TRUE(body.geometry.Dm_inv[0].isApprox(Eigen::Matrix3d::Identity()));
    EXPECT_NEAR(body.geometry.rest_volumes[0], 1.0 / 6.0, 1e-12);

    for (double mass : body.vertex_masses) {
        EXPECT_NEAR(mass, 0.5, 1e-12);
    }
    EXPECT_EQ(body.info.vertex_count, 4u);
    EXPECT_EQ(body.fixed_vertices.size(), 4u);
}

TEST(TetBodyTest, DegenerateTetThrows) {
    TetBody body{};
    body.geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(1.0, 1.0, 0.0),
    };
    body.geometry.tets = {{{0, 1, 2, 3}}};

    EXPECT_THROW(body.precompute(), std::invalid_argument);
}

TEST(TetBodyTest, TetBlockGeneration) {
    TetBody body = generate_tet_block(2, 2, 2, 1.0, Eigen::Vector3d::Zero());

    EXPECT_EQ(body.vertex_count(), 27u);
    EXPECT_EQ(body.tet_count(), 48u);

    EXPECT_NO_THROW(body.precompute());
    EXPECT_EQ(body.geometry.Dm_inv.size(), body.tet_count());
    EXPECT_EQ(body.geometry.rest_volumes.size(), body.tet_count());
    EXPECT_TRUE(std::all_of(body.geometry.rest_volumes.begin(), body.geometry.rest_volumes.end(), [](double volume) {
        return volume > 0.0;
    }));
}

TEST(TetBodyTest, FixedVerticesMask) {
    TetBody body = generate_tet_block(3, 3, 3, 1.0, Eigen::Vector3d::Zero());
    body.fixed_vertices.assign(body.vertex_count(), false);

    const double top_y = 3.0;
    for (std::size_t i = 0; i < body.vertex_count(); ++i) {
        if (std::abs(body.geometry.rest_positions[i].y() - top_y) < 1e-12) {
            body.fixed_vertices[i] = true;
        }
    }

    body.precompute();

    const auto fixed_count = static_cast<std::size_t>(std::count(body.fixed_vertices.begin(),
                                                                 body.fixed_vertices.end(),
                                                                 true));
    EXPECT_EQ(fixed_count, 16u);
}

}  // namespace rtr::system::physics::ipc::test
