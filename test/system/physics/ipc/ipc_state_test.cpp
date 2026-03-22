#include <gtest/gtest.h>

#include "rtr/system/physics/ipc/core/ipc_state.hpp"

namespace rtr::system::physics::ipc::test {

TEST(IPCStateTest, ResizeAndDofCount) {
    IPCState state{};
    state.resize(4);

    EXPECT_EQ(state.vertex_count(), 4u);
    EXPECT_EQ(state.dof_count(), 12u);
    EXPECT_EQ(state.x.size(), 12);
    EXPECT_EQ(state.x_prev.size(), 12);
    EXPECT_EQ(state.v.size(), 12);
    EXPECT_EQ(state.mass_diag.size(), 12);
}

TEST(IPCStateTest, PositionSegment) {
    IPCState state{};
    state.resize(4);

    for (Eigen::Index i = 0; i < state.x.size(); ++i) {
        state.x[i] = static_cast<double>(i);
    }

    EXPECT_TRUE(state.position(0).isApprox(Eigen::Vector3d(0.0, 1.0, 2.0)));
    EXPECT_TRUE(state.position(2).isApprox(Eigen::Vector3d(6.0, 7.0, 8.0)));

    state.position(1) = Eigen::Vector3d(10.0, 11.0, 12.0);
    EXPECT_TRUE(state.x.segment<3>(3).isApprox(Eigen::Vector3d(10.0, 11.0, 12.0)));
}

TEST(IPCStateTest, ResizeZero) {
    IPCState state{};
    state.resize(0);

    EXPECT_EQ(state.vertex_count(), 0u);
    EXPECT_EQ(state.dof_count(), 0u);
    EXPECT_EQ(state.x.size(), 0);
    EXPECT_EQ(state.mass_diag.size(), 0);
}

}  // namespace rtr::system::physics::ipc::test
