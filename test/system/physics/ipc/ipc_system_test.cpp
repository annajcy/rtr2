#include <gtest/gtest.h>

#include <Eigen/Core>

#include "rtr/system/physics/ipc/core/ipc_system.hpp"

namespace rtr::system::physics::ipc {
namespace {

TetBody make_single_tet_body() {
    TetBody body{};
    body.geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
    };
    body.geometry.tets = {{{0, 1, 2, 3}}};
    body.material = FixedCorotatedMaterial{
        .mass_density = 2.0,
        .youngs_modulus = 100.0,
        .poisson_ratio = 0.3,
    };
    return body;
}

TEST(IPCSystemTest, InitializeAssemblesGlobalStateFromTetBodies) {
    IPCSystem system(IPCConfig{.dt = 0.1, .gravity = Eigen::Vector3d::Zero()});
    TetBody body = make_single_tet_body();

    system.add_tet_body(body);
    system.initialize();

    ASSERT_EQ(system.tet_body_count(), 1u);
    const IPCState& state = system.state();
    EXPECT_EQ(state.dof_count(), 12u);
    EXPECT_TRUE(state.x.isApprox(state.x_prev, 0.0));
    EXPECT_TRUE(state.v.isZero(0.0));
    EXPECT_GT(state.mass_diag.minCoeff(), 0.0);

    const auto positions = system.get_body_positions(0);
    ASSERT_EQ(positions.size(), 4u);
    for (std::size_t i = 0; i < positions.size(); ++i) {
        EXPECT_TRUE(positions[i].isApprox(body.geometry.rest_positions[i], 0.0));
    }

    const auto& stored_body = system.tet_body(0);
    EXPECT_EQ(stored_body.info.dof_offset, 0u);
    EXPECT_EQ(stored_body.info.vertex_count, 4u);
}

TEST(IPCSystemTest, StepMovesFreeBodyUnderGravity) {
    IPCSystem system(IPCConfig{
        .dt = 0.1,
        .gravity = Eigen::Vector3d(0.0, -9.81, 0.0),
        .solver_params = NewtonSolverParams{
            .max_iterations = 8,
            .gradient_tolerance = 1e-8,
            .dx_tolerance = 1e-10,
            .regularization = 1e-6,
        },
    });
    system.add_tet_body(make_single_tet_body());
    system.initialize();

    const auto initial_positions = system.get_body_positions(0);
    system.step();
    const auto next_positions = system.get_body_positions(0);

    ASSERT_EQ(initial_positions.size(), next_positions.size());
    for (std::size_t i = 0; i < initial_positions.size(); ++i) {
        EXPECT_NEAR(next_positions[i].x(), initial_positions[i].x(), 1e-6);
        EXPECT_NEAR(next_positions[i].z(), initial_positions[i].z(), 1e-6);
        EXPECT_NEAR(next_positions[i].y(), initial_positions[i].y() - 0.0981, 1e-4);
    }
}

TEST(IPCSystemTest, FixedVerticesRemainPinnedAndVelocityIsZero) {
    IPCSystem system(IPCConfig{
        .dt = 0.1,
        .gravity = Eigen::Vector3d(0.0, -9.81, 0.0),
        .solver_params = NewtonSolverParams{
            .max_iterations = 12,
            .gradient_tolerance = 1e-8,
            .dx_tolerance = 1e-10,
            .regularization = 1e-6,
        },
    });

    TetBody body = make_single_tet_body();
    body.fixed_vertices = {true, false, false, false};
    const Eigen::Vector3d fixed_rest = body.geometry.rest_positions[0];

    system.add_tet_body(std::move(body));
    system.initialize();
    system.step();

    const auto positions = system.get_body_positions(0);
    ASSERT_EQ(positions.size(), 4u);
    EXPECT_TRUE(positions[0].isApprox(fixed_rest, 1e-8));
    EXPECT_TRUE(system.state().v.segment<3>(0).isZero(1e-12));
    EXPECT_LT(positions[1].y(), 0.0);
}

TEST(IPCSystemTest, EmptySystemStepIsNoOp) {
    IPCSystem system{};
    EXPECT_NO_THROW(system.step());
    EXPECT_EQ(system.state().dof_count(), 0u);
}

}  // namespace
}  // namespace rtr::system::physics::ipc
