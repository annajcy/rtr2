#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "rtr/system/physics/ipc/core/ipc_state.hpp"
#include "rtr/system/physics/ipc/energy/inertial_energy.hpp"
#include "rtr/system/physics/ipc/energy/material_energy.hpp"
#include "rtr/system/physics/ipc/energy/material_model/tet_fixed_corotated.hpp"
#include "rtr/system/physics/ipc/model/tet_body.hpp"
#include "rtr/system/physics/ipc/solver/newton_solver.hpp"

namespace rtr::system::physics::ipc {
namespace {

NewtonProblem make_inertial_problem(const IPCState& state, const Eigen::VectorXd& x_hat, double dt) {
    return NewtonProblem{
        .compute_energy =
            [&state, &x_hat, dt](const Eigen::VectorXd& x) {
                return InertialEnergy::compute_energy(InertialEnergy::Input{
                    .x = x,
                    .x_hat = x_hat,
                    .mass_diag = state.mass_diag,
                    .dt = dt,
                });
            },
        .compute_gradient =
            [&state, &x_hat, dt](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
                InertialEnergy::compute_gradient(InertialEnergy::Input{
                    .x = x,
                    .x_hat = x_hat,
                    .mass_diag = state.mass_diag,
                    .dt = dt,
                }, gradient);
            },
        .compute_hessian_triplets =
            [&state, &x_hat, dt](const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) {
                InertialEnergy::compute_hessian_triplets(InertialEnergy::Input{
                    .x = x,
                    .x_hat = x_hat,
                    .mass_diag = state.mass_diag,
                    .dt = dt,
                }, triplets);
            },
    };
}

TetBody make_solver_test_body() {
    TetBody body{};
    body.geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
    };
    body.geometry.tets = {{{0, 1, 2, 3}}};
    body.density = 2.0;
    body.youngs_modulus = 200.0;
    body.poisson_ratio = 0.3;
    body.precompute();
    body.info.dof_offset = 0;
    return body;
}

NewtonProblem make_total_problem(const IPCState& state,
                                 const Eigen::VectorXd& x_hat,
                                 double dt,
                                 const TetBody& body,
                                 const FixedCorotatedMaterial& material) {
    return NewtonProblem{
        .compute_energy =
            [&state, &x_hat, dt, &body, &material](const Eigen::VectorXd& x) {
                return InertialEnergy::compute_energy(InertialEnergy::Input{
                           .x = x,
                           .x_hat = x_hat,
                           .mass_diag = state.mass_diag,
                           .dt = dt,
                       }) +
                       MaterialEnergy<FixedCorotatedMaterial>::compute_energy(
                           MaterialEnergy<FixedCorotatedMaterial>::Input{
                               .body = body,
                               .x = x,
                               .material = material,
                           }
                       );
            },
        .compute_gradient =
            [&state, &x_hat, dt, &body, &material](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
                InertialEnergy::compute_gradient(InertialEnergy::Input{
                    .x = x,
                    .x_hat = x_hat,
                    .mass_diag = state.mass_diag,
                    .dt = dt,
                }, gradient);
                MaterialEnergy<FixedCorotatedMaterial>::compute_gradient(
                    MaterialEnergy<FixedCorotatedMaterial>::Input{
                        .body = body,
                        .x = x,
                        .material = material,
                    },
                    gradient
                );
            },
        .compute_hessian_triplets =
            [&state, &x_hat, dt, &body, &material](const Eigen::VectorXd& x,
                                                   std::vector<Eigen::Triplet<double>>& triplets) {
                InertialEnergy::compute_hessian_triplets(InertialEnergy::Input{
                    .x = x,
                    .x_hat = x_hat,
                    .mass_diag = state.mass_diag,
                    .dt = dt,
                }, triplets);
                MaterialEnergy<FixedCorotatedMaterial>::compute_hessian_triplets(
                    MaterialEnergy<FixedCorotatedMaterial>::Input{
                        .body = body,
                        .x = x,
                        .material = material,
                    },
                    triplets
                );
            },
    };
}

double masked_inf_norm(const NewtonProblem& problem,
                       const Eigen::VectorXd& x,
                       const std::vector<bool>& free_dof_mask) {
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(x.size());
    problem.compute_gradient(x, gradient);
    double norm = 0.0;
    for (Eigen::Index i = 0; i < gradient.size(); ++i) {
        if (!free_dof_mask[static_cast<std::size_t>(i)]) {
            continue;
        }
        norm = std::max(norm, std::abs(gradient[i]));
    }
    return norm;
}

TEST(NewtonSolverTest, PureInertialEnergyConvergesInOneStep) {
    IPCState state{};
    state.resize(2);
    state.x << 1.0, -2.0, 3.0, -4.0, 5.0, -6.0;
    state.mass_diag << 2.0, 2.0, 2.0, 3.0, 3.0, 3.0;

    Eigen::VectorXd x_hat(6);
    x_hat << -1.0, 2.0, -3.0, 4.0, -5.0, 6.0;

    const NewtonProblem problem = make_inertial_problem(state, x_hat, 0.5);
    const std::vector<bool> free_dof_mask(state.dof_count(), true);

    const NewtonSolverResult result = solve(
        state,
        x_hat,
        free_dof_mask,
        problem,
        NewtonSolverParams{
            .max_iterations = 4,
            .gradient_tolerance = 1e-10,
            .dx_tolerance = 1e-12,
            .regularization = 0.0,
        }
    );

    EXPECT_TRUE(result.converged);
    EXPECT_EQ(result.iterations, 1);
    EXPECT_NEAR(result.final_energy, 0.0, 1e-12);
    EXPECT_LE(result.final_gradient_norm, 1e-10);
    EXPECT_TRUE(state.x.isApprox(x_hat, 1e-12));
}

TEST(NewtonSolverTest, FixedDofsRemainUnchanged) {
    IPCState state{};
    state.resize(2);
    state.x << 3.0, 4.0, 5.0, -1.0, -2.0, -3.0;
    state.mass_diag = Eigen::VectorXd::Ones(6);

    Eigen::VectorXd x_hat(6);
    x_hat << 10.0, 11.0, 12.0, 1.0, 2.0, 3.0;

    const NewtonProblem problem = make_inertial_problem(state, x_hat, 1.0);
    std::vector<bool> free_dof_mask(6, true);
    free_dof_mask[0] = false;
    free_dof_mask[1] = false;
    free_dof_mask[2] = false;

    const Eigen::Vector3d fixed_before = state.x.segment<3>(0);
    const NewtonSolverResult result = solve(
        state,
        x_hat,
        free_dof_mask,
        problem,
        NewtonSolverParams{
            .max_iterations = 4,
            .gradient_tolerance = 1e-10,
            .dx_tolerance = 1e-12,
            .regularization = 1e-10,
        }
    );

    EXPECT_TRUE(result.converged);
    EXPECT_TRUE(state.x.segment<3>(0).isApprox(fixed_before, 1e-12));
    EXPECT_TRUE(state.x.segment<3>(3).isApprox(x_hat.segment<3>(3), 1e-12));
}

TEST(NewtonSolverTest, TotalEnergyGradientNormDecreasesForTetProblem) {
    TetBody body = make_solver_test_body();
    IPCState state{};
    state.resize(body.vertex_count());

    for (std::size_t vertex = 0; vertex < body.vertex_count(); ++vertex) {
        state.position(vertex) = body.geometry.rest_positions[vertex];
        state.mass_diag.segment<3>(static_cast<Eigen::Index>(3 * vertex)).setConstant(body.vertex_masses[vertex]);
    }

    state.x[3] += 0.25;
    state.x[7] -= 0.18;
    state.x[11] += 0.12;

    Eigen::VectorXd x_hat(12);
    for (std::size_t vertex = 0; vertex < body.vertex_count(); ++vertex) {
        x_hat.segment<3>(static_cast<Eigen::Index>(3 * vertex)) = body.geometry.rest_positions[vertex];
    }

    const FixedCorotatedMaterial material{};
    const NewtonProblem problem = make_total_problem(state, x_hat, 0.1, body, material);
    const std::vector<bool> free_dof_mask(state.dof_count(), true);

    const double initial_gradient_norm = masked_inf_norm(problem, state.x, free_dof_mask);
    const double initial_energy = problem.compute_energy(state.x);

    const NewtonSolverResult result = solve(
        state,
        x_hat,
        free_dof_mask,
        problem,
        NewtonSolverParams{
            .max_iterations = 8,
            .gradient_tolerance = 1e-8,
            .dx_tolerance = 1e-10,
            .regularization = 1e-6,
        }
    );

    EXPECT_TRUE(state.x.allFinite());
    EXPECT_TRUE(std::isfinite(result.final_energy));
    EXPECT_TRUE(std::isfinite(result.final_gradient_norm));
    EXPECT_LT(result.final_energy, initial_energy);
    EXPECT_LT(result.final_gradient_norm, initial_gradient_norm);
}

}  // namespace
}  // namespace rtr::system::physics::ipc
