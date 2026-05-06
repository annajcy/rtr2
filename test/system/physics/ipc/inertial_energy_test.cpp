#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "rtr/system/physics/ipc/energy/inertial_energy.hpp"

namespace rtr::system::physics::ipc {
namespace {

TEST(InertialEnergyTest, ReturnsZeroAtInertialPrediction) {
    Eigen::VectorXd x(6);
    x << 1.0, -2.0, 3.0, 4.0, -5.0, 6.0;
    const Eigen::VectorXd x_hat = x;

    Eigen::VectorXd mass_diag(6);
    mass_diag << 2.0, 2.0, 2.0, 3.0, 3.0, 3.0;
    const InertialEnergy::Input input{
        .x = x,
        .x_hat = x_hat,
        .mass_diag = mass_diag,
        .dt = 0.25,
    };

    EXPECT_DOUBLE_EQ(InertialEnergy::compute_energy(input), 0.0);

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(6);
    InertialEnergy::compute_gradient(input, gradient);
    EXPECT_TRUE(gradient.isZero(0.0));
}

TEST(InertialEnergyTest, ComputesEnergyGradientAndDiagonalHessian) {
    Eigen::VectorXd x(6);
    x << 1.0, -2.0, 3.0, 0.5, 0.0, -1.0;
    Eigen::VectorXd x_hat = Eigen::VectorXd::Zero(6);

    Eigen::VectorXd mass_diag(6);
    mass_diag << 2.0, 2.0, 2.0, 3.0, 3.0, 3.0;

    const double dt = 0.5;
    const double inv_dt_sq = 4.0;
    const InertialEnergy::Input input{
        .x = x,
        .x_hat = x_hat,
        .mass_diag = mass_diag,
        .dt = dt,
    };

    const double expected_energy = 0.5 * inv_dt_sq *
                                   (2.0 * (1.0 * 1.0 + 4.0 + 9.0) + 3.0 * (0.25 + 0.0 + 1.0));
    EXPECT_DOUBLE_EQ(InertialEnergy::compute_energy(input), expected_energy);

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(6);
    InertialEnergy::compute_gradient(input, gradient);

    Eigen::VectorXd expected_gradient(6);
    expected_gradient << 8.0, -16.0, 24.0, 6.0, 0.0, -12.0;
    EXPECT_TRUE(gradient.isApprox(expected_gradient));

    std::vector<Eigen::Triplet<double>> triplets{};
    InertialEnergy::compute_hessian_triplets(input, triplets);

    ASSERT_EQ(triplets.size(), 6u);
    for (Eigen::Index i = 0; i < 6; ++i) {
        EXPECT_EQ(triplets[static_cast<std::size_t>(i)].row(), i);
        EXPECT_EQ(triplets[static_cast<std::size_t>(i)].col(), i);
        EXPECT_DOUBLE_EQ(triplets[static_cast<std::size_t>(i)].value(), inv_dt_sq * mass_diag[i]);
    }
}

TEST(InertialEnergyTest, RejectsInvalidSizes) {
    const Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
    const Eigen::VectorXd x_hat = Eigen::VectorXd::Zero(3);
    const Eigen::VectorXd mass_diag = Eigen::VectorXd::Ones(6);
    Eigen::VectorXd small_gradient = Eigen::VectorXd::Zero(3);
    std::vector<Eigen::Triplet<double>> triplets{};
    const InertialEnergy::Input bad_energy_input{
        .x = x,
        .x_hat = x_hat,
        .mass_diag = mass_diag,
        .dt = 0.1,
    };
    const InertialEnergy::Input bad_gradient_input{
        .x = x,
        .x_hat = x,
        .mass_diag = mass_diag,
        .dt = 0.1,
    };
    const Eigen::VectorXd bad_mass_diag = Eigen::VectorXd::Ones(5);
    const InertialEnergy::Input bad_hessian_input{
        .x = x,
        .x_hat = x,
        .mass_diag = bad_mass_diag,
        .dt = 0.1,
    };

    EXPECT_THROW(InertialEnergy::compute_energy(bad_energy_input), std::invalid_argument);
    EXPECT_THROW(InertialEnergy::compute_gradient(bad_gradient_input, small_gradient), std::invalid_argument);
    EXPECT_THROW(InertialEnergy::compute_hessian_triplets(bad_hessian_input, triplets), std::invalid_argument);
}

}  // namespace
}  // namespace rtr::system::physics::ipc
