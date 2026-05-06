#include <gtest/gtest.h>

#include <Eigen/Core>

#include "rtr/system/physics/ipc/energy/gravity_energy.hpp"

namespace rtr::system::physics::ipc {
namespace {

TEST(GravityEnergyTest, ReturnsZeroForZeroGravity) {
    Eigen::VectorXd x(6);
    x << 1.0, 2.0, 3.0, -4.0, 5.0, -6.0;

    Eigen::VectorXd mass_diag(6);
    mass_diag << 2.0, 2.0, 2.0, 3.0, 3.0, 3.0;

    const Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
    const GravityEnergy::Input input{
        .x = x,
        .mass_diag = mass_diag,
        .gravity = gravity,
    };
    EXPECT_DOUBLE_EQ(GravityEnergy::compute_energy(input), 0.0);

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(6);
    GravityEnergy::compute_gradient(input, gradient);
    EXPECT_TRUE(gradient.isZero(0.0));
}

TEST(GravityEnergyTest, ComputesLinearPotentialAndConstantGradient) {
    Eigen::VectorXd x(6);
    x << 1.0, 2.0, 3.0, -4.0, 5.0, -6.0;

    Eigen::VectorXd mass_diag(6);
    mass_diag << 2.0, 2.0, 2.0, 3.0, 3.0, 3.0;

    const Eigen::Vector3d gravity(0.0, -9.81, 0.0);
    const GravityEnergy::Input input{
        .x = x,
        .mass_diag = mass_diag,
        .gravity = gravity,
    };
    const double expected_energy = -(2.0 * gravity.y() * 2.0 + 3.0 * gravity.y() * 5.0);
    EXPECT_DOUBLE_EQ(GravityEnergy::compute_energy(input), expected_energy);

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(6);
    GravityEnergy::compute_gradient(input, gradient);

    Eigen::VectorXd expected_gradient(6);
    expected_gradient << 0.0, 19.62, 0.0, 0.0, 29.43, 0.0;
    EXPECT_TRUE(gradient.isApprox(expected_gradient, 1e-12));
}

TEST(GravityEnergyTest, RejectsInvalidSizes) {
    const Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
    const Eigen::VectorXd mass_diag = Eigen::VectorXd::Ones(3);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(6);
    const GravityEnergy::Input bad_input{
        .x = x,
        .mass_diag = mass_diag,
        .gravity = Eigen::Vector3d::Zero(),
    };

    EXPECT_THROW(GravityEnergy::compute_energy(bad_input), std::invalid_argument);
    EXPECT_THROW(GravityEnergy::compute_gradient(bad_input, gradient), std::invalid_argument);
}

}  // namespace
}  // namespace rtr::system::physics::ipc
