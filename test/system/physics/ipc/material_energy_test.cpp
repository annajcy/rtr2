#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "rtr/system/physics/ipc/energy/material_energy.hpp"
#include "rtr/system/physics/ipc/energy/material_model/tet_fixed_corotated.hpp"

namespace rtr::system::physics::ipc {
namespace {

static_assert(Energy<MaterialEnergy<FixedCorotatedMaterial>>);

TetBody make_single_tet_body() {
    TetBody body{};
    body.geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
    };
    body.geometry.tets = {{{0, 1, 2, 3}}};
    body.youngs_modulus = 100.0;
    body.poisson_ratio = 0.3;
    body.precompute();
    return body;
}

Eigen::Matrix3d build_F(const TetBody& body, const Eigen::VectorXd& x) {
    Eigen::Matrix3d Ds;
    const Eigen::Vector3d x0 = x.segment<3>(0);
    Ds.col(0) = x.segment<3>(3) - x0;
    Ds.col(1) = x.segment<3>(6) - x0;
    Ds.col(2) = x.segment<3>(9) - x0;
    return Ds * body.geometry.Dm_inv[0];
}

TEST(MaterialEnergyTest, SingleTetEnergyMatchesDirectMaterialCall) {
    TetBody body = make_single_tet_body();
    FixedCorotatedMaterial material{};

    Eigen::VectorXd x(12);
    x << 0.0, 0.0, 0.0,
         1.1, 0.0, 0.0,
         0.0, 0.95, 0.0,
         0.0, 0.0, 1.02;

    const MaterialEnergy<FixedCorotatedMaterial>::Input input{
        .body = body,
        .x = x,
        .material = material,
    };

    const Eigen::Matrix3d F = build_F(body, x);
    const double expected_energy = material.compute_energy(
        F,
        body.geometry.rest_volumes[0],
        body.youngs_modulus,
        body.poisson_ratio
    );

    EXPECT_NEAR(MaterialEnergy<FixedCorotatedMaterial>::compute_energy(input), expected_energy, 1e-12);
}

TEST(MaterialEnergyTest, GradientMatchesFiniteDifferenceOfTotalEnergy) {
    TetBody body = make_single_tet_body();
    FixedCorotatedMaterial material{};

    Eigen::VectorXd x(12);
    x << 0.02, -0.01, 0.00,
         1.05, 0.03, 0.00,
         0.01, 0.98, 0.04,
         -0.02, 0.01, 1.02;

    const MaterialEnergy<FixedCorotatedMaterial>::Input input{
        .body = body,
        .x = x,
        .material = material,
    };

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(12);
    MaterialEnergy<FixedCorotatedMaterial>::compute_gradient(input, gradient);
    ASSERT_EQ(gradient.size(), 12);
    EXPECT_TRUE(gradient.allFinite());

    constexpr double epsilon = 1e-6;
    for (Eigen::Index i = 0; i < x.size(); ++i) {
        Eigen::VectorXd x_plus = x;
        Eigen::VectorXd x_minus = x;
        x_plus[i] += epsilon;
        x_minus[i] -= epsilon;

        const MaterialEnergy<FixedCorotatedMaterial>::Input plus_input{
            .body = body,
            .x = x_plus,
            .material = material,
        };
        const MaterialEnergy<FixedCorotatedMaterial>::Input minus_input{
            .body = body,
            .x = x_minus,
            .material = material,
        };

        const double energy_plus = MaterialEnergy<FixedCorotatedMaterial>::compute_energy(plus_input);
        const double energy_minus = MaterialEnergy<FixedCorotatedMaterial>::compute_energy(minus_input);
        const double finite_difference = (energy_plus - energy_minus) / (2.0 * epsilon);

        const double tolerance = 1e-4 * std::max(1.0, std::abs(finite_difference));
        EXPECT_NEAR(gradient[i], finite_difference, tolerance);
    }
}

TEST(MaterialEnergyTest, HessianTripletsAreFiniteAndWithinGlobalSize) {
    TetBody body = make_single_tet_body();
    FixedCorotatedMaterial material{};

    Eigen::VectorXd x(12);
    x << 0.01, 0.00, -0.01,
         1.02, 0.01, 0.00,
         0.00, 1.01, 0.02,
         -0.01, 0.03, 0.97;

    const MaterialEnergy<FixedCorotatedMaterial>::Input input{
        .body = body,
        .x = x,
        .material = material,
    };

    std::vector<Eigen::Triplet<double>> triplets{};
    MaterialEnergy<FixedCorotatedMaterial>::compute_hessian_triplets(input, triplets);

    ASSERT_FALSE(triplets.empty());
    for (const auto& triplet : triplets) {
        EXPECT_GE(triplet.row(), 0);
        EXPECT_GE(triplet.col(), 0);
        EXPECT_LT(triplet.row(), x.size());
        EXPECT_LT(triplet.col(), x.size());
        EXPECT_TRUE(std::isfinite(triplet.value()));
    }
}

}  // namespace
}  // namespace rtr::system::physics::ipc
