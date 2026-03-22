#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "rtr/system/physics/ipc/energy/material_model/tet_fixed_corotated.hpp"
#include "rtr/system/physics/ipc/energy/material_model/tet_material_model_concept.hpp"

namespace rtr::system::physics::ipc {
namespace {

static_assert(TetMaterialModel<FixedCorotatedMaterial>);

Eigen::Matrix<double, 9, 1> flatten(const Eigen::Matrix3d& matrix) {
    return Eigen::Map<const Eigen::Matrix<double, 9, 1>>(matrix.data());
}

TEST(FixedCorotatedMaterialTest, IdentityHasZeroEnergyAndStress) {
    const FixedCorotatedMaterial material{};
    const Eigen::Matrix3d F = Eigen::Matrix3d::Identity();

    EXPECT_DOUBLE_EQ(material.compute_energy(F, 1.0, 1e5, 0.3), 0.0);
    EXPECT_TRUE(material.compute_pk1(F, 1.0, 1e5, 0.3).isZero(1e-12));

    const auto hessian = material.compute_hessian(F, 1.0, 1e5, 0.3);
    EXPECT_TRUE(hessian.allFinite());
}

TEST(FixedCorotatedMaterialTest, Pk1MatchesFiniteDifferenceEnergyGradient) {
    const FixedCorotatedMaterial material{};
    Eigen::Matrix3d F;
    F << 1.05, 0.02, 0.00,
         0.01, 0.97, 0.03,
         0.00, -0.02, 1.01;

    constexpr double rest_volume = 0.5;
    constexpr double youngs_modulus = 100.0;
    constexpr double poisson_ratio = 0.3;
    constexpr double epsilon = 1e-6;

    const Eigen::Matrix<double, 9, 1> expected =
        rest_volume * flatten(material.compute_pk1(F, rest_volume, youngs_modulus, poisson_ratio));

    for (int j = 0; j < 9; ++j) {
        Eigen::Matrix3d F_plus = F;
        Eigen::Matrix3d F_minus = F;
        Eigen::Map<Eigen::Matrix<double, 9, 1>>(F_plus.data())[j] += epsilon;
        Eigen::Map<Eigen::Matrix<double, 9, 1>>(F_minus.data())[j] -= epsilon;

        const double energy_plus = material.compute_energy(F_plus, rest_volume, youngs_modulus, poisson_ratio);
        const double energy_minus = material.compute_energy(F_minus, rest_volume, youngs_modulus, poisson_ratio);
        const double finite_difference = (energy_plus - energy_minus) / (2.0 * epsilon);

        const double tolerance = 1e-5 * std::max(1.0, std::abs(expected[j]));
        EXPECT_NEAR(finite_difference, expected[j], tolerance);
    }
}

TEST(FixedCorotatedMaterialTest, StretchAndCompressionIncreaseEnergy) {
    const FixedCorotatedMaterial material{};
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d stretch = Eigen::Matrix3d::Identity();
    stretch(0, 0) = 1.1;
    Eigen::Matrix3d compression = Eigen::Matrix3d::Identity();
    compression(0, 0) = 0.9;

    const double identity_energy = material.compute_energy(identity, 1.0, 1e5, 0.3);
    const double stretch_energy = material.compute_energy(stretch, 1.0, 1e5, 0.3);
    const double compression_energy = material.compute_energy(compression, 1.0, 1e5, 0.3);

    EXPECT_GT(stretch_energy, identity_energy);
    EXPECT_GT(compression_energy, identity_energy);
}

TEST(FixedCorotatedMaterialTest, HessianMatchesFiniteDifferenceOfPk1) {
    const FixedCorotatedMaterial material{};
    Eigen::Matrix3d F;
    F << 1.03, 0.02, -0.01,
         0.01, 0.98, 0.04,
         -0.02, 0.01, 1.05;

    constexpr double rest_volume = 0.5;
    constexpr double youngs_modulus = 150.0;
    constexpr double poisson_ratio = 0.25;
    constexpr double epsilon = 1e-6;

    const Eigen::Matrix<double, 9, 9> analytic_hessian =
        material.compute_hessian(F, rest_volume, youngs_modulus, poisson_ratio);

    for (int j = 0; j < 9; ++j) {
        Eigen::Matrix3d F_plus = F;
        Eigen::Matrix3d F_minus = F;
        Eigen::Map<Eigen::Matrix<double, 9, 1>>(F_plus.data())[j] += epsilon;
        Eigen::Map<Eigen::Matrix<double, 9, 1>>(F_minus.data())[j] -= epsilon;

        const Eigen::Matrix3d P_plus = material.compute_pk1(F_plus, rest_volume, youngs_modulus, poisson_ratio);
        const Eigen::Matrix3d P_minus = material.compute_pk1(F_minus, rest_volume, youngs_modulus, poisson_ratio);
        const Eigen::Matrix<double, 9, 1> finite_difference =
            (flatten(P_plus) - flatten(P_minus)) / (2.0 * epsilon);

        for (int i = 0; i < 9; ++i) {
            const double tolerance = 2e-4 * std::max(1.0, std::abs(finite_difference[i]));
            EXPECT_NEAR(analytic_hessian(i, j), finite_difference[i], tolerance);
        }
    }
}

}  // namespace
}  // namespace rtr::system::physics::ipc
