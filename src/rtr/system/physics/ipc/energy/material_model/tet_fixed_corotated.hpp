#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

#include "rtr/system/physics/ipc/energy/material_model/tet_material_model_concept.hpp"

namespace rtr::system::physics::ipc {

namespace fixed_corotated_detail {

struct LameParameters {
    double mu{0.0};
    double lambda{0.0};
};

struct RotationData {
    Eigen::Matrix3d UD{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d V{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d signed_singular_values{Eigen::Vector3d::Ones()};
};

inline void validate_material_inputs(const Eigen::Matrix3d& F,
                                     double rest_volume,
                                     double youngs_modulus,
                                     double poisson_ratio) {
    if (!F.allFinite()) {
        throw std::invalid_argument("FixedCorotatedMaterial requires F to be finite.");
    }
    if (!std::isfinite(rest_volume) || rest_volume <= 0.0) {
        throw std::invalid_argument("FixedCorotatedMaterial requires rest_volume to be finite and positive.");
    }
    if (!std::isfinite(youngs_modulus) || youngs_modulus <= 0.0) {
        throw std::invalid_argument("FixedCorotatedMaterial requires youngs_modulus to be finite and positive.");
    }
    if (!std::isfinite(poisson_ratio) || poisson_ratio <= -1.0 || poisson_ratio >= 0.5) {
        throw std::invalid_argument("FixedCorotatedMaterial requires poisson_ratio in (-1, 0.5).");
    }
}

inline LameParameters compute_lame_parameters(double youngs_modulus, double poisson_ratio) {
    return LameParameters{
        .mu = youngs_modulus / (2.0 * (1.0 + poisson_ratio)),
        .lambda = (youngs_modulus * poisson_ratio) /
                  ((1.0 + poisson_ratio) * (1.0 - 2.0 * poisson_ratio)),
    };
}

inline RotationData compute_rotation_data(const Eigen::Matrix3d& F) {
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        F,
        Eigen::ComputeFullU | Eigen::ComputeFullV
    );

    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
    Eigen::Vector3d signed_singular_values = svd.singularValues();
    if ((U * V.transpose()).determinant() < 0.0) {
        D(2, 2) = -1.0;
        signed_singular_values[2] *= -1.0;
    }

    const Eigen::Matrix3d UD = U * D;
    return RotationData{
        .UD = UD,
        .V = V,
        .R = UD * V.transpose(),
        .signed_singular_values = signed_singular_values,
    };
}

inline Eigen::Matrix3d compute_cofactor_matrix(const Eigen::Matrix3d& F) {
    Eigen::Matrix3d cofactor;
    cofactor.col(0) = F.col(1).cross(F.col(2));
    cofactor.col(1) = F.col(2).cross(F.col(0));
    cofactor.col(2) = F.col(0).cross(F.col(1));
    return cofactor;
}

inline Eigen::Matrix<double, 9, 1> flatten_matrix(const Eigen::Matrix3d& matrix) {
    return Eigen::Map<const Eigen::Matrix<double, 9, 1>>(matrix.data());
}

inline Eigen::Matrix3d differential_rotation(const Eigen::Matrix3d& dF, const RotationData& rotation_data) {
    const Eigen::Matrix3d M = rotation_data.R.transpose() * dF - dF.transpose() * rotation_data.R;
    const Eigen::Matrix3d B = rotation_data.V.transpose() * M * rotation_data.V;

    Eigen::Matrix3d omega_hat = Eigen::Matrix3d::Zero();
    constexpr double kMinDenominator = 1e-12;
    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            const double denominator =
                rotation_data.signed_singular_values[i] + rotation_data.signed_singular_values[j];
            if (std::abs(denominator) <= kMinDenominator) {
                continue;
            }
            const double omega_ij = B(i, j) / denominator;
            omega_hat(i, j) = omega_ij;
            omega_hat(j, i) = -omega_ij;
        }
    }

    return rotation_data.UD * omega_hat * rotation_data.V.transpose();
}

inline Eigen::Matrix3d differential_cofactor_matrix(const Eigen::Matrix3d& F, const Eigen::Matrix3d& dF) {
    Eigen::Matrix3d differential = Eigen::Matrix3d::Zero();
    differential.col(0) = dF.col(1).cross(F.col(2)) + F.col(1).cross(dF.col(2));
    differential.col(1) = dF.col(2).cross(F.col(0)) + F.col(2).cross(dF.col(0));
    differential.col(2) = dF.col(0).cross(F.col(1)) + F.col(0).cross(dF.col(1));
    return differential;
}

}  // namespace fixed_corotated_detail

struct FixedCorotatedMaterial {
    double compute_energy(const Eigen::Matrix3d& F,
                          double rest_volume,
                          double youngs_modulus,
                          double poisson_ratio) const {
        fixed_corotated_detail::validate_material_inputs(F, rest_volume, youngs_modulus, poisson_ratio);

        const auto lame = fixed_corotated_detail::compute_lame_parameters(youngs_modulus, poisson_ratio);
        const fixed_corotated_detail::RotationData rotation_data =
            fixed_corotated_detail::compute_rotation_data(F);
        const double J = F.determinant();
        const double density =
            lame.mu * (F - rotation_data.R).squaredNorm() + 0.5 * lame.lambda * std::pow(J - 1.0, 2.0);
        return rest_volume * density;
    }

    Eigen::Matrix3d compute_pk1(const Eigen::Matrix3d& F,
                                double rest_volume,
                                double youngs_modulus,
                                double poisson_ratio) const {
        fixed_corotated_detail::validate_material_inputs(F, rest_volume, youngs_modulus, poisson_ratio);

        const auto lame = fixed_corotated_detail::compute_lame_parameters(youngs_modulus, poisson_ratio);
        const fixed_corotated_detail::RotationData rotation_data =
            fixed_corotated_detail::compute_rotation_data(F);
        const double J = F.determinant();
        const Eigen::Matrix3d cofactor = fixed_corotated_detail::compute_cofactor_matrix(F);
        return 2.0 * lame.mu * (F - rotation_data.R) + lame.lambda * (J - 1.0) * cofactor;
    }

    Eigen::Matrix<double, 9, 9> compute_hessian(const Eigen::Matrix3d& F,
                                                double rest_volume,
                                                double youngs_modulus,
                                                double poisson_ratio) const {
        fixed_corotated_detail::validate_material_inputs(F, rest_volume, youngs_modulus, poisson_ratio);

        const auto lame = fixed_corotated_detail::compute_lame_parameters(youngs_modulus, poisson_ratio);
        const fixed_corotated_detail::RotationData rotation_data =
            fixed_corotated_detail::compute_rotation_data(F);
        const double J = F.determinant();
        const Eigen::Matrix3d cofactor = fixed_corotated_detail::compute_cofactor_matrix(F);

        Eigen::Matrix<double, 9, 9> hessian = Eigen::Matrix<double, 9, 9>::Zero();

        for (int j = 0; j < 9; ++j) {
            Eigen::Matrix3d dF = Eigen::Matrix3d::Zero();
            Eigen::Map<Eigen::Matrix<double, 9, 1>>(dF.data())[j] = 1.0;

            const Eigen::Matrix3d dR = fixed_corotated_detail::differential_rotation(dF, rotation_data);
            const Eigen::Matrix3d dCof = fixed_corotated_detail::differential_cofactor_matrix(F, dF);
            const double dJ = cofactor.cwiseProduct(dF).sum();
            const Eigen::Matrix3d dP = 2.0 * lame.mu * (dF - dR) +
                                       lame.lambda * (dJ * cofactor + (J - 1.0) * dCof);

            hessian.col(j) = fixed_corotated_detail::flatten_matrix(dP);
        }

        return hessian;
    }
};

static_assert(TetMaterialModel<FixedCorotatedMaterial>);

}  // namespace rtr::system::physics::ipc
