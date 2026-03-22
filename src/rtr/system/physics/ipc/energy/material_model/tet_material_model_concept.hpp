#pragma once

#include <concepts>

#include <Eigen/Core>

namespace rtr::system::physics::ipc {

template <typename M>
concept TetMaterialModel = requires(const M& material,
                                    const Eigen::Matrix3d& F,
                                    double rest_volume,
                                    double youngs_modulus,
                                    double poisson_ratio) {
    { material.compute_energy(F, rest_volume, youngs_modulus, poisson_ratio) } -> std::convertible_to<double>;
    { material.compute_pk1(F, rest_volume, youngs_modulus, poisson_ratio) } -> std::convertible_to<Eigen::Matrix3d>;
    { material.compute_hessian(F, rest_volume, youngs_modulus, poisson_ratio) }
        -> std::convertible_to<Eigen::Matrix<double, 9, 9>>;
};

}  // namespace rtr::system::physics::ipc
