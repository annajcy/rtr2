#pragma once

#include <concepts>

#include <Eigen/Core>

namespace rtr::system::physics::ipc {

template <typename M>
concept TetMaterialModel = requires(const M& material,
                                    const Eigen::Matrix3d& F,
                                    double rest_volume) {
    { material.density() } -> std::convertible_to<double>;
    { material.compute_energy(F, rest_volume) } -> std::convertible_to<double>;
    { material.compute_pk1(F, rest_volume) } -> std::convertible_to<Eigen::Matrix3d>;
    { material.compute_hessian(F, rest_volume) } -> std::convertible_to<Eigen::Matrix<double, 9, 9>>;
};

}  // namespace rtr::system::physics::ipc
