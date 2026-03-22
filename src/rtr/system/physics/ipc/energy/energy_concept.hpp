#pragma once

#include <concepts>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

namespace rtr::system::physics::ipc {

template <typename T>
concept Energy = requires(const typename T::Input& input,
                          Eigen::VectorXd& gradient,
                          std::vector<Eigen::Triplet<double>>& triplets) {
    typename T::Input;
    { T::compute_energy(input) } -> std::convertible_to<double>;
    { T::compute_gradient(input, gradient) } -> std::same_as<void>;
    { T::compute_hessian_triplets(input, triplets) } -> std::same_as<void>;
};

}  // namespace rtr::system::physics::ipc
