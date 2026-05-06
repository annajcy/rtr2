#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>

namespace rtr::system::physics::ipc {

template <int Dofs>
struct DistanceResult {
    static constexpr Eigen::Index kDofs = Dofs;
    using Gradient = Eigen::Matrix<double, Dofs, 1>;
    using Hessian = Eigen::Matrix<double, Dofs, Dofs>;

    double distance_squared{0.0};
    Gradient gradient{Gradient::Zero()};
    Hessian hessian{Hessian::Zero()};
};

namespace detail {

inline constexpr double kMinEdgeLengthSquared = 1e-24;
inline constexpr double kMinTriangleAreaSquared = 1e-24;
inline constexpr double kParallelThreshold = 1e-18;

template <typename Derived>
inline void require_finite_point(const Eigen::MatrixBase<Derived>& point, const char* label) {
    if (!point.allFinite()) {
        throw std::invalid_argument(std::string(label) + " must be finite.");
    }
}

template <typename Result>
inline void require_finite_result(const Result& result, const char* label) {
    if (!std::isfinite(result.distance_squared) || !result.gradient.allFinite() || !result.hessian.allFinite()) {
        throw std::runtime_error(std::string(label) + " produced a non-finite result.");
    }
}

template <typename Result>
inline void assign_distance_result(const DistanceResult<Result::kDofs>& source, Result& target) {
    target.distance_squared = source.distance_squared;
    target.gradient = source.gradient;
    target.hessian = source.hessian;
}

template <typename Derived>
inline auto squared_norm(const Eigen::MatrixBase<Derived>& vector) {
    return vector.dot(vector);
}

template <typename DerivedA, typename DerivedB>
inline auto cross(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedB>& b) {
    using Scalar = typename DerivedA::Scalar;
    Eigen::Matrix<Scalar, 3, 1> result;
    result << a.y() * b.z() - a.z() * b.y(),
        a.z() * b.x() - a.x() * b.z(),
        a.x() * b.y() - a.y() * b.x();
    return result;
}

template <int Dofs>
struct SecondOrderAutoDiff {
    using FirstDerivative = Eigen::Matrix<double, Dofs, 1>;
    using InnerScalar = Eigen::AutoDiffScalar<FirstDerivative>;
    using SecondDerivative = Eigen::Matrix<InnerScalar, Dofs, 1>;
    using Scalar = Eigen::AutoDiffScalar<SecondDerivative>;

    static Scalar make_variable(const double value, const Eigen::Index index) {
        InnerScalar inner_value(value);
        inner_value.derivatives() = FirstDerivative::Unit(index);

        SecondDerivative outer_derivatives;
        for (Eigen::Index derivative_index = 0; derivative_index < Dofs; ++derivative_index) {
            outer_derivatives[derivative_index].value() = (derivative_index == index) ? 1.0 : 0.0;
            outer_derivatives[derivative_index].derivatives().setZero();
        }

        return Scalar(inner_value, outer_derivatives);
    }
};

template <int Dofs, typename Fn>
inline DistanceResult<Dofs> evaluate_distance_expression(const Eigen::Matrix<double, Dofs, 1>& dofs,
                                                         Fn&& expression) {
    using AutoDiff = SecondOrderAutoDiff<Dofs>;

    Eigen::Matrix<typename AutoDiff::Scalar, Dofs, 1> variables;
    for (Eigen::Index index = 0; index < Dofs; ++index) {
        variables[index] = AutoDiff::make_variable(dofs[index], index);
    }

    const auto value = std::forward<Fn>(expression)(variables);

    DistanceResult<Dofs> result;
    result.distance_squared = value.value().value();
    result.gradient = value.value().derivatives();

    typename DistanceResult<Dofs>::Hessian hessian;
    for (Eigen::Index row = 0; row < Dofs; ++row) {
        hessian.row(row) = value.derivatives()[row].derivatives().transpose();
    }
    result.hessian = 0.5 * (hessian + hessian.transpose());
    require_finite_result(result, "Distance expression");
    return result;
}

template <int FromPoints, int ToPoints, typename ToResult>
inline void embed_distance_result(const DistanceResult<3 * FromPoints>& source,
                                  const std::array<int, FromPoints>& point_map,
                                  ToResult& target) {
    target.distance_squared = source.distance_squared;
    target.gradient.setZero();
    target.hessian.setZero();

    for (int source_point = 0; source_point < FromPoints; ++source_point) {
        const int target_point = point_map[static_cast<std::size_t>(source_point)];
        target.gradient.template segment<3>(3 * target_point) =
            source.gradient.template segment<3>(3 * source_point);

        for (int source_point_col = 0; source_point_col < FromPoints; ++source_point_col) {
            const int target_point_col = point_map[static_cast<std::size_t>(source_point_col)];
            target.hessian.template block<3, 3>(3 * target_point, 3 * target_point_col) =
                source.hessian.template block<3, 3>(3 * source_point, 3 * source_point_col);
        }
    }
}

}  // namespace detail
}  // namespace rtr::system::physics::ipc
