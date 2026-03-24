#pragma once

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "rtr/system/physics/ipc/geometry/edge_edge_distance.hpp"
#include "rtr/system/physics/ipc/geometry/point_edge_distance.hpp"
#include "rtr/system/physics/ipc/geometry/point_point_distance.hpp"
#include "rtr/system/physics/ipc/geometry/point_triangle_distance.hpp"

namespace rtr::system::physics::ipc::test {

template <typename DistanceKernel>
struct DistanceInputAdapter;

template <>
struct DistanceInputAdapter<PointPointDistance> {
    static constexpr int kDofs = 6;
    using Vector = Eigen::Matrix<double, kDofs, 1>;
    using Input = PointPointDistance::Input;

    static Vector pack(const Input& input) {
        Vector values;
        values << input.p0, input.p1;
        return values;
    }

    static Input unpack(const Vector& values) {
        return {
            .p0 = values.template segment<3>(0),
            .p1 = values.template segment<3>(3),
        };
    }
};

template <>
struct DistanceInputAdapter<PointEdgeDistance> {
    static constexpr int kDofs = 9;
    using Vector = Eigen::Matrix<double, kDofs, 1>;
    using Input = PointEdgeDistance::Input;

    static Vector pack(const Input& input) {
        Vector values;
        values << input.p, input.e0, input.e1;
        return values;
    }

    static Input unpack(const Vector& values) {
        return {
            .p = values.template segment<3>(0),
            .e0 = values.template segment<3>(3),
            .e1 = values.template segment<3>(6),
        };
    }
};

template <>
struct DistanceInputAdapter<PointTriangleDistance> {
    static constexpr int kDofs = 12;
    using Vector = Eigen::Matrix<double, kDofs, 1>;
    using Input = PointTriangleDistance::Input;

    static Vector pack(const Input& input) {
        Vector values;
        values << input.p, input.t0, input.t1, input.t2;
        return values;
    }

    static Input unpack(const Vector& values) {
        return {
            .p = values.template segment<3>(0),
            .t0 = values.template segment<3>(3),
            .t1 = values.template segment<3>(6),
            .t2 = values.template segment<3>(9),
        };
    }
};

template <>
struct DistanceInputAdapter<EdgeEdgeDistance> {
    static constexpr int kDofs = 12;
    using Vector = Eigen::Matrix<double, kDofs, 1>;
    using Input = EdgeEdgeDistance::Input;

    static Vector pack(const Input& input) {
        Vector values;
        values << input.ea0, input.ea1, input.eb0, input.eb1;
        return values;
    }

    static Input unpack(const Vector& values) {
        return {
            .ea0 = values.template segment<3>(0),
            .ea1 = values.template segment<3>(3),
            .eb0 = values.template segment<3>(6),
            .eb1 = values.template segment<3>(9),
        };
    }
};

template <typename DistanceKernel>
void expect_matches_finite_difference(const typename DistanceKernel::Input& input,
                                      const double epsilon = 1e-7,
                                      const double relative_tolerance = 1e-5) {
    using Adapter = DistanceInputAdapter<DistanceKernel>;
    using Vector = typename Adapter::Vector;

    const auto analytic = DistanceKernel::compute(input);
    ASSERT_TRUE(std::isfinite(analytic.distance_squared));
    ASSERT_TRUE(analytic.gradient.allFinite());
    ASSERT_TRUE(analytic.hessian.allFinite());
    EXPECT_TRUE(analytic.hessian.isApprox(analytic.hessian.transpose(), 1e-9));

    const Vector dofs = Adapter::pack(input);

    Vector gradient_fd;
    for (int index = 0; index < Adapter::kDofs; ++index) {
        Vector plus = dofs;
        Vector minus = dofs;
        plus[index] += epsilon;
        minus[index] -= epsilon;

        const double energy_plus = DistanceKernel::compute(Adapter::unpack(plus)).distance_squared;
        const double energy_minus = DistanceKernel::compute(Adapter::unpack(minus)).distance_squared;
        gradient_fd[index] = (energy_plus - energy_minus) / (2.0 * epsilon);
    }

    for (int index = 0; index < Adapter::kDofs; ++index) {
        const double tolerance = relative_tolerance * std::max(1.0, std::abs(gradient_fd[index]));
        EXPECT_NEAR(analytic.gradient[index], gradient_fd[index], tolerance);
    }

    Eigen::Matrix<double, Adapter::kDofs, Adapter::kDofs> hessian_fd;
    for (int column = 0; column < Adapter::kDofs; ++column) {
        Vector plus = dofs;
        Vector minus = dofs;
        plus[column] += epsilon;
        minus[column] -= epsilon;

        const auto gradient_plus = DistanceKernel::compute(Adapter::unpack(plus)).gradient;
        const auto gradient_minus = DistanceKernel::compute(Adapter::unpack(minus)).gradient;
        hessian_fd.col(column) = (gradient_plus - gradient_minus) / (2.0 * epsilon);
    }

    for (int row = 0; row < Adapter::kDofs; ++row) {
        for (int column = 0; column < Adapter::kDofs; ++column) {
            const double tolerance = relative_tolerance * std::max(1.0, std::abs(hessian_fd(row, column)));
            EXPECT_NEAR(analytic.hessian(row, column), hessian_fd(row, column), tolerance);
        }
    }
}

}  // namespace rtr::system::physics::ipc::test
