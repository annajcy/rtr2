#include <limits>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "distance_test_utils.hpp"

namespace rtr::system::physics::ipc {
namespace {

TEST(PointPointDistanceTest, ComputesExpectedClosedFormResult) {
    const auto result = PointPointDistance::compute({
        .p0 = Eigen::Vector3d(1.0, 2.0, 3.0),
        .p1 = Eigen::Vector3d(-1.0, 0.0, 4.0),
    });

    EXPECT_DOUBLE_EQ(result.distance_squared, 9.0);

    Eigen::Matrix<double, 6, 1> expected_gradient;
    expected_gradient << 4.0, 4.0, -2.0, -4.0, -4.0, 2.0;
    EXPECT_TRUE(result.gradient.isApprox(expected_gradient, 1e-12));

    Eigen::Matrix<double, 6, 6> expected_hessian = Eigen::Matrix<double, 6, 6>::Zero();
    expected_hessian.topLeftCorner<3, 3>().diagonal().setConstant(2.0);
    expected_hessian.bottomRightCorner<3, 3>().diagonal().setConstant(2.0);
    expected_hessian.topRightCorner<3, 3>().diagonal().setConstant(-2.0);
    expected_hessian.bottomLeftCorner<3, 3>().diagonal().setConstant(-2.0);
    EXPECT_TRUE(result.hessian.isApprox(expected_hessian, 1e-12));
}

TEST(PointPointDistanceTest, MatchesFiniteDifference) {
    test::expect_matches_finite_difference<PointPointDistance>({
        .p0 = Eigen::Vector3d(0.2, -0.4, 1.1),
        .p1 = Eigen::Vector3d(-0.5, 0.8, 0.3),
    });
}

TEST(PointPointDistanceTest, RejectsNonFiniteInput) {
    EXPECT_THROW(
        PointPointDistance::compute({
            .p0 = Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
            .p1 = Eigen::Vector3d::Zero(),
        }),
        std::invalid_argument);
}

}  // namespace
}  // namespace rtr::system::physics::ipc
