#include <limits>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "distance_test_utils.hpp"

namespace rtr::system::physics::ipc {
namespace {

TEST(PointEdgeDistanceTest, CoversInteriorAndEndpointRegions) {
    const auto interior = PointEdgeDistance::compute({
        .p = Eigen::Vector3d(0.5, 1.0, 0.2),
        .e0 = Eigen::Vector3d(0.0, 0.0, 0.0),
        .e1 = Eigen::Vector3d(2.0, 0.0, 0.0),
    });
    EXPECT_EQ(interior.region, PointEdgeRegion::EdgeInterior);
    EXPECT_NEAR(interior.distance_squared, 1.04, 1e-12);

    const auto endpoint0 = PointEdgeDistance::compute({
        .p = Eigen::Vector3d(-1.0, 0.25, -0.5),
        .e0 = Eigen::Vector3d(0.0, 0.0, 0.0),
        .e1 = Eigen::Vector3d(2.0, 0.0, 0.0),
    });
    EXPECT_EQ(endpoint0.region, PointEdgeRegion::Endpoint0);
    EXPECT_NEAR(endpoint0.distance_squared, 1.3125, 1e-12);

    const auto endpoint1 = PointEdgeDistance::compute({
        .p = Eigen::Vector3d(3.0, -0.5, 0.25),
        .e0 = Eigen::Vector3d(0.0, 0.0, 0.0),
        .e1 = Eigen::Vector3d(2.0, 0.0, 0.0),
    });
    EXPECT_EQ(endpoint1.region, PointEdgeRegion::Endpoint1);
    EXPECT_NEAR(endpoint1.distance_squared, 1.3125, 1e-12);
}

TEST(PointEdgeDistanceTest, MatchesFiniteDifferenceAcrossRegions) {
    test::expect_matches_finite_difference<PointEdgeDistance>({
        .p = Eigen::Vector3d(0.5, 1.0, 0.2),
        .e0 = Eigen::Vector3d(0.0, 0.0, 0.0),
        .e1 = Eigen::Vector3d(2.0, 0.0, 0.0),
    });
    test::expect_matches_finite_difference<PointEdgeDistance>({
        .p = Eigen::Vector3d(-1.0, 0.25, -0.5),
        .e0 = Eigen::Vector3d(0.0, 0.0, 0.0),
        .e1 = Eigen::Vector3d(2.0, 0.0, 0.0),
    });
    test::expect_matches_finite_difference<PointEdgeDistance>({
        .p = Eigen::Vector3d(3.0, -0.5, 0.25),
        .e0 = Eigen::Vector3d(0.0, 0.0, 0.0),
        .e1 = Eigen::Vector3d(2.0, 0.0, 0.0),
    });
}

TEST(PointEdgeDistanceTest, RejectsDegenerateEdgesAndNonFiniteInput) {
    EXPECT_THROW(
        PointEdgeDistance::compute({
            .p = Eigen::Vector3d::Zero(),
            .e0 = Eigen::Vector3d::Zero(),
            .e1 = Eigen::Vector3d::Zero(),
        }),
        std::invalid_argument);

    EXPECT_THROW(
        PointEdgeDistance::compute({
            .p = Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.0, 0.0),
            .e0 = Eigen::Vector3d::Zero(),
            .e1 = Eigen::Vector3d::UnitX(),
        }),
        std::invalid_argument);
}

}  // namespace
}  // namespace rtr::system::physics::ipc
