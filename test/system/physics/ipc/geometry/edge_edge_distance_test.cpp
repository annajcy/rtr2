#include <array>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "distance_test_utils.hpp"

namespace rtr::system::physics::ipc {
namespace {

struct EdgeEdgeCase {
    EdgeEdgeDistance::Input input;
    EdgeEdgeRegion region;
    double distance_squared;
};

TEST(EdgeEdgeDistanceTest, CoversInteriorFallbackAndParallelPaths) {
    const std::array<EdgeEdgeCase, 6> cases{{
        {{
             .ea0 = Eigen::Vector3d(0.0, 0.0, 0.0),
             .ea1 = Eigen::Vector3d(1.0, 0.0, 0.0),
             .eb0 = Eigen::Vector3d(0.5, -1.0, 1.0),
             .eb1 = Eigen::Vector3d(0.5, 1.0, 1.0),
         },
         EdgeEdgeRegion::InteriorInterior,
         1.0},
        {{
             .ea0 = Eigen::Vector3d(0.0, 0.0, 0.0),
             .ea1 = Eigen::Vector3d(1.0, 0.0, 0.0),
             .eb0 = Eigen::Vector3d(-0.4, -1.0, 0.2),
             .eb1 = Eigen::Vector3d(-0.4, 1.0, 0.2),
         },
         EdgeEdgeRegion::Ea0OnEdgeB,
         0.2},
        {{
             .ea0 = Eigen::Vector3d(0.0, 0.0, 0.0),
             .ea1 = Eigen::Vector3d(1.0, 0.0, 0.0),
             .eb0 = Eigen::Vector3d(-1.0, 1.0, 0.5),
             .eb1 = Eigen::Vector3d(-1.0, 2.0, 0.5),
         },
         EdgeEdgeRegion::Ea0OnEdgeB,
         2.25},
        {{
             .ea0 = Eigen::Vector3d(0.0, 0.0, 0.0),
             .ea1 = Eigen::Vector3d(2.0, 0.0, 0.0),
             .eb0 = Eigen::Vector3d(0.0, 1.0, 0.25),
             .eb1 = Eigen::Vector3d(2.0, 1.0, 0.25),
         },
         EdgeEdgeRegion::ParallelDegenerate,
         1.0625},
        {{
             .ea0 = Eigen::Vector3d(0.0, 0.0, 0.0),
             .ea1 = Eigen::Vector3d(2.0, 0.0, 0.0),
             .eb0 = Eigen::Vector3d(0.0, 1.0, 0.0),
             .eb1 = Eigen::Vector3d(2.0, 1.0, 2e-10),
         },
         EdgeEdgeRegion::ParallelDegenerate,
         1.0},
        {{
             .ea0 = Eigen::Vector3d(0.0, 0.0, 0.0),
             .ea1 = Eigen::Vector3d(1e-3, 0.0, 0.0),
             .eb0 = Eigen::Vector3d(0.5, -1.0, 0.3),
             .eb1 = Eigen::Vector3d(0.5, 1.0, 0.3),
         },
         EdgeEdgeRegion::Ea1OnEdgeB,
         0.339001},
    }};

    for (const auto& test_case : cases) {
        const auto result = EdgeEdgeDistance::compute(test_case.input);
        EXPECT_EQ(result.region, test_case.region);
        EXPECT_NEAR(result.distance_squared, test_case.distance_squared, 1e-8);
        EXPECT_TRUE(result.gradient.allFinite());
        EXPECT_TRUE(result.hessian.allFinite());
    }
}

TEST(EdgeEdgeDistanceTest, MatchesFiniteDifferenceAcrossRepresentativeCases) {
    test::expect_matches_finite_difference<EdgeEdgeDistance>({
        .ea0 = Eigen::Vector3d(0.0, 0.0, 0.0),
        .ea1 = Eigen::Vector3d(1.0, 0.0, 0.0),
        .eb0 = Eigen::Vector3d(0.5, -1.0, 1.0),
        .eb1 = Eigen::Vector3d(0.5, 1.0, 1.0),
    });
    test::expect_matches_finite_difference<EdgeEdgeDistance>({
        .ea0 = Eigen::Vector3d(0.0, 0.0, 0.0),
        .ea1 = Eigen::Vector3d(1.0, 0.0, 0.0),
        .eb0 = Eigen::Vector3d(-0.4, -1.0, 0.2),
        .eb1 = Eigen::Vector3d(-0.4, 1.0, 0.2),
    });
}

TEST(EdgeEdgeDistanceTest, RejectsZeroLengthEdges) {
    EXPECT_THROW(
        EdgeEdgeDistance::compute({
            .ea0 = Eigen::Vector3d::Zero(),
            .ea1 = Eigen::Vector3d::Zero(),
            .eb0 = Eigen::Vector3d::UnitY(),
            .eb1 = Eigen::Vector3d::UnitY() + Eigen::Vector3d::UnitZ(),
        }),
        std::invalid_argument);
}

}  // namespace
}  // namespace rtr::system::physics::ipc
