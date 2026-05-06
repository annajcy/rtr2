#include <array>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "distance_test_utils.hpp"

namespace rtr::system::physics::ipc {
namespace {

struct PointTriangleCase {
    PointTriangleDistance::Input input;
    PointTriangleRegion region;
};

TEST(PointTriangleDistanceTest, CoversAllDocumentedRegions) {
    const std::array<PointTriangleCase, 8> cases{{
        {{
             .p = Eigen::Vector3d(0.4, 0.5, 1.2),
             .t0 = Eigen::Vector3d(0.0, 0.0, 0.0),
             .t1 = Eigen::Vector3d(2.0, 0.0, 0.0),
             .t2 = Eigen::Vector3d(0.0, 2.0, 0.0),
         },
         PointTriangleRegion::Face},
        {{
             .p = Eigen::Vector3d(0.6, -0.8, 0.3),
             .t0 = Eigen::Vector3d(0.0, 0.0, 0.0),
             .t1 = Eigen::Vector3d(2.0, 0.0, 0.0),
             .t2 = Eigen::Vector3d(0.0, 2.0, 0.0),
         },
         PointTriangleRegion::Edge01},
        {{
             .p = Eigen::Vector3d(-0.7, 0.6, 0.4),
             .t0 = Eigen::Vector3d(0.0, 0.0, 0.0),
             .t1 = Eigen::Vector3d(2.0, 0.0, 0.0),
             .t2 = Eigen::Vector3d(0.0, 2.0, 0.0),
         },
         PointTriangleRegion::Edge02},
        {{
             .p = Eigen::Vector3d(1.4, 1.4, 0.2),
             .t0 = Eigen::Vector3d(0.0, 0.0, 0.0),
             .t1 = Eigen::Vector3d(2.0, 0.0, 0.0),
             .t2 = Eigen::Vector3d(0.0, 2.0, 0.0),
         },
         PointTriangleRegion::Edge12},
        {{
             .p = Eigen::Vector3d(-0.8, -0.6, 0.5),
             .t0 = Eigen::Vector3d(0.0, 0.0, 0.0),
             .t1 = Eigen::Vector3d(2.0, 0.0, 0.0),
             .t2 = Eigen::Vector3d(0.0, 2.0, 0.0),
         },
         PointTriangleRegion::Vertex0},
        {{
             .p = Eigen::Vector3d(2.7, -0.4, 0.3),
             .t0 = Eigen::Vector3d(0.0, 0.0, 0.0),
             .t1 = Eigen::Vector3d(2.0, 0.0, 0.0),
             .t2 = Eigen::Vector3d(0.0, 2.0, 0.0),
         },
         PointTriangleRegion::Vertex1},
        {{
             .p = Eigen::Vector3d(-0.4, 2.6, 0.4),
             .t0 = Eigen::Vector3d(0.0, 0.0, 0.0),
             .t1 = Eigen::Vector3d(2.0, 0.0, 0.0),
             .t2 = Eigen::Vector3d(0.0, 2.0, 0.0),
         },
         PointTriangleRegion::Vertex2},
        {{
             .p = Eigen::Vector3d(0.3, 0.8, 0.2),
             .t0 = Eigen::Vector3d(0.0, 0.0, 0.0),
             .t1 = Eigen::Vector3d(1.0, 0.0, 0.0),
             .t2 = Eigen::Vector3d(2.0, 0.0, 0.0),
         },
         PointTriangleRegion::DegenerateTriangle},
    }};

    for (const auto& test_case : cases) {
        const auto result = PointTriangleDistance::compute(test_case.input);
        EXPECT_EQ(result.region, test_case.region);
        EXPECT_TRUE(std::isfinite(result.distance_squared));
        EXPECT_TRUE(result.gradient.allFinite());
        EXPECT_TRUE(result.hessian.allFinite());
    }
}

TEST(PointTriangleDistanceTest, MatchesFiniteDifferenceAcrossRepresentativeRegions) {
    test::expect_matches_finite_difference<PointTriangleDistance>({
        .p = Eigen::Vector3d(0.4, 0.5, 1.2),
        .t0 = Eigen::Vector3d(0.0, 0.0, 0.0),
        .t1 = Eigen::Vector3d(2.0, 0.0, 0.0),
        .t2 = Eigen::Vector3d(0.0, 2.0, 0.0),
    });
    test::expect_matches_finite_difference<PointTriangleDistance>({
        .p = Eigen::Vector3d(-1.4, -1.2, 0.6),
        .t0 = Eigen::Vector3d(0.0, 0.0, 0.0),
        .t1 = Eigen::Vector3d(2.0, 0.0, 0.0),
        .t2 = Eigen::Vector3d(0.0, 2.0, 0.0),
    });
}

}  // namespace
}  // namespace rtr::system::physics::ipc
