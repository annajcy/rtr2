#include <gtest/gtest.h>

#include "rtr/system/physics/ipc/geometry/distance_concept.hpp"
#include "rtr/system/physics/ipc/geometry/edge_edge_distance.hpp"
#include "rtr/system/physics/ipc/geometry/point_edge_distance.hpp"
#include "rtr/system/physics/ipc/geometry/point_point_distance.hpp"
#include "rtr/system/physics/ipc/geometry/point_triangle_distance.hpp"

namespace rtr::system::physics::ipc {
namespace {

static_assert(Distance<PointPointDistance>);
static_assert(Distance<PointEdgeDistance>);
static_assert(Distance<PointTriangleDistance>);
static_assert(Distance<EdgeEdgeDistance>);

TEST(DistanceConceptTest, DocumentsCurrentDistanceKernelCoverage) {
    EXPECT_TRUE((Distance<PointPointDistance>));
    EXPECT_TRUE((Distance<PointEdgeDistance>));
    EXPECT_TRUE((Distance<PointTriangleDistance>));
    EXPECT_TRUE((Distance<EdgeEdgeDistance>));
}

}  // namespace
}  // namespace rtr::system::physics::ipc
