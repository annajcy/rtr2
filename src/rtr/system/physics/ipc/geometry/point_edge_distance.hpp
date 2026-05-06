#pragma once

#include <array>

#include "rtr/system/physics/ipc/geometry/distance_common.hpp"
#include "rtr/system/physics/ipc/geometry/point_point_distance.hpp"

namespace rtr::system::physics::ipc {

enum class PointEdgeRegion {
    EdgeInterior,
    Endpoint0,
    Endpoint1,
};

struct PointEdgeDistanceResult : DistanceResult<9> {
    PointEdgeRegion region{PointEdgeRegion::EdgeInterior};
};

struct PointEdgeDistance {
    struct Input {
        Eigen::Vector3d p;
        Eigen::Vector3d e0;
        Eigen::Vector3d e1;
    };

    using Result = PointEdgeDistanceResult;

    static Result compute(const Input& input) {
        detail::require_finite_point(input.p, "PointEdgeDistance p");
        detail::require_finite_point(input.e0, "PointEdgeDistance e0");
        detail::require_finite_point(input.e1, "PointEdgeDistance e1");

        const Eigen::Vector3d edge = input.e1 - input.e0;
        const double edge_length_squared = edge.squaredNorm();
        if (!std::isfinite(edge_length_squared) || edge_length_squared <= detail::kMinEdgeLengthSquared) {
            throw std::invalid_argument("PointEdgeDistance requires a non-degenerate edge.");
        }

        const double alpha = (input.p - input.e0).dot(edge) / edge_length_squared;
        if (alpha <= 0.0) {
            const auto pp = PointPointDistance::compute({.p0 = input.p, .p1 = input.e0});
            Result result;
            detail::embed_distance_result<2, 3>(pp, {0, 1}, result);
            result.region = PointEdgeRegion::Endpoint0;
            return result;
        }

        if (alpha >= 1.0) {
            const auto pp = PointPointDistance::compute({.p0 = input.p, .p1 = input.e1});
            Result result;
            detail::embed_distance_result<2, 3>(pp, {0, 2}, result);
            result.region = PointEdgeRegion::Endpoint1;
            return result;
        }

        Eigen::Matrix<double, 9, 1> dofs;
        dofs << input.p, input.e0, input.e1;

        const auto base_result = detail::evaluate_distance_expression<9>(dofs, [](const auto& x) {
            const auto p = x.template segment<3>(0);
            const auto e0 = x.template segment<3>(3);
            const auto e1 = x.template segment<3>(6);
            const auto edge = e1 - e0;
            return detail::squared_norm(detail::cross(edge, p - e0)) / detail::squared_norm(edge);
        });

        Result result;
        detail::assign_distance_result(base_result, result);
        result.region = PointEdgeRegion::EdgeInterior;
        return result;
    }
};

inline PointEdgeDistanceResult point_edge_distance(const Eigen::Vector3d& p,
                                                   const Eigen::Vector3d& e0,
                                                   const Eigen::Vector3d& e1) {
    return PointEdgeDistance::compute({.p = p, .e0 = e0, .e1 = e1});
}

}  // namespace rtr::system::physics::ipc
