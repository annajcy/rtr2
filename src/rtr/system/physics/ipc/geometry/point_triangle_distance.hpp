#pragma once

#include <array>

#include "rtr/system/physics/ipc/geometry/distance_common.hpp"
#include "rtr/system/physics/ipc/geometry/point_edge_distance.hpp"
#include "rtr/system/physics/ipc/geometry/point_point_distance.hpp"

namespace rtr::system::physics::ipc {

enum class PointTriangleRegion {
    Face,
    Edge01,
    Edge02,
    Edge12,
    Vertex0,
    Vertex1,
    Vertex2,
    DegenerateTriangle,
};

struct PointTriangleDistanceResult : DistanceResult<12> {
    PointTriangleRegion region{PointTriangleRegion::Face};
};

struct PointTriangleDistance {
    struct Input {
        Eigen::Vector3d p;
        Eigen::Vector3d t0;
        Eigen::Vector3d t1;
        Eigen::Vector3d t2;
    };

    using Result = PointTriangleDistanceResult;

private:
    static Result embed_point_point(const PointPointDistance::Input& input,
                                    const std::array<int, 2>& point_map,
                                    const PointTriangleRegion region) {
        const auto pp = PointPointDistance::compute(input);
        Result result;
        detail::embed_distance_result<2, 4>(pp, point_map, result);
        result.region = region;
        return result;
    }

    static Result embed_point_edge(const PointEdgeDistance::Input& input,
                                   const std::array<int, 3>& point_map,
                                   const PointTriangleRegion region) {
        const auto pe = PointEdgeDistance::compute(input);
        Result result;
        detail::embed_distance_result<3, 4>(pe, point_map, result);
        result.region = region;
        return result;
    }

    static Result compute_degenerate_triangle(const Input& input) {
        Result best_result;
        bool has_candidate = false;

        const auto consider = [&](const auto& candidate) {
            if (!has_candidate || candidate.distance_squared < best_result.distance_squared) {
                best_result = candidate;
                has_candidate = true;
            }
        };

        consider(embed_point_point({.p0 = input.p, .p1 = input.t0}, {0, 1}, PointTriangleRegion::DegenerateTriangle));
        consider(embed_point_point({.p0 = input.p, .p1 = input.t1}, {0, 2}, PointTriangleRegion::DegenerateTriangle));
        consider(embed_point_point({.p0 = input.p, .p1 = input.t2}, {0, 3}, PointTriangleRegion::DegenerateTriangle));

        if ((input.t1 - input.t0).squaredNorm() > detail::kMinEdgeLengthSquared) {
            consider(embed_point_edge({.p = input.p, .e0 = input.t0, .e1 = input.t1},
                                      {0, 1, 2},
                                      PointTriangleRegion::DegenerateTriangle));
        }
        if ((input.t2 - input.t0).squaredNorm() > detail::kMinEdgeLengthSquared) {
            consider(embed_point_edge({.p = input.p, .e0 = input.t0, .e1 = input.t2},
                                      {0, 1, 3},
                                      PointTriangleRegion::DegenerateTriangle));
        }
        if ((input.t2 - input.t1).squaredNorm() > detail::kMinEdgeLengthSquared) {
            consider(embed_point_edge({.p = input.p, .e0 = input.t1, .e1 = input.t2},
                                      {0, 2, 3},
                                      PointTriangleRegion::DegenerateTriangle));
        }

        return best_result;
    }

public:
    static Result compute(const Input& input) {
        detail::require_finite_point(input.p, "PointTriangleDistance p");
        detail::require_finite_point(input.t0, "PointTriangleDistance t0");
        detail::require_finite_point(input.t1, "PointTriangleDistance t1");
        detail::require_finite_point(input.t2, "PointTriangleDistance t2");

        const Eigen::Vector3d ab = input.t1 - input.t0;
        const Eigen::Vector3d ac = input.t2 - input.t0;
        const double area_squared = detail::cross(ab, ac).squaredNorm();
        if (!std::isfinite(area_squared) || area_squared <= detail::kMinTriangleAreaSquared) {
            return compute_degenerate_triangle(input);
        }

        const Eigen::Vector3d ap = input.p - input.t0;
        const double d1 = ab.dot(ap);
        const double d2 = ac.dot(ap);
        if (d1 <= 0.0 && d2 <= 0.0) {
            return embed_point_point({.p0 = input.p, .p1 = input.t0}, {0, 1}, PointTriangleRegion::Vertex0);
        }

        const Eigen::Vector3d bp = input.p - input.t1;
        const double d3 = ab.dot(bp);
        const double d4 = ac.dot(bp);
        if (d3 >= 0.0 && d4 <= d3) {
            return embed_point_point({.p0 = input.p, .p1 = input.t1}, {0, 2}, PointTriangleRegion::Vertex1);
        }

        const double vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
            return embed_point_edge({.p = input.p, .e0 = input.t0, .e1 = input.t1},
                                    {0, 1, 2},
                                    PointTriangleRegion::Edge01);
        }

        const Eigen::Vector3d cp = input.p - input.t2;
        const double d5 = ab.dot(cp);
        const double d6 = ac.dot(cp);
        if (d6 >= 0.0 && d5 <= d6) {
            return embed_point_point({.p0 = input.p, .p1 = input.t2}, {0, 3}, PointTriangleRegion::Vertex2);
        }

        const double vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
            return embed_point_edge({.p = input.p, .e0 = input.t0, .e1 = input.t2},
                                    {0, 1, 3},
                                    PointTriangleRegion::Edge02);
        }

        const double va = d3 * d6 - d5 * d4;
        if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
            return embed_point_edge({.p = input.p, .e0 = input.t1, .e1 = input.t2},
                                    {0, 2, 3},
                                    PointTriangleRegion::Edge12);
        }

        Eigen::Matrix<double, 12, 1> dofs;
        dofs << input.p, input.t0, input.t1, input.t2;

        const auto base_result = detail::evaluate_distance_expression<12>(dofs, [](const auto& x) {
            const auto p = x.template segment<3>(0);
            const auto t0 = x.template segment<3>(3);
            const auto t1 = x.template segment<3>(6);
            const auto t2 = x.template segment<3>(9);
            const auto normal = detail::cross(t1 - t0, t2 - t0);
            const auto signed_distance_numerator = (p - t0).dot(normal);
            return (signed_distance_numerator * signed_distance_numerator) / detail::squared_norm(normal);
        });

        Result result;
        detail::assign_distance_result(base_result, result);
        result.region = PointTriangleRegion::Face;
        return result;
    }
};

inline PointTriangleDistanceResult point_triangle_distance(const Eigen::Vector3d& p,
                                                           const Eigen::Vector3d& t0,
                                                           const Eigen::Vector3d& t1,
                                                           const Eigen::Vector3d& t2) {
    return PointTriangleDistance::compute({.p = p, .t0 = t0, .t1 = t1, .t2 = t2});
}

}  // namespace rtr::system::physics::ipc
