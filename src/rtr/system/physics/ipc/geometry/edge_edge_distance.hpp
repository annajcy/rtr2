#pragma once

#include <array>

#include "rtr/system/physics/ipc/geometry/distance_common.hpp"
#include "rtr/system/physics/ipc/geometry/point_edge_distance.hpp"

namespace rtr::system::physics::ipc {

enum class EdgeEdgeRegion {
    InteriorInterior,
    Ea0OnEdgeB,
    Ea1OnEdgeB,
    Eb0OnEdgeA,
    Eb1OnEdgeA,
    ParallelDegenerate,
};

struct EdgeEdgeDistanceResult : DistanceResult<12> {
    EdgeEdgeRegion region{EdgeEdgeRegion::InteriorInterior};
};

struct EdgeEdgeDistance {
    struct Input {
        Eigen::Vector3d ea0;
        Eigen::Vector3d ea1;
        Eigen::Vector3d eb0;
        Eigen::Vector3d eb1;
    };

    using Result = EdgeEdgeDistanceResult;

private:
    static Result embed_point_edge(const PointEdgeDistance::Input& input,
                                   const std::array<int, 3>& point_map,
                                   const EdgeEdgeRegion region) {
        const auto pe = PointEdgeDistance::compute(input);
        Result result;
        detail::embed_distance_result<3, 4>(pe, point_map, result);
        result.region = region;
        return result;
    }

    static Result compute_fallback_candidates(const Input& input, const EdgeEdgeRegion region_override) {
        Result best_result;
        bool has_candidate = false;

        const auto consider = [&](const Result& candidate) {
            if (!has_candidate || candidate.distance_squared < best_result.distance_squared) {
                best_result = candidate;
                has_candidate = true;
            }
        };

        consider(embed_point_edge({.p = input.ea0, .e0 = input.eb0, .e1 = input.eb1},
                                  {0, 2, 3},
                                  region_override == EdgeEdgeRegion::ParallelDegenerate ? region_override
                                                                                       : EdgeEdgeRegion::Ea0OnEdgeB));
        consider(embed_point_edge({.p = input.ea1, .e0 = input.eb0, .e1 = input.eb1},
                                  {1, 2, 3},
                                  region_override == EdgeEdgeRegion::ParallelDegenerate ? region_override
                                                                                       : EdgeEdgeRegion::Ea1OnEdgeB));
        consider(embed_point_edge({.p = input.eb0, .e0 = input.ea0, .e1 = input.ea1},
                                  {2, 0, 1},
                                  region_override == EdgeEdgeRegion::ParallelDegenerate ? region_override
                                                                                       : EdgeEdgeRegion::Eb0OnEdgeA));
        consider(embed_point_edge({.p = input.eb1, .e0 = input.ea0, .e1 = input.ea1},
                                  {3, 0, 1},
                                  region_override == EdgeEdgeRegion::ParallelDegenerate ? region_override
                                                                                       : EdgeEdgeRegion::Eb1OnEdgeA));
        return best_result;
    }

public:
    static Result compute(const Input& input) {
        detail::require_finite_point(input.ea0, "EdgeEdgeDistance ea0");
        detail::require_finite_point(input.ea1, "EdgeEdgeDistance ea1");
        detail::require_finite_point(input.eb0, "EdgeEdgeDistance eb0");
        detail::require_finite_point(input.eb1, "EdgeEdgeDistance eb1");

        const Eigen::Vector3d edge_a = input.ea1 - input.ea0;
        const Eigen::Vector3d edge_b = input.eb1 - input.eb0;
        const double edge_a_length_squared = edge_a.squaredNorm();
        const double edge_b_length_squared = edge_b.squaredNorm();
        if (!std::isfinite(edge_a_length_squared) || edge_a_length_squared <= detail::kMinEdgeLengthSquared ||
            !std::isfinite(edge_b_length_squared) || edge_b_length_squared <= detail::kMinEdgeLengthSquared) {
            throw std::invalid_argument("EdgeEdgeDistance requires two non-degenerate edges.");
        }

        const Eigen::Vector3d cross_product = detail::cross(edge_a, edge_b);
        const double cross_norm_squared = cross_product.squaredNorm();
        if (cross_norm_squared < detail::kParallelThreshold) {
            return compute_fallback_candidates(input, EdgeEdgeRegion::ParallelDegenerate);
        }

        const Eigen::Vector3d w0 = input.ea0 - input.eb0;
        const double a = edge_a.dot(edge_a);
        const double b = edge_a.dot(edge_b);
        const double c = edge_b.dot(edge_b);
        const double d = edge_a.dot(w0);
        const double e = edge_b.dot(w0);
        const double denom = a * c - b * b;

        const double s = (b * e - c * d) / denom;
        const double t = (a * e - b * d) / denom;
        if (s > 0.0 && s < 1.0 && t > 0.0 && t < 1.0) {
            Eigen::Matrix<double, 12, 1> dofs;
            dofs << input.ea0, input.ea1, input.eb0, input.eb1;

            const auto base_result = detail::evaluate_distance_expression<12>(dofs, [](const auto& x) {
                const auto ea0 = x.template segment<3>(0);
                const auto ea1 = x.template segment<3>(3);
                const auto eb0 = x.template segment<3>(6);
                const auto eb1 = x.template segment<3>(9);
                const auto edge_a = ea1 - ea0;
                const auto edge_b = eb1 - eb0;
                const auto normal = detail::cross(edge_a, edge_b);
                const auto signed_distance_numerator = (ea0 - eb0).dot(normal);
                return (signed_distance_numerator * signed_distance_numerator) / detail::squared_norm(normal);
            });

            Result result;
            detail::assign_distance_result(base_result, result);
            result.region = EdgeEdgeRegion::InteriorInterior;
            return result;
        }

        return compute_fallback_candidates(input, EdgeEdgeRegion::InteriorInterior);
    }
};

inline EdgeEdgeDistanceResult edge_edge_distance(const Eigen::Vector3d& ea0,
                                                 const Eigen::Vector3d& ea1,
                                                 const Eigen::Vector3d& eb0,
                                                 const Eigen::Vector3d& eb1) {
    return EdgeEdgeDistance::compute({.ea0 = ea0, .ea1 = ea1, .eb0 = eb0, .eb1 = eb1});
}

}  // namespace rtr::system::physics::ipc
