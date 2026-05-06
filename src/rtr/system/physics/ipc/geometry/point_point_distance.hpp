#pragma once

#include "rtr/system/physics/ipc/geometry/distance_common.hpp"

namespace rtr::system::physics::ipc {

struct PointPointDistanceResult : DistanceResult<6> {};

struct PointPointDistance {
    struct Input {
        Eigen::Vector3d p0;
        Eigen::Vector3d p1;
    };

    using Result = PointPointDistanceResult;

    static Result compute(const Input& input) {
        detail::require_finite_point(input.p0, "PointPointDistance p0");
        detail::require_finite_point(input.p1, "PointPointDistance p1");

        Eigen::Matrix<double, 6, 1> dofs;
        dofs << input.p0, input.p1;

        const auto base_result = detail::evaluate_distance_expression<6>(dofs, [](const auto& x) {
            const auto p0 = x.template segment<3>(0);
            const auto p1 = x.template segment<3>(3);
            return detail::squared_norm(p0 - p1);
        });

        Result result;
        detail::assign_distance_result(base_result, result);
        return result;
    }
};

inline PointPointDistanceResult point_point_distance(const Eigen::Vector3d& p0,
                                                     const Eigen::Vector3d& p1) {
    return PointPointDistance::compute({.p0 = p0, .p1 = p1});
}

}  // namespace rtr::system::physics::ipc
