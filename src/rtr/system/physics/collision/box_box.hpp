#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include "rtr/system/physics/collider.hpp"
#include "rtr/system/physics/collision/base.hpp"

namespace rtr::system::physics::detail {

enum class BoxBoxAxisKind {
    FaceA,
    FaceB,
    EdgeEdge,
};

struct BoxBoxBestAxis {
    pbpt::math::Float overlap{std::numeric_limits<pbpt::math::Float>::infinity()};
    pbpt::math::Vec3  normal{0.0f, 1.0f, 0.0f};
    BoxBoxAxisKind    kind{BoxBoxAxisKind::FaceA};
    int               index_a{0};
    int               index_b{0};
};

inline std::array<pbpt::math::Vec3, 3> box_axes(const WorldBox& box) {
    return {
        pbpt::math::normalize(box.rotation * pbpt::math::Vec3{1.0f, 0.0f, 0.0f}),
        pbpt::math::normalize(box.rotation * pbpt::math::Vec3{0.0f, 1.0f, 0.0f}),
        pbpt::math::normalize(box.rotation * pbpt::math::Vec3{0.0f, 0.0f, 1.0f}),
    };
}

inline pbpt::math::Float project_box_radius(const WorldBox& box, const std::array<pbpt::math::Vec3, 3>& axes,
                                            const pbpt::math::Vec3& axis) {
    return box.half_extents.x() * std::abs(pbpt::math::dot(axes[0], axis)) +
           box.half_extents.y() * std::abs(pbpt::math::dot(axes[1], axis)) +
           box.half_extents.z() * std::abs(pbpt::math::dot(axes[2], axis));
}

inline pbpt::math::Vec3 support_point(const WorldBox& box, const std::array<pbpt::math::Vec3, 3>& axes,
                                      const pbpt::math::Vec3& direction) {
    pbpt::math::Vec3 point = box.center;
    for (int i = 0; i < 3; ++i) {
        const auto sign = pbpt::math::dot(direction, axes[i]) >= 0.0f ? 1.0f : -1.0f;
        point += axes[i] * (box.half_extents[i] * sign);
    }
    return point;
}

inline std::pair<pbpt::math::Vec3, pbpt::math::Vec3> edge_segment(const WorldBox& box,
                                                                  const std::array<pbpt::math::Vec3, 3>& axes,
                                                                  int edge_axis, const pbpt::math::Vec3& outward_dir) {
    pbpt::math::Vec3 midpoint = box.center;
    for (int i = 0; i < 3; ++i) {
        if (i == edge_axis) {
            continue;
        }
        const auto sign = pbpt::math::dot(outward_dir, axes[i]) >= 0.0f ? 1.0f : -1.0f;
        midpoint += axes[i] * (box.half_extents[i] * sign);
    }

    const auto edge_offset = axes[edge_axis] * box.half_extents[edge_axis];
    return {midpoint - edge_offset, midpoint + edge_offset};
}

inline std::pair<pbpt::math::Vec3, pbpt::math::Vec3> closest_points_on_segments(const pbpt::math::Vec3& p1,
                                                                                 const pbpt::math::Vec3& q1,
                                                                                 const pbpt::math::Vec3& p2,
                                                                                 const pbpt::math::Vec3& q2) {
    constexpr pbpt::math::Float kEpsilon = 1e-6f;

    const auto d1 = q1 - p1;
    const auto d2 = q2 - p2;
    const auto r  = p1 - p2;

    const auto a = pbpt::math::dot(d1, d1);
    const auto e = pbpt::math::dot(d2, d2);
    const auto f = pbpt::math::dot(d2, r);

    pbpt::math::Float s = 0.0f;
    pbpt::math::Float t = 0.0f;

    if (a <= kEpsilon && e <= kEpsilon) {
        return {p1, p2};
    }

    if (a <= kEpsilon) {
        t = pbpt::math::clamp(f / e, 0.0f, 1.0f);
    } else {
        const auto c = pbpt::math::dot(d1, r);
        if (e <= kEpsilon) {
            s = pbpt::math::clamp(-c / a, 0.0f, 1.0f);
        } else {
            const auto b     = pbpt::math::dot(d1, d2);
            const auto denom = a * e - b * b;

            if (std::abs(denom) > kEpsilon) {
                s = pbpt::math::clamp((b * f - c * e) / denom, 0.0f, 1.0f);
            }

            t = (b * s + f) / e;
            if (t < 0.0f) {
                t = 0.0f;
                s = pbpt::math::clamp(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = pbpt::math::clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    return {p1 + d1 * s, p2 + d2 * t};
}

inline void update_best_axis(BoxBoxBestAxis& best, pbpt::math::Float overlap, const pbpt::math::Vec3& normal,
                             BoxBoxAxisKind kind, int index_a, int index_b) {
    if (overlap < best.overlap) {
        best.overlap = overlap;
        best.normal  = normal;
        best.kind    = kind;
        best.index_a = index_a;
        best.index_b = index_b;
    }
}

inline pbpt::math::Vec3 approximate_face_contact_point(const WorldBox& a, const std::array<pbpt::math::Vec3, 3>& axes_a,
                                                       const WorldBox& b, const std::array<pbpt::math::Vec3, 3>& axes_b,
                                                       const pbpt::math::Vec3& normal) {
    const auto support_a = support_point(a, axes_a, normal);
    const auto support_b = support_point(b, axes_b, -normal);
    return (support_a + support_b) * 0.5f;
}

inline pbpt::math::Vec3 approximate_edge_contact_point(const WorldBox& a, const std::array<pbpt::math::Vec3, 3>& axes_a,
                                                       int edge_axis_a, const WorldBox& b,
                                                       const std::array<pbpt::math::Vec3, 3>& axes_b, int edge_axis_b,
                                                       const pbpt::math::Vec3& normal) {
    const auto [a0, a1] = edge_segment(a, axes_a, edge_axis_a, normal);
    const auto [b0, b1] = edge_segment(b, axes_b, edge_axis_b, -normal);
    const auto [pa, pb] = closest_points_on_segments(a0, a1, b0, b1);
    return (pa + pb) * 0.5f;
}

}  // namespace rtr::system::physics::detail

namespace rtr::system::physics {

template <>
struct ContactPairTrait<WorldBox, WorldBox> {
    static ContactResult generate(const WorldBox& a, const WorldBox& b) {
        constexpr pbpt::math::Float kEpsilon = 1e-6f;

        const auto axes_a = detail::box_axes(a);
        const auto axes_b = detail::box_axes(b);
        const auto delta  = b.center - a.center;

        std::array<std::array<pbpt::math::Float, 3>, 3> rotation{};
        std::array<std::array<pbpt::math::Float, 3>, 3> abs_rotation{};
        std::array<pbpt::math::Float, 3>                translation_a{};
        std::array<pbpt::math::Float, 3>                translation_b{};

        for (int i = 0; i < 3; ++i) {
            translation_a[i] = pbpt::math::dot(delta, axes_a[i]);
            translation_b[i] = pbpt::math::dot(delta, axes_b[i]);
            for (int j = 0; j < 3; ++j) {
                rotation[i][j]     = pbpt::math::dot(axes_a[i], axes_b[j]);
                abs_rotation[i][j] = std::abs(rotation[i][j]) + kEpsilon;
            }
        }

        detail::BoxBoxBestAxis best_axis{};

        for (int i = 0; i < 3; ++i) {
            const auto ra      = a.half_extents[i];
            const auto rb      = b.half_extents.x() * abs_rotation[i][0] + b.half_extents.y() * abs_rotation[i][1] +
                            b.half_extents.z() * abs_rotation[i][2];
            const auto dist    = std::abs(translation_a[i]);
            const auto overlap = ra + rb - dist;
            if (overlap <= 0.0f) {
                return ContactResult{};
            }

            auto normal = axes_a[i];
            if (translation_a[i] < 0.0f) {
                normal = -normal;
            }
            detail::update_best_axis(best_axis, overlap, normal, detail::BoxBoxAxisKind::FaceA, i, -1);
        }

        for (int j = 0; j < 3; ++j) {
            const auto ra      = a.half_extents.x() * abs_rotation[0][j] + a.half_extents.y() * abs_rotation[1][j] +
                            a.half_extents.z() * abs_rotation[2][j];
            const auto rb      = b.half_extents[j];
            const auto dist    = std::abs(translation_b[j]);
            const auto overlap = ra + rb - dist;
            if (overlap <= 0.0f) {
                return ContactResult{};
            }

            auto normal = axes_b[j];
            if (translation_b[j] < 0.0f) {
                normal = -normal;
            }
            detail::update_best_axis(best_axis, overlap, normal, detail::BoxBoxAxisKind::FaceB, -1, j);
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                const auto axis = pbpt::math::cross(axes_a[i], axes_b[j]);
                const auto axis_length_sq = pbpt::math::dot(axis, axis);
                if (axis_length_sq <= kEpsilon * kEpsilon) {
                    continue;
                }

                const auto axis_normalized = axis / std::sqrt(axis_length_sq);
                const auto ra              = detail::project_box_radius(a, axes_a, axis_normalized);
                const auto rb              = detail::project_box_radius(b, axes_b, axis_normalized);
                const auto dist            = std::abs(pbpt::math::dot(delta, axis_normalized));
                const auto overlap         = ra + rb - dist;
                if (overlap <= 0.0f) {
                    return ContactResult{};
                }

                auto normal = axis_normalized;
                if (pbpt::math::dot(delta, normal) < 0.0f) {
                    normal = -normal;
                }
                detail::update_best_axis(best_axis, overlap, normal, detail::BoxBoxAxisKind::EdgeEdge, i, j);
            }
        }

        ContactResult result{};
        result.normal      = best_axis.normal;
        result.penetration = best_axis.overlap;

        if (best_axis.kind == detail::BoxBoxAxisKind::EdgeEdge) {
            result.point = detail::approximate_edge_contact_point(
                a, axes_a, best_axis.index_a, b, axes_b, best_axis.index_b, result.normal);
        } else {
            result.point = detail::approximate_face_contact_point(a, axes_a, b, axes_b, result.normal);
        }

        return result;
    }
};

}  // namespace rtr::system::physics
