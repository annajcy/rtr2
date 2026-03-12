#pragma once

#include "rtr/system/physics/collision/collider_shape.hpp"
#include "rtr/system/physics/collision/contact.hpp"
#include "rtr/system/physics/collision/plane_common.hpp"

namespace rtr::system::physics {

template <>
struct ContactPairTrait<WorldMesh, WorldPlane> {
    static ContactResult generate(const WorldMesh& mesh, const WorldPlane& plane) {
        return detail::average_penetrating_points_against_plane(mesh.vertices, plane);
    }
};

template <>
struct ContactPairTrait<WorldPlane, WorldMesh> {
    static ContactResult generate(const WorldPlane& plane, const WorldMesh& mesh) {
        auto result = ContactPairTrait<WorldMesh, WorldPlane>::generate(mesh, plane);
        result.normal = -result.normal;
        return result;
    }
};

}  // namespace rtr::system::physics
