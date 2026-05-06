#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include <pbpt/math/math.h>

#include "rtr/system/physics/ipc/model/tet_body.hpp"
#include "rtr/utils/obj_types.hpp"

namespace rtr::system::physics::ipc {

struct TetSurfaceMapping {
    std::vector<uint32_t> surface_indices{};
    std::vector<uint32_t> surface_vertex_ids{};
};

namespace detail::tet_to_mesh {

inline void recompute_mesh_normals(utils::ObjMeshData& mesh);

struct Face {
    uint32_t v[3];

    bool operator==(const Face& other) const {
        return v[0] == other.v[0] && v[1] == other.v[1] && v[2] == other.v[2];
    }
};

struct FaceHash {
    std::size_t operator()(const Face& face) const {
        std::size_t hash = face.v[0];
        hash ^= face.v[1] + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= face.v[2] + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        return hash;
    }
};

struct FaceData {
    int count{0};
    std::array<uint32_t, 3> oriented_vertices{};
};

inline pbpt::math::Vec3 to_pbpt_vec3(const Eigen::Vector3d& value) {
    return pbpt::math::Vec3(
        static_cast<float>(value.x()),
        static_cast<float>(value.y()),
        static_cast<float>(value.z())
    );
}

inline pbpt::math::Vec3 read_position_from_dofs(const Eigen::VectorXd& positions, uint32_t vertex_id) {
    const Eigen::Index base = static_cast<Eigen::Index>(3u * vertex_id);
    if (base + 2 >= positions.size()) {
        throw std::out_of_range("tet_to_mesh positions index out of range.");
    }
    return pbpt::math::Vec3(
        static_cast<float>(positions[base + 0]),
        static_cast<float>(positions[base + 1]),
        static_cast<float>(positions[base + 2])
    );
}

inline void validate_surface_mapping(const TetSurfaceMapping& surface,
                                     std::size_t vertex_count,
                                     std::size_t vertex_offset) {
    for (const uint32_t vertex_id : surface.surface_vertex_ids) {
        if (vertex_offset + static_cast<std::size_t>(vertex_id) >= vertex_count) {
            throw std::out_of_range("TetSurfaceMapping surface vertex id out of range.");
        }
    }
    for (const uint32_t vertex_id : surface.surface_indices) {
        if (vertex_offset + static_cast<std::size_t>(vertex_id) >= vertex_count) {
            throw std::out_of_range("TetSurfaceMapping surface index out of range.");
        }
    }
}

inline void validate_surface_mapping(const TetSurfaceMapping& surface, std::size_t vertex_count) {
    for (const uint32_t vertex_id : surface.surface_vertex_ids) {
        if (vertex_id >= vertex_count) {
            throw std::out_of_range("TetSurfaceMapping surface vertex id out of range.");
        }
    }
    for (const uint32_t vertex_id : surface.surface_indices) {
        if (vertex_id >= vertex_count) {
            throw std::out_of_range("TetSurfaceMapping surface index out of range.");
        }
    }
}

template <typename PositionReader>
inline utils::ObjMeshData build_surface_mesh_from_mapping(const TetSurfaceMapping& surface,
                                                          PositionReader&& read_position) {
    utils::ObjMeshData mesh{};
    mesh.vertices.reserve(surface.surface_vertex_ids.size());
    mesh.indices.reserve(surface.surface_indices.size());

    std::unordered_map<uint32_t, uint32_t> old_to_new{};
    old_to_new.reserve(surface.surface_vertex_ids.size());

    for (uint32_t new_index = 0; new_index < surface.surface_vertex_ids.size(); ++new_index) {
        const uint32_t old_index = surface.surface_vertex_ids[new_index];
        old_to_new.emplace(old_index, new_index);

        utils::ObjVertex vertex{};
        vertex.position = read_position(old_index);
        vertex.uv = pbpt::math::Vec2(0.0f);
        vertex.normal = pbpt::math::Vec3(0.0f);
        mesh.vertices.push_back(vertex);
    }

    for (const uint32_t old_index : surface.surface_indices) {
        const auto found = old_to_new.find(old_index);
        if (found == old_to_new.end()) {
            throw std::invalid_argument("TetSurfaceMapping indices reference a non-surface vertex.");
        }
        mesh.indices.push_back(found->second);
    }

    recompute_mesh_normals(mesh);
    return mesh;
}

inline void recompute_mesh_normals(utils::ObjMeshData& mesh) {
    if (mesh.indices.size() % 3u != 0u) {
        throw std::invalid_argument("ObjMeshData indices must be triangles (size % 3 == 0).");
    }

    std::vector<pbpt::math::Vec3> accum_normals(mesh.vertices.size(), pbpt::math::Vec3(0.0f));
    for (std::size_t i = 0; i + 2u < mesh.indices.size(); i += 3u) {
        const uint32_t i0 = mesh.indices[i + 0u];
        const uint32_t i1 = mesh.indices[i + 1u];
        const uint32_t i2 = mesh.indices[i + 2u];
        if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() || i2 >= mesh.vertices.size()) {
            throw std::out_of_range("ObjMeshData index out of range during normal recompute.");
        }

        const pbpt::math::Vec3& p0 = mesh.vertices[i0].position;
        const pbpt::math::Vec3& p1 = mesh.vertices[i1].position;
        const pbpt::math::Vec3& p2 = mesh.vertices[i2].position;
        const pbpt::math::Vec3 face_normal = pbpt::math::normalize(pbpt::math::cross(p1 - p0, p2 - p0));
        accum_normals[i0] += face_normal;
        accum_normals[i1] += face_normal;
        accum_normals[i2] += face_normal;
    }

    for (std::size_t vertex_index = 0; vertex_index < mesh.vertices.size(); ++vertex_index) {
        if (pbpt::math::length(accum_normals[vertex_index]) > 0.0f) {
            mesh.vertices[vertex_index].normal = pbpt::math::normalize(accum_normals[vertex_index]);
        } else {
            mesh.vertices[vertex_index].normal = pbpt::math::Vec3(0.0f, 1.0f, 0.0f);
        }
    }
}

}  // namespace detail::tet_to_mesh

inline TetSurfaceMapping build_tet_surface_mapping(const TetGeometry& geometry) {
    std::unordered_map<detail::tet_to_mesh::Face, detail::tet_to_mesh::FaceData, detail::tet_to_mesh::FaceHash>
        face_map{};
    face_map.reserve(geometry.tet_count() * 4u);

    auto add_face = [&](uint32_t a, uint32_t b, uint32_t c) {
        std::array<uint32_t, 3> sorted = {a, b, c};
        std::sort(sorted.begin(), sorted.end());
        detail::tet_to_mesh::Face face{{sorted[0], sorted[1], sorted[2]}};

        auto& data = face_map[face];
        data.count += 1;
        data.oriented_vertices = {a, b, c};
    };

    for (const auto& tet : geometry.tets) {
        for (const std::size_t vertex_index : tet) {
            if (vertex_index >= geometry.vertex_count()) {
                throw std::out_of_range("TetGeometry tetrahedron vertex index out of range.");
            }
        }

        add_face(static_cast<uint32_t>(tet[0]), static_cast<uint32_t>(tet[1]), static_cast<uint32_t>(tet[3]));
        add_face(static_cast<uint32_t>(tet[1]), static_cast<uint32_t>(tet[2]), static_cast<uint32_t>(tet[3]));
        add_face(static_cast<uint32_t>(tet[2]), static_cast<uint32_t>(tet[0]), static_cast<uint32_t>(tet[3]));
        add_face(static_cast<uint32_t>(tet[0]), static_cast<uint32_t>(tet[2]), static_cast<uint32_t>(tet[1]));
    }

    TetSurfaceMapping result{};
    std::vector<bool> is_surface_vertex(geometry.vertex_count(), false);

    for (const auto& [_, data] : face_map) {
        if (data.count != 1) {
            continue;
        }
        for (int i = 0; i < 3; ++i) {
            const uint32_t vertex_id = data.oriented_vertices[static_cast<std::size_t>(i)];
            result.surface_indices.push_back(vertex_id);
            if (!is_surface_vertex[vertex_id]) {
                is_surface_vertex[vertex_id] = true;
                result.surface_vertex_ids.push_back(vertex_id);
            }
        }
    }

    std::sort(result.surface_vertex_ids.begin(), result.surface_vertex_ids.end());
    return result;
}

inline TetSurfaceMapping build_tet_surface_mapping(const TetBody& body) {
    return build_tet_surface_mapping(body.geometry);
}

inline utils::ObjMeshData tet_dofs_to_surface_mesh(const Eigen::VectorXd& positions,
                                                   const TetSurfaceMapping& surface,
                                                   std::size_t vertex_offset = 0u) {
    if ((positions.size() % 3) != 0) {
        throw std::invalid_argument("tet_dofs_to_surface_mesh positions size must be divisible by 3.");
    }
    const std::size_t vertex_count = static_cast<std::size_t>(positions.size() / 3);
    detail::tet_to_mesh::validate_surface_mapping(surface, vertex_count, vertex_offset);
    return detail::tet_to_mesh::build_surface_mesh_from_mapping(surface, [&](uint32_t vertex_id) {
        return detail::tet_to_mesh::read_position_from_dofs(
            positions,
            static_cast<uint32_t>(vertex_offset + static_cast<std::size_t>(vertex_id))
        );
    });
}

inline utils::ObjMeshData tet_rest_to_surface_mesh(const TetGeometry& geometry, const TetSurfaceMapping& surface) {
    detail::tet_to_mesh::validate_surface_mapping(surface, geometry.vertex_count());
    return detail::tet_to_mesh::build_surface_mesh_from_mapping(surface, [&](uint32_t vertex_id) {
        return detail::tet_to_mesh::to_pbpt_vec3(geometry.rest_positions[vertex_id]);
    });
}

inline void update_surface_mesh_from_tet_dofs(utils::ObjMeshData& mesh,
                                              const Eigen::VectorXd& positions,
                                              const TetSurfaceMapping& surface,
                                              std::size_t vertex_offset = 0u) {
    if (mesh.vertices.size() != surface.surface_vertex_ids.size()) {
        throw std::invalid_argument(
            "update_surface_mesh_from_tet_dofs mesh vertex count must match surface vertex ids."
        );
    }
    if (mesh.indices.size() != surface.surface_indices.size()) {
        throw std::invalid_argument(
            "update_surface_mesh_from_tet_dofs mesh index count must match surface indices."
        );
    }
    if ((positions.size() % 3) != 0) {
        throw std::invalid_argument("update_surface_mesh_from_tet_dofs positions size must be divisible by 3.");
    }

    const std::size_t vertex_count = static_cast<std::size_t>(positions.size() / 3);
    detail::tet_to_mesh::validate_surface_mapping(surface, vertex_count, vertex_offset);

    for (std::size_t i = 0; i < surface.surface_vertex_ids.size(); ++i) {
        mesh.vertices[i].position = detail::tet_to_mesh::read_position_from_dofs(
            positions,
            static_cast<uint32_t>(vertex_offset + static_cast<std::size_t>(surface.surface_vertex_ids[i]))
        );
    }

    detail::tet_to_mesh::recompute_mesh_normals(mesh);
}

}  // namespace rtr::system::physics::ipc
