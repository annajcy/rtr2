#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <new>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "rtr/system/physics/ipc/energy/material_model/tet_fixed_corotated.hpp"
#include "rtr/system/physics/ipc/model/tet_body.hpp"
#include "rtr/utils/obj_types.hpp"

#ifdef RTR_HAS_FTETWILD
#include <floattetwild/FloatTetwild.h>
#include <floattetwild/MeshIO.hpp>
#include <floattetwild/Parameters.h>
#include <floattetwild/Types.hpp>
#include <geogram/basic/common.h>
#include <geogram/mesh/mesh.h>
#endif

namespace rtr::system::physics::ipc {

struct TetMeshingParams {
    double ideal_edge_length_rel{0.05};
    double eps_rel{1e-3};
    int max_its{80};
    bool skip_simplify{false};
    bool quiet{true};
};

struct TetMeshingResult {
    TetGeometry geometry{};
    bool success{false};
    std::string error_message{};
};

inline std::vector<Eigen::Vector3d> obj_mesh_to_eigen_positions(const utils::ObjMeshData& mesh);
inline std::vector<std::array<uint32_t, 3>> obj_mesh_to_triangle_indices(const utils::ObjMeshData& mesh);

namespace detail::mesh_to_tet {

inline void validate_obj_mesh(const utils::ObjMeshData& mesh) {
    if (mesh.vertices.empty()) {
        throw std::invalid_argument("mesh_to_tet requires ObjMeshData vertices to be non-empty.");
    }
    if (mesh.indices.empty()) {
        throw std::invalid_argument("mesh_to_tet requires ObjMeshData indices to be non-empty.");
    }
    (void)obj_mesh_to_triangle_indices(mesh);
}

inline TetMeshingResult make_failure(std::string message) {
    TetMeshingResult result{};
    result.success = false;
    result.error_message = std::move(message);
    return result;
}

inline TetGeometry build_tet_geometry_from_matrices(const Eigen::MatrixXd& vertices,
                                                    const Eigen::MatrixXi& tets) {
    constexpr double kMinTetDeterminant = 1e-12;

    if (vertices.cols() != 3) {
        throw std::invalid_argument("fTetWild output vertex matrix must have 3 columns.");
    }
    if (tets.cols() != 4) {
        throw std::invalid_argument("fTetWild output tet matrix must have 4 columns.");
    }
    if (vertices.rows() == 0 || tets.rows() == 0) {
        throw std::runtime_error("fTetWild tetrahedralization returned an empty volume mesh.");
    }

    TetGeometry geometry{};
    geometry.rest_positions.reserve(static_cast<std::size_t>(vertices.rows()));
    geometry.tets.reserve(static_cast<std::size_t>(tets.rows()));

    for (Eigen::Index row = 0; row < vertices.rows(); ++row) {
        geometry.rest_positions.emplace_back(vertices(row, 0), vertices(row, 1), vertices(row, 2));
    }

    for (Eigen::Index row = 0; row < tets.rows(); ++row) {
        std::array<std::size_t, 4> tet{};
        for (Eigen::Index col = 0; col < 4; ++col) {
            const int vertex_index = tets(row, col);
            if (vertex_index < 0 || vertex_index >= vertices.rows()) {
                throw std::out_of_range("fTetWild output tet index out of range.");
            }
            tet[static_cast<std::size_t>(col)] = static_cast<std::size_t>(vertex_index);
        }

        const Eigen::Vector3d& x0 = geometry.rest_positions[tet[0]];
        Eigen::Matrix3d dm{};
        dm.col(0) = geometry.rest_positions[tet[1]] - x0;
        dm.col(1) = geometry.rest_positions[tet[2]] - x0;
        dm.col(2) = geometry.rest_positions[tet[3]] - x0;

        const double determinant = dm.determinant();
        if (std::abs(determinant) <= kMinTetDeterminant) {
            throw std::runtime_error("fTetWild returned a degenerate tetrahedron.");
        }
        if (determinant < 0.0) {
            std::swap(tet[1], tet[2]);
        }

        geometry.tets.push_back(tet);
    }

    return geometry;
}

#ifdef RTR_HAS_FTETWILD
inline void initialize_ftetwild_runtime() {
    static std::once_flag once{};
    std::call_once(once, [] {
        GEO::initialize();
    });
}

inline void build_geo_surface_mesh(const utils::ObjMeshData& mesh, GEO::Mesh& surface_mesh) {
    validate_obj_mesh(mesh);

    const std::vector<Eigen::Vector3d> positions = obj_mesh_to_eigen_positions(mesh);
    const std::vector<std::array<uint32_t, 3>> triangles = obj_mesh_to_triangle_indices(mesh);

    std::vector<floatTetWild::Vector3> points{};
    points.reserve(positions.size());
    for (const Eigen::Vector3d& position : positions) {
        points.emplace_back(position.x(), position.y(), position.z());
    }

    std::vector<floatTetWild::Vector3i> faces{};
    faces.reserve(triangles.size());
    for (const auto& triangle : triangles) {
        faces.emplace_back(
            static_cast<int>(triangle[0]),
            static_cast<int>(triangle[1]),
            static_cast<int>(triangle[2])
        );
    }

    std::vector<int> face_tags(faces.size(), 0);
    floatTetWild::MeshIO::load_mesh(points, faces, surface_mesh, face_tags);
}
#endif

}  // namespace detail::mesh_to_tet

inline std::vector<Eigen::Vector3d> obj_mesh_to_eigen_positions(const utils::ObjMeshData& mesh) {
    std::vector<Eigen::Vector3d> positions{};
    positions.reserve(mesh.vertices.size());

    for (const auto& vertex : mesh.vertices) {
        positions.emplace_back(
            static_cast<double>(vertex.position.x()),
            static_cast<double>(vertex.position.y()),
            static_cast<double>(vertex.position.z())
        );
    }

    return positions;
}

inline std::vector<std::array<uint32_t, 3>> obj_mesh_to_triangle_indices(const utils::ObjMeshData& mesh) {
    if ((mesh.indices.size() % 3u) != 0u) {
        throw std::invalid_argument("ObjMeshData indices must be triangles (size % 3 == 0).");
    }

    std::vector<std::array<uint32_t, 3>> triangles{};
    triangles.reserve(mesh.indices.size() / 3u);

    for (std::size_t i = 0; i + 2u < mesh.indices.size(); i += 3u) {
        const uint32_t i0 = mesh.indices[i + 0u];
        const uint32_t i1 = mesh.indices[i + 1u];
        const uint32_t i2 = mesh.indices[i + 2u];
        if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() || i2 >= mesh.vertices.size()) {
            throw std::out_of_range("ObjMeshData indices out of range.");
        }
        triangles.push_back({i0, i1, i2});
    }

    return triangles;
}

inline bool ftetwild_available() {
#ifdef RTR_HAS_FTETWILD
    return true;
#else
    return false;
#endif
}

inline TetMeshingResult tetrahedralize_obj_mesh(const utils::ObjMeshData& mesh,
                                                const TetMeshingParams& params = {}) {
#ifndef RTR_HAS_FTETWILD
    (void)mesh;
    (void)params;
    return detail::mesh_to_tet::make_failure("fTetWild not available (RTR_HAS_FTETWILD=0).");
#else
    try {
        detail::mesh_to_tet::validate_obj_mesh(mesh);
        detail::mesh_to_tet::initialize_ftetwild_runtime();

        GEO::Mesh surface_mesh{};
        detail::mesh_to_tet::build_geo_surface_mesh(mesh, surface_mesh);

        floatTetWild::Parameters ftetwild_params{};
        ftetwild_params.ideal_edge_length_rel = params.ideal_edge_length_rel;
        ftetwild_params.eps_rel = params.eps_rel;
        ftetwild_params.max_its = params.max_its;
        ftetwild_params.is_quiet = params.quiet;
        ftetwild_params.log_level = params.quiet ? 6 : 3;

        Eigen::MatrixXd output_vertices{};
        Eigen::MatrixXi output_tets{};
        const int result_code = floatTetWild::tetrahedralization(
            surface_mesh,
            ftetwild_params,
            output_vertices,
            output_tets,
            -1,
            params.skip_simplify
        );
        if (result_code != 0) {
            return detail::mesh_to_tet::make_failure(
                "fTetWild tetrahedralization failed with error code " + std::to_string(result_code) + "."
            );
        }

        TetMeshingResult result{};
        result.geometry = detail::mesh_to_tet::build_tet_geometry_from_matrices(output_vertices, output_tets);
        result.success = true;
        return result;
    } catch (const std::exception& e) {
        return detail::mesh_to_tet::make_failure(e.what());
    } catch (...) {
        return detail::mesh_to_tet::make_failure("fTetWild tetrahedralization failed with an unknown error.");
    }
#endif
}

inline TetGeometry obj_mesh_to_tet_geometry(const utils::ObjMeshData& mesh,
                                            const TetMeshingParams& params = {}) {
    TetMeshingResult result = tetrahedralize_obj_mesh(mesh, params);
    if (!result.success) {
        throw std::runtime_error("obj_mesh_to_tet_geometry failed: " + result.error_message);
    }
    return std::move(result.geometry);
}

inline TetBody obj_mesh_to_tet_body(const utils::ObjMeshData& mesh,
                                    TetMaterialVariant material = FixedCorotatedMaterial{},
                                    const TetMeshingParams& params = {}) {
    TetBody body{};
    body.material = std::move(material);
    body.info.type = IPCBodyType::Tet;
    body.geometry = obj_mesh_to_tet_geometry(mesh, params);
    body.info.vertex_count = body.geometry.vertex_count();
    return body;
}

}  // namespace rtr::system::physics::ipc
