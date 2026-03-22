#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <type_traits>
#include <variant>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>

#include "rtr/system/physics/ipc/energy/material_model/tet_material_variant.hpp"
#include "rtr/system/physics/ipc/model/ipc_body.hpp"

namespace rtr::system::physics::ipc {

struct TetGeometry {
    std::vector<Eigen::Vector3d> rest_positions{};
    std::vector<std::array<std::size_t, 4>> tets{};

    std::vector<Eigen::Matrix3d> Dm_inv{};
    std::vector<double> rest_volumes{};

    std::size_t vertex_count() const { return rest_positions.size(); }
    std::size_t tet_count() const { return tets.size(); }

private:
    struct TetRestData {
        Eigen::Matrix3d Dm_inv{Eigen::Matrix3d::Identity()};
        double volume{0.0};
    };

    void validate() const {
        if (rest_positions.empty()) {
            throw std::invalid_argument("TetGeometry requires at least one rest position.");
        }
        if (tets.empty()) {
            throw std::invalid_argument("TetGeometry requires at least one tetrahedron.");
        }
    }

    TetRestData precompute_tet_rest_data(const std::array<std::size_t, 4>& tet) const {
        constexpr double kMinDeterminantMagnitude = 1e-12;

        for (const auto vertex_index : tet) {
            if (vertex_index >= vertex_count()) {
                throw std::out_of_range("TetBody tetrahedron vertex index out of range.");
            }
        }

        const Eigen::Vector3d& X0 = rest_positions[tet[0]];
        const Eigen::Vector3d& X1 = rest_positions[tet[1]];
        const Eigen::Vector3d& X2 = rest_positions[tet[2]];
        const Eigen::Vector3d& X3 = rest_positions[tet[3]];

        Eigen::Matrix3d Dm;
        Dm.col(0) = X1 - X0;
        Dm.col(1) = X2 - X0;
        Dm.col(2) = X3 - X0;

        const double determinant = Dm.determinant();
        const double abs_determinant = std::abs(determinant);
        if (!std::isfinite(abs_determinant) || abs_determinant <= kMinDeterminantMagnitude) {
            throw std::invalid_argument("TetBody encountered a degenerate tetrahedron.");
        }

        return TetRestData{
            .Dm_inv = Dm.inverse(),
            .volume = abs_determinant / 6.0,
        };
    }

public:
    void precompute_rest_data() {
        validate();
        Dm_inv.clear();
        rest_volumes.clear();
        Dm_inv.reserve(tet_count());
        rest_volumes.reserve(tet_count());

        for (const auto& tet : tets) {
            const TetRestData rest_data = precompute_tet_rest_data(tet);
            Dm_inv.push_back(rest_data.Dm_inv);
            rest_volumes.push_back(rest_data.volume);
        }
    }
};

struct TetBody {
    IPCBodyInfo info{.type = IPCBodyType::Tet};
    TetGeometry geometry{};

    std::vector<double> vertex_masses{};
    TetMaterialVariant material{FixedCorotatedMaterial{}};

    std::vector<bool> fixed_vertices{};

    std::size_t vertex_count() const { return geometry.vertex_count(); }
    std::size_t tet_count() const { return geometry.tet_count(); }

private:
    void normalize_fixed_vertices() {
        if (fixed_vertices.empty()) {
            fixed_vertices.assign(vertex_count(), false);
        } else if (fixed_vertices.size() != vertex_count()) {
            throw std::invalid_argument("TetBody fixed_vertices size must match geometry.rest_positions size.");
        }
    }

public:
    void precompute() {
        info.type = IPCBodyType::Tet;
        info.vertex_count = vertex_count();
        geometry.precompute_rest_data();
        normalize_fixed_vertices();
        vertex_masses.assign(vertex_count(), 0.0);

        const double density = std::visit(
            [](const auto& tet_material) {
                return static_cast<double>(tet_material.density());
            },
            material
        );

        for (std::size_t tet_index = 0; tet_index < geometry.tet_count(); ++tet_index) {
            const double tet_vertex_mass = density * geometry.rest_volumes[tet_index] / 4.0;
            for (const auto vertex_index : geometry.tets[tet_index]) {
                vertex_masses[vertex_index] += tet_vertex_mass;
            }
        }
    }
};

inline TetGeometry generate_tet_geometry_block(std::size_t nx,
                                               std::size_t ny,
                                               std::size_t nz,
                                               double spacing,
                                               const Eigen::Vector3d& origin) {
    if (nx == 0 || ny == 0 || nz == 0) {
        throw std::invalid_argument("generate_tet_geometry_block requires nx, ny, nz to be positive.");
    }
    if (!std::isfinite(spacing) || spacing <= 0.0) {
        throw std::invalid_argument("generate_tet_geometry_block spacing must be finite and positive.");
    }

    TetGeometry geometry{};

    const auto stride_x = nx + 1;
    const auto stride_y = ny + 1;
    const auto stride_z = nz + 1;
    const auto total_vertices = stride_x * stride_y * stride_z;

    geometry.rest_positions.reserve(total_vertices);

    auto vertex_index = [stride_x, stride_y](std::size_t i, std::size_t j, std::size_t k) {
        return i + stride_x * (j + stride_y * k);
    };

    for (std::size_t k = 0; k <= nz; ++k) {
        for (std::size_t j = 0; j <= ny; ++j) {
            for (std::size_t i = 0; i <= nx; ++i) {
                geometry.rest_positions.push_back(
                    origin + spacing * Eigen::Vector3d(static_cast<double>(i),
                                                       static_cast<double>(j),
                                                       static_cast<double>(k))
                );
            }
        }
    }

    auto orient_tet = [&geometry](std::array<std::size_t, 4> tet) {
        Eigen::Matrix3d Dm;
        const Eigen::Vector3d& X0 = geometry.rest_positions[tet[0]];
        Dm.col(0) = geometry.rest_positions[tet[1]] - X0;
        Dm.col(1) = geometry.rest_positions[tet[2]] - X0;
        Dm.col(2) = geometry.rest_positions[tet[3]] - X0;
        if (Dm.determinant() < 0.0) {
            std::swap(tet[1], tet[2]);
        }
        return tet;
    };

    geometry.tets.reserve(nx * ny * nz * 6);
    for (std::size_t k = 0; k < nz; ++k) {
        for (std::size_t j = 0; j < ny; ++j) {
            for (std::size_t i = 0; i < nx; ++i) {
                const std::size_t v000 = vertex_index(i,     j,     k);
                const std::size_t v100 = vertex_index(i + 1, j,     k);
                const std::size_t v010 = vertex_index(i,     j + 1, k);
                const std::size_t v110 = vertex_index(i + 1, j + 1, k);
                const std::size_t v001 = vertex_index(i,     j,     k + 1);
                const std::size_t v101 = vertex_index(i + 1, j,     k + 1);
                const std::size_t v011 = vertex_index(i,     j + 1, k + 1);
                const std::size_t v111 = vertex_index(i + 1, j + 1, k + 1);

                geometry.tets.push_back(orient_tet({v000, v100, v110, v111}));
                geometry.tets.push_back(orient_tet({v000, v100, v101, v111}));
                geometry.tets.push_back(orient_tet({v000, v001, v101, v111}));
                geometry.tets.push_back(orient_tet({v000, v001, v011, v111}));
                geometry.tets.push_back(orient_tet({v000, v010, v110, v111}));
                geometry.tets.push_back(orient_tet({v000, v010, v011, v111}));
            }
        }
    }

    return geometry;
}

inline TetBody generate_tet_block(std::size_t nx,
                                  std::size_t ny,
                                  std::size_t nz,
                                  double spacing,
                                  const Eigen::Vector3d& origin) {
    TetBody body{};
    body.info.type = IPCBodyType::Tet;
    body.geometry = generate_tet_geometry_block(nx, ny, nz, spacing, origin);
    body.info.vertex_count = body.vertex_count();
    return body;
}

}  // namespace rtr::system::physics::ipc
