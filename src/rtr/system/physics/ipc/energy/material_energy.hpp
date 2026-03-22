#pragma once

#include <array>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "rtr/system/physics/ipc/energy/energy_concept.hpp"
#include "rtr/system/physics/ipc/energy/material_model/tet_material_model_concept.hpp"
#include "rtr/system/physics/ipc/model/tet_body.hpp"

namespace rtr::system::physics::ipc {

namespace material_energy_detail {

inline void validate_tet_body(const TetBody& body) {
    if (body.geometry.tets.empty()) {
        throw std::invalid_argument("MaterialEnergy requires TetBody to contain tetrahedra.");
    }
    if (body.geometry.Dm_inv.size() != body.tet_count() || body.geometry.rest_volumes.size() != body.tet_count()) {
        throw std::invalid_argument("MaterialEnergy requires TetBody geometry precompute to be available.");
    }
    if ((body.info.dof_offset % 3u) != 0u) {
        throw std::invalid_argument("MaterialEnergy requires body dof_offset to be aligned to vertex-major DOFs.");
    }
}

template <typename Material>
inline void validate_input(const TetBody& body, const Eigen::VectorXd& x, const Material& material) {
    (void)material;
    validate_tet_body(body);
    if ((x.size() % 3) != 0) {
        throw std::invalid_argument("MaterialEnergy requires x to have size 3N.");
    }
    const std::size_t required_dofs = body.info.dof_offset + 3u * body.vertex_count();
    if (required_dofs > static_cast<std::size_t>(x.size())) {
        throw std::out_of_range("MaterialEnergy body slice exceeds x size.");
    }
}

inline Eigen::Vector3d read_position(const Eigen::VectorXd& x,
                                     std::size_t dof_offset,
                                     std::size_t vertex_index) {
    const Eigen::Index base = static_cast<Eigen::Index>(dof_offset + 3u * vertex_index);
    return x.segment<3>(base);
}

inline Eigen::Matrix3d build_Ds(const Eigen::VectorXd& x,
                                std::size_t dof_offset,
                                const std::array<std::size_t, 4>& tet) {
    const Eigen::Vector3d x0 = read_position(x, dof_offset, tet[0]);
    Eigen::Matrix3d Ds;
    Ds.col(0) = read_position(x, dof_offset, tet[1]) - x0;
    Ds.col(1) = read_position(x, dof_offset, tet[2]) - x0;
    Ds.col(2) = read_position(x, dof_offset, tet[3]) - x0;
    return Ds;
}

inline std::array<Eigen::Vector3d, 4> compute_shape_gradients(const Eigen::Matrix3d& Dm_inv) {
    const Eigen::Matrix3d Dm_inv_T = Dm_inv.transpose();
    std::array<Eigen::Vector3d, 4> gradients{};
    gradients[1] = Dm_inv_T.col(0);
    gradients[2] = Dm_inv_T.col(1);
    gradients[3] = Dm_inv_T.col(2);
    gradients[0] = -(gradients[1] + gradients[2] + gradients[3]);
    return gradients;
}

inline Eigen::Matrix<double, 9, 3> build_dFdx_matrix(const Eigen::Vector3d& grad_N) {
    Eigen::Matrix<double, 9, 3> dFdx = Eigen::Matrix<double, 9, 3>::Zero();
    for (int col = 0; col < 3; ++col) {
        for (int row = 0; row < 3; ++row) {
            dFdx(row + 3 * col, row) = grad_N[col];
        }
    }
    return dFdx;
}

}  // namespace material_energy_detail

template <TetMaterialModel Material>
struct MaterialEnergy {
    struct Input {
        const TetBody& body;
        const Eigen::VectorXd& x;
        const Material& material;
    };

private:
    static void validate_input(const Input& input) {
        material_energy_detail::validate_input(input.body, input.x, input.material);
    }

public:
    static double compute_energy(const Input& input) {
        validate_input(input);

        double total_energy = 0.0;
        for (std::size_t tet_index = 0; tet_index < input.body.tet_count(); ++tet_index) {
            const auto& tet = input.body.geometry.tets[tet_index];
            const Eigen::Matrix3d Ds =
                material_energy_detail::build_Ds(input.x, input.body.info.dof_offset, tet);
            const Eigen::Matrix3d F = Ds * input.body.geometry.Dm_inv[tet_index];
            total_energy += input.material.compute_energy(
                F,
                input.body.geometry.rest_volumes[tet_index],
                input.body.youngs_modulus,
                input.body.poisson_ratio
            );
        }
        return total_energy;
    }

    static void compute_gradient(const Input& input, Eigen::VectorXd& gradient) {
        validate_input(input);
        if (gradient.size() != input.x.size()) {
            throw std::invalid_argument("MaterialEnergy gradient must match x size.");
        }

        for (std::size_t tet_index = 0; tet_index < input.body.tet_count(); ++tet_index) {
            const auto& tet = input.body.geometry.tets[tet_index];
            const Eigen::Matrix3d Ds =
                material_energy_detail::build_Ds(input.x, input.body.info.dof_offset, tet);
            const Eigen::Matrix3d& Dm_inv = input.body.geometry.Dm_inv[tet_index];
            const Eigen::Matrix3d F = Ds * Dm_inv;
            const double rest_volume = input.body.geometry.rest_volumes[tet_index];
            const Eigen::Matrix3d P = input.material.compute_pk1(
                F,
                rest_volume,
                input.body.youngs_modulus,
                input.body.poisson_ratio
            );
            const auto grad_N = material_energy_detail::compute_shape_gradients(Dm_inv);

            for (int local_vertex = 0; local_vertex < 4; ++local_vertex) {
                const Eigen::Vector3d local_gradient = rest_volume * P * grad_N[local_vertex];
                const Eigen::Index base =
                    static_cast<Eigen::Index>(input.body.info.dof_offset + 3u * tet[local_vertex]);
                gradient.segment<3>(base) += local_gradient;
            }
        }
    }

    static void compute_hessian_triplets(const Input& input,
                                         std::vector<Eigen::Triplet<double>>& triplets) {
        validate_input(input);

        for (std::size_t tet_index = 0; tet_index < input.body.tet_count(); ++tet_index) {
            const auto& tet = input.body.geometry.tets[tet_index];
            const Eigen::Matrix3d Ds =
                material_energy_detail::build_Ds(input.x, input.body.info.dof_offset, tet);
            const Eigen::Matrix3d& Dm_inv = input.body.geometry.Dm_inv[tet_index];
            const Eigen::Matrix3d F = Ds * Dm_inv;
            const double rest_volume = input.body.geometry.rest_volumes[tet_index];
            Eigen::Matrix<double, 9, 9> hessian_F = input.material.compute_hessian(
                F,
                rest_volume,
                input.body.youngs_modulus,
                input.body.poisson_ratio
            );
            hessian_F = 0.5 * (hessian_F + hessian_F.transpose());

            const auto grad_N = material_energy_detail::compute_shape_gradients(Dm_inv);
            std::array<Eigen::Matrix<double, 9, 3>, 4> dFdx{};
            for (int local_vertex = 0; local_vertex < 4; ++local_vertex) {
                dFdx[local_vertex] = material_energy_detail::build_dFdx_matrix(grad_N[local_vertex]);
            }

            for (int a = 0; a < 4; ++a) {
                const Eigen::Index row_base =
                    static_cast<Eigen::Index>(input.body.info.dof_offset + 3u * tet[static_cast<std::size_t>(a)]);
                for (int b = 0; b < 4; ++b) {
                    const Eigen::Index col_base =
                        static_cast<Eigen::Index>(input.body.info.dof_offset + 3u * tet[static_cast<std::size_t>(b)]);
                    const Eigen::Matrix3d block =
                        rest_volume * dFdx[static_cast<std::size_t>(a)].transpose() * hessian_F *
                        dFdx[static_cast<std::size_t>(b)];

                    for (int i = 0; i < 3; ++i) {
                        for (int j = 0; j < 3; ++j) {
                            triplets.emplace_back(row_base + i, col_base + j, block(i, j));
                        }
                    }
                }
            }
        }
    }
};

}  // namespace rtr::system::physics::ipc
