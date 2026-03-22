#pragma once

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "rtr/system/physics/ipc/core/ipc_state.hpp"
#include "rtr/system/physics/ipc/energy/gravity_energy.hpp"
#include "rtr/system/physics/ipc/energy/inertial_energy.hpp"
#include "rtr/system/physics/ipc/energy/material_energy.hpp"
#include "rtr/system/physics/ipc/model/tet_body.hpp"
#include "rtr/system/physics/ipc/solver/newton_solver.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::system::physics::ipc {

struct IPCConfig {
    double dt{0.01};
    Eigen::Vector3d gravity{0.0, -9.81, 0.0};
    NewtonSolverParams solver_params{};
};

class IPCSystem {
public:
    explicit IPCSystem(IPCConfig config = {}) : m_config(std::move(config)) {}

    void add_tet_body(TetBody body) {
        m_tet_bodies.push_back(std::move(body));
        m_initialized = false;
    }

    void initialize() {
        std::size_t total_vertices = 0;
        std::size_t dof_offset = 0;
        for (auto& body : m_tet_bodies) {
            body.precompute();
            body.info.dof_offset = dof_offset;
            body.info.vertex_count = body.vertex_count();
            total_vertices += body.vertex_count();
            dof_offset += 3u * body.vertex_count();
        }

        m_state.resize(total_vertices);
        m_state.x_prev = m_state.x;
        m_state.v.setZero();
        m_state.mass_diag.setZero();

        for (const auto& body : m_tet_bodies) {
            const std::size_t base_vertex = body.info.dof_offset / 3u;
            for (std::size_t local_vertex = 0; local_vertex < body.vertex_count(); ++local_vertex) {
                const std::size_t global_vertex = base_vertex + local_vertex;
                const Eigen::Index base = static_cast<Eigen::Index>(3u * global_vertex);
                m_state.x.segment<3>(base) = body.geometry.rest_positions[local_vertex];
                m_state.x_prev.segment<3>(base) = body.geometry.rest_positions[local_vertex];
                m_state.mass_diag.segment<3>(base).setConstant(body.vertex_masses[local_vertex]);
            }
        }

        m_x_hat = m_state.x;
        build_free_dof_mask();
        m_initialized = true;
    }

    void step() {
        if (!m_initialized) {
            if (m_tet_bodies.empty()) {
                return;
            }
            throw std::logic_error("IPCSystem must be initialized before stepping.");
        }
        if (m_state.dof_count() == 0) {
            return;
        }

        m_state.x_prev = m_state.x;
        compute_x_hat();

        const NewtonProblem problem{
            .compute_energy = [this](const Eigen::VectorXd& x) { return compute_total_energy(x); },
            .compute_gradient = [this](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
                compute_total_gradient(x, gradient);
            },
            .compute_hessian_triplets =
                [this](const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) {
                    compute_total_hessian(x, triplets);
                },
        };

        const NewtonSolverResult result =
            solve(m_state, m_x_hat, m_free_dof_mask, problem, m_config.solver_params);
        if (!result.converged) {
            utils::get_logger("system.physics.ipc")->warn(
                "IPCSystem Newton solve did not converge (iterations={}, final_energy={:.6e}, final_gradient={:.6e}).",
                result.iterations,
                result.final_energy,
                result.final_gradient_norm
            );
        }

        m_state.v = (m_state.x - m_state.x_prev) / m_config.dt;
        for (Eigen::Index i = 0; i < m_state.v.size(); ++i) {
            if (!m_free_dof_mask[static_cast<std::size_t>(i)]) {
                m_state.v[i] = 0.0;
            }
        }
    }

    const IPCState& state() const { return m_state; }
    const TetBody& tet_body(std::size_t index) const { return m_tet_bodies.at(index); }
    TetBody& tet_body(std::size_t index) { return m_tet_bodies.at(index); }
    std::size_t tet_body_count() const { return m_tet_bodies.size(); }

    std::vector<Eigen::Vector3d> get_body_positions(std::size_t body_index) const {
        ensure_initialized();
        const auto& body = m_tet_bodies.at(body_index);
        std::vector<Eigen::Vector3d> positions{};
        positions.reserve(body.vertex_count());
        const std::size_t base_vertex = body.info.dof_offset / 3u;
        for (std::size_t local_vertex = 0; local_vertex < body.vertex_count(); ++local_vertex) {
            positions.push_back(m_state.position(base_vertex + local_vertex));
        }
        return positions;
    }

private:
    IPCConfig m_config{};
    IPCState m_state{};
    std::vector<TetBody> m_tet_bodies{};
    Eigen::VectorXd m_x_hat{};
    std::vector<bool> m_free_dof_mask{};
    bool m_initialized{false};

    void ensure_initialized() const {
        if (!m_initialized) {
            throw std::logic_error("IPCSystem must be initialized before stepping or reading body state.");
        }
    }

    void compute_x_hat() { m_x_hat = m_state.x_prev + m_config.dt * m_state.v; }

    void build_free_dof_mask() {
        m_free_dof_mask.assign(m_state.dof_count(), true);
        for (const auto& body : m_tet_bodies) {
            const std::size_t base_vertex = body.info.dof_offset / 3u;
            for (std::size_t local_vertex = 0; local_vertex < body.fixed_vertices.size(); ++local_vertex) {
                if (!body.fixed_vertices[local_vertex]) {
                    continue;
                }
                const std::size_t global_vertex = base_vertex + local_vertex;
                for (int axis = 0; axis < 3; ++axis) {
                    m_free_dof_mask[3u * global_vertex + static_cast<std::size_t>(axis)] = false;
                }
            }
        }
    }

    double compute_total_energy(const Eigen::VectorXd& x) const {
        double total_energy = 0.0;
        total_energy += InertialEnergy::compute_energy(InertialEnergy::Input{
            .x = x,
            .x_hat = m_x_hat,
            .mass_diag = m_state.mass_diag,
            .dt = m_config.dt,
        });
        total_energy += GravityEnergy::compute_energy(GravityEnergy::Input{
            .x = x,
            .mass_diag = m_state.mass_diag,
            .gravity = m_config.gravity,
        });
        for (const auto& body : m_tet_bodies) {
            total_energy += material_energy_variant::compute_energy(body, x);
        }
        return total_energy;
    }

    void compute_total_gradient(const Eigen::VectorXd& x, Eigen::VectorXd& gradient) const {
        if (gradient.size() != x.size()) {
            throw std::invalid_argument("IPCSystem total gradient buffer must match x size.");
        }

        gradient.setZero();
        InertialEnergy::compute_gradient(InertialEnergy::Input{
            .x = x,
            .x_hat = m_x_hat,
            .mass_diag = m_state.mass_diag,
            .dt = m_config.dt,
        }, gradient);
        GravityEnergy::compute_gradient(GravityEnergy::Input{
            .x = x,
            .mass_diag = m_state.mass_diag,
            .gravity = m_config.gravity,
        }, gradient);
        for (const auto& body : m_tet_bodies) {
            material_energy_variant::compute_gradient(body, x, gradient);
        }
    }

    void compute_total_hessian(const Eigen::VectorXd& x,
                               std::vector<Eigen::Triplet<double>>& triplets) const {
        triplets.clear();
        InertialEnergy::compute_hessian_triplets(InertialEnergy::Input{
            .x = x,
            .x_hat = m_x_hat,
            .mass_diag = m_state.mass_diag,
            .dt = m_config.dt,
        }, triplets);
        for (const auto& body : m_tet_bodies) {
            material_energy_variant::compute_hessian_triplets(body, x, triplets);
        }
    }
};

}  // namespace rtr::system::physics::ipc
