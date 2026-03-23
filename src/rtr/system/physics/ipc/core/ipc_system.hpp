#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
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

using IPCBodyID = std::uint64_t;
inline constexpr IPCBodyID kInvalidIPCBodyId = std::numeric_limits<IPCBodyID>::max();

struct IPCConfig {
    double dt{0.01};
    Eigen::Vector3d gravity{0.0, -9.81, 0.0};
    NewtonSolverParams solver_params{};
};

class IPCSystem {
public:
    explicit IPCSystem(IPCConfig config = {}) : m_config(std::move(config)) {}

    IPCBodyID create_tet_body(TetBody body) {
        const IPCBodyID body_id = m_next_body_id++;
        m_tet_bodies.emplace(body_id, std::move(body));
        m_body_order.push_back(body_id);
        m_needs_rebuild = true;
        return body_id;
    }

    bool has_tet_body(IPCBodyID id) const {
        return id != kInvalidIPCBodyId && m_tet_bodies.contains(id);
    }

    bool remove_tet_body(IPCBodyID id) {
        if (!has_tet_body(id)) {
            return false;
        }

        m_tet_bodies.erase(id);
        m_body_order.erase(
            std::remove(m_body_order.begin(), m_body_order.end(), id),
            m_body_order.end()
        );
        m_body_layouts.erase(id);
        m_needs_rebuild = true;

        if (m_tet_bodies.empty()) {
            clear_runtime_state();
        }
        return true;
    }

    void initialize() {
        rebuild_runtime_state();
    }

    void step() {
        if (m_needs_rebuild || (!m_initialized && !m_tet_bodies.empty())) {
            rebuild_runtime_state();
        }
        if (!m_initialized || m_state.dof_count() == 0) {
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
    bool initialized() const { return m_initialized; }
    std::size_t tet_body_count() const { return m_tet_bodies.size(); }
    const TetBody& get_tet_body(IPCBodyID id) const { return m_tet_bodies.at(id); }
    TetBody& get_tet_body(IPCBodyID id) { return m_tet_bodies.at(id); }

    void refresh_tet_body_runtime_data(IPCBodyID id) {
        auto& body = m_tet_bodies.at(id);
        const std::size_t original_vertex_count = body.vertex_count();

        body.precompute();
        body.info.vertex_count = body.vertex_count();

        if (!m_initialized || m_needs_rebuild) {
            return;
        }
        if (body.vertex_count() != original_vertex_count) {
            m_needs_rebuild = true;
            return;
        }

        const std::size_t base_vertex = body.info.dof_offset / 3u;
        for (std::size_t local_vertex = 0; local_vertex < body.vertex_count(); ++local_vertex) {
            const std::size_t global_vertex = base_vertex + local_vertex;
            const Eigen::Index base = static_cast<Eigen::Index>(3u * global_vertex);
            m_state.mass_diag.segment<3>(base).setConstant(body.vertex_masses[local_vertex]);
        }
        build_free_dof_mask();
        compute_x_hat();
    }

    std::vector<Eigen::Vector3d> get_body_positions(IPCBodyID body_id) const {
        ensure_initialized();
        const auto& body = m_tet_bodies.at(body_id);
        std::vector<Eigen::Vector3d> positions{};
        positions.reserve(body.vertex_count());
        const std::size_t base_vertex = body.info.dof_offset / 3u;
        for (std::size_t local_vertex = 0; local_vertex < body.vertex_count(); ++local_vertex) {
            positions.push_back(m_state.position(base_vertex + local_vertex));
        }
        return positions;
    }

private:
    struct BodyLayout {
        std::size_t dof_offset{0};
        std::size_t vertex_count{0};
    };

    struct BodyRuntimeState {
        std::vector<Eigen::Vector3d> x{};
        std::vector<Eigen::Vector3d> x_prev{};
        std::vector<Eigen::Vector3d> v{};
    };

    IPCConfig m_config{};
    IPCState m_state{};
    std::unordered_map<IPCBodyID, TetBody> m_tet_bodies{};
    std::vector<IPCBodyID> m_body_order{};
    std::unordered_map<IPCBodyID, BodyLayout> m_body_layouts{};
    IPCBodyID m_next_body_id{0};
    Eigen::VectorXd m_x_hat{};
    std::vector<bool> m_free_dof_mask{};
    bool m_initialized{false};
    bool m_needs_rebuild{false};

    void ensure_initialized() const {
        if (!m_initialized || m_needs_rebuild) {
            throw std::logic_error("IPCSystem must be initialized before stepping or reading body state.");
        }
    }

    void clear_runtime_state() {
        m_state.resize(0);
        m_x_hat.resize(0);
        m_free_dof_mask.clear();
        m_body_layouts.clear();
        m_initialized = false;
        m_needs_rebuild = false;
    }

    void rebuild_runtime_state() {
        if (m_tet_bodies.empty()) {
            clear_runtime_state();
            return;
        }

        std::unordered_map<IPCBodyID, BodyRuntimeState> previous_body_states{};
        previous_body_states.reserve(m_body_layouts.size());
        for (const auto& [body_id, layout] : m_body_layouts) {
            if (!m_tet_bodies.contains(body_id)) {
                continue;
            }
            if (layout.vertex_count == 0 || layout.dof_offset + 3u * layout.vertex_count > m_state.dof_count()) {
                continue;
            }

            BodyRuntimeState runtime_state{};
            runtime_state.x.reserve(layout.vertex_count);
            runtime_state.x_prev.reserve(layout.vertex_count);
            runtime_state.v.reserve(layout.vertex_count);

            const std::size_t base_vertex = layout.dof_offset / 3u;
            for (std::size_t local_vertex = 0; local_vertex < layout.vertex_count; ++local_vertex) {
                const std::size_t global_vertex = base_vertex + local_vertex;
                runtime_state.x.push_back(m_state.position(global_vertex));
                runtime_state.x_prev.push_back(m_state.x_prev.segment<3>(static_cast<Eigen::Index>(3u * global_vertex)));
                runtime_state.v.push_back(m_state.v.segment<3>(static_cast<Eigen::Index>(3u * global_vertex)));
            }

            previous_body_states.emplace(body_id, std::move(runtime_state));
        }

        std::size_t total_vertices = 0;
        std::size_t dof_offset = 0;
        for (const IPCBodyID body_id : m_body_order) {
            auto& body = m_tet_bodies.at(body_id);
            body.precompute();
            body.info.dof_offset = dof_offset;
            body.info.vertex_count = body.vertex_count();
            total_vertices += body.vertex_count();
            dof_offset += 3u * body.vertex_count();
        }

        m_state.resize(total_vertices);
        m_state.x.setZero();
        m_state.x_prev.setZero();
        m_state.v.setZero();
        m_state.mass_diag.setZero();
        m_body_layouts.clear();

        for (const IPCBodyID body_id : m_body_order) {
            const auto& body = m_tet_bodies.at(body_id);
            const std::size_t base_vertex = body.info.dof_offset / 3u;
            const auto previous_state = previous_body_states.find(body_id);
            const bool can_restore_previous_state =
                previous_state != previous_body_states.end() &&
                previous_state->second.x.size() == body.vertex_count();

            for (std::size_t local_vertex = 0; local_vertex < body.vertex_count(); ++local_vertex) {
                const std::size_t global_vertex = base_vertex + local_vertex;
                const Eigen::Index base = static_cast<Eigen::Index>(3u * global_vertex);
                const Eigen::Vector3d rest_position = body.geometry.rest_positions[local_vertex];

                if (can_restore_previous_state) {
                    m_state.x.segment<3>(base) = previous_state->second.x[local_vertex];
                    m_state.x_prev.segment<3>(base) = previous_state->second.x_prev[local_vertex];
                    m_state.v.segment<3>(base) = previous_state->second.v[local_vertex];
                } else {
                    m_state.x.segment<3>(base) = rest_position;
                    m_state.x_prev.segment<3>(base) = rest_position;
                    m_state.v.segment<3>(base).setZero();
                }

                m_state.mass_diag.segment<3>(base).setConstant(body.vertex_masses[local_vertex]);
            }

            m_body_layouts.emplace(body_id, BodyLayout{
                .dof_offset = body.info.dof_offset,
                .vertex_count = body.vertex_count(),
            });
        }

        build_free_dof_mask();
        compute_x_hat();
        m_initialized = true;
        m_needs_rebuild = false;
    }

    void compute_x_hat() { m_x_hat = m_state.x_prev + m_config.dt * m_state.v; }

    void build_free_dof_mask() {
        m_free_dof_mask.assign(m_state.dof_count(), true);
        for (const IPCBodyID body_id : m_body_order) {
            const auto& body = m_tet_bodies.at(body_id);
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
        for (const IPCBodyID body_id : m_body_order) {
            total_energy += material_energy_variant::compute_energy(m_tet_bodies.at(body_id), x);
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
        for (const IPCBodyID body_id : m_body_order) {
            material_energy_variant::compute_gradient(m_tet_bodies.at(body_id), x, gradient);
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
        for (const IPCBodyID body_id : m_body_order) {
            material_energy_variant::compute_hessian_triplets(m_tet_bodies.at(body_id), x, triplets);
        }
    }
};

}  // namespace rtr::system::physics::ipc
