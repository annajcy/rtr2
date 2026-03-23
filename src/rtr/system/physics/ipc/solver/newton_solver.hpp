#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>

#include "rtr/system/physics/ipc/core/ipc_state.hpp"
#include "rtr/system/physics/ipc/solver/line_search.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::system::physics::ipc {

struct NewtonSolverParams {
    int max_iterations{50};
    double gradient_tolerance{1e-6};
    double dx_tolerance{1e-8};
    double regularization{1e-8};
    bool use_psd_projection{false};
};

struct NewtonSolverResult {
    int iterations{0};
    double final_gradient_norm{0.0};
    double final_energy{0.0};
    bool converged{false};
};

struct NewtonProblem {
    std::function<double(const Eigen::VectorXd& x)> compute_energy{};
    std::function<void(const Eigen::VectorXd& x, Eigen::VectorXd& gradient)> compute_gradient{};
    std::function<void(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets)>
        compute_hessian_triplets{};
};

namespace detail::newton_solver {

inline void validate_problem(const NewtonProblem& problem) {
    if (!problem.compute_energy || !problem.compute_gradient || !problem.compute_hessian_triplets) {
        throw std::invalid_argument("Newton solver requires energy, gradient, and hessian callbacks.");
    }
}

inline void validate_inputs(const IPCState& state,
                            const Eigen::VectorXd& x_hat,
                            const std::vector<bool>& free_dof_mask,
                            const NewtonProblem& problem,
                            const NewtonSolverParams& params) {
    validate_problem(problem);
    if (state.x.size() == 0) {
        throw std::invalid_argument("Newton solver requires a non-empty IPCState.");
    }
    if (x_hat.size() != state.x.size()) {
        throw std::invalid_argument("Newton solver requires x_hat to match state.x size.");
    }
    if (free_dof_mask.size() != static_cast<std::size_t>(state.x.size())) {
        throw std::invalid_argument("Newton solver requires free_dof_mask to match state.x size.");
    }
    if (params.max_iterations < 0) {
        throw std::invalid_argument("Newton solver requires max_iterations to be non-negative.");
    }
    if (!std::isfinite(params.gradient_tolerance) || params.gradient_tolerance < 0.0) {
        throw std::invalid_argument("Newton solver requires gradient_tolerance to be finite and non-negative.");
    }
    if (!std::isfinite(params.dx_tolerance) || params.dx_tolerance < 0.0) {
        throw std::invalid_argument("Newton solver requires dx_tolerance to be finite and non-negative.");
    }
    if (!std::isfinite(params.regularization) || params.regularization < 0.0) {
        throw std::invalid_argument("Newton solver requires regularization to be finite and non-negative.");
    }
}

inline std::vector<Eigen::Index> collect_free_dofs(const std::vector<bool>& free_dof_mask) {
    std::vector<Eigen::Index> free_dofs{};
    free_dofs.reserve(free_dof_mask.size());
    for (std::size_t i = 0; i < free_dof_mask.size(); ++i) {
        if (free_dof_mask[i]) {
            free_dofs.push_back(static_cast<Eigen::Index>(i));
        }
    }
    return free_dofs;
}

inline double masked_inf_norm(const Eigen::VectorXd& vector, const std::vector<bool>& free_dof_mask) {
    double norm = 0.0;
    for (Eigen::Index i = 0; i < vector.size(); ++i) {
        if (!free_dof_mask[static_cast<std::size_t>(i)]) {
            continue;
        }
        norm = std::max(norm, std::abs(vector[i]));
    }
    return norm;
}

inline void zero_fixed_gradient(Eigen::VectorXd& gradient, const std::vector<bool>& free_dof_mask) {
    for (Eigen::Index i = 0; i < gradient.size(); ++i) {
        if (!free_dof_mask[static_cast<std::size_t>(i)]) {
            gradient[i] = 0.0;
        }
    }
}

inline Eigen::SparseMatrix<double> build_reduced_hessian(
    Eigen::Index dof_count,
    const std::vector<Eigen::Triplet<double>>& full_triplets,
    const std::vector<Eigen::Index>& free_dofs,
    double regularization
) {
    std::vector<Eigen::Index> reduced_index(static_cast<std::size_t>(dof_count), Eigen::Index{-1});
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(free_dofs.size()); ++i) {
        reduced_index[static_cast<std::size_t>(free_dofs[static_cast<std::size_t>(i)])] = i;
    }

    std::vector<Eigen::Triplet<double>> reduced_triplets{};
    reduced_triplets.reserve(full_triplets.size() + static_cast<std::size_t>(free_dofs.size()));
    for (const auto& triplet : full_triplets) {
        if (triplet.row() < 0 || triplet.row() >= dof_count || triplet.col() < 0 || triplet.col() >= dof_count) {
            throw std::out_of_range("Newton solver received Hessian triplets outside the global DOF range.");
        }

        const Eigen::Index reduced_row = reduced_index[static_cast<std::size_t>(triplet.row())];
        const Eigen::Index reduced_col = reduced_index[static_cast<std::size_t>(triplet.col())];
        if (reduced_row < 0 || reduced_col < 0) {
            continue;
        }
        reduced_triplets.emplace_back(reduced_row, reduced_col, triplet.value());
    }

    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(free_dofs.size()); ++i) {
        reduced_triplets.emplace_back(i, i, regularization);
    }

    Eigen::SparseMatrix<double> reduced_hessian(
        static_cast<Eigen::Index>(free_dofs.size()),
        static_cast<Eigen::Index>(free_dofs.size())
    );
    reduced_hessian.setFromTriplets(
        reduced_triplets.begin(),
        reduced_triplets.end(),
        [](const double lhs, const double rhs) { return lhs + rhs; }
    );
    reduced_hessian.makeCompressed();
    return reduced_hessian;
}

inline bool solve_reduced_system(const Eigen::SparseMatrix<double>& reduced_hessian,
                                 const Eigen::VectorXd& reduced_gradient,
                                 Eigen::VectorXd& reduced_dx) {
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver{};
    solver.compute(reduced_hessian);
    if (solver.info() != Eigen::Success) {
        return false;
    }

    reduced_dx = solver.solve(-reduced_gradient);
    if (solver.info() != Eigen::Success || !reduced_dx.allFinite()) {
        return false;
    }
    return true;
}

inline Eigen::VectorXd expand_reduced_step(Eigen::Index dof_count,
                                           const std::vector<Eigen::Index>& free_dofs,
                                           const Eigen::VectorXd& reduced_dx) {
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(dof_count);
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(free_dofs.size()); ++i) {
        dx[free_dofs[static_cast<std::size_t>(i)]] = reduced_dx[i];
    }
    return dx;
}

inline double evaluate_gradient_norm(const NewtonProblem& problem,
                                     const Eigen::VectorXd& x,
                                     const std::vector<bool>& free_dof_mask) {
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(x.size());
    problem.compute_gradient(x, gradient);
    zero_fixed_gradient(gradient, free_dof_mask);
    return masked_inf_norm(gradient, free_dof_mask);
}

}  // namespace detail::newton_solver

inline NewtonSolverResult solve(IPCState& state,
                                const Eigen::VectorXd& x_hat,
                                const std::vector<bool>& free_dof_mask,
                                const NewtonProblem& problem,
                                const NewtonSolverParams& params = {}) {
    using namespace detail::newton_solver;

    validate_inputs(state, x_hat, free_dof_mask, problem, params);

    auto logger = utils::get_logger("system.physics.ipc.solver");
    if (params.use_psd_projection) {
        logger->warn("Newton solver PSD projection is not implemented yet; continuing without it.");
    }

    NewtonSolverResult result{};
    result.final_energy = problem.compute_energy(state.x);
    if (!std::isfinite(result.final_energy)) {
        throw std::runtime_error("Newton solver received a non-finite initial energy.");
    }

    const std::vector<Eigen::Index> free_dofs = collect_free_dofs(free_dof_mask);
    if (free_dofs.empty()) {
        result.converged = true;
        return result;
    }

    for (int iteration = 0; iteration < params.max_iterations; ++iteration) {
        result.iterations = iteration;

        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(state.x.size());
        problem.compute_gradient(state.x, gradient);
        if (!gradient.allFinite()) {
            throw std::runtime_error("Newton solver received a non-finite gradient.");
        }
        zero_fixed_gradient(gradient, free_dof_mask);
        const double gradient_norm = masked_inf_norm(gradient, free_dof_mask);
        result.final_gradient_norm = gradient_norm;

        const double current_energy = problem.compute_energy(state.x);
        if (!std::isfinite(current_energy)) {
            throw std::runtime_error("Newton solver received a non-finite energy during iteration.");
        }
        result.final_energy = current_energy;

        if (gradient_norm <= params.gradient_tolerance) {
            result.converged = true;
            return result;
        }

        std::vector<Eigen::Triplet<double>> hessian_triplets{};
        problem.compute_hessian_triplets(state.x, hessian_triplets);

        Eigen::VectorXd reduced_gradient(static_cast<Eigen::Index>(free_dofs.size()));
        for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(free_dofs.size()); ++i) {
            reduced_gradient[i] = gradient[free_dofs[static_cast<std::size_t>(i)]];
        }

        Eigen::VectorXd reduced_dx{};
        bool solved = false;
        double regularization = std::max(params.regularization, 0.0);
        for (int retry = 0; retry < 6; ++retry) {
            const Eigen::SparseMatrix<double> reduced_hessian =
                build_reduced_hessian(state.x.size(), hessian_triplets, free_dofs, regularization);
            solved = solve_reduced_system(reduced_hessian, reduced_gradient, reduced_dx);
            if (solved) {
                break;
            }
            regularization = (regularization > 0.0) ? regularization * 10.0 : 1e-8;
        }

        if (!solved) {
            logger->warn("Newton solver failed to factorize the reduced Hessian.");
            return result;
        }

        Eigen::VectorXd dx = expand_reduced_step(state.x.size(), free_dofs, reduced_dx);
        if (!dx.allFinite()) {
            throw std::runtime_error("Newton solver produced a non-finite Newton step.");
        }
        const double dx_norm = masked_inf_norm(dx, free_dof_mask);
        if (dx_norm <= params.dx_tolerance) {
            result.converged = true;
            return result;
        }

        const double directional_derivative = gradient.dot(dx);
        const auto line_search_result = backtracking_line_search(
            [&state, &problem, &dx](double alpha) {
                const Eigen::VectorXd trial_x = state.x + alpha * dx;
                return problem.compute_energy(trial_x);
            },
            current_energy,
            directional_derivative
        );

        logger->info(
            "Newton iter={} energy={:.6e} |g|_inf={:.6e} |dx|_inf={:.6e} alpha={:.6e} success={}",
            iteration,
            current_energy,
            gradient_norm,
            dx_norm,
            line_search_result.alpha,
            line_search_result.success
        );

        if (!line_search_result.success) {
            logger->warn("Newton solver line search failed to find a decreasing step.");
            return result;
        }

        state.x += line_search_result.alpha * dx;
        if (!state.x.allFinite()) {
            throw std::runtime_error("Newton solver updated state.x to a non-finite value.");
        }

        const double post_gradient_norm = evaluate_gradient_norm(problem, state.x, free_dof_mask);
        result.iterations = iteration + 1;
        result.final_energy = line_search_result.energy;
        result.final_gradient_norm = post_gradient_norm;
        if (post_gradient_norm <= params.gradient_tolerance ||
            line_search_result.alpha * dx_norm <= params.dx_tolerance) {
            result.converged = true;
            return result;
        }
    }

    result.final_energy = problem.compute_energy(state.x);
    result.final_gradient_norm = evaluate_gradient_norm(problem, state.x, free_dof_mask);
    return result;
}

}  // namespace rtr::system::physics::ipc
