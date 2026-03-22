#pragma once

#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace rtr::system::physics::ipc {

struct LineSearchResult {
    double alpha{0.0};
    double energy{0.0};
    bool success{false};
};

using EnergyFunction = std::function<double(double alpha)>;

inline LineSearchResult backtracking_line_search(
    EnergyFunction energy_fn,
    double current_energy,
    double directional_derivative,
    double alpha_init = 1.0,
    double shrink = 0.5,
    double armijo_c = 1e-4,
    int max_iterations = 20
) {
    if (!energy_fn) {
        throw std::invalid_argument("backtracking_line_search requires a valid energy function.");
    }
    if (!std::isfinite(current_energy)) {
        throw std::invalid_argument("backtracking_line_search requires current_energy to be finite.");
    }
    if (!std::isfinite(directional_derivative)) {
        throw std::invalid_argument("backtracking_line_search requires directional_derivative to be finite.");
    }
    if (!std::isfinite(alpha_init) || alpha_init <= 0.0) {
        throw std::invalid_argument("backtracking_line_search requires alpha_init to be finite and positive.");
    }
    if (!std::isfinite(shrink) || shrink <= 0.0 || shrink >= 1.0) {
        throw std::invalid_argument("backtracking_line_search requires shrink to be in (0, 1).");
    }
    if (!std::isfinite(armijo_c) || armijo_c <= 0.0 || armijo_c >= 1.0) {
        throw std::invalid_argument("backtracking_line_search requires armijo_c to be in (0, 1).");
    }
    if (max_iterations < 0) {
        throw std::invalid_argument("backtracking_line_search requires max_iterations to be non-negative.");
    }

    if (directional_derivative >= 0.0) {
        return LineSearchResult{
            .alpha = 0.0,
            .energy = current_energy,
            .success = false,
        };
    }

    double alpha = alpha_init;
    double trial_energy = std::numeric_limits<double>::quiet_NaN();
    for (int iteration = 0; iteration <= max_iterations; ++iteration) {
        trial_energy = energy_fn(alpha);
        if (trial_energy <= current_energy + armijo_c * alpha * directional_derivative) {
            return LineSearchResult{
                .alpha = alpha,
                .energy = trial_energy,
                .success = true,
            };
        }
        alpha *= shrink;
    }

    return LineSearchResult{
        .alpha = alpha,
        .energy = trial_energy,
        .success = false,
    };
}

}  // namespace rtr::system::physics::ipc
