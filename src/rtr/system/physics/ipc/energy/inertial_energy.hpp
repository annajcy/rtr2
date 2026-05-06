#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

namespace rtr::system::physics::ipc {

struct InertialEnergy {
    struct Input {
        const Eigen::VectorXd& x;
        const Eigen::VectorXd& x_hat;
        const Eigen::VectorXd& mass_diag;
        double dt;
    };

private:
    static void validate_inputs(const Input& input) {
        if (!std::isfinite(input.dt) || input.dt <= 0.0) {
            throw std::invalid_argument("InertialEnergy requires dt to be finite and positive.");
        }
        if (input.x.size() != input.x_hat.size() || input.x.size() != input.mass_diag.size()) {
            throw std::invalid_argument("InertialEnergy requires x, x_hat, and mass_diag to have the same size.");
        }
        if ((input.x.size() % 3) != 0) {
            throw std::invalid_argument("InertialEnergy requires vectors to have size 3N.");
        }
    }

public:

    static double compute_energy(const Input& input) {
        validate_inputs(input);
        const double inv_dt_sq = 1.0 / (input.dt * input.dt);
        const Eigen::VectorXd delta = input.x - input.x_hat;
        return 0.5 * inv_dt_sq * input.mass_diag.dot(delta.cwiseProduct(delta));
    }

    static void compute_gradient(const Input& input, Eigen::VectorXd& gradient) {
        validate_inputs(input);
        if (gradient.size() != input.x.size()) {
            throw std::invalid_argument("InertialEnergy gradient must match x size.");
        }

        const double inv_dt_sq = 1.0 / (input.dt * input.dt);
        gradient.array() += inv_dt_sq * input.mass_diag.array() * (input.x - input.x_hat).array();
    }

    static void compute_hessian_triplets(const Input& input,
                                         std::vector<Eigen::Triplet<double>>& triplets) {
        validate_inputs(input);

        const double inv_dt_sq = 1.0 / (input.dt * input.dt);
        triplets.reserve(triplets.size() + static_cast<std::size_t>(input.mass_diag.size()));
        for (Eigen::Index i = 0; i < input.mass_diag.size(); ++i) {
            triplets.emplace_back(i, i, inv_dt_sq * input.mass_diag[i]);
        }
    }
};

}  // namespace rtr::system::physics::ipc
