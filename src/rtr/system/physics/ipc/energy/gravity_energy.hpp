#pragma once

#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/SparseCore>

namespace rtr::system::physics::ipc {

struct GravityEnergy {
    struct Input {
        const Eigen::VectorXd& x;
        const Eigen::VectorXd& mass_diag;
        const Eigen::Vector3d& gravity;
    };

private:
    static void validate_inputs(const Input& input) {
        if (input.x.size() != input.mass_diag.size()) {
            throw std::invalid_argument("GravityEnergy requires x and mass_diag to have the same size.");
        }
        if ((input.x.size() % 3) != 0) {
            throw std::invalid_argument("GravityEnergy requires vectors to have size 3N.");
        }
    }

public:

    static double compute_energy(const Input& input) {
        validate_inputs(input);

        double energy = 0.0;
        for (Eigen::Index i = 0; i < input.x.size(); i += 3) {
            energy -= input.mass_diag[i + 0] * input.gravity.x() * input.x[i + 0];
            energy -= input.mass_diag[i + 1] * input.gravity.y() * input.x[i + 1];
            energy -= input.mass_diag[i + 2] * input.gravity.z() * input.x[i + 2];
        }
        return energy;
    }

    static void compute_gradient(const Input& input, Eigen::VectorXd& gradient) {
        validate_inputs(input);
        if (gradient.size() != input.mass_diag.size()) {
            throw std::invalid_argument("GravityEnergy gradient must match mass_diag size.");
        }

        for (Eigen::Index i = 0; i < gradient.size(); i += 3) {
            gradient[i + 0] -= input.mass_diag[i + 0] * input.gravity.x();
            gradient[i + 1] -= input.mass_diag[i + 1] * input.gravity.y();
            gradient[i + 2] -= input.mass_diag[i + 2] * input.gravity.z();
        }
    }

    static void compute_hessian_triplets(const Input& input,
                                         std::vector<Eigen::Triplet<double>>& triplets) {
        validate_inputs(input);
        (void)triplets;
    }
};

}  // namespace rtr::system::physics::ipc
