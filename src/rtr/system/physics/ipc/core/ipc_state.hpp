#pragma once

#include <cassert>
#include <cstddef>

#include <Eigen/Core>

namespace rtr::system::physics::ipc {

struct IPCState {
    Eigen::VectorXd x{};
    Eigen::VectorXd x_prev{};
    Eigen::VectorXd v{};
    Eigen::VectorXd mass_diag{};

    void resize(std::size_t vertex_count) {
        const auto dof_count = static_cast<Eigen::Index>(3 * vertex_count);
        x.setZero(dof_count);
        x_prev.setZero(dof_count);
        v.setZero(dof_count);
        mass_diag.setZero(dof_count);
    }

    std::size_t vertex_count() const {
        assert((x.size() % 3) == 0);
        return static_cast<std::size_t>(x.size() / 3);
    }

    std::size_t dof_count() const { return static_cast<std::size_t>(x.size()); }

    Eigen::Ref<Eigen::Vector3d> position(std::size_t vertex_index) {
        assert(vertex_index < vertex_count());
        return x.segment<3>(static_cast<Eigen::Index>(3 * vertex_index));
    }

    Eigen::Ref<const Eigen::Vector3d> position(std::size_t vertex_index) const {
        assert(vertex_index < vertex_count());
        return x.segment<3>(static_cast<Eigen::Index>(3 * vertex_index));
    }
};

}  // namespace rtr::system::physics::ipc
