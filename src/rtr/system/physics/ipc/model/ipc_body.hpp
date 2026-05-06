#pragma once

#include <cstddef>

namespace rtr::system::physics::ipc {

enum class IPCBodyType {
    Tet,
    Shell,
    Obstacle,
};

struct IPCBodyInfo {
    IPCBodyType type{};          // Concrete IPC body category.
    std::size_t dof_offset{0};   // First global DOF index in IPCState.
    std::size_t vertex_count{0}; // Number of vertices owned by this body.
    bool enabled{true};          // Skip body during assembly/solve when false.
};

}  // namespace rtr::system::physics::ipc
