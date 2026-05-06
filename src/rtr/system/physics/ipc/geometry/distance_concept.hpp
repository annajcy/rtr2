#pragma once

#include <concepts>

namespace rtr::system::physics::ipc {

template <typename T>
concept Distance = requires(const typename T::Input& input,
                            const typename T::Result& result) {
    typename T::Input;
    typename T::Result;
    { T::compute(input) } -> std::same_as<typename T::Result>;
    { result.distance_squared } -> std::convertible_to<double>;
    result.gradient;
    result.hessian;
};

}  // namespace rtr::system::physics::ipc
