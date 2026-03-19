#pragma once

#include <cstdint>

namespace rtr::system::physics {

using ClothID = std::int32_t;
using VertexID = std::int32_t;
using EdgeID = std::int32_t;
using TriangleID = std::int32_t;

inline constexpr ClothID kInvalidClothId = -1;
inline constexpr VertexID kInvalidVertexID = -1;
inline constexpr EdgeID kInvalidEdgeID = -1;
inline constexpr TriangleID kInvalidTriangleID = -1;

}  // namespace rtr::system::physics
