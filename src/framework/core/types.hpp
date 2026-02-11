#pragma once

#include <cstdint>

namespace rtr::framework::core {

using SceneId = std::uint64_t;
using GameObjectId = std::uint64_t;
using ComponentId = std::uint64_t;

constexpr SceneId kInvalidSceneId = 0;
constexpr GameObjectId kInvalidGameObjectId = 0;
constexpr ComponentId kInvalidComponentId = 0;

} // namespace rtr::framework::core
