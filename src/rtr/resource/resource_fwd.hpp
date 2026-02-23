#pragma once

#include "rtr/rhi/frame_constants.hpp"

namespace rtr::resource {

struct MeshResourceKind;
struct TextureResourceKind;

template <std::uint32_t FramesInFlight, class Kind0, class Kind1, class... Kinds>
class ResourceManagerT;

using ResourceManager = ResourceManagerT<rhi::kFramesInFlight, MeshResourceKind, TextureResourceKind>;

} // namespace rtr::resource
