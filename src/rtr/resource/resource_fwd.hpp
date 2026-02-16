#pragma once

namespace rtr::resource {

struct MeshResourceKind;
struct TextureResourceKind;

template <class Kind0, class Kind1, class... Kinds>
class ResourceManagerT;

using ResourceManager = ResourceManagerT<MeshResourceKind, TextureResourceKind>;

} // namespace rtr::resource
