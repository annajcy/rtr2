#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

namespace rtr::resource {

struct MeshResourceKind;
struct TextureResourceKind;

template <class KindTag>
struct ResourceHandle {
    std::uint64_t value{0};

    bool is_valid() const {
        return value != 0;
    }

    bool operator==(const ResourceHandle& other) const {
        return value == other.value;
    }
};

using MeshHandle = ResourceHandle<MeshResourceKind>;
using TextureHandle = ResourceHandle<TextureResourceKind>;

} // namespace rtr::resource

namespace std {

template <class KindTag>
struct hash<rtr::resource::ResourceHandle<KindTag>> {
    size_t operator()(const rtr::resource::ResourceHandle<KindTag>& handle) const noexcept {
        return hash<std::uint64_t>{}(handle.value);
    }
};

} // namespace std
