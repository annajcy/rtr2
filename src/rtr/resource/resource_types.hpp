#pragma once

#include <cstdint>

namespace rtr::resource {

struct MeshHandle {
    std::uint64_t value{0};

    bool is_valid() const {
        return value != 0;
    }

    bool operator==(const MeshHandle& other) const {
        return value == other.value;
    }
};

struct TextureHandle {
    std::uint64_t value{0};

    bool is_valid() const {
        return value != 0;
    }

    bool operator==(const TextureHandle& other) const {
        return value == other.value;
    }
};

} // namespace rtr::resource

namespace std {

template <>
struct hash<rtr::resource::MeshHandle> {
    size_t operator()(const rtr::resource::MeshHandle& handle) const noexcept {
        return hash<std::uint64_t>{}(handle.value);
    }
};

template <>
struct hash<rtr::resource::TextureHandle> {
    size_t operator()(const rtr::resource::TextureHandle& handle) const noexcept {
        return hash<std::uint64_t>{}(handle.value);
    }
};

} // namespace std
