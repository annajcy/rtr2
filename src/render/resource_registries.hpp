#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "rhi/buffer.hpp"
#include "rhi/texture.hpp"
#include "vulkan/vulkan_raii.hpp"

namespace rtr::render {

template <class T>
class TypedResourceRegistry {
public:
    using ResourceMap = std::unordered_map<std::string, T*>;

private:
    std::vector<ResourceMap> m_per_frame;
    ResourceMap m_global;

public:
    explicit TypedResourceRegistry(uint32_t frames_in_flight = 0)
        : m_per_frame(frames_in_flight) {}

    void clear_frame(uint32_t frame_index) {
        frame_resources(frame_index).clear();
    }

    void clear_global() {
        m_global.clear();
    }

    void clear_all_resources() {
        for (auto& per_frame_map : m_per_frame) {
            per_frame_map.clear();
        }
        m_global.clear();
    }

    void set_frame_resource(uint32_t frame_index, const std::string& name, T& resource) {
        auto& per_frame_map = frame_resources(frame_index);
        if (m_global.find(name) != m_global.end()) {
            throw std::runtime_error(
                "Resource name conflict: '" + name +
                "' already exists in global scope and cannot be set in per-frame scope (frame index: " +
                std::to_string(frame_index) + ")."
            );
        }
        per_frame_map[name] = &resource;
    }

    void set_global_resource(const std::string& name, T& resource) {
        for (size_t frame_index = 0; frame_index < m_per_frame.size(); ++frame_index) {
            if (m_per_frame[frame_index].find(name) != m_per_frame[frame_index].end()) {
                throw std::runtime_error(
                    "Resource name conflict: '" + name +
                    "' already exists in per-frame scope (frame index: " +
                    std::to_string(frame_index) + ") and cannot be set in global scope."
                );
            }
        }
        m_global[name] = &resource;
    }

    T& get_perframe_resource(uint32_t frame_index, const std::string& name) const {
        const auto& per_frame_map = frame_resources(frame_index);
        auto per_frame_it = per_frame_map.find(name);
        if (per_frame_it != per_frame_map.end()) {
            return *per_frame_it->second;
        }
        throw std::runtime_error(
            "Per-frame resource not found: '" + name + "' (frame index: " + std::to_string(frame_index) + ")."
        );
    }

    T& get_global_resource(const std::string& name) const {
        auto global_it = m_global.find(name);
        if (global_it != m_global.end()) {
            return *global_it->second;
        }
        throw std::runtime_error("Global resource not found: '" + name + "'.");
    }

    bool has_perframe_resource(uint32_t frame_index, const std::string& name) const {
        const auto& per_frame_map = frame_resources(frame_index);
        return per_frame_map.find(name) != per_frame_map.end();
    }

    bool has_global_resource(const std::string& name) const {
        return m_global.find(name) != m_global.end();
    }

private:
    ResourceMap& frame_resources(uint32_t frame_index) {
        validate_frame_index(frame_index);
        return m_per_frame[frame_index];
    }

    const ResourceMap& frame_resources(uint32_t frame_index) const {
        validate_frame_index(frame_index);
        return m_per_frame[frame_index];
    }

    void validate_frame_index(uint32_t frame_index) const {
        if (frame_index >= m_per_frame.size()) {
            throw std::runtime_error(
                "Invalid frame index: " + std::to_string(frame_index) +
                " (frames in flight: " + std::to_string(m_per_frame.size()) + ")."
            );
        }
    }
};

using BufferRegistry = TypedResourceRegistry<rhi::Buffer>;
using DescriptorSetRegistry = TypedResourceRegistry<vk::raii::DescriptorSet>;
using ImageRegistry = TypedResourceRegistry<rhi::Image>;
inline constexpr const char* kBuiltinDepthImageResourceName = "__builtin.depth_image";

template <class... Ts>
class ResourceRegistryAggregate {
private:
    std::tuple<TypedResourceRegistry<Ts>...> m_registries;

    static std::tuple<TypedResourceRegistry<Ts>...> make_registries(uint32_t frames_count) {
        return std::tuple<TypedResourceRegistry<Ts>...>{TypedResourceRegistry<Ts>(frames_count)...};
    }

public:
    explicit ResourceRegistryAggregate(uint32_t frames_count = 0)
        : m_registries(make_registries(frames_count)) {}

    void clear_frame(uint32_t frame_index) {
        (registry<Ts>().clear_frame(frame_index), ...);
    }

    void clear_global() {
        (registry<Ts>().clear_global(), ...);
    }

    void clear_all() {
        (registry<Ts>().clear_all_resources(), ...);
    }

    template <class T>
    TypedResourceRegistry<T>& registry() {
        return std::get<TypedResourceRegistry<T>>(m_registries);
    }

    template <class T>
    const TypedResourceRegistry<T>& registry() const {
        return std::get<TypedResourceRegistry<T>>(m_registries);
    }
};

class ResourceRegistries final : public ResourceRegistryAggregate<rhi::Buffer, vk::raii::DescriptorSet, rhi::Image> {
public:
    using ResourceRegistryAggregate::ResourceRegistryAggregate;
};

} // namespace rtr::render
