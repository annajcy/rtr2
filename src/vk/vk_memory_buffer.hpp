#pragma once

#include <optional>
#include <tuple>
#include <utility>

#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vulkan_structs.hpp"

namespace rtr {

inline std::optional<uint32_t> find_memory_type(
    const vk::PhysicalDeviceMemoryProperties& mem_properties,
    uint32_t type_filter,
    vk::MemoryPropertyFlags properties
) {
    for (uint32_t i = 0; i < mem_properties.memoryTypeCount; i++) {
        if ((type_filter & (1 << i)) && 
            (mem_properties.memoryTypes[i].propertyFlags & properties) == properties) {
            return i;
        }
    }
    return std::nullopt;
}

inline std::optional<std::pair<vk::raii::Buffer, vk::raii::DeviceMemory>> make_buffer_with_memory(
    const vk::raii::Device& device,
    const vk::raii::PhysicalDevice& physical_device,
    vk::DeviceSize size,
    vk::BufferUsageFlags usage,
    vk::MemoryPropertyFlags properties
) {
    vk::BufferCreateInfo buffer_create_info{};
    buffer_create_info.size = size;
    buffer_create_info.usage = usage;
    buffer_create_info.sharingMode = vk::SharingMode::eExclusive;

    vk::raii::Buffer buffer{ device, buffer_create_info };

    vk::MemoryRequirements mem_requirements = buffer.getMemoryRequirements();

    vk::PhysicalDeviceMemoryProperties mem_properties = physical_device.getMemoryProperties();
    if (auto memory_type_index_opt = find_memory_type(
        mem_properties,
        mem_requirements.memoryTypeBits,
        properties
    )) {
        auto memory_type_index = memory_type_index_opt.value();

        vk::MemoryAllocateInfo alloc_info{};
        alloc_info.allocationSize = mem_requirements.size;
        alloc_info.memoryTypeIndex = memory_type_index;

        vk::raii::DeviceMemory buffer_memory{ device, alloc_info };

        buffer.bindMemory(
            *buffer_memory,
            vk::DeviceSize(0)
        );

        return std::make_pair(std::move(buffer), std::move(buffer_memory));
    } else {
        return std::nullopt;
    }
}

inline std::optional<std::tuple<vk::raii::Buffer, vk::raii::DeviceMemory, void*>> make_mapped_buffer_with_memory(
    const vk::raii::Device& device,
    const vk::raii::PhysicalDevice& physical_device,
    vk::DeviceSize size,
    vk::BufferUsageFlags usage,
    vk::MemoryPropertyFlags properties
) {
    if (auto buffer_with_memory_opt = make_buffer_with_memory(
        device,
        physical_device,
        size,
        usage,
        properties | vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent)
    ) {
        auto [buffer, buffer_memory] = std::move(buffer_with_memory_opt.value());
        void* mapped_ptr = buffer_memory.mapMemory(0, size);
        return std::make_tuple(
            std::move(buffer),
            std::move(buffer_memory),
            mapped_ptr
        );
    } else {
        return std::nullopt;
    }
}

template<typename Mapper>
inline void map_memory(
    const vk::raii::DeviceMemory& memory,
    vk::DeviceSize size,
    vk::DeviceSize offset,
    Mapper&& mapper,
    vk::MemoryMapFlags flags = vk::MemoryMapFlags{}
) {
    void* data = memory.mapMemory(
        offset,
        size,
        flags
    );
    mapper(data);
    memory.unmapMemory();
}

} // namespace rtr