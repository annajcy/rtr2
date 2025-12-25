#pragma once

#include <optional>
#include <tuple>
#include <utility>

#include "device.hpp"
#include "command.hpp"

#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_core.h"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vulkan_structs.hpp"

namespace rtr::core {

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


class Buffer {
private:
    Device* m_device{};
    vk::raii::Buffer m_buffer{nullptr};
    vk::raii::DeviceMemory m_buffer_memory{nullptr};
    vk::DeviceSize m_size{0};
    vk::BufferUsageFlags m_usage{};
    vk::MemoryPropertyFlags m_properties{};
    void* m_mapped_data{nullptr};
    bool m_is_mapped{ false };

public:  
    static Buffer create_host_visible_buffer(
        Device* device,
        vk::DeviceSize size,
        vk::BufferUsageFlags usage
    ) {
        return Buffer(
            device,
            size,
            usage,
            vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent
        );
    }

    static Buffer create_device_local_buffer(
        Device* device,
        vk::DeviceSize size,
        vk::BufferUsageFlags usage
    ) {
        return Buffer(
            device,
            size,
            usage,
            vk::MemoryPropertyFlagBits::eDeviceLocal
        );
    }

    static void copy_buffer(
        Device* device,
        vk::Buffer src,
        vk::Buffer dst,
        vk::DeviceSize size
    ) {
        CommandPool command_pool(device, vk::CommandPoolCreateFlagBits::eTransient);
        auto cmd = command_pool.create_command_buffer();
        
        cmd.record_and_submit([&](CommandBuffer& cmd) {
            vk::BufferCopy buffer_copy{};
            buffer_copy.srcOffset = 0;
            buffer_copy.dstOffset = 0;
            buffer_copy.size = size;
            cmd.get().copyBuffer(src, dst, buffer_copy);
        });
        
        device->queue().waitIdle();
    }

    static Buffer create_device_local_with_data(
        Device* device,
        const void* data,
        vk::DeviceSize size,
        vk::BufferUsageFlags usage
    ) {
        auto buffer = create_device_local_buffer(
            device, 
            size, 
            usage | vk::BufferUsageFlagBits::eTransferDst
        );
        
        auto staging_buffer = create_host_visible_buffer(
            device, 
            size, 
            vk::BufferUsageFlagBits::eTransferSrc
        );
        
        staging_buffer.map();
        std::memcpy(staging_buffer.mapped_data(), data, size);
        staging_buffer.unmap();
        
        copy_buffer(device, *staging_buffer.buffer(), *buffer.buffer(), size);
        
        return buffer;
    }

public:
    Buffer(
        Device* device,
        vk::DeviceSize size,
        vk::BufferUsageFlags usage,
        vk::MemoryPropertyFlags properties
    ) : m_device(device), m_size(size), m_usage(usage), m_properties(properties) {
        auto buffer_with_memory_opt = make_buffer_with_memory(
            device->device(),
            device->physical_device(),
            size,
            usage,
            properties
        );

        if (!buffer_with_memory_opt.has_value()) {
            throw std::runtime_error("Failed to create buffer.");
        }

        auto [buffer, buffer_memory] = std::move(buffer_with_memory_opt.value());
        m_buffer = std::move(buffer);
        m_buffer_memory = std::move(buffer_memory);
    }

    Buffer(const Buffer&) = delete;
    Buffer& operator=(const Buffer&) = delete;
    Buffer(Buffer&&) = default;
    Buffer& operator=(Buffer&&) = default;

    ~Buffer() {
        if (m_is_mapped) {
            unmap();
        }
    }

    const vk::raii::Buffer& buffer() const { return m_buffer; }
    const vk::raii::DeviceMemory& buffer_memory() const { return m_buffer_memory; }
    vk::DeviceSize size() const { return m_size; }
    vk::BufferUsageFlags usage() const { return m_usage; }
    vk::MemoryPropertyFlags properties() const { return m_properties; }
    const Device* device() const { return m_device; }

    bool is_mapped() const { return m_is_mapped; }

    void map(
        vk::DeviceSize size = VK_WHOLE_SIZE,
        vk::DeviceSize offset = 0
    ) {
        if (m_is_mapped) {
            throw std::runtime_error("Buffer is already mapped.");
        }
        m_mapped_data = m_buffer_memory.mapMemory(
            offset,
            size,
            vk::MemoryMapFlags{}
        );
        m_is_mapped = true;
    }

    void unmap() {
        if (!m_is_mapped) {
            throw std::runtime_error("Buffer is not mapped.");
        }
        m_buffer_memory.unmapMemory();
        m_mapped_data = nullptr;
        m_is_mapped = false;
    }

    void* mapped_data() const {
        if (!m_is_mapped) {
            throw std::runtime_error("Buffer is not mapped.");
        }
        return m_mapped_data;
    }
};


}