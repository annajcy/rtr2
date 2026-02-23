#pragma once

#include <cstring>
#include <functional>
#include <utility>

#include "rtr/rhi/common.hpp"
#include "device.hpp"

#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_raii.hpp"


namespace rtr::rhi {

class Buffer {
private:
    std::reference_wrapper<Device> m_device;
    vk::raii::Buffer m_buffer{nullptr};
    vk::raii::DeviceMemory m_buffer_memory{nullptr};
    vk::DeviceSize m_size{0};
    vk::BufferUsageFlags m_usage{};
    vk::MemoryPropertyFlags m_properties{};
    void* m_mapped_data{nullptr};
    bool m_is_mapped{ false };

public:  
    static Buffer create_host_visible_buffer(
        Device& device,
        vk::DeviceSize size,
        vk::BufferUsageFlags usage = {}
    ) {
        return Buffer(
            device,
            size,
            usage,
            vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent
        );
    }

    static Buffer create_device_local_buffer(
        Device& device,
        vk::DeviceSize size,
        vk::BufferUsageFlags usage = {}
    ) {
        return Buffer(
            device,
            size,
            usage,
            vk::MemoryPropertyFlagBits::eDeviceLocal
        );
    }

public:
    Buffer(
        Device& device,
        vk::DeviceSize size,
        vk::BufferUsageFlags usage = {},
        vk::MemoryPropertyFlags properties = {}
    ) : m_device(device), m_size(size), m_usage(usage), m_properties(properties) {
        auto buffer_with_memory_opt = make_buffer_with_memory(
            device.device(),
            device.physical_device(),
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

    // 移动构造
    Buffer(Buffer&& other) noexcept 
        : m_device(other.m_device.get()),
        m_buffer(std::move(other.m_buffer)),
        m_buffer_memory(std::move(other.m_buffer_memory)),
        m_size(other.m_size),
        m_usage(other.m_usage),
        m_properties(other.m_properties),
        m_mapped_data(other.m_mapped_data),
        m_is_mapped(other.m_is_mapped) // 接管状态
    {
        // 重置源对象状态
        other.m_mapped_data = nullptr;
        other.m_is_mapped = false;
        other.m_size = 0;
    }

    // 移动赋值类似，需要先处理自己的析构逻辑
    Buffer& operator=(Buffer&& other) noexcept {
        if (this != &other) {
            // 释放当前资源
            if (m_is_mapped) {
                unmap();
            }

            m_device = other.m_device;
            m_buffer = std::move(other.m_buffer);
            m_buffer_memory = std::move(other.m_buffer_memory);
            m_size = other.m_size;
            m_usage = other.m_usage;
            m_properties = other.m_properties;
            m_mapped_data = other.m_mapped_data;
            m_is_mapped = other.m_is_mapped;

            // 重置源对象状态
            other.m_mapped_data = nullptr;
            other.m_is_mapped = false;
            other.m_size = 0;
        }
        return *this;
    }

    Buffer(const Buffer&) = delete;
    Buffer& operator=(const Buffer&) = delete;

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
    const Device& device() const { return m_device.get(); }

    bool is_mapped() const { return m_is_mapped; }

    void map(
        vk::DeviceSize size = vk::WholeSize,
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


} // namespace rtr::rhi
