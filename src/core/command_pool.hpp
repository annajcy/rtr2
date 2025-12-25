#pragma once

#include "device.hpp"
#include "vulkan/vulkan_raii.hpp"
#include <vector>
#include <functional>

namespace rtr::core {

class CommandPool {
private:
    Device* m_device;
    vk::raii::CommandPool m_command_pool{nullptr};

public:
    CommandPool(Device* device, vk::CommandPoolCreateFlags flags)
        : m_device(device) {
        vk::CommandPoolCreateInfo create_info{};
        create_info.flags = flags;
        create_info.queueFamilyIndex = device->queue_family_index();
        m_command_pool = vk::raii::CommandPool(device->device(), create_info);
    }

    vk::raii::CommandBuffer allocate_command_buffer(vk::CommandBufferLevel level = vk::CommandBufferLevel::ePrimary) {
        vk::CommandBufferAllocateInfo alloc_info{};
        alloc_info.commandPool = *m_command_pool;
        alloc_info.level = level;
        alloc_info.commandBufferCount = 1;

        auto buffers = m_device->device().allocateCommandBuffers(alloc_info);
        return std::move(buffers.front());
    }

    std::vector<vk::raii::CommandBuffer> allocate_command_buffers(uint32_t count, vk::CommandBufferLevel level = vk::CommandBufferLevel::ePrimary) {
        vk::CommandBufferAllocateInfo alloc_info{};
        alloc_info.commandPool = *m_command_pool;
        alloc_info.level = level;
        alloc_info.commandBufferCount = count;

        auto buffers = m_device->device().allocateCommandBuffers(alloc_info);
        std::vector<vk::raii::CommandBuffer> result;
        result.reserve(buffers.size());
        for (auto& buffer : buffers) {
            result.emplace_back(std::move(buffer));
        }
        return result;
    }

    template<typename Func>
    void execute_single_time_commands(Func&& recorder) {
        auto command_buffer = allocate_command_buffer();
        
        vk::CommandBufferBeginInfo begin_info{};
        begin_info.flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit;
        command_buffer.begin(begin_info);

        recorder(*command_buffer);

        command_buffer.end();

        vk::SubmitInfo submit_info{};
        submit_info.commandBufferCount = 1;
        submit_info.pCommandBuffers = &*command_buffer;

        m_device->queue().submit(submit_info, nullptr);
        m_device->queue().waitIdle();
    }

    const vk::raii::CommandPool& command_pool() const { return m_command_pool; }
    const Device* device() const { return m_device; }

};

}
