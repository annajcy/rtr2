#pragma once

#include "device.hpp"
#include "vulkan/vulkan_raii.hpp"
#include <vector>

namespace rtr::core {

class CommandBuffer {
private:
    Device* m_device;
    vk::raii::CommandBuffer m_command_buffer;
    bool m_is_recording = false;

public:
    CommandBuffer(Device* device, vk::raii::CommandBuffer&& command_buffer)
        : m_device(device)
        , m_command_buffer(std::move(command_buffer)) {
    }

    // Begin recording
    void begin(vk::CommandBufferUsageFlags usage_flags = {}) {
        if (m_is_recording) {
            throw std::runtime_error("CommandBuffer is already recording");
        }

        vk::CommandBufferBeginInfo begin_info{};
        begin_info.flags = usage_flags;
        m_command_buffer.begin(begin_info);
        m_is_recording = true;
    }

    // End recording
    void end() {
        if (!m_is_recording) {
            throw std::runtime_error("CommandBuffer is not recording");
        }

        m_command_buffer.end();
        m_is_recording = false;
    }

    // Reset command buffer
    void reset(vk::CommandBufferResetFlags flags = {}) {
        m_command_buffer.reset(flags);
        m_is_recording = false;
    }

    // Record commands using lambda
    template<typename Func>
    void record(Func&& recorder, vk::CommandBufferUsageFlags usage_flags = {}) {
        begin(usage_flags);
        recorder(*this);
        end();
    }

    // Submit to queue with optional synchronization
    struct SubmitInfo {
        std::vector<vk::Semaphore> wait_semaphores;
        std::vector<vk::PipelineStageFlags> wait_stages;
        std::vector<vk::Semaphore> signal_semaphores;
        std::optional<vk::Fence> fence;
    };

    void submit(const SubmitInfo& submit_info = {}) {
        if (m_is_recording) {
            throw std::runtime_error("Cannot submit while recording");
        }

        vk::SubmitInfo vk_submit_info{};
        
        if (!submit_info.wait_semaphores.empty()) {
            vk_submit_info.waitSemaphoreCount = static_cast<uint32_t>(submit_info.wait_semaphores.size());
            vk_submit_info.pWaitSemaphores = submit_info.wait_semaphores.data();
            vk_submit_info.pWaitDstStageMask = submit_info.wait_stages.data();
        }

        vk::CommandBuffer cmd_buf = *m_command_buffer;
        vk_submit_info.commandBufferCount = 1;
        vk_submit_info.pCommandBuffers = &cmd_buf;

        if (!submit_info.signal_semaphores.empty()) {
            vk_submit_info.signalSemaphoreCount = static_cast<uint32_t>(submit_info.signal_semaphores.size());
            vk_submit_info.pSignalSemaphores = submit_info.signal_semaphores.data();
        }

        vk::Fence fence_handle = submit_info.fence.value_or(VK_NULL_HANDLE);
        m_device->queue().submit(vk_submit_info, fence_handle);
    }

    // Record and submit in one call
    template<typename Func>
    void record_and_submit(Func&& recorder, 
                          const SubmitInfo& submit_info = {},
                          vk::CommandBufferUsageFlags usage_flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit) {
        record(std::forward<Func>(recorder), usage_flags);
        submit(submit_info);
    }

    // Get underlying command buffer for direct access
    const vk::raii::CommandBuffer& get() const { return m_command_buffer; }
    vk::raii::CommandBuffer& get() { return m_command_buffer; }

    // Convenient accessors for common operations
    void bind_pipeline(vk::PipelineBindPoint bind_point, const vk::raii::Pipeline& pipeline) {
        m_command_buffer.bindPipeline(bind_point, *pipeline);
    }

    void bind_descriptor_sets(vk::PipelineBindPoint bind_point,
                             const vk::raii::PipelineLayout& layout,
                             uint32_t first_set,
                             const vk::raii::DescriptorSet& descriptor_set) {
        m_command_buffer.bindDescriptorSets(bind_point, *layout, first_set, *descriptor_set, {});
    }

    void bind_vertex_buffers(uint32_t first_binding,
                            const std::vector<vk::Buffer>& buffers,
                            const std::vector<vk::DeviceSize>& offsets) {
        m_command_buffer.bindVertexBuffers(first_binding, buffers, offsets);
    }

    void bind_index_buffer(vk::Buffer buffer, vk::DeviceSize offset, vk::IndexType index_type) {
        m_command_buffer.bindIndexBuffer(buffer, offset, index_type);
    }

    void set_viewport(const vk::Viewport& viewport) {
        m_command_buffer.setViewport(0, viewport);
    }

    void set_scissor(const vk::Rect2D& scissor) {
        m_command_buffer.setScissor(0, scissor);
    }

    void begin_rendering(const vk::RenderingInfo& rendering_info) {
        m_command_buffer.beginRendering(rendering_info);
    }

    void end_rendering() {
        m_command_buffer.endRendering();
    }

    void draw(uint32_t vertex_count, uint32_t instance_count = 1, 
             uint32_t first_vertex = 0, uint32_t first_instance = 0) {
        m_command_buffer.draw(vertex_count, instance_count, first_vertex, first_instance);
    }

    void draw_indexed(uint32_t index_count, uint32_t instance_count = 1,
                     uint32_t first_index = 0, int32_t vertex_offset = 0, uint32_t first_instance = 0) {
        m_command_buffer.drawIndexed(index_count, instance_count, first_index, vertex_offset, first_instance);
    }

    void pipeline_barrier_2(const vk::DependencyInfo& dependency_info) {
        m_command_buffer.pipelineBarrier2(dependency_info);
    }

    Device* device() const { return m_device; }
};

class CommandPool {
private:
    Device* m_device;
    vk::raii::CommandPool m_pool{nullptr};

public:
    CommandPool(Device* device, vk::CommandPoolCreateFlags flags)
        : m_device(device) {
        vk::CommandPoolCreateInfo create_info{};
        create_info.flags = flags;
        create_info.queueFamilyIndex = device->queue_family_index();
        m_pool = vk::raii::CommandPool(device->device(), create_info);
    }

    vk::raii::CommandBuffer allocate_command_buffer(vk::CommandBufferLevel level = vk::CommandBufferLevel::ePrimary) {
        vk::CommandBufferAllocateInfo alloc_info{};
        alloc_info.commandPool = *m_pool;
        alloc_info.level = level;
        alloc_info.commandBufferCount = 1;

        auto buffers = m_device->device().allocateCommandBuffers(alloc_info);
        return std::move(buffers.front());
    }

    std::vector<vk::raii::CommandBuffer> allocate_command_buffers(uint32_t count, vk::CommandBufferLevel level = vk::CommandBufferLevel::ePrimary) {
        vk::CommandBufferAllocateInfo alloc_info{};
        alloc_info.commandPool = *m_pool;
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

    // Create a CommandBuffer wrapper for easier command recording and submission
    CommandBuffer create_command_buffer(vk::CommandBufferLevel level = vk::CommandBufferLevel::ePrimary) {
        auto raw_buffer = allocate_command_buffer(level);
        return CommandBuffer(m_device, std::move(raw_buffer));
    }

    std::vector<CommandBuffer> create_command_buffers(uint32_t count, vk::CommandBufferLevel level = vk::CommandBufferLevel::ePrimary) {
        auto raw_buffers = allocate_command_buffers(count, level);
        std::vector<CommandBuffer> result;
        result.reserve(raw_buffers.size());
        for (auto& raw_buffer : raw_buffers) {
            result.emplace_back(m_device, std::move(raw_buffer));
        }
        return result;
    }

    const vk::raii::CommandPool& command_pool() const { return m_pool; }
    const Device* device() const { return m_device; }

};

}
