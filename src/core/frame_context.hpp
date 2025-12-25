#pragma once

#include "command.hpp"
#include "core/buffer.hpp"
#include "device.hpp"
#include "vulkan/vulkan_raii.hpp"
#include <unordered_map>
#include <string>
#include <stdexcept>
#include <any>

namespace rtr::core {

/**
 * @brief Context for a single frame rendering
 * 
 * Provides access to current frame's resources. Does not store frame indices -
 * the Renderer is responsible for selecting the correct per-frame resources.
 * This design ensures render passes only see "current frame" without knowing
 * which frame-in-flight it is, which is ideal for RenderGraph systems.
 */
class FrameContext {

public:
    class Builder {
    public:
        std::unordered_map<std::string, Buffer*> buffers;
        std::unordered_map<std::string, const vk::raii::DescriptorSet*> descriptor_sets;
        std::unordered_map<std::string, std::any> custom_resources;

        Builder& add_buffer(const std::string& name, Buffer* buffer) {
            buffers[name] = buffer;
            return *this;
        }

        Builder& add_descriptor_set(const std::string& name, const vk::raii::DescriptorSet& descriptor_set) {
            descriptor_sets[name] = &descriptor_set;
            return *this;
        }

        FrameContext build(
            Device* device,
            CommandBuffer* cmd,
            const vk::raii::ImageView* swapchain_image_view,
            const vk::Image* swapchain_image
        ) {
            return FrameContext(
                device,
                cmd,
                swapchain_image_view,
                swapchain_image,
                buffers,
                descriptor_sets
            );
        }
          
    };

private:
    Device* m_device;
    CommandBuffer* m_cmd;
    const vk::raii::ImageView* m_swapchain_image_view;  // Current swapchain image view to render to
    const vk::Image* m_swapchain_image;  // Corresponding VkImage for barriers
    
    // Current frame's resources (non-owning pointers, already indexed by Renderer)
    std::unordered_map<std::string, Buffer*> m_buffers;
    std::unordered_map<std::string, const vk::raii::DescriptorSet*> m_descriptor_sets;

public:
    FrameContext(Device* device,
                 CommandBuffer* cmd,
                 const vk::raii::ImageView* swapchain_image_view,
                 const vk::Image* swapchain_image,
                const std::unordered_map<std::string, Buffer*> buffers,
                const std::unordered_map<std::string, const vk::raii::DescriptorSet*> descriptor_sets)
        : m_device(device),
          m_cmd(cmd),
          m_swapchain_image_view(swapchain_image_view),
          m_swapchain_image(swapchain_image),
          m_buffers(buffers),
          m_descriptor_sets(descriptor_sets) {}
    
    const vk::raii::ImageView& swapchain_image_view() const { return *m_swapchain_image_view; }
    const vk::Image& swapchain_image() const { return *m_swapchain_image; }

    Device* device() const { return m_device; }
    CommandBuffer* cmd() const { return m_cmd; }
    
    // Convenience accessors with validation
    Buffer& get_buffer(const std::string& name) {
        auto it = m_buffers.find(name);
        if (it == m_buffers.end()) {
            throw std::runtime_error("Buffer not found: " + name);
        }
        return *it->second;
    }
    
    const vk::raii::DescriptorSet& get_descriptor_set(const std::string& name) {
        auto it = m_descriptor_sets.find(name);
        if (it == m_descriptor_sets.end()) {
            throw std::runtime_error("DescriptorSet not found: " + name);
        }
        return *it->second;
    }
    
    // Check if resource exists
    bool has_buffer(const std::string& name) const {
        return m_buffers.find(name) != m_buffers.end();
    }
    
    bool has_descriptor_set(const std::string& name) const {
        return m_descriptor_sets.find(name) != m_descriptor_sets.end();
    }
};

} // namespace rtr::core
