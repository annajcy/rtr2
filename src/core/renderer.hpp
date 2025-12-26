#pragma once

#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "device.hpp"
#include "buffer.hpp"
#include "swap_chain.hpp"
#include "window.hpp"
#include "vulkan/vulkan_raii.hpp"

#include "command.hpp"

namespace rtr::core {

class ResourceRegistry {
public:
    using BufferMap = std::unordered_map<std::string, Buffer*>;
    using DescriptorSetMap = std::unordered_map<std::string, const vk::raii::DescriptorSet*>;

    ResourceRegistry(uint32_t frames_in_flight = 0) { resize(frames_in_flight); }

    void resize(uint32_t frames_in_flight) {
        m_buffers.assign(frames_in_flight, {});
        m_descriptor_sets.assign(frames_in_flight, {});
    }

    void clear_frame(uint32_t frame_index) {
        m_buffers[frame_index].clear();
        m_descriptor_sets[frame_index].clear();
    }

    void set_buffer(uint32_t frame_index, const std::string& name, Buffer* buffer) {
        m_buffers[frame_index][name] = buffer;
    }

    void set_descriptor_set(uint32_t frame_index, const std::string& name, const vk::raii::DescriptorSet& set) {
        m_descriptor_sets[frame_index][name] = &set;
    }

    Buffer& get_buffer(uint32_t frame_index, const std::string& name) const {
        const auto& buffers = m_buffers[frame_index];
        auto it = buffers.find(name);
        if (it == buffers.end()) {
            throw std::runtime_error("Buffer not found: " + name);
        }
        return *it->second;
    }

    const vk::raii::DescriptorSet& get_descriptor_set(uint32_t frame_index, const std::string& name) const {
        const auto& sets = m_descriptor_sets[frame_index];
        auto it = sets.find(name);
        if (it == sets.end()) {
            throw std::runtime_error("DescriptorSet not found: " + name);
        }
        return *it->second;
    }

    bool has_buffer(uint32_t frame_index, const std::string& name) const {
        return m_buffers[frame_index].find(name) != m_buffers[frame_index].end();
    }

    bool has_descriptor_set(uint32_t frame_index, const std::string& name) const {
        return m_descriptor_sets[frame_index].find(name) != m_descriptor_sets[frame_index].end();
    }

private:
    std::vector<BufferMap> m_buffers;
    std::vector<DescriptorSetMap> m_descriptor_sets;
};

/**
 * @brief Context for a single frame rendering
 * 
 * Provides access to the per-frame command buffer and swapchain image handles.
 * The Renderer selects the correct frame resources, so render passes only see
 * the "current frame" without tracking frame-in-flight indices.
 */
class FrameContext {

private:
    Device* m_device;
    CommandBuffer* m_cmd;
    const vk::raii::ImageView* m_swapchain_image_view;  // Current swapchain image view to render to
    const vk::Image* m_swapchain_image;  // Corresponding VkImage for barriers
    ResourceRegistry* m_registry;
    uint32_t m_frame_index = 0;
    
public:
    FrameContext(Device* device,
                 CommandBuffer* cmd,
                 const vk::raii::ImageView& swapchain_image_view,
                 const vk::Image& swapchain_image,
                 ResourceRegistry* registry,
                 uint32_t frame_index)
        : m_device(device),
          m_cmd(cmd),
          m_swapchain_image_view(&swapchain_image_view),
          m_swapchain_image(&swapchain_image),
          m_registry(registry),
          m_frame_index(frame_index) {}
    
    const vk::raii::ImageView& swapchain_image_view() const { return *m_swapchain_image_view; }
    const vk::Image& swapchain_image() const { return *m_swapchain_image; }

    Device* device() const { return m_device; }
    CommandBuffer* cmd() const { return m_cmd; }

    Buffer& get_buffer(const std::string& name) {
        return m_registry->get_buffer(m_frame_index, name);
    }

    const vk::raii::DescriptorSet& get_descriptor_set(const std::string& name) {
        return m_registry->get_descriptor_set(m_frame_index, name);
    }

    bool has_buffer(const std::string& name) const {
        return m_registry->has_buffer(m_frame_index, name);
    }

    bool has_descriptor_set(const std::string& name) const {
        return m_registry->has_descriptor_set(m_frame_index, name);
    }
};

/**
 * @brief Renderer manages frame-in-flight synchronization and swapchain presentation
 * 
 * Responsibilities:
 * - Own and manage SwapChain lifecycle
 * - Handle frame-in-flight synchronization
 * - Manage per-frame command buffers and sync objects
 * - Build FrameContext for each frame
 * - Handle swapchain recreation on window resize
 * 
 * Usage:
 *   Renderer renderer(device, window, max_frames_in_flight);
 *   while (!window->should_close()) {
 *       renderer.draw_frame([](FrameContext& ctx) {
 *           // Record rendering commands
 *       });
 *   }
 */
class Renderer {
public:
 /**
     * @brief Per-frame resources (command buffer + synchronization objects)
     */
    struct PerFrameResources {
        vk::raii::Semaphore image_available_semaphore{nullptr};
        vk::raii::Semaphore render_finished_semaphore{nullptr};
        vk::raii::Fence in_flight_fence{nullptr};
        CommandBuffer command_buffer;
        
        // Constructor taking CommandBuffer by move
        PerFrameResources(CommandBuffer&& cmd)
            : command_buffer(std::move(cmd)) {}
        
        PerFrameResources(PerFrameResources&&) = default;
        PerFrameResources& operator=(PerFrameResources&&) = default;
    };

private:
    Device* m_device;
    Window* m_window;
    
    std::unique_ptr<SwapChain> m_swapchain;
    std::unique_ptr<CommandPool> m_command_pool;
    
    uint32_t m_max_frames_in_flight;
    uint32_t m_current_frame = 0;
    uint32_t m_current_image = 0;
    bool m_framebuffer_resized = false;
    
    std::vector<PerFrameResources> m_per_frame_resources;
    ResourceRegistry m_resource_registry;
    std::function<void(uint32_t, ResourceRegistry&)> m_frame_resource_provider;
    
public:
    // Callback type for user rendering logic
    using RenderCallback = std::function<void(FrameContext&)>;
    
    /**
     * @brief Construct a new Renderer
     * 
     * @param device Vulkan device (non-owning)
     * @param window Window for swapchain surface (non-owning)
     * @param max_frames_in_flight Number of frames that can be processed concurrently
     */
    Renderer(Device* device, 
             Window* window, 
             uint32_t max_frames_in_flight = 2)
        : m_device(device)
        , m_window(window)
        , m_max_frames_in_flight(max_frames_in_flight)
        , m_resource_registry(max_frames_in_flight) {
        
        // Create swapchain
        m_swapchain = std::make_unique<SwapChain>(device);
        
        // Create command pool with reset capability
        m_command_pool = std::make_unique<CommandPool>(
            device, 
            vk::CommandPoolCreateFlagBits::eResetCommandBuffer
        );
        
        // Initialize per-frame resources
        init_per_frame_resources();
    }
    
    ~Renderer() = default;
    
    // Non-copyable
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    /**
     * @brief Set a provider that registers per-frame resources into the registry
     */
    void set_frame_resource_provider(std::function<void(uint32_t, ResourceRegistry&)> provider) {
        m_frame_resource_provider = std::move(provider);
    }
    
    /**
     * @brief Main rendering function - acquires image, records commands, submits, presents
     * 
     * @param callback User-provided function to record rendering commands
     */
    void draw_frame(RenderCallback callback) {
        auto& frame_res = m_per_frame_resources[m_current_frame];
        
        // 1. Wait for previous frame to finish
        vk::Result wait_result = m_device->device().waitForFences(
            *frame_res.in_flight_fence,
            VK_TRUE,
            UINT64_MAX
        );
        
        if (wait_result != vk::Result::eSuccess) {
            std::cerr << "Failed to wait for fence" << std::endl;
            return;
        }
        
        // 2. Acquire next swapchain image
        auto [result, image_index] = m_swapchain->acquire_next_image(
            frame_res.image_available_semaphore
        );
        
        if (result == vk::Result::eErrorOutOfDateKHR) {
            // Swapchain is out of date, recreate it
            recreate_swapchain();
            return;
        }
        
        if (result != vk::Result::eSuccess && result != vk::Result::eSuboptimalKHR) {
            throw std::runtime_error("Failed to acquire swapchain image");
        }
        
        m_current_image = image_index;
        
        // 3. Reset per-frame resource registry entries
        m_resource_registry.clear_frame(m_current_frame);
        
        // 4. Let caller register per-frame resources (if provided)
        if (m_frame_resource_provider) {
            m_frame_resource_provider(m_current_frame, m_resource_registry);
        }
        
        // 5. Build FrameContext
        FrameContext frame_ctx = build_frame_context();
        
        // 6. Reset command buffer and let user record commands
        frame_res.command_buffer.reset();
        callback(frame_ctx);
        
        // 7. Reset fence before submission
        m_device->device().resetFences(*frame_res.in_flight_fence);
        
        // 8. Submit command buffer
        CommandBuffer::SubmitInfo submit_info;
        submit_info.wait_semaphores = {*frame_res.image_available_semaphore};
        submit_info.wait_stages = {vk::PipelineStageFlagBits::eColorAttachmentOutput};
        submit_info.signal_semaphores = {*frame_res.render_finished_semaphore};
        submit_info.fence = *frame_res.in_flight_fence;
        
        frame_res.command_buffer.submit(submit_info);
        
        // 9. Present image
        // With VK_EXT_swapchain_maintenance1, we can use a fence with present
        // Wait for the fence to complete before reusing it
        wait_result = m_device->device().waitForFences(*frame_res.in_flight_fence, VK_TRUE, UINT64_MAX);
        if (wait_result != vk::Result::eSuccess) {
            std::cerr << "Failed to wait for fence before present" << std::endl;
            return;
        }
        m_device->device().resetFences(*frame_res.in_flight_fence);
        
        vk::Result present_result = m_swapchain->present(
            image_index,
            frame_res.render_finished_semaphore,
            frame_res.in_flight_fence  // Reuse in_flight_fence for present
        );
        
        // 10. Check if swapchain needs recreation
        bool needs_recreation = false;
        
        if (present_result == vk::Result::eErrorOutOfDateKHR) {
            needs_recreation = true;
        } else if (present_result == vk::Result::eSuboptimalKHR) {
            std::cout << "Swapchain suboptimal during presentation." << std::endl;
            needs_recreation = true;
        } else if (present_result != vk::Result::eSuccess) {
            throw std::runtime_error("Failed to present swapchain image");
        }
        
        if (needs_recreation || m_framebuffer_resized) {
            m_framebuffer_resized = false;
            recreate_swapchain();
        }
        
        // 9. Advance to next frame
        m_current_frame = (m_current_frame + 1) % m_max_frames_in_flight;
    }
    
    /**
     * @brief Notify renderer of window resize (triggers swapchain recreation)
     * 
     * @param width New window width
     * @param height New window height
     */
    void on_window_resized(uint32_t width, uint32_t height) {
        std::cout << "Renderer: Window resized to (" << width << ", " << height << ")" << std::endl;
        m_framebuffer_resized = true;
    }
    
    /**
     * @brief Get current render extent (swapchain extent)
     */
    vk::Extent2D render_extent() const { return m_swapchain->extent(); }
    
    /**
     * @brief Get current render format (swapchain image format)
     */
    vk::Format render_format() const { return m_swapchain->image_format(); }
    
    /**
     * @brief Get number of swapchain images
     */
    uint32_t image_count() const { 
        return static_cast<uint32_t>(m_swapchain->images().size()); 
    }

    /**
     * @brief Get maximum frames in flight
     */
    uint32_t max_frames_in_flight() const { return m_max_frames_in_flight; }
    
    /**
     * @brief Get current frame index (for accessing per-frame resources)
     */
    uint32_t current_frame_index() const { return m_current_frame; }
    
    /**
     * @brief Get current swapchain image index
     */
    uint32_t current_image_index() const { return m_current_image; }
    
    /**
     * @brief Get per-frame resources for a specific frame index
     */
    const PerFrameResources& get_frame_resources(uint32_t frame_index) const {
        return m_per_frame_resources[frame_index];
    }
    
    /**
     * @brief Get current frame's resources
     */
    const PerFrameResources& current_frame_resources() const {
        return m_per_frame_resources[m_current_frame];
    }
    
    /**
     * @brief Access to device (for advanced usage)
     */
    Device* device() const { return m_device; }
    
    /**
     * @brief Access resource registry for the current renderer
     */
    ResourceRegistry& resource_registry() { return m_resource_registry; }
    const ResourceRegistry& resource_registry() const { return m_resource_registry; }
    
    /**
     * @brief Access to swapchain (for advanced usage)
     */
    const SwapChain& swapchain() const { return *m_swapchain; }

    /**
     * @brief Initialize per-frame resources (command buffers + sync objects)
     */
    void init_per_frame_resources() {
        m_per_frame_resources.clear();
        m_per_frame_resources.reserve(m_max_frames_in_flight);
        
        vk::SemaphoreCreateInfo semaphore_info{};
        vk::FenceCreateInfo fence_info{};
        fence_info.flags = vk::FenceCreateFlagBits::eSignaled;  // Start signaled
        
        // Create command buffers
        auto command_buffers = m_command_pool->create_command_buffers(m_max_frames_in_flight);
        
        for (uint32_t i = 0; i < m_max_frames_in_flight; ++i) {
            PerFrameResources resources(std::move(command_buffers[i]));
            
            resources.image_available_semaphore = vk::raii::Semaphore(
                m_device->device(), semaphore_info
            );
            
            resources.render_finished_semaphore = vk::raii::Semaphore(
                m_device->device(), semaphore_info
            );
            
            resources.in_flight_fence = vk::raii::Fence(
                m_device->device(), fence_info
            );
            
            m_per_frame_resources.push_back(std::move(resources));
        }
    }
    
    /**
     * @brief Recreate swapchain (called on window resize or out-of-date)
     */
    void recreate_swapchain() {
        // Wait for device to be idle before recreating
        m_device->device().waitIdle();
        
        // Recreate swapchain
        m_swapchain->recreate();
        
        std::cout << "Swapchain recreated with extent (" 
                  << m_swapchain->extent().width << ", " 
                  << m_swapchain->extent().height << ")" << std::endl;
    }
    
    /**
     * @brief Build FrameContext for current frame
     */
    FrameContext build_frame_context() {
        auto& frame_res = m_per_frame_resources[m_current_frame];
        
        return FrameContext(
            m_device,
            &frame_res.command_buffer,
            m_swapchain->image_views()[m_current_image],
            m_swapchain->images()[m_current_image],
            &m_resource_registry,
            m_current_frame
        );
    }
};

} // namespace rtr::core
