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


struct DepthResources {
    vk::raii::Image image{nullptr};
    vk::raii::DeviceMemory memory{nullptr};
    vk::raii::ImageView view{nullptr};
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
    ResourceRegistry* m_registry;
    const vk::raii::ImageView* m_swapchain_image_view;  // Current swapchain image view to render to
    const vk::Image* m_swapchain_image;  // Corresponding VkImage for barriers
    const DepthResources* m_depth_resources;
    uint32_t m_frame_index = 0;
    
public:
    FrameContext(Device* device,
                CommandBuffer* cmd,
                ResourceRegistry* registry,
                const vk::raii::ImageView& swapchain_image_view,
                const vk::Image& swapchain_image,
                const DepthResources& depth_resources,
                uint32_t frame_index)
        : m_device(device),
          m_cmd(cmd),
          m_swapchain_image_view(&swapchain_image_view),
          m_swapchain_image(&swapchain_image),
          m_depth_resources(&depth_resources),
          m_registry(registry),
          m_frame_index(frame_index) {}
    
    const vk::raii::ImageView& swapchain_image_view() const { return *m_swapchain_image_view; }
    const vk::Image& swapchain_image() const { return *m_swapchain_image; }
    const DepthResources& depth_resources() const { return *m_depth_resources; }

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

    std::vector<DepthResources> m_depth_resources;
    vk::Format m_depth_format{vk::Format::eD32Sfloat};

    std::vector<vk::raii::Semaphore> m_render_finished_semaphores;
    
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

        init_depth_resources();
        init_render_finished_semaphores();
        
        // Initialize per-frame resources
        init_per_frame_resources();
    }
    
    ~Renderer() = default;
    
    // Non-copyable
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    void init_render_finished_semaphores() {
        m_render_finished_semaphores.clear();
        vk::SemaphoreCreateInfo semaphore_info{};
        
        // 为每一张 Swapchain Image 创建一个对应的信号量
        for (size_t i = 0; i < m_swapchain->images().size(); i++) {
            m_render_finished_semaphores.emplace_back(m_device->device(), semaphore_info);
        }
    }

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

        m_device->device().resetFences(*frame_res.in_flight_fence);
        
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
 
        // 7. Submit command buffer
        CommandBuffer::SubmitInfo submit_info;
        submit_info.wait_semaphores = {*frame_res.image_available_semaphore};
        submit_info.wait_stages = {vk::PipelineStageFlagBits::eColorAttachmentOutput};
        submit_info.signal_semaphores = {*m_render_finished_semaphores[image_index]};
        submit_info.fence = *frame_res.in_flight_fence;
        
        frame_res.command_buffer.submit(submit_info);
        
        // 8. Present swapchain image
        vk::Result present_result = m_swapchain->present(
            image_index,
            m_render_finished_semaphores[image_index],
            nullptr
        );
        
        // 9. Check if swapchain needs recreation
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
             // Wait for device to be idle before recreating
            m_device->device().waitIdle();
            recreate_swapchain();
            init_depth_resources();
            init_render_finished_semaphores();
        }
        
        // 10. Advance to next frame
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

    vk::Format depth_format() const { return m_depth_format; }
    
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
            
            resources.in_flight_fence = vk::raii::Fence(
                m_device->device(), fence_info
            );

            m_per_frame_resources.push_back(std::move(resources));
        }
    }

    void init_depth_resources() {
        m_depth_resources.resize(m_swapchain->images().size());
        for (auto& depth : m_depth_resources) {
            create_depth_resources(depth);
        }
    }
    
    /**
     * @brief Recreate swapchain (called on window resize or out-of-date)
     */
    void recreate_swapchain() {
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
            &m_resource_registry,
            m_swapchain->image_views()[m_current_image],
            m_swapchain->images()[m_current_image],
            m_depth_resources[m_current_image],
            m_current_frame
        );
    }

    void recreate_depth_resources() {
        for (auto& depth : m_depth_resources) {
            create_depth_resources(depth);
        }
    }

    void create_depth_resources(DepthResources& depth) {
        vk::ImageCreateInfo image_info{};
        image_info.imageType = vk::ImageType::e2D;
        auto extent = m_swapchain->extent();
        image_info.extent = vk::Extent3D{extent.width, extent.height, 1};
        image_info.mipLevels = 1;
        image_info.arrayLayers = 1;
        image_info.format = m_depth_format;
        image_info.tiling = vk::ImageTiling::eOptimal;
        image_info.initialLayout = vk::ImageLayout::eUndefined;
        image_info.usage = vk::ImageUsageFlagBits::eDepthStencilAttachment;
        image_info.samples = vk::SampleCountFlagBits::e1;
        image_info.sharingMode = vk::SharingMode::eExclusive;

        depth.image = vk::raii::Image(m_device->device(), image_info);

        auto mem_requirements = depth.image.getMemoryRequirements();
        auto mem_properties = m_device->physical_device().getMemoryProperties();
        auto mem_type_index = find_memory_type(
            mem_properties,
            mem_requirements.memoryTypeBits,
            vk::MemoryPropertyFlagBits::eDeviceLocal);
        if (!mem_type_index.has_value()) {
            throw std::runtime_error("Failed to find memory for depth buffer.");
        }

        vk::MemoryAllocateInfo alloc_info{};
        alloc_info.allocationSize = mem_requirements.size;
        alloc_info.memoryTypeIndex = mem_type_index.value();
        depth.memory = vk::raii::DeviceMemory(m_device->device(), alloc_info);
        depth.image.bindMemory(*depth.memory, 0);

        vk::ImageViewCreateInfo view_info{};
        view_info.image = *depth.image;
        view_info.viewType = vk::ImageViewType::e2D;
        view_info.format = m_depth_format;
        view_info.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eDepth;
        view_info.subresourceRange.baseMipLevel = 0;
        view_info.subresourceRange.levelCount = 1;
        view_info.subresourceRange.baseArrayLayer = 0;
        view_info.subresourceRange.layerCount = 1;
        depth.view = vk::raii::ImageView(m_device->device(), view_info);

        // Transition to depth attachment layout once.
        CommandPool temp_pool(m_device, vk::CommandPoolCreateFlagBits::eTransient);
        auto cmd = temp_pool.create_command_buffer();
        cmd.record_and_submit([&](CommandBuffer& recorder) {
            vk::ImageMemoryBarrier2 barrier{};
            barrier.oldLayout = vk::ImageLayout::eUndefined;
            barrier.newLayout = vk::ImageLayout::eDepthAttachmentOptimal;
            barrier.srcStageMask = vk::PipelineStageFlagBits2::eTopOfPipe;
            barrier.srcAccessMask = vk::AccessFlagBits2::eNone;
            barrier.dstStageMask = vk::PipelineStageFlagBits2::eEarlyFragmentTests;
            barrier.dstAccessMask = vk::AccessFlagBits2::eDepthStencilAttachmentWrite | vk::AccessFlagBits2::eDepthStencilAttachmentRead;
            barrier.image = *depth.image;
            barrier.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eDepth;
            barrier.subresourceRange.baseMipLevel = 0;
            barrier.subresourceRange.levelCount = 1;
            barrier.subresourceRange.baseArrayLayer = 0;
            barrier.subresourceRange.layerCount = 1;

            vk::DependencyInfo dep{};
            dep.imageMemoryBarrierCount = 1;
            dep.pImageMemoryBarriers = &barrier;
            recorder.pipeline_barrier_2(dep);
        });
        m_device->queue().waitIdle();
    }
};

} // namespace rtr::core
