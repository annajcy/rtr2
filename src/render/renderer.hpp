#pragma once

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "rhi/buffer.hpp"
#include "rhi/command.hpp"
#include "rhi/context.hpp"
#include "rhi/device.hpp"
#include "rhi/texture.hpp"
#include "rhi/window.hpp"
#include "render/frame_scheduler.hpp"
#include "vulkan/vulkan_raii.hpp"

namespace rtr::render {

class ResourceRegistry {
public:
    using BufferMap = std::unordered_map<std::string, rhi::Buffer*>;
    using DescriptorSetMap = std::unordered_map<std::string, const vk::raii::DescriptorSet*>;

    explicit ResourceRegistry(uint32_t frames_in_flight = 0) { resize(frames_in_flight); }

    void resize(uint32_t frames_in_flight) {
        m_buffers.assign(frames_in_flight, {});
        m_descriptor_sets.assign(frames_in_flight, {});
    }

    void clear_frame(uint32_t frame_index) {
        m_buffers[frame_index].clear();
        m_descriptor_sets[frame_index].clear();
    }

    void set_buffer(uint32_t frame_index, const std::string& name, rhi::Buffer* buffer) {
        m_buffers[frame_index][name] = buffer;
    }

    void set_descriptor_set(uint32_t frame_index, const std::string& name, const vk::raii::DescriptorSet& set) {
        m_descriptor_sets[frame_index][name] = &set;
    }

    rhi::Buffer& get_buffer(uint32_t frame_index, const std::string& name) const {
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

class FrameContext {
private:
    rhi::Device* m_device{};
    rhi::CommandBuffer* m_cmd{};
    ResourceRegistry* m_registry{};
    const vk::raii::ImageView* m_swapchain_image_view{};
    const vk::Image* m_swapchain_image{};
    const rhi::Image* m_depth_image{};
    uint32_t m_frame_index{0};

public:
    FrameContext(
        rhi::Device* device,
        rhi::CommandBuffer* cmd,
        ResourceRegistry* registry,
        const vk::raii::ImageView& swapchain_image_view,
        const vk::Image& swapchain_image,
        const rhi::Image& depth_image,
        uint32_t frame_index
    )
        : m_device(device),
          m_cmd(cmd),
          m_registry(registry),
          m_swapchain_image_view(&swapchain_image_view),
          m_swapchain_image(&swapchain_image),
          m_depth_image(&depth_image),
          m_frame_index(frame_index) {}

    const vk::raii::ImageView& swapchain_image_view() const { return *m_swapchain_image_view; }
    const vk::Image& swapchain_image() const { return *m_swapchain_image; }
    const rhi::Image& depth_image() const { return *m_depth_image; }

    rhi::Device* device() const { return m_device; }
    rhi::CommandBuffer* cmd() const { return m_cmd; }

    rhi::Buffer& get_buffer(const std::string& name) {
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

class Renderer {
public:
    using PerFrameResources = FrameScheduler::PerFrameResources;
    using PerImageResources = FrameScheduler::PerImageResources;
    using RenderCallback = std::function<void(FrameContext&)>;

private:
    std::unique_ptr<rhi::Window> m_window{};
    std::unique_ptr<rhi::Context> m_context{};
    std::unique_ptr<rhi::Device> m_device{};

    std::unique_ptr<FrameScheduler> m_frame_scheduler{};
    ResourceRegistry m_resource_registry;
    std::function<void(uint32_t, ResourceRegistry&)> m_frame_resource_provider;

public:
    Renderer(
        int width,
        int height,
        std::string title,
        uint32_t max_frames_in_flight = 2
    )
        : m_resource_registry(max_frames_in_flight) {
        m_window = std::make_unique<rhi::Window>(width, height, title);
        m_window->set_user_pointer(this);
        m_window->set_framebuffer_size_callback([](GLFWwindow* window, int width, int height) {
            auto* renderer = reinterpret_cast<Renderer*>(glfwGetWindowUserPointer(window));
            if (!renderer) {
                return;
            }
            renderer->on_window_resized(
                static_cast<uint32_t>(width),
                static_cast<uint32_t>(height)
            );
        });

        rhi::ContextCreateInfo context_info{};
        context_info.app_name = m_window->title();
        context_info.instance_extensions = m_window->required_extensions();
        context_info.surface_creator = [this](const vk::raii::Instance& instance) {
            return m_window->create_vk_surface(instance);
        };
        // context_info.framebuffer_size = [this]() {
        //     return m_window->framebuffer_size();
        // };
        m_context = std::make_unique<rhi::Context>(std::move(context_info));
        m_device = std::make_unique<rhi::Device>(m_context.get());
        m_frame_scheduler = std::make_unique<FrameScheduler>(
            m_window.get(),
            m_context.get(),
            m_device.get(),
            max_frames_in_flight
        );
    }

    ~Renderer() = default;

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    void set_frame_resource_provider(std::function<void(uint32_t, ResourceRegistry&)> provider) {
        m_frame_resource_provider = std::move(provider);
    }

    void draw_frame(RenderCallback callback) {
        auto ticket_opt = m_frame_scheduler->begin_frame();
        if (!ticket_opt.has_value()) {
            return;
        }
        auto ticket = ticket_opt.value();

        m_resource_registry.clear_frame(ticket.frame_index);
        if (m_frame_resource_provider) {
            m_frame_resource_provider(ticket.frame_index, m_resource_registry);
        }

        FrameContext frame_ctx = build_frame_context(ticket);
        ticket.command_buffer->reset();
        callback(frame_ctx);

        m_frame_scheduler->submit_and_present(ticket);
    }

    void on_window_resized(uint32_t width, uint32_t height) {
        m_frame_scheduler->on_window_resized(width, height);
    }

    const rhi::Device& device() const { return *m_device.get(); }
    const rhi::Context& context() const { return *m_context.get(); }
    const rhi::Window& window() const { return *m_window.get(); }
    const render::FrameScheduler& frame_scheduler() const { return *m_frame_scheduler.get(); }

    rhi::Device& device() { return *m_device.get(); }
    rhi::Context& context() { return *m_context.get(); }
    rhi::Window& window() { return *m_window.get(); }
    render::FrameScheduler& frame_scheduler() { return *m_frame_scheduler.get(); }

    ResourceRegistry& resource_registry() { return m_resource_registry; }
    const ResourceRegistry& resource_registry() const { return m_resource_registry; }

private:
    FrameContext build_frame_context(const FrameScheduler::FrameTicket& ticket) {
        return FrameContext(
            m_device.get(),
            ticket.command_buffer,
            &m_resource_registry,
            m_frame_scheduler->swapchain().image_views()[ticket.image_index],
            m_frame_scheduler->swapchain().images()[ticket.image_index],
            m_frame_scheduler->per_image_resources()[ticket.image_index].depth_image,
            ticket.frame_index
        );
    }
};

} // namespace rtr::render
