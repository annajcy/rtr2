#pragma once

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
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
        return m_per_frame[frame_index];
    }

    const ResourceMap& frame_resources(uint32_t frame_index) const {
        return m_per_frame[frame_index];
    }
};

using BufferRegistry = TypedResourceRegistry<rhi::Buffer>;
using DescriptorSetRegistry = TypedResourceRegistry<vk::raii::DescriptorSet>;

template <class... Ts>
class ResourceRegistryAggregate {
private:
    std::tuple<TypedResourceRegistry<Ts>...> m_registries;
    uint32_t m_frame_count{0};

    static std::tuple<TypedResourceRegistry<Ts>...> make_registries(uint32_t frames_count) {
        return std::tuple<TypedResourceRegistry<Ts>...>{TypedResourceRegistry<Ts>(frames_count)...};
    }

public:
    explicit ResourceRegistryAggregate(uint32_t frames_count = 0)
        : m_registries(make_registries(frames_count)),
          m_frame_count(frames_count) {}

    void clear_frame(uint32_t frame_index) {
        (registry<Ts>().clear_frame(frame_index), ...);
    }

    void clear_all() {
        for (uint32_t i = 0; i < m_frame_count; ++i) {
            clear_frame(i);
        }
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

using ResourceRegistries = ResourceRegistryAggregate<rhi::Buffer, vk::raii::DescriptorSet>;

class IFrameResourceBinder {
public:
    virtual ~IFrameResourceBinder() = default;

    virtual void bind_static_resources(ResourceRegistries& /*registries*/) {}
    virtual void bind_frame_resources(uint32_t frame_index, ResourceRegistries& registries) = 0;
};

class FrameContext {
private:
    rhi::Device* m_device{};
    rhi::CommandBuffer* m_cmd{};
    ResourceRegistries* m_registries{};
    const vk::raii::ImageView* m_swapchain_image_view{};
    const vk::Image* m_swapchain_image{};
    const rhi::Image* m_depth_image{};
    uint32_t m_frame_index{0};

public:
    FrameContext(
        rhi::Device* device,
        rhi::CommandBuffer* cmd,
        ResourceRegistries* registries,
        const vk::raii::ImageView& swapchain_image_view,
        const vk::Image& swapchain_image,
        const rhi::Image& depth_image,
        uint32_t frame_index
    )
        : m_device(device),
          m_cmd(cmd),
          m_registries(registries),
          m_swapchain_image_view(&swapchain_image_view),
          m_swapchain_image(&swapchain_image),
          m_depth_image(&depth_image),
          m_frame_index(frame_index) {}

    const rhi::CommandBuffer& cmd() const { return *m_cmd; }
    rhi::CommandBuffer& cmd() { return *m_cmd; }

    const vk::raii::ImageView& swapchain_image_view() const { return *m_swapchain_image_view; }
    const vk::Image& swapchain_image() const { return *m_swapchain_image; }
    const rhi::Image& depth_image() const { return *m_depth_image; }

    rhi::Buffer& get_buffer(const std::string& name) {
        auto& registry = m_registries->registry<rhi::Buffer>();
        if (registry.has_perframe_resource(m_frame_index, name)) {
            return registry.get_perframe_resource(m_frame_index, name);
        }
        return registry.get_global_resource(name);
    }

    vk::raii::DescriptorSet& get_descriptor_set(const std::string& name) {
        auto& registry = m_registries->registry<vk::raii::DescriptorSet>();
        if (registry.has_perframe_resource(m_frame_index, name)) {
            return registry.get_perframe_resource(m_frame_index, name);
        }
        return registry.get_global_resource(name);
    }

    bool has_buffer(const std::string& name) const {
        return m_registries->registry<rhi::Buffer>().has_perframe_resource(m_frame_index, name) ||
               m_registries->registry<rhi::Buffer>().has_global_resource(name);
    }

    bool has_descriptor_set(const std::string& name) const {
        return m_registries->registry<vk::raii::DescriptorSet>().has_perframe_resource(m_frame_index, name) ||
               m_registries->registry<vk::raii::DescriptorSet>().has_global_resource(name);
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
    ResourceRegistries m_resource_registries;

    std::vector<IFrameResourceBinder*> m_frame_resource_binders;

public:
    Renderer(
        int width, int height,
        std::string title,
        uint32_t max_frames_in_flight = 2
    )
        : m_resource_registries(max_frames_in_flight) {
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

    void bind_resource() {
        m_resource_registries.clear_all();
        // bind static resources
        for (auto* binder : m_frame_resource_binders) {
            binder->bind_static_resources(m_resource_registries);
        }
        // bind per-frame resources
        for (uint32_t i = 0; i < m_frame_scheduler->max_frames_in_flight(); ++i) {
            for (auto* binder : m_frame_resource_binders) {
                binder->bind_frame_resources(i, m_resource_registries);
            }
        }   
    }

    void register_frame_resource_binder(IFrameResourceBinder& binder) {
        auto it = std::find(m_frame_resource_binders.begin(), m_frame_resource_binders.end(), &binder);
        if (it != m_frame_resource_binders.end()) {
            throw std::runtime_error("Frame resource binder already registered.");
        }
        m_frame_resource_binders.push_back(&binder);
    }

    void unregister_frame_resource_binder(IFrameResourceBinder& binder) {
        auto it = std::find(m_frame_resource_binders.begin(), m_frame_resource_binders.end(), &binder);
        if (it == m_frame_resource_binders.end()) {
            return;
        }
        m_frame_resource_binders.erase(it);
    }

    void draw_frame(RenderCallback callback) {
        bind_resource();
        auto ticket_opt = m_frame_scheduler->begin_frame();
        if (!ticket_opt.has_value()) {
            return;
        }
        auto ticket = ticket_opt.value();
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
    rhi::Device& device() { return *m_device.get(); }
    rhi::Context& context() { return *m_context.get(); }
    rhi::Window& window() { return *m_window.get(); }

    const render::FrameScheduler& frame_scheduler() const { return *m_frame_scheduler.get(); }
    render::FrameScheduler& frame_scheduler() { return *m_frame_scheduler.get(); }

    ResourceRegistries& resource_registries() { return m_resource_registries; }
    const ResourceRegistries& resource_registries() const { return m_resource_registries; }
    BufferRegistry& buffer_registry() { return m_resource_registries.registry<rhi::Buffer>(); }
    const BufferRegistry& buffer_registry() const { return m_resource_registries.registry<rhi::Buffer>(); }
    DescriptorSetRegistry& descriptor_registry() { return m_resource_registries.registry<vk::raii::DescriptorSet>(); }
    const DescriptorSetRegistry& descriptor_registry() const { return m_resource_registries.registry<vk::raii::DescriptorSet>(); }

private:
    FrameContext build_frame_context(const FrameScheduler::FrameTicket& ticket) {
        return FrameContext(
            m_device.get(),
            ticket.command_buffer,
            &m_resource_registries,
            m_frame_scheduler->swapchain().image_views()[ticket.image_index],
            m_frame_scheduler->swapchain().images()[ticket.image_index],
            m_frame_scheduler->per_frame_resources()[ticket.frame_index].depth_image,
            ticket.frame_index
        );
    }
};

} // namespace rtr::render
