#pragma once

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rhi/context.hpp"
#include "rhi/device.hpp"
#include "rhi/window.hpp"
#include "render/frame_context.hpp"
#include "render/frame_scheduler.hpp"
#include "render/imgui_renderer.hpp"
#include "render/pipeline.hpp"
#include "render/resource_registries.hpp"

namespace rtr::render {

class Renderer {
public:
    using PerFrameResources = FrameScheduler::PerFrameResources;
    using PerImageResources = FrameScheduler::PerImageResources;
    using RenderCallback = std::function<void(FrameContext&)>;
    using UiCallback = std::function<void()>;

private:
    std::unique_ptr<rhi::Window> m_window{};
    std::unique_ptr<rhi::Context> m_context{};
    std::unique_ptr<rhi::Device> m_device{};

    std::unique_ptr<FrameScheduler> m_frame_scheduler{};
    std::unique_ptr<ImGuiLayer> m_imgui_renderer{};
    std::unique_ptr<IRenderPipeline> m_active_pipeline{};
    
    UiCallback m_ui_callback{};
    
    rhi::Window::WindowResizeEvent::ActionHandle m_window_resize_handle{0};

    ResourceRegistries m_resource_registries;
    bool m_static_bindings_dirty{true};
    std::vector<bool> m_perframe_bindings_dirty;
    uint64_t m_last_swapchain_generation{0};

public:
    Renderer(
        int width, int height,
        std::string title,
        uint32_t max_frames_in_flight = 2
    )
        : m_resource_registries(max_frames_in_flight),
          m_perframe_bindings_dirty(max_frames_in_flight, true) {
        m_window = std::make_unique<rhi::Window>(width, height, title);
        m_window_resize_handle = m_window->window_resize_event().add([this](int resize_width, int resize_height) {
            on_window_resized(
                static_cast<uint32_t>(resize_width),
                static_cast<uint32_t>(resize_height)
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
        m_imgui_renderer = std::make_unique<ImGuiLayer>(
            m_device.get(),
            m_context.get(),
            m_window.get(),
            m_frame_scheduler->image_count(),
            m_frame_scheduler->render_format(),
            m_frame_scheduler->depth_format()
        );
        m_last_swapchain_generation = m_frame_scheduler->swapchain_state().generation;
    }

    ~Renderer() {
        if (m_window && m_window_resize_handle != 0) {
            m_window->window_resize_event().remove(m_window_resize_handle);
            m_window_resize_handle = 0;
        }
    }

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    PipelineRuntime build_pipeline_runtime() const {
        return PipelineRuntime{
            .device = m_device.get(),
            .context = m_context.get(),
            .window = m_window.get(),
            .frame_count = m_frame_scheduler->max_frames_in_flight(),
            .image_count = m_frame_scheduler->image_count(),
            .color_format = m_frame_scheduler->render_format(),
            .depth_format = m_frame_scheduler->depth_format()
        };
    }

    void invalidate_static_bindings() {
        m_static_bindings_dirty = true;
    }

    void invalidate_all_perframe_bindings() {
        std::fill(m_perframe_bindings_dirty.begin(), m_perframe_bindings_dirty.end(), true);
    }

    void invalidate_perframe_binding(uint32_t frame_index) {
        validate_perframe_binding_index(frame_index);
        m_perframe_bindings_dirty[frame_index] = true;
    }

    void invalidate_all_bindings() {
        invalidate_static_bindings();
        invalidate_all_perframe_bindings();
    }

    void set_pipeline(std::unique_ptr<IRenderPipeline> pipeline) {
        if (!pipeline) {
            throw std::runtime_error("set_pipeline received null pipeline.");
        }
        if (m_active_pipeline) {
            throw std::runtime_error("Renderer pipeline is immutable at runtime and cannot be replaced.");
        }
        m_active_pipeline = std::move(pipeline);
        m_active_pipeline->on_swapchain_state_changed(m_frame_scheduler->swapchain_state());
        invalidate_all_bindings();
    }

    IRenderPipeline* pipeline() { return m_active_pipeline.get(); }
    const IRenderPipeline* pipeline() const { return m_active_pipeline.get(); }

    void set_ui_callback(UiCallback cb) {
        if (!cb) {
            throw std::runtime_error("UI callback must be valid.");
        }
        m_ui_callback = std::move(cb);
    }

    void clear_ui_callback() {
        m_ui_callback = UiCallback{};
    }

    bool imgui_wants_capture_mouse() const {
        return m_imgui_renderer && m_imgui_renderer->wants_capture_mouse();
    }

    bool imgui_wants_capture_keyboard() const {
        return m_imgui_renderer && m_imgui_renderer->wants_capture_keyboard();
    }

    void draw_frame() {
        if (!m_active_pipeline) {
            throw std::runtime_error("No active pipeline. Call set_pipeline(...) before draw_frame().");
        }

        auto ticket_opt = m_frame_scheduler->begin_frame();
        if (!ticket_opt.has_value()) {
            return;
        }
        auto ticket = ticket_opt.value();
        handle_swapchain_state_change(m_frame_scheduler->swapchain_state());

        if (m_static_bindings_dirty) {
            m_resource_registries.clear_global();
            m_active_pipeline->bind_static_resources(m_resource_registries);
            m_static_bindings_dirty = false;
        }

        validate_perframe_binding_index(ticket.frame_index);
        if (m_perframe_bindings_dirty[ticket.frame_index]) {
            m_resource_registries.clear_frame(ticket.frame_index);
            m_active_pipeline->bind_frame_resources(ticket.frame_index, m_resource_registries);
            m_perframe_bindings_dirty[ticket.frame_index] = false;
        }

        m_imgui_renderer->begin_frame();
        m_imgui_renderer->build_dockspace();
        if (m_ui_callback) {
            m_ui_callback();
        }
        ImDrawData* imgui_draw_data = m_imgui_renderer->prepare_draw_data();

        FrameContext frame_ctx = build_frame_context(ticket);
        ticket.command_buffer->reset();
        ticket.command_buffer->record([&](rhi::CommandBuffer& cb) {
            m_active_pipeline->render(frame_ctx);
            record_imgui_overlay(frame_ctx, cb.command_buffer(), imgui_draw_data);
            transition_swapchain_to_present(cb.command_buffer(), frame_ctx.swapchain_image());
        }, vk::CommandBufferUsageFlagBits::eOneTimeSubmit);
        m_frame_scheduler->submit_and_present(ticket);
    }

    void on_window_resized(uint32_t width, uint32_t height) {
        m_frame_scheduler->on_window_resized(width, height);
        if (m_active_pipeline) {
            m_active_pipeline->on_resize(width, height);
        }
        invalidate_all_bindings();
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

    void handle_swapchain_state_change(const FrameScheduler::SwapchainState& state) {
        if (state.generation == m_last_swapchain_generation) {
            return;
        }

        m_imgui_renderer->on_swapchain_recreated(state.image_count, state.color_format, state.depth_format);
        if (m_active_pipeline) {
            m_active_pipeline->on_swapchain_state_changed(state);
        }
        invalidate_all_perframe_bindings();
        m_last_swapchain_generation = state.generation;
    }

    void record_imgui_overlay(
        const FrameContext& ctx,
        const vk::raii::CommandBuffer& command_buffer,
        ImDrawData* draw_data
    ) {
        if (draw_data == nullptr) {
            return;
        }

        vk::RenderingAttachmentInfo color_attachment_info{};
        color_attachment_info.imageView = *ctx.swapchain_image_view();
        color_attachment_info.imageLayout = vk::ImageLayout::eColorAttachmentOptimal;
        color_attachment_info.loadOp = vk::AttachmentLoadOp::eLoad;
        color_attachment_info.storeOp = vk::AttachmentStoreOp::eStore;

        vk::RenderingAttachmentInfo depth_attachment_info{};
        depth_attachment_info.imageView = *ctx.depth_image().image_view();
        depth_attachment_info.imageLayout = vk::ImageLayout::eDepthAttachmentOptimal;
        depth_attachment_info.loadOp = vk::AttachmentLoadOp::eLoad;
        depth_attachment_info.storeOp = vk::AttachmentStoreOp::eStore;

        vk::RenderingInfo rendering_info{};
        rendering_info.renderArea.offset = vk::Offset2D{0, 0};
        rendering_info.renderArea.extent = ctx.render_extent();
        rendering_info.layerCount = 1;
        rendering_info.colorAttachmentCount = 1;
        rendering_info.pColorAttachments = &color_attachment_info;
        rendering_info.pDepthAttachment = &depth_attachment_info;

        command_buffer.beginRendering(rendering_info);
        m_imgui_renderer->render_draw_data(command_buffer, draw_data);
        command_buffer.endRendering();
    }

    void transition_swapchain_to_present(const vk::raii::CommandBuffer& command_buffer, const vk::Image& swapchain_image) {
        vk::ImageMemoryBarrier2 to_present{};
        to_present.srcStageMask = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        to_present.dstStageMask = vk::PipelineStageFlagBits2::eBottomOfPipe;
        to_present.srcAccessMask = vk::AccessFlagBits2::eColorAttachmentWrite;
        to_present.dstAccessMask = vk::AccessFlagBits2::eNone;
        to_present.oldLayout = vk::ImageLayout::eColorAttachmentOptimal;
        to_present.newLayout = vk::ImageLayout::ePresentSrcKHR;
        to_present.image = swapchain_image;
        to_present.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
        to_present.subresourceRange.baseMipLevel = 0;
        to_present.subresourceRange.levelCount = 1;
        to_present.subresourceRange.baseArrayLayer = 0;
        to_present.subresourceRange.layerCount = 1;

        vk::DependencyInfo to_present_dep{};
        to_present_dep.imageMemoryBarrierCount = 1;
        to_present_dep.pImageMemoryBarriers = &to_present;
        command_buffer.pipelineBarrier2(to_present_dep);
    }

    void validate_perframe_binding_index(uint32_t frame_index) const {
        if (frame_index >= m_perframe_bindings_dirty.size()) {
            throw std::runtime_error(
                "Invalid per-frame dirty index: " + std::to_string(frame_index) +
                " (frames in flight: " + std::to_string(m_perframe_bindings_dirty.size()) + ")."
            );
        }
    }

    FrameContext build_frame_context(const FrameScheduler::FrameTicket& ticket) {
        return FrameContext(
            m_device.get(),
            ticket.command_buffer,
            &m_resource_registries,
            m_frame_scheduler->swapchain().image_views()[ticket.image_index],
            m_frame_scheduler->swapchain().images()[ticket.image_index],
            m_frame_scheduler->render_extent(),
            ticket.frame_index
        );
    }
};

} // namespace rtr::render
