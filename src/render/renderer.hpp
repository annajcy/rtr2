#pragma once

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rhi/context.hpp"
#include "rhi/device.hpp"
#include "rhi/window.hpp"
#include "render/frame_context.hpp"
#include "render/frame_scheduler.hpp"
#include "render/pipeline.hpp"

namespace rtr::render {

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
    std::unique_ptr<IRenderPipeline> m_active_pipeline{};

    rhi::Window::WindowResizeEvent::ActionHandle m_window_resize_handle{0};
    uint64_t m_last_swapchain_generation{0};

public:
    Renderer(
        int width,
        int height,
        std::string title,
        uint32_t max_frames_in_flight = 2
    ) {
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

    void set_pipeline(std::unique_ptr<IRenderPipeline> pipeline) {
        if (!pipeline) {
            throw std::runtime_error("set_pipeline received null pipeline.");
        }
        if (m_active_pipeline) {
            throw std::runtime_error("Renderer pipeline is immutable at runtime and cannot be replaced.");
        }
        m_active_pipeline = std::move(pipeline);
        m_active_pipeline->on_swapchain_state_changed(m_frame_scheduler->swapchain_state());
    }

    IRenderPipeline* pipeline() { return m_active_pipeline.get(); }
    const IRenderPipeline* pipeline() const { return m_active_pipeline.get(); }

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

        FrameContext frame_ctx = build_frame_context(ticket);
        ticket.command_buffer->reset();
        ticket.command_buffer->record([&](rhi::CommandBuffer& cb) {
            m_active_pipeline->render(frame_ctx);
            transition_swapchain_to_present(cb.command_buffer(), frame_ctx.swapchain_image());
        }, vk::CommandBufferUsageFlagBits::eOneTimeSubmit);

        m_frame_scheduler->submit_and_present(ticket);
    }

    void on_window_resized(uint32_t width, uint32_t height) {
        m_frame_scheduler->on_window_resized(width, height);
        if (m_active_pipeline) {
            m_active_pipeline->on_resize(static_cast<int>(width), static_cast<int>(height));
        }
    }

    const rhi::Device& device() const { return *m_device.get(); }
    const rhi::Context& context() const { return *m_context.get(); }
    const rhi::Window& window() const { return *m_window.get(); }
    rhi::Device& device() { return *m_device.get(); }
    rhi::Context& context() { return *m_context.get(); }
    rhi::Window& window() { return *m_window.get(); }

    const render::FrameScheduler& frame_scheduler() const { return *m_frame_scheduler.get(); }
    render::FrameScheduler& frame_scheduler() { return *m_frame_scheduler.get(); }

private:
    void handle_swapchain_state_change(const FrameScheduler::SwapchainState& state) {
        if (state.generation == m_last_swapchain_generation) {
            return;
        }

        if (m_active_pipeline) {
            m_active_pipeline->on_swapchain_state_changed(state);
        }
        m_last_swapchain_generation = state.generation;
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

    FrameContext build_frame_context(const FrameScheduler::FrameTicket& ticket) {
        return FrameContext(
            m_device.get(),
            ticket.command_buffer,
            m_frame_scheduler->swapchain().image_views()[ticket.image_index],
            m_frame_scheduler->swapchain().images()[ticket.image_index],
            m_frame_scheduler->render_extent(),
            ticket.frame_index
        );
    }
};

} // namespace rtr::render
