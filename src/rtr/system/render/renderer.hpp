#pragma once

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rtr/rhi/command.hpp"
#include "rtr/rhi/context.hpp"
#include "rtr/rhi/device.hpp"
#include "rtr/rhi/window.hpp"
#include "rtr/system/render/frame_context.hpp"
#include "rtr/system/render/frame_scheduler.hpp"
#include "rtr/system/render/output_backend.hpp"
#include "rtr/system/render/render_pipeline.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::system::render {

class IRenderer {
public:
    virtual ~IRenderer() = default;

    virtual PipelineRuntime build_pipeline_runtime() = 0;
    virtual RenderPipeline* pipeline() = 0;
    virtual const RenderPipeline* pipeline() const = 0;
    virtual void draw_frame() = 0;

    virtual const rhi::Device& device() const = 0;
    virtual const rhi::Context& context() const = 0;
    virtual const rhi::Window& window() const = 0;
    virtual const FrameScheduler& frame_scheduler() const = 0;

    virtual rhi::Device& device() = 0;
    virtual rhi::Context& context() = 0;
    virtual rhi::Window& window() = 0;
    virtual FrameScheduler& frame_scheduler() = 0;
};

template <typename TBackend>
class RendererT : public IRenderer {
    static_assert(RenderOutputBackendConcept<TBackend>, "RendererT requires a valid render output backend.");

public:
    using PerFrameResources       = FrameScheduler::PerFrameResources;
    using PerImageResources       = FrameScheduler::PerImageResources;
    using RenderCallback          = std::function<void(FrameContext&)>;
    using ComputeRecordCallback   = std::function<void(rhi::CommandBuffer&)>;
    using ComputeCompleteCallback = std::function<void()>;

    class ComputeJob {
    private:
        rhi::Device& m_device;
        rhi::CommandBuffer m_command_buffer;
        vk::raii::Fence m_fence{nullptr};
        mutable ComputeCompleteCallback m_on_complete{};
        mutable bool m_on_complete_invoked{false};

        ComputeJob(
            rhi::Device& device,
            rhi::CommandBuffer&& command_buffer,
            vk::raii::Fence&& fence,
            ComputeCompleteCallback&& on_complete
        )
            : m_device(device),
              m_command_buffer(std::move(command_buffer)),
              m_fence(std::move(fence)),
              m_on_complete(std::move(on_complete)) {}

        friend class RendererT<TBackend>;

    public:
        ComputeJob() = delete;
        ~ComputeJob() noexcept {
            if (!valid()) {
                return;
            }
            try {
                wait();
            } catch (...) {
            }
        }

        ComputeJob(const ComputeJob&) = delete;
        ComputeJob& operator=(const ComputeJob&) = delete;
        ComputeJob(ComputeJob&&) noexcept = default;
        ComputeJob& operator=(ComputeJob&&) noexcept = delete;

        bool valid() const { return static_cast<vk::Fence>(m_fence) != vk::Fence{}; }

        void set_on_complete(ComputeCompleteCallback on_complete) {
            m_on_complete = std::move(on_complete);
            m_on_complete_invoked = false;
        }

        bool is_done() const {
            if (!valid()) {
                return false;
            }
            const vk::Result result = m_device.device().waitForFences(*m_fence, VK_TRUE, 0);
            if (result == vk::Result::eSuccess) {
                invoke_on_complete_if_needed();
                return true;
            }
            if (result == vk::Result::eTimeout) {
                return false;
            }
            throw std::runtime_error("ComputeJob status query failed.");
        }

        void wait(uint64_t timeout_ns = UINT64_MAX) const {
            if (!valid()) {
                throw std::runtime_error("ComputeJob is invalid.");
            }

            const vk::Result result = m_device.device().waitForFences(*m_fence, VK_TRUE, timeout_ns);
            if (result == vk::Result::eSuccess) {
                invoke_on_complete_if_needed();
                return;
            }
            if (result == vk::Result::eTimeout) {
                throw std::runtime_error("ComputeJob wait timed out.");
            }
            throw std::runtime_error("ComputeJob wait failed.");
        }

    private:
        void invoke_on_complete_if_needed() const {
            if (m_on_complete_invoked || !m_on_complete) {
                return;
            }
            m_on_complete_invoked = true;
            m_on_complete();
        }
    };

private:
    static rhi::ContextCreateInfo make_context_create_info(rhi::Window& window) {
        rhi::ContextCreateInfo context_info{};
        context_info.app_name = window.title();
        context_info.instance_extensions = window.required_extensions();
        context_info.surface_creator = [&window](const vk::raii::Instance& instance) {
            return window.create_vk_surface(instance);
        };
        return context_info;
    }

    static std::filesystem::path resolve_shader_root_dir() {
        if (const char* env_root = std::getenv("RTR_SHADER_ROOT"); env_root != nullptr && env_root[0] != '\0') {
            return std::filesystem::path(env_root);
        }
#ifdef RTR_DEFAULT_SHADER_OUTPUT_DIR
        return std::filesystem::path(RTR_DEFAULT_SHADER_OUTPUT_DIR);
#else
        return {};
#endif
    }

    rhi::Window m_window;
    rhi::Context m_context;
    rhi::Device m_device;

    rhi::CommandPool m_compute_command_pool;
    FrameScheduler m_frame_scheduler;
    std::unique_ptr<RenderPipeline> m_active_pipeline{};
    TBackend m_output_backend;

    utils::SubscriptionToken m_window_resize_subscription{};
    uint64_t m_last_swapchain_generation{0};

public:
    RendererT(int width, int height, std::string title)
        : m_window(width, height, title),
          m_context(make_context_create_info(m_window)),
          m_device(m_context),
          m_compute_command_pool(
              m_device,
              vk::CommandPoolCreateFlagBits::eTransient | vk::CommandPoolCreateFlagBits::eResetCommandBuffer
          ),
          m_frame_scheduler(m_window, m_context, m_device),
          m_output_backend(build_backend_services()) {
        m_window_resize_subscription =
            m_window.window_resize_event().subscribe([this](int resize_width, int resize_height) {
                on_window_resized(static_cast<uint32_t>(resize_width), static_cast<uint32_t>(resize_height));
            });
        m_last_swapchain_generation = m_frame_scheduler.swapchain_state().generation;
    }

    ~RendererT() noexcept override {
        try {
            m_device.wait_idle();
        } catch (const std::exception& ex) {
            try {
                auto logger = utils::get_logger("system.render.renderer");
                logger->error("Renderer::~Renderer wait_idle failed: {}", ex.what());
            } catch (...) {
            }
        } catch (...) {
            try {
                auto logger = utils::get_logger("system.render.renderer");
                logger->error("Renderer::~Renderer wait_idle failed with unknown exception.");
            } catch (...) {
            }
        }
    }

    RendererT(const RendererT&) = delete;
    RendererT& operator=(const RendererT&) = delete;

    PipelineRuntime build_pipeline_runtime() override {
        return PipelineRuntime{
            .device = m_device,
            .context = m_context,
            .window = m_window,
            .image_count = m_frame_scheduler.image_count(),
            .color_format = m_frame_scheduler.render_format(),
            .depth_format = m_frame_scheduler.depth_format(),
            .shader_root_dir = resolve_shader_root_dir()
        };
    }

    void set_pipeline(std::unique_ptr<RenderPipeline> pipeline) {
        if (!pipeline) {
            throw std::runtime_error("set_pipeline received null pipeline.");
        }
        if (m_active_pipeline) {
            throw std::runtime_error("Renderer pipeline is immutable at runtime and cannot be replaced.");
        }
        m_active_pipeline = std::move(pipeline);
        m_active_pipeline->on_swapchain_state_changed(m_frame_scheduler.swapchain_state());
    }

    TBackend& output_backend() { return m_output_backend; }
    const TBackend& output_backend() const { return m_output_backend; }

    RenderPipeline* pipeline() override { return m_active_pipeline.get(); }
    const RenderPipeline* pipeline() const override { return m_active_pipeline.get(); }

    void compute(const ComputeRecordCallback& record, ComputeCompleteCallback on_complete = {}) {
        auto job = compute_async(record, std::move(on_complete));
        job.wait();
    }

    ComputeJob compute_async(const ComputeRecordCallback& record, ComputeCompleteCallback on_complete = {}) {
        if (!record) {
            throw std::runtime_error("compute_async received empty callback.");
        }

        auto command_buffer = m_compute_command_pool.create_command_buffer();
        command_buffer.record(
            [&](rhi::CommandBuffer& cb) { record(cb); },
            vk::CommandBufferUsageFlagBits::eOneTimeSubmit
        );

        vk::raii::Fence fence(m_device.device(), vk::FenceCreateInfo{});
        rhi::CommandBuffer::SubmitInfo submit_info{};
        submit_info.fence = *fence;
        command_buffer.submit(submit_info);

        return ComputeJob(m_device, std::move(command_buffer), std::move(fence), std::move(on_complete));
    }

    void draw_frame() override {
        if (!m_active_pipeline) {
            throw std::runtime_error("No active pipeline. Call set_pipeline(...) before draw_frame().");
        }

        auto ticket_opt = m_output_backend.begin_frame();
        if (!ticket_opt.has_value()) {
            return;
        }

        const auto ticket = ticket_opt.value();
        FrameContext frame_ctx = build_frame_context(ticket);
        ticket.command_buffer->reset();
        ticket.command_buffer->record(
            [&](rhi::CommandBuffer& cb) {
                (void)cb;
                m_active_pipeline->render(frame_ctx);
                m_output_backend.record_output(*m_active_pipeline, frame_ctx);
            },
            vk::CommandBufferUsageFlagBits::eOneTimeSubmit
        );
        m_output_backend.end_frame(ticket);
    }

    void on_window_resized(uint32_t width, uint32_t height) {
        m_frame_scheduler.on_window_resized(width, height);
        if (m_active_pipeline) {
            m_active_pipeline->on_resize(static_cast<int>(width), static_cast<int>(height));
        }
    }

    const rhi::Device& device() const override { return m_device; }
    const rhi::Context& context() const override { return m_context; }
    const rhi::Window& window() const override { return m_window; }
    const FrameScheduler& frame_scheduler() const override { return m_frame_scheduler; }
    rhi::Device& device() override { return m_device; }
    rhi::Context& context() override { return m_context; }
    rhi::Window& window() override { return m_window; }
    FrameScheduler& frame_scheduler() override { return m_frame_scheduler; }

private:
    RenderBackendServices build_backend_services() {
        return RenderBackendServices{
            .device = m_device,
            .context = m_context,
            .window = m_window,
            .frame_scheduler = m_frame_scheduler,
            .on_swapchain_state_change = [this](const FrameScheduler::SwapchainState& state) {
                handle_swapchain_state_change(state);
            }
        };
    }

    void handle_swapchain_state_change(const FrameScheduler::SwapchainState& state) {
        if (state.generation == m_last_swapchain_generation) {
            return;
        }

        if (m_active_pipeline) {
            m_active_pipeline->on_swapchain_state_changed(state);
        }
        m_last_swapchain_generation = state.generation;
    }

    FrameContext build_frame_context(const RenderFrameTicket& ticket) {
        return FrameContext(
            &m_device,
            ticket.command_buffer,
            ticket.render_extent,
            ticket.frame_index,
            ticket.output_target
        );
    }
};

using Renderer = RendererT<SwapchainOutputBackend>;

}  // namespace rtr::system::render
