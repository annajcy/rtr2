#pragma once

#include <concepts>
#include <functional>
#include <optional>
#include <stdexcept>
#include <utility>

#include "rtr/rhi/context.hpp"
#include "rtr/rhi/window.hpp"
#include "rtr/system/render/frame_context.hpp"
#include "rtr/system/render/frame_scheduler.hpp"
#include "rtr/system/render/pass/present_pass.hpp"
#include "rtr/system/render/render_pipeline.hpp"

namespace rtr::system::render {

struct RenderFrameTicket {
    uint32_t frame_index{0};
    uint32_t image_index{0};
    rhi::CommandBuffer* command_buffer{};
    vk::Extent2D render_extent{};
    std::optional<RenderOutputTarget> output_target{};
};

template <typename T>
concept RenderOutputBackendConcept = requires(
    T& backend,
    RenderPipeline& pipeline,
    FrameContext& frame_ctx,
    const RenderFrameTicket& ticket
) {
    { backend.begin_frame() } -> std::same_as<std::optional<RenderFrameTicket>>;
    { backend.record_output(pipeline, frame_ctx) } -> std::same_as<void>;
    { backend.end_frame(ticket) } -> std::same_as<void>;
};

struct RenderBackendServices {
    rhi::Device& device;
    rhi::Context& context;
    rhi::Window& window;
    FrameScheduler& frame_scheduler;
    std::function<void(const FrameScheduler::SwapchainState&)> on_swapchain_state_change;
};

class SwapchainFrameOutputBackendBase {
protected:
    RenderBackendServices m_services;

    explicit SwapchainFrameOutputBackendBase(RenderBackendServices services)
        : m_services(std::move(services)) {}

    std::optional<RenderFrameTicket> begin_swapchain_frame() {
        auto ticket_opt = m_services.frame_scheduler.begin_frame();
        if (!ticket_opt.has_value()) {
            return std::nullopt;
        }

        const auto& scheduler = m_services.frame_scheduler;
        m_services.on_swapchain_state_change(scheduler.swapchain_state());

        auto ticket = ticket_opt.value();
        return RenderFrameTicket{
            .frame_index = ticket.frame_index,
            .image_index = ticket.image_index,
            .command_buffer = &ticket.command_buffer,
            .render_extent = scheduler.render_extent(),
            .output_target = RenderOutputTarget{
                .image_view = &scheduler.swapchain().image_views()[ticket.image_index],
                .image = &scheduler.swapchain().images()[ticket.image_index],
                .expected_layout = vk::ImageLayout::ePresentSrcKHR,
                .extent = scheduler.render_extent()
            }
        };
    }

    void end_swapchain_frame(const RenderFrameTicket& ticket) {
        if (ticket.command_buffer == nullptr) {
            throw std::runtime_error("Swapchain frame ticket command buffer is null.");
        }
        FrameScheduler::FrameTicket scheduler_ticket{
            .frame_index = ticket.frame_index,
            .image_index = ticket.image_index,
            .command_buffer = *ticket.command_buffer
        };
        m_services.frame_scheduler.submit_and_present(scheduler_ticket);
    }

    static void transition_output_target_to_present(
        const vk::raii::CommandBuffer& command_buffer,
        const RenderOutputTarget& output_target
    ) {
        vk::ImageMemoryBarrier2 to_present{};
        to_present.srcStageMask = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        to_present.dstStageMask = vk::PipelineStageFlagBits2::eBottomOfPipe;
        to_present.srcAccessMask = vk::AccessFlagBits2::eColorAttachmentWrite;
        to_present.dstAccessMask = vk::AccessFlagBits2::eNone;
        to_present.oldLayout = vk::ImageLayout::eColorAttachmentOptimal;
        to_present.newLayout = vk::ImageLayout::ePresentSrcKHR;
        to_present.image = *output_target.image;
        to_present.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
        to_present.subresourceRange.baseMipLevel = 0;
        to_present.subresourceRange.levelCount = 1;
        to_present.subresourceRange.baseArrayLayer = 0;
        to_present.subresourceRange.layerCount = 1;

        vk::DependencyInfo dependency{};
        dependency.imageMemoryBarrierCount = 1;
        dependency.pImageMemoryBarriers = &to_present;
        command_buffer.pipelineBarrier2(dependency);
    }
};

class SwapchainOutputBackend final : public SwapchainFrameOutputBackendBase {
private:
    PresentPass m_present_pass{};

public:
    explicit SwapchainOutputBackend(RenderBackendServices services)
        : SwapchainFrameOutputBackendBase(std::move(services)) {}

    std::optional<RenderFrameTicket> begin_frame() {
        return begin_swapchain_frame();
    }

    void record_output(RenderPipeline& pipeline, FrameContext& frame_ctx) {
        auto final_output = pipeline.final_output(frame_ctx.frame_index());
        m_present_pass.execute(
            frame_ctx,
            PresentPass::RenderPassResources{
                .src_color = final_output.color,
                .src_extent = final_output.extent
            }
        );
        transition_output_target_to_present(frame_ctx.cmd().command_buffer(), frame_ctx.output_target());
    }

    void end_frame(const RenderFrameTicket& ticket) {
        end_swapchain_frame(ticket);
    }
};

class OfflineImageOutputBackend final {
public:
    explicit OfflineImageOutputBackend(RenderBackendServices /*services*/) {}

    std::optional<RenderFrameTicket> begin_frame() {
        throw std::logic_error("OfflineImageOutputBackend is not implemented in step2.");
    }

    void record_output(RenderPipeline&, FrameContext&) {
        throw std::logic_error("OfflineImageOutputBackend is not implemented in step2.");
    }

    void end_frame(const RenderFrameTicket&) {
        throw std::logic_error("OfflineImageOutputBackend is not implemented in step2.");
    }
};

}  // namespace rtr::system::render
