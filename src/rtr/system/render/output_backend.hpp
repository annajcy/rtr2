#pragma once

#include <concepts>
#include <filesystem>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <fmt/format.h>

#include "rtr/rhi/image_readback.hpp"
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

struct OfflineImageOutputConfig {
    bool export_frames{true};
    std::filesystem::path output_dir{"output/frames"};
    std::string filename_pattern{"frame_{:06d}.png"};
    uint32_t starting_output_frame_index{0};
};

class OfflineImageOutputBackend final : public SwapchainFrameOutputBackendBase {
private:
    PresentPass m_present_pass{};
    OfflineImageOutputConfig m_config{};
    uint32_t m_next_output_frame_index{0};
    std::optional<rtr::rhi::detail::PendingImageReadback> m_pending_readback{};
    std::optional<std::filesystem::path> m_pending_output_path{};

public:
    explicit OfflineImageOutputBackend(
        RenderBackendServices services,
        OfflineImageOutputConfig config = {}
    )
        : SwapchainFrameOutputBackendBase(std::move(services)),
          m_config(std::move(config)),
          m_next_output_frame_index(m_config.starting_output_frame_index) {}

    std::optional<RenderFrameTicket> begin_frame() {
        return begin_swapchain_frame();
    }

    void record_output(RenderPipeline& pipeline, FrameContext& frame_ctx) {
        auto final_output = pipeline.final_output(frame_ctx.frame_index());

        if (frame_ctx.has_output_target()) {
            m_present_pass.execute(
                frame_ctx,
                PresentPass::RenderPassResources{
                    .src_color = final_output.color,
                    .src_extent = final_output.extent
                }
            );
        }

        if (m_config.export_frames) {
            const auto output_path = m_config.output_dir /
                                     fmt::format(fmt::runtime(m_config.filename_pattern), m_next_output_frame_index);
            m_pending_readback = rhi::detail::record_readback_copy(
                m_services.device,
                frame_ctx.cmd(),
                rhi::ReadbackImageDesc{
                    .image = *final_output.color.image.image(),
                    .format = final_output.color.image.format(),
                    .extent = final_output.extent,
                    .current_layout = final_output.color.layout
                }
            );
            m_pending_output_path = output_path;
            ++m_next_output_frame_index;
        }

        if (frame_ctx.has_output_target()) {
            transition_output_target_to_present(frame_ctx.cmd().command_buffer(), frame_ctx.output_target());
        }
    }

    void end_frame(const RenderFrameTicket& ticket) {
        end_swapchain_frame(ticket);
        if (m_config.export_frames) {
            wait_for_frame_completion(ticket.frame_index);
            flush_pending_readback();
        }
    }

private:
    void wait_for_frame_completion(uint32_t frame_index) {
        const auto& per_frame = m_services.frame_scheduler.per_frame_resources();
        if (frame_index >= per_frame.size()) {
            throw std::runtime_error("OfflineImageOutputBackend frame index out of range.");
        }
        const vk::Result result = m_services.device.device().waitForFences(
            *per_frame[frame_index].in_flight_fence,
            VK_TRUE,
            UINT64_MAX
        );
        if (result != vk::Result::eSuccess) {
            throw std::runtime_error("OfflineImageOutputBackend failed waiting for frame fence.");
        }
    }

    void flush_pending_readback() {
        if (!m_pending_readback.has_value() || !m_pending_output_path.has_value()) {
            return;
        }

        auto image = rhi::detail::finalize_readback_copy(std::move(*m_pending_readback));
        rhi::save_png(*m_pending_output_path, image);
        m_pending_readback.reset();
        m_pending_output_path.reset();
    }
};

}  // namespace rtr::system::render
