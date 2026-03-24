#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <utility>

#include "rtr/editor/core/editor_capture.hpp"
#include "rtr/editor/core/editor_host.hpp"
#include "rtr/editor/render/editor_imgui_pass.hpp"
#include "rtr/system/render/output_backend.hpp"

namespace rtr::editor::render {

class EditorOutputBackend final : public system::render::SwapchainFrameOutputBackendBase,
                                  public IEditorInputCaptureSource {
private:
    system::render::PipelineRuntime m_runtime;
    std::shared_ptr<EditorHost> m_editor_host{};
    std::unique_ptr<EditorImGuiPass> m_editor_pass{};
    std::uint64_t m_last_swapchain_generation{0};

public:
    explicit EditorOutputBackend(system::render::RenderBackendServices services)
        : system::render::SwapchainFrameOutputBackendBase(services),
          m_runtime{
              .device = services.device,
              .context = services.context,
              .window = services.window,
              .image_count = services.frame_scheduler.image_count(),
              .color_format = services.frame_scheduler.render_format(),
              .depth_format = services.frame_scheduler.depth_format(),
              .shader_root_dir = {}
          },
          m_last_swapchain_generation(services.frame_scheduler.swapchain_state().generation) {}

    void bind_editor_host(std::shared_ptr<EditorHost> editor_host) {
        m_editor_host = std::move(editor_host);
        if (!m_editor_host) {
            m_editor_pass.reset();
            return;
        }
        m_editor_pass = std::make_unique<EditorImGuiPass>(m_runtime, m_editor_host);
    }

    bool has_editor_host() const {
        return static_cast<bool>(m_editor_pass);
    }

    std::optional<system::render::RenderFrameTicket> begin_frame() {
        if (!m_editor_pass) {
            throw std::runtime_error("EditorOutputBackend requires bind_editor_host(...) before rendering.");
        }
        auto ticket = begin_swapchain_frame();
        if (!ticket.has_value()) {
            return std::nullopt;
        }

        const auto state = m_services.frame_scheduler.swapchain_state();
        m_runtime.image_count = state.image_count;
        m_runtime.color_format = state.color_format;
        m_runtime.depth_format = state.depth_format;
        if (state.generation != m_last_swapchain_generation) {
            m_editor_pass->on_swapchain_recreated(state.image_count, state.color_format, state.depth_format);
            m_last_swapchain_generation = state.generation;
        }
        return ticket;
    }

    void record_output(system::render::RenderPipeline& pipeline, system::render::FrameContext& frame_ctx) {
        if (!m_editor_pass) {
            throw std::runtime_error("EditorOutputBackend requires bind_editor_host(...) before rendering.");
        }

        auto final_output = pipeline.final_output(frame_ctx.frame_index());
        auto& cmd = frame_ctx.cmd().command_buffer();
        auto& scene_image = final_output.color.image;
        auto& scene_layout = final_output.color.layout;

        vk::PipelineStageFlags2 src_stage = vk::PipelineStageFlagBits2::eTopOfPipe;
        vk::AccessFlags2 src_access = vk::AccessFlagBits2::eNone;
        if (scene_layout == vk::ImageLayout::eColorAttachmentOptimal) {
            src_stage = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
            src_access = vk::AccessFlagBits2::eColorAttachmentWrite;
        } else if (scene_layout == vk::ImageLayout::eGeneral) {
            src_stage = vk::PipelineStageFlagBits2::eComputeShader;
            src_access = vk::AccessFlagBits2::eShaderStorageWrite;
        } else if (scene_layout == vk::ImageLayout::eTransferDstOptimal) {
            src_stage = vk::PipelineStageFlagBits2::eTransfer;
            src_access = vk::AccessFlagBits2::eTransferWrite;
        }

        vk::ImageMemoryBarrier2 scene_to_sampled{};
        scene_to_sampled.srcStageMask = src_stage;
        scene_to_sampled.dstStageMask = vk::PipelineStageFlagBits2::eFragmentShader;
        scene_to_sampled.srcAccessMask = src_access;
        scene_to_sampled.dstAccessMask = vk::AccessFlagBits2::eShaderRead;
        scene_to_sampled.oldLayout = scene_layout;
        scene_to_sampled.newLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
        scene_to_sampled.image = *scene_image.image();
        scene_to_sampled.subresourceRange = {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};

        vk::ImageMemoryBarrier2 output_to_color{};
        output_to_color.srcStageMask = vk::PipelineStageFlagBits2::eTopOfPipe;
        output_to_color.dstStageMask = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
        output_to_color.srcAccessMask = vk::AccessFlagBits2::eNone;
        output_to_color.dstAccessMask = vk::AccessFlagBits2::eColorAttachmentWrite;
        output_to_color.oldLayout = vk::ImageLayout::eUndefined;
        output_to_color.newLayout = vk::ImageLayout::eColorAttachmentOptimal;
        output_to_color.image = *frame_ctx.output_target().image;
        output_to_color.subresourceRange = {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};

        std::array<vk::ImageMemoryBarrier2, 2> barriers{scene_to_sampled, output_to_color};
        vk::DependencyInfo dependency{};
        dependency.imageMemoryBarrierCount = static_cast<uint32_t>(barriers.size());
        dependency.pImageMemoryBarriers = barriers.data();
        cmd.pipelineBarrier2(dependency);

        scene_layout = vk::ImageLayout::eShaderReadOnlyOptimal;
        m_editor_pass->set_scene_viewport_size_callback(
            [&pipeline](std::uint32_t width, std::uint32_t height) {
                pipeline.publish_event<system::render::SceneViewportResizeEvent>(
                    system::render::SceneViewportResizeEvent{.width = width, .height = height}
                );
            }
        );
        m_editor_pass->execute(
            frame_ctx,
            EditorImGuiPass::RenderPassResources{
                .scene_image_view = *scene_image.image_view(),
                .scene_image_layout = scene_layout,
                .scene_extent = final_output.extent
            }
        );
        transition_output_target_to_present(cmd, frame_ctx.output_target());
    }

    void end_frame(const system::render::RenderFrameTicket& ticket) {
        end_swapchain_frame(ticket);
    }

    bool wants_imgui_capture_mouse() const override {
        return m_editor_pass ? m_editor_pass->wants_capture_mouse() : false;
    }

    bool wants_imgui_capture_keyboard() const override {
        return m_editor_pass ? m_editor_pass->wants_capture_keyboard() : false;
    }
};

}  // namespace rtr::editor::render
