#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>

#include "rtr/rhi/command.hpp"
#include "rtr/rhi/device.hpp"

namespace rtr::system::render {

struct RenderOutputTarget {
    const vk::raii::ImageView* image_view{};
    const vk::Image* image{};
    vk::ImageLayout expected_layout{vk::ImageLayout::eUndefined};
    vk::Extent2D extent{};

    bool valid() const {
        return image_view != nullptr && image != nullptr &&
               extent.width > 0 && extent.height > 0;
    }
};

class FrameContext {
private:
    rhi::Device* m_device{};
    rhi::CommandBuffer* m_cmd{};
    std::optional<RenderOutputTarget> m_output_target{};
    vk::Extent2D m_render_extent{};
    uint32_t m_frame_index{0};

public:
    FrameContext(
        rhi::Device* device,
        rhi::CommandBuffer* cmd,
        const vk::Extent2D& render_extent,
        uint32_t frame_index,
        std::optional<RenderOutputTarget> output_target = std::nullopt
    )
        : m_device(device),
          m_cmd(cmd),
          m_output_target(std::move(output_target)),
          m_render_extent(render_extent),
          m_frame_index(frame_index) {}

    const rhi::CommandBuffer& cmd() const { return *m_cmd; }
    rhi::CommandBuffer& cmd() { return *m_cmd; }

    const vk::Extent2D& render_extent() const { return m_render_extent; }
    uint32_t frame_index() const { return m_frame_index; }
    const rhi::Device& device() const { return *m_device; }
    rhi::Device& device() { return *m_device; }

    bool has_output_target() const {
        return m_output_target.has_value() && m_output_target->valid();
    }

    const RenderOutputTarget& output_target() const {
        if (!has_output_target()) {
            throw std::runtime_error("FrameContext output target is not available.");
        }
        return *m_output_target;
    }

    RenderOutputTarget& output_target() {
        if (!has_output_target()) {
            throw std::runtime_error("FrameContext output target is not available.");
        }
        return *m_output_target;
    }

    // Compatibility accessors for code paths that still treat the output target
    // like a swapchain attachment. New code should prefer output_target().
    const vk::raii::ImageView& swapchain_image_view() const { return *output_target().image_view; }
    const vk::Image& swapchain_image() const { return *output_target().image; }
};

} // namespace rtr::system::render
