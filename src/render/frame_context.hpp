#pragma once

#include <cstdint>

#include "rhi/command.hpp"
#include "rhi/device.hpp"

namespace rtr::render {

class FrameContext {
private:
    rhi::Device* m_device{};
    rhi::CommandBuffer* m_cmd{};
    const vk::raii::ImageView* m_swapchain_image_view{};
    const vk::Image* m_swapchain_image{};
    vk::Extent2D m_render_extent{};
    uint32_t m_frame_index{0};

public:
    FrameContext(
        rhi::Device* device,
        rhi::CommandBuffer* cmd,
        const vk::raii::ImageView& swapchain_image_view,
        const vk::Image& swapchain_image,
        const vk::Extent2D& render_extent,
        uint32_t frame_index
    )
        : m_device(device),
          m_cmd(cmd),
          m_swapchain_image_view(&swapchain_image_view),
          m_swapchain_image(&swapchain_image),
          m_render_extent(render_extent),
          m_frame_index(frame_index) {}

    const rhi::CommandBuffer& cmd() const { return *m_cmd; }
    rhi::CommandBuffer& cmd() { return *m_cmd; }

    const vk::raii::ImageView& swapchain_image_view() const { return *m_swapchain_image_view; }
    const vk::Image& swapchain_image() const { return *m_swapchain_image; }
    const vk::Extent2D& render_extent() const { return m_render_extent; }
    uint32_t frame_index() const { return m_frame_index; }
    const rhi::Device& device() const { return *m_device; }
    rhi::Device& device() { return *m_device; }
};

} // namespace rtr::render
