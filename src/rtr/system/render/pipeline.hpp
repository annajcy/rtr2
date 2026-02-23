#pragma once

#include <memory>
#include <stdexcept>
#include <vector>

#include "rtr/resource/resource_fwd.hpp"
#include "rtr/system/render/frame_context.hpp"
#include "rtr/system/render/frame_scheduler.hpp"
#include "rtr/rhi/buffer.hpp"
#include "rtr/rhi/context.hpp"
#include "rtr/rhi/device.hpp"
#include "rtr/rhi/texture.hpp"
#include "rtr/rhi/window.hpp"
#include "vulkan/vulkan_enums.hpp"
#include <cstdint>

namespace rtr::framework::core {
class World;
}

namespace rtr::system::input {
class InputSystem;
}

namespace rtr::system::render {

struct PipelineRuntime {
    rhi::Device* device{};
    rhi::Context* context{};
    rhi::Window* window{};
    uint32_t frame_count{0};
    uint32_t image_count{0};
    vk::Format color_format{vk::Format::eUndefined};
    vk::Format depth_format{vk::Format::eUndefined};

    bool is_valid() const {
        return device != nullptr && context != nullptr && window != nullptr;
    }
};

class IRenderPipeline {
public:
    virtual ~IRenderPipeline() = default;

    virtual void on_resize(int width, int height) {}
    virtual void on_swapchain_state_changed(const FrameScheduler::SwapchainState& state) {}

    // Renderer owns command buffer begin/end/reset/submit; pipeline only records draw commands.
    virtual void render(FrameContext& ctx) = 0;
};

struct FramePrepareContext {
    framework::core::World& world;
    resource::ResourceManager& resources;
    system::input::InputSystem& input;
    std::uint64_t frame_serial{0};
    double delta_seconds{0.0};
};

class IFramePreparePipeline {
public:
    virtual ~IFramePreparePipeline() = default;
    virtual void prepare_frame(const FramePrepareContext& ctx) = 0;
};

struct SwapchainChangeSummary {
    bool extent_changed{false};
    bool image_count_changed{false};
    bool color_format_changed{false};
    bool depth_format_changed{false};

    bool extent_or_depth_changed() const {
        return extent_changed || depth_format_changed;
    }

    bool color_or_depth_changed() const {
        return color_format_changed || depth_format_changed;
    }
};

class RenderPipelineBase : public IRenderPipeline {
protected:
    rhi::Device* m_device{};
    rhi::Context* m_context{};
    rhi::Window* m_window{};
    uint32_t m_frame_count{0};
    uint32_t m_image_count{0};
    vk::Format m_color_format{vk::Format::eUndefined};
    vk::Format m_depth_format{vk::Format::eUndefined};
    vk::Extent2D m_swapchain_extent{};

public:
    explicit RenderPipelineBase(const PipelineRuntime& runtime)
        : m_device(runtime.device),
          m_context(runtime.context),
          m_window(runtime.window),
          m_frame_count(runtime.frame_count),
          m_image_count(runtime.image_count),
          m_color_format(runtime.color_format),
          m_depth_format(runtime.depth_format) {
        if (!runtime.is_valid()) {
            throw std::runtime_error("RenderPipelineBase requires valid device/context/window.");
        }
    }

    void on_swapchain_state_changed(const FrameScheduler::SwapchainState& state) final {
        SwapchainChangeSummary diff{
            .extent_changed =
                (m_swapchain_extent.width != state.extent.width) ||
                (m_swapchain_extent.height != state.extent.height),
            .image_count_changed = (m_image_count != state.image_count),
            .color_format_changed = (m_color_format != state.color_format),
            .depth_format_changed = (m_depth_format != state.depth_format)
        };

        m_swapchain_extent = state.extent;
        m_image_count = state.image_count;
        m_color_format = state.color_format;
        m_depth_format = state.depth_format;

        handle_swapchain_state_change(state, diff);
    }

protected:
    virtual void handle_swapchain_state_change(
        const FrameScheduler::SwapchainState& state,
        const SwapchainChangeSummary& diff
    ) = 0;

    bool has_valid_extent() const {
        return m_swapchain_extent.width > 0 && m_swapchain_extent.height > 0;
    }

    std::vector<std::unique_ptr<rhi::Buffer>> make_per_frame_mapped_uniform_buffers(
        vk::DeviceSize size,
        vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eUniformBuffer
    ) const {
        std::vector<std::unique_ptr<rhi::Buffer>> buffers;
        buffers.reserve(m_frame_count);
        for (uint32_t i = 0; i < m_frame_count; ++i) {
            auto buffer = std::make_unique<rhi::Buffer>(
                rhi::Buffer::create_host_visible_buffer(
                    m_device,
                    size,
                    usage
                )
            );
            buffer->map();
            buffers.emplace_back(std::move(buffer));
        }
        return buffers;
    }

    std::vector<std::unique_ptr<rhi::Image>> make_per_frame_depth_images(
        const vk::Extent2D& extent,
        vk::Format depth_format
    ) const {
        std::vector<std::unique_ptr<rhi::Image>> depth_images;
        depth_images.reserve(m_frame_count);
        for (uint32_t i = 0; i < m_frame_count; ++i) {
            depth_images.emplace_back(std::make_unique<rhi::Image>(
                rhi::Image::create_depth_image(
                    m_device,
                    extent.width,
                    extent.height,
                    depth_format
                )
            ));
        }
        return depth_images;
    }
};

} // namespace rtr::system::render
