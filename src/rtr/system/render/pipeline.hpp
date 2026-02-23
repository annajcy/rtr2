#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "rtr/resource/resource_fwd.hpp"
#include "rtr/system/render/frame_context.hpp"
#include "rtr/system/render/frame_scheduler.hpp"
#include "rtr/rhi/buffer.hpp"
#include "rtr/rhi/context.hpp"
#include "rtr/rhi/device.hpp"
#include "rtr/rhi/frame_constants.hpp"
#include "rtr/rhi/texture.hpp"
#include "rtr/rhi/window.hpp"
#include "vulkan/vulkan_enums.hpp"

namespace rtr::framework::core {
class World;
}

namespace rtr::system::input {
class InputSystem;
}

namespace rtr::system::render {

struct PipelineRuntime {
    rhi::Device& device;
    rhi::Context& context;
    rhi::Window& window;
    uint32_t image_count{0};
    vk::Format color_format{vk::Format::eUndefined};
    vk::Format depth_format{vk::Format::eUndefined};
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
    rhi::Device& m_device;
    rhi::Context& m_context;
    rhi::Window& m_window;
    uint32_t m_image_count{0};
    vk::Format m_color_format{vk::Format::eUndefined};
    vk::Format m_depth_format{vk::Format::eUndefined};
    vk::Extent2D m_swapchain_extent{};

public:
    explicit RenderPipelineBase(const PipelineRuntime& runtime)
        : m_device(runtime.device),
          m_context(runtime.context),
          m_window(runtime.window),
          m_image_count(runtime.image_count),
          m_color_format(runtime.color_format),
          m_depth_format(runtime.depth_format) {}

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

    template <typename TComponent, typename Factory, std::size_t... I>
    static std::array<TComponent, rhi::kFramesInFlight> make_frame_array_impl(Factory&& factory, std::index_sequence<I...>) {
        return {{factory(static_cast<uint32_t>(I))...}};
    }

    template <typename TComponent, typename Factory>
    static std::array<TComponent, rhi::kFramesInFlight> make_frame_array(Factory&& factory) {
        return make_frame_array_impl<TComponent>(std::forward<Factory>(factory), std::make_index_sequence<rhi::kFramesInFlight>{});
    }

    template <typename TComponent, std::size_t... I>
    static std::array<TComponent, rhi::kFramesInFlight> vector_to_frame_array_impl(
        std::vector<TComponent>&& values,
        std::index_sequence<I...>
    ) {
        return {{std::move(values[I])...}};
    }

    template <typename TComponent>
    static std::array<TComponent, rhi::kFramesInFlight> vector_to_frame_array(std::vector<TComponent>&& values, std::string_view label) {
        if (values.size() != rhi::kFramesInFlight) {
            throw std::runtime_error(std::string(label) + " size mismatch with kFramesInFlight.");
        }
        return vector_to_frame_array_impl<TComponent>(std::move(values), std::make_index_sequence<rhi::kFramesInFlight>{});
    }

    std::array<rhi::Buffer, rhi::kFramesInFlight> make_per_frame_mapped_uniform_buffers(
        vk::DeviceSize size,
        vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eUniformBuffer
    ) const {
        return make_frame_array<rhi::Buffer>([&](uint32_t) {
            auto buffer = rhi::Buffer::create_host_visible_buffer(m_device, size, usage);
            buffer.map();
            return buffer;
        });
    }

    std::array<rhi::Image, rhi::kFramesInFlight> make_per_frame_depth_images(
        const vk::Extent2D& extent,
        vk::Format depth_format
    ) const {
        return make_frame_array<rhi::Image>([&](uint32_t) {
            return rhi::Image::create_depth_image(
                m_device,
                extent.width,
                extent.height,
                depth_format
            );
        });
    }
};

} // namespace rtr::system::render
