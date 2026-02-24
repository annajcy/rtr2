#pragma once

#include <array>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "rtr/resource/resource_fwd.hpp"
#include "rtr/rhi/buffer.hpp"
#include "rtr/rhi/context.hpp"
#include "rtr/rhi/device.hpp"
#include "rtr/rhi/frame_constants.hpp"
#include "rtr/rhi/texture.hpp"
#include "rtr/rhi/window.hpp"
#include "rtr/system/render/frame_context.hpp"
#include "rtr/system/render/frame_scheduler.hpp"
#include "rtr/utils/event_center.hpp"
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
    std::filesystem::path shader_root_dir{};
};

struct FramePrepareContext {
    framework::core::World& world;
    resource::ResourceManager& resources;
    system::input::InputSystem& input;
    std::uint64_t frame_serial{0};
    double delta_seconds{0.0};
};

struct SceneViewportResizeEvent {
    std::uint32_t width{0};
    std::uint32_t height{0};
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

class RenderPipeline {
protected:
    rhi::Device& m_device;
    rhi::Context& m_context;
    rhi::Window& m_window;
    uint32_t m_image_count{0};
    vk::Format m_color_format{vk::Format::eUndefined};
    vk::Format m_depth_format{vk::Format::eUndefined};
    vk::Extent2D m_swapchain_extent{};

private:
    utils::TypedEventCenter m_events{};

public:
    explicit RenderPipeline(const PipelineRuntime& runtime)
        : m_device(runtime.device),
          m_context(runtime.context),
          m_window(runtime.window),
          m_image_count(runtime.image_count),
          m_color_format(runtime.color_format),
          m_depth_format(runtime.depth_format) {}

    virtual ~RenderPipeline() = default;

    virtual void render(FrameContext& ctx) = 0;
    virtual void prepare_frame(const FramePrepareContext& ctx) {
        (void)ctx;
    }

    virtual void on_resize(int width, int height) {
        (void)width;
        (void)height;
    }

    virtual void wait_for_scene_target_rebuild() {
        m_device.wait_idle();
    }

    void on_swapchain_state_changed(const FrameScheduler::SwapchainState& state) {
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

    template <typename TEvent>
    utils::SubscriptionToken subscribe_event(std::function<void(const TEvent&)> action) {
        return m_events.subscribe<TEvent>(std::move(action));
    }

    template <typename TEvent>
    void publish_event(const TEvent& event) {
        m_events.publish<TEvent>(event);
    }

protected:
    static std::filesystem::path resolve_shader_root_dir(const PipelineRuntime& runtime) {
        if (!runtime.shader_root_dir.empty()) {
            return runtime.shader_root_dir;
        }
        if (const char* env_root = std::getenv("RTR_SHADER_ROOT");
            env_root != nullptr && env_root[0] != '\0') {
            return std::filesystem::path(env_root);
        }
#ifdef RTR_DEFAULT_SHADER_OUTPUT_DIR
        return std::filesystem::path(RTR_DEFAULT_SHADER_OUTPUT_DIR);
#else
        return {};
#endif
    }

    static std::filesystem::path resolve_shader_path(const PipelineRuntime& runtime, std::string_view filename) {
        if (filename.empty()) {
            throw std::invalid_argument("Shader filename must not be empty.");
        }
        const std::filesystem::path root = resolve_shader_root_dir(runtime);
        if (root.empty()) {
            throw std::runtime_error("Shader root directory is not configured.");
        }
        return (root / std::filesystem::path(filename)).lexically_normal();
    }

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
