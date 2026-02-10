#pragma once

#include "render/frame_context.hpp"
#include "render/resource_registries.hpp"
#include "render/frame_scheduler.hpp"
#include "rhi/context.hpp"
#include "rhi/device.hpp"
#include "rhi/window.hpp"
#include "vulkan/vulkan_enums.hpp"
#include <cstdint>

namespace rtr::render {

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

    virtual void bind_static_resources(ResourceRegistries& regs) = 0;
    virtual void bind_frame_resources(uint32_t frame_index, ResourceRegistries& regs) = 0;
    // Renderer owns command buffer begin/end/reset/submit; pipeline only records draw commands.
    virtual void render(FrameContext& ctx) = 0;
};

} // namespace rtr::render
