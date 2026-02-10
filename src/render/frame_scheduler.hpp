#pragma once

#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include "rhi/command.hpp"
#include "rhi/device.hpp"
#include "rhi/swap_chain.hpp"

namespace rtr::render {

/**
 * @brief Vulkan frame orchestration utility.
 *
 * This class owns swapchain + per-frame/per-image synchronization and provides
 * a begin/submit-present API so higher layers can focus on recording commands.
 */
class FrameScheduler {
public:
    struct PerFrameResources {
        rhi::CommandBuffer command_buffer;

        vk::raii::Semaphore image_available_semaphore{nullptr};
        vk::raii::Fence in_flight_fence{nullptr};
    };

    struct PerImageResources {
        vk::raii::Semaphore render_finished_semaphore{nullptr};
    };

    struct FrameTicket {
        uint32_t frame_index{0};
        uint32_t image_index{0};
        rhi::CommandBuffer* command_buffer{nullptr};
    };

    struct SwapchainState {
        uint64_t generation{0};
        vk::Extent2D extent{};
        uint32_t image_count{0};
        vk::Format color_format{vk::Format::eUndefined};
        vk::Format depth_format{vk::Format::eUndefined};
    };

private:
    rhi::Window* m_window{};
    rhi::Context* m_context{};
    rhi::Device* m_device{};

    std::unique_ptr<rhi::SwapChain> m_swapchain;
    std::unique_ptr<rhi::CommandPool> m_command_pool;

    uint32_t m_max_frames_in_flight{0};
    uint32_t m_current_frame_index{0};
    uint32_t m_current_image_index{0};
    bool m_framebuffer_resized{false};
    uint64_t m_swapchain_generation{1};

    std::vector<PerImageResources> m_per_image_resources;
    std::vector<PerFrameResources> m_per_frame_resources;

    vk::Format m_depth_format{vk::Format::eD32Sfloat};

public:
    explicit FrameScheduler(rhi::Window* window, rhi::Context* context, rhi::Device* device, uint32_t max_frames_in_flight = 2)
        : m_window(window),
          m_context(context),
          m_device(device),
          m_max_frames_in_flight(max_frames_in_flight) {

        m_swapchain = std::make_unique<rhi::SwapChain>(window, context, device);
        m_command_pool = std::make_unique<rhi::CommandPool>(
            device,
            vk::CommandPoolCreateFlagBits::eResetCommandBuffer
        );
        m_depth_format = find_supported_format(
            {vk::Format::eD32Sfloat, vk::Format::eD32SfloatS8Uint, vk::Format::eD24UnormS8Uint},
            vk::ImageTiling::eOptimal,
            vk::FormatFeatureFlags::BitsType::eDepthStencilAttachment
        );

        init_per_image_resource();
        init_per_frame_resources();
    }

    FrameScheduler(const FrameScheduler&) = delete;
    FrameScheduler& operator=(const FrameScheduler&) = delete;

    std::optional<FrameTicket> begin_frame() {
        auto& frame_res = m_per_frame_resources[m_current_frame_index];

        vk::Result wait_result = m_device->device().waitForFences(
            *frame_res.in_flight_fence,
            VK_TRUE,
            UINT64_MAX
        );
        if (wait_result != vk::Result::eSuccess) {
            std::cerr << "Failed to wait for fence" << std::endl;
            return std::nullopt;
        }

        m_device->device().resetFences(*frame_res.in_flight_fence);

        auto [result, image_index] = m_swapchain->acquire_next_image(
            frame_res.image_available_semaphore
        );
        if (result == vk::Result::eErrorOutOfDateKHR) {
            m_device->device().waitIdle();
            recreate_swapchain_resources();
            return std::nullopt;
        }
        if (result != vk::Result::eSuccess && result != vk::Result::eSuboptimalKHR) {
            throw std::runtime_error("Failed to acquire swapchain image");
        }

        m_current_image_index = image_index;
        return FrameTicket{
            .frame_index = m_current_frame_index,
            .image_index = m_current_image_index,
            .command_buffer = &frame_res.command_buffer
        };
    }

    void submit_and_present(const FrameTicket& ticket) {
        auto& frame_res = m_per_frame_resources[ticket.frame_index];
        auto& image_res = m_per_image_resources[ticket.image_index];

        rhi::CommandBuffer::SubmitInfo submit_info;
        submit_info.wait_semaphores = {*frame_res.image_available_semaphore};
        submit_info.wait_stages = {vk::PipelineStageFlagBits::eColorAttachmentOutput};
        submit_info.signal_semaphores = {*image_res.render_finished_semaphore};
        submit_info.fence = *frame_res.in_flight_fence;
        ticket.command_buffer->submit(submit_info);

        vk::Result present_result = m_swapchain->present(
            ticket.image_index,
            image_res.render_finished_semaphore,
            nullptr
        );

        bool needs_recreation = false;
        if (present_result == vk::Result::eErrorOutOfDateKHR) {
            needs_recreation = true;
        } else if (present_result == vk::Result::eSuboptimalKHR) {
            std::cout << "Swapchain suboptimal during presentation." << std::endl;
            needs_recreation = true;
        } else if (present_result != vk::Result::eSuccess) {
            throw std::runtime_error("Failed to present swapchain image");
        }

        if (needs_recreation || m_framebuffer_resized) {
            m_framebuffer_resized = false;
            m_device->device().waitIdle();
            recreate_swapchain_resources();
        }

        m_current_frame_index = (m_current_frame_index + 1) % m_max_frames_in_flight;
    }

    void on_window_resized(uint32_t width, uint32_t height) {
        std::cout << "FrameScheduler: Window resized to (" << width << ", " << height << ")" << std::endl;
        m_framebuffer_resized = true;
    }

    vk::Extent2D render_extent() const { return m_swapchain->extent(); }
    vk::Format render_format() const { return m_swapchain->image_format(); }
    uint32_t image_count() const { return static_cast<uint32_t>(m_swapchain->images().size()); }
    uint32_t max_frames_in_flight() const { return m_max_frames_in_flight; }
    uint32_t current_frame_index() const { return m_current_frame_index; }
    uint32_t current_image_index() const { return m_current_image_index; }
    const vk::Format& depth_format() const { return m_depth_format; }
    SwapchainState swapchain_state() const {
        return SwapchainState{
            .generation = m_swapchain_generation,
            .extent = m_swapchain->extent(),
            .image_count = static_cast<uint32_t>(m_swapchain->images().size()),
            .color_format = m_swapchain->image_format(),
            .depth_format = m_depth_format
        };
    }

    const std::vector<PerImageResources>& per_image_resources() const { return m_per_image_resources; }
    std::vector<PerImageResources>& per_image_resources() { return m_per_image_resources; }
    const std::vector<PerFrameResources>& per_frame_resources() const { return m_per_frame_resources; }
    const rhi::SwapChain& swapchain() const { return *m_swapchain; }

private:
    void recreate_swapchain_resources() {
        init_swapchain();
        init_per_image_resource();
        init_per_frame_resources();
        ++m_swapchain_generation;
    }

    void init_swapchain() {
        m_swapchain.reset();
        m_swapchain = std::make_unique<rhi::SwapChain>(m_window, m_context, m_device);
    }

    void init_per_image_resource() {
        m_per_image_resources.clear();
        m_per_image_resources.reserve(m_swapchain->images().size());
        vk::SemaphoreCreateInfo semaphore_info{};

        for (size_t i = 0; i < m_swapchain->images().size(); ++i) {
            PerImageResources image_res{
                .render_finished_semaphore = vk::raii::Semaphore(m_device->device(), semaphore_info)
            };
            m_per_image_resources.push_back(std::move(image_res));
        }
    }

    void init_per_frame_resources() {
        m_per_frame_resources.clear();
        m_per_frame_resources.reserve(m_max_frames_in_flight);

        vk::SemaphoreCreateInfo semaphore_info{};
        vk::FenceCreateInfo fence_info{};
        fence_info.flags = vk::FenceCreateFlagBits::eSignaled;

        auto command_buffers = m_command_pool->create_command_buffers(m_max_frames_in_flight);
        for (uint32_t i = 0; i < m_max_frames_in_flight; ++i) {
            PerFrameResources resources{
                .command_buffer = std::move(command_buffers[i]),
                .image_available_semaphore = vk::raii::Semaphore(
                    m_device->device(),
                    semaphore_info
                ),
                .in_flight_fence = vk::raii::Fence(
                    m_device->device(),
                    fence_info
                )
            };
            m_per_frame_resources.push_back(std::move(resources));
        }
    }

    vk::Format find_supported_format(
        const std::vector<vk::Format>& candidates,
        vk::ImageTiling tiling,
        vk::FormatFeatureFlags features
    ) const {
        for (const auto format : candidates) {
            vk::FormatProperties props = m_device->physical_device().getFormatProperties(format);
            if (tiling == vk::ImageTiling::eLinear && (props.linearTilingFeatures & features) == features) {
                return format;
            }
            if (tiling == vk::ImageTiling::eOptimal && (props.optimalTilingFeatures & features) == features) {
                return format;
            }
        }
        throw std::runtime_error("failed to find supported format!");
    }
};

} // namespace rtr::render
