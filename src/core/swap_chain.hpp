#pragma once

#include <algorithm>
#include <functional>
#include <cstdlib>
#include <optional>
#include <utility>
#include <vector>
#include <iostream>

#include "device.hpp"
#include "window.hpp"
#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vulkan_structs.hpp"

namespace rtr::core {

class SwapChain {
private:
    Device* m_device;

    vk::raii::SwapchainKHR m_swapchain{nullptr};
    std::vector<vk::Image> m_images;
    std::vector<vk::raii::ImageView> m_image_views;
    vk::Format m_image_format;
    vk::Extent2D m_extent;

public:
    SwapChain(Device* device) : m_device(device) {
        auto surface_formats = m_device->physical_device().getSurfaceFormatsKHR(*m_device->context()->surface());
        auto surface_format = choose_surface_format(surface_formats);
        auto present_mode = choose_present_mode(m_device->physical_device().getSurfacePresentModesKHR(*m_device->context()->surface()));
        auto capabilities = m_device->physical_device().getSurfaceCapabilitiesKHR(*m_device->context()->surface());
        auto extent = choose_extent(capabilities);

        uint32_t image_count = capabilities.minImageCount + 1;
        if (capabilities.maxImageCount > 0 && image_count > capabilities.maxImageCount) {
            image_count = capabilities.maxImageCount;
        }

        std::cout << image_count << " swapchain images will be created with extent ("
                  << extent.width << ", " << extent.height << ")." << std::endl;

        vk::SwapchainCreateInfoKHR create_info{};
        create_info.surface = *m_device->context()->surface();
        create_info.minImageCount = image_count;
        create_info.imageFormat = surface_format.format;
        create_info.imageColorSpace = surface_format.colorSpace;
        create_info.imageExtent = extent;
        create_info.imageArrayLayers = 1;
        create_info.imageUsage = vk::ImageUsageFlagBits::eColorAttachment;

        create_info.imageSharingMode = vk::SharingMode::eExclusive;
        create_info.preTransform = capabilities.currentTransform;
        create_info.compositeAlpha = vk::CompositeAlphaFlagBitsKHR::eOpaque;
        create_info.presentMode = present_mode;
        create_info.clipped = VK_TRUE;
        create_info.oldSwapchain = VK_NULL_HANDLE;

        m_swapchain = vk::raii::SwapchainKHR(m_device->device(), create_info);
        m_images = m_swapchain.getImages();
        m_image_format = surface_format.format;
        m_extent = extent;

        create_image_views();
    }

    ~SwapChain() {
        m_image_views.clear();
        m_swapchain.clear();
    }

    SwapChain(const SwapChain&) = delete;
    SwapChain& operator=(const SwapChain&) = delete;

    std::pair<vk::Result, uint32_t> acquire_next_image(const vk::raii::Semaphore& semaphore) {
        try {
            return m_swapchain.acquireNextImage(UINT64_MAX, *semaphore, nullptr);
        } catch (const vk::OutOfDateKHRError&) {
            return {vk::Result::eErrorOutOfDateKHR, 0};
        }
    }

    vk::Result present(uint32_t image_index, const vk::raii::Semaphore& wait_semaphore, const vk::raii::Fence& present_fence) {
        vk::SwapchainKHR swapchains[] = {*m_swapchain};
        vk::Semaphore wait_semaphores[] = {*wait_semaphore};
        vk::Fence fences[] = {*present_fence};
        
        vk::SwapchainPresentFenceInfoEXT present_fence_info{};
        present_fence_info.swapchainCount = 1;
        present_fence_info.pFences = fences;

        vk::PresentInfoKHR present_info{};
        present_info.waitSemaphoreCount = 1;
        present_info.pWaitSemaphores = wait_semaphores;
        present_info.swapchainCount = 1;
        present_info.pSwapchains = swapchains;
        present_info.pImageIndices = &image_index;

        vk::StructureChain<vk::PresentInfoKHR, vk::SwapchainPresentFenceInfoEXT> present_info_chain{
            present_info,
            present_fence_info
        };

        try {
            return m_device->queue().presentKHR(present_info_chain.get<vk::PresentInfoKHR>());
        } catch (const vk::OutOfDateKHRError&) {
            return vk::Result::eErrorOutOfDateKHR;
        }
    }

    vk::Format image_format() const { return m_image_format; }
    vk::Extent2D extent() const { return m_extent; }
    const std::vector<vk::raii::ImageView>& image_views() const { return m_image_views; }
    const std::vector<vk::Image>& images() const { return m_images; }

private:

    void create_image_views() {
        m_image_views.clear();
        m_image_views.reserve(m_images.size());

        for (const auto& image : m_images) {
            vk::ImageViewCreateInfo create_info{};
            create_info.image = image;
            create_info.viewType = vk::ImageViewType::e2D;
            create_info.format = m_image_format;
            create_info.components.r = vk::ComponentSwizzle::eIdentity;
            create_info.components.g = vk::ComponentSwizzle::eIdentity;
            create_info.components.b = vk::ComponentSwizzle::eIdentity;
            create_info.components.a = vk::ComponentSwizzle::eIdentity;
            create_info.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
            create_info.subresourceRange.baseMipLevel = 0;
            create_info.subresourceRange.levelCount = 1;
            create_info.subresourceRange.baseArrayLayer = 0;
            create_info.subresourceRange.layerCount = 1;

            m_image_views.emplace_back(m_device->device(), create_info);
        }
    }

    vk::SurfaceFormatKHR choose_surface_format(const std::vector<vk::SurfaceFormatKHR>& available_formats) {
        for (const auto& available_format : available_formats) {
            if (available_format.format == vk::Format::eB8G8R8A8Srgb &&
                available_format.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear) {
                return available_format;
            }
        }
        return available_formats[0];
    }

    vk::PresentModeKHR choose_present_mode(const std::vector<vk::PresentModeKHR>& available_present_modes) {
        for (const auto& available_present_mode : available_present_modes) {
            if (available_present_mode == vk::PresentModeKHR::eMailbox) {
                return available_present_mode;
            }
        }
        return vk::PresentModeKHR::eFifo;
    }

    vk::Extent2D choose_extent(const vk::SurfaceCapabilitiesKHR& capabilities) {
        if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
            return capabilities.currentExtent;
        } else {
            auto [width, height] = m_device->context()->window()->framebuffer_size();

            vk::Extent2D actual_extent = {
                static_cast<uint32_t>(width),
                static_cast<uint32_t>(height)
            };

            actual_extent.width = std::clamp(actual_extent.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
            actual_extent.height = std::clamp(actual_extent.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height);

            return actual_extent;
        }
    }

    const Device* device() const {
        return m_device;
    }
};

}