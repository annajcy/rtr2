#pragma once

#include <algorithm>
#include <functional>
#include <cstdlib>
#include <optional>
#include <utility>
#include <vector>

#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vulkan_structs.hpp"

namespace rtr {

inline std::optional<std::pair<vk::raii::SwapchainKHR, std::vector<vk::raii::ImageView>>> make_swapchain_with_image_views(
    const vk::raii::Device& device,
    const vk::SwapchainCreateInfoKHR& swapchain_create_info
) {
    vk::raii::SwapchainKHR swapchain{ device, swapchain_create_info };
    std::vector<vk::Image> images = swapchain.getImages();

    std::vector<vk::raii::ImageView> image_views{};
    image_views.reserve(images.size());
    for (const auto& image : images) {
        vk::ImageViewCreateInfo image_view_create_info{};
        image_view_create_info.image = image;
        image_view_create_info.viewType = vk::ImageViewType::e2D;
        image_view_create_info.format = swapchain_create_info.imageFormat;
        image_view_create_info.components.r = vk::ComponentSwizzle::eIdentity;
        image_view_create_info.components.g = vk::ComponentSwizzle::eIdentity;
        image_view_create_info.components.b = vk::ComponentSwizzle::eIdentity;
        image_view_create_info.components.a = vk::ComponentSwizzle::eIdentity;
        image_view_create_info.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
        image_view_create_info.subresourceRange.baseMipLevel = 0;
        image_view_create_info.subresourceRange.levelCount = 1;
        image_view_create_info.subresourceRange.baseArrayLayer = 0;
        image_view_create_info.subresourceRange.layerCount = 1;
        image_views.emplace_back(device, image_view_create_info);
    }

    return std::make_pair(std::move(swapchain), std::move(image_views));
}

};