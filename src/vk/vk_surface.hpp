#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <cstdlib>
#include <utility>
#include <vector>

#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vk_platform.h"
#include "vulkan/vulkan_structs.hpp"

namespace rtr {

inline vk::SurfaceFormatKHR select_surface_format(
    const std::vector<vk::SurfaceFormatKHR>& available_formats,
    std::function<bool(vk::SurfaceFormatKHR)> checker
) {
    for (const auto& format : available_formats) {
        if (checker(format)) {
            return format;
        }
    }
    return *available_formats.begin();
}

inline vk::PresentModeKHR select_present_mode(
    const std::vector<vk::PresentModeKHR>& available_present_modes,
    vk::PresentModeKHR preferred_mode
) {
    for (const auto& present_mode : available_present_modes) {
        if (present_mode == preferred_mode) {
            return present_mode;
        }
    }
    return vk::PresentModeKHR::eFifo;
}

inline std::pair<vk::Extent2D, int> select_swapchain_image_property(
    const vk::SurfaceCapabilitiesKHR& capabilities,
    uint32_t width,
    uint32_t height,
    uint32_t desired_image_count = 3
) {
    vk::Extent2D extent;
    if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
        extent = capabilities.currentExtent;
    } else {
        extent = vk::Extent2D{
            std::clamp(width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width),
            std::clamp(height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height)
        };
    }
    uint32_t image_count = std::max(desired_image_count, capabilities.minImageCount);
    if (capabilities.maxImageCount > 0 && image_count > capabilities.maxImageCount) {
        image_count = capabilities.maxImageCount;
    }
    return { extent, image_count };
}

}