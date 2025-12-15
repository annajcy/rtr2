#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>
#include <vector>

#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vk_platform.h"
#include "vulkan/vulkan_structs.hpp"

namespace rtr {

template<typename ... Features>
inline std::optional<vk::raii::Device> make_device(
    const vk::raii::PhysicalDevice& physical_device,
    const std::vector<std::string>& required_device_extensions,
    const vk::StructureChain<Features...>& feature_chain,
    const uint32_t& device_queue_family_index
) {
    std::vector<float> queue_priorities { 0.0f };
    vk::DeviceQueueCreateInfo queue_create_info{};
    queue_create_info.queueFamilyIndex = device_queue_family_index;
    queue_create_info.queueCount = static_cast<uint32_t>(queue_priorities.size());
    queue_create_info.pQueuePriorities = queue_priorities.data();

    std::vector<vk::DeviceQueueCreateInfo> device_queue_create_infos {
        queue_create_info
    };

    vk::DeviceCreateInfo device_create_info{};
    device_create_info.pNext = &std::get<0>(feature_chain);
    device_create_info.queueCreateInfoCount = static_cast<uint32_t>(device_queue_create_infos.size());
    device_create_info.pQueueCreateInfos = device_queue_create_infos.data();

    std::vector<const char*> required_extensions_cstr{};
    std::ranges::transform(
        required_device_extensions,
        std::back_inserter(required_extensions_cstr),
        [](const std::string& ext) { return ext.c_str(); }
    );

    device_create_info.enabledExtensionCount = static_cast<uint32_t>(required_extensions_cstr.size());
    device_create_info.ppEnabledExtensionNames = required_extensions_cstr.data();

    return vk::raii::Device(physical_device, device_create_info);
}

};