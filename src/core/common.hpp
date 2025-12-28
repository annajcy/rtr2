// Utility free functions shared across rtr::core
#pragma once

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <optional>
#include <ranges>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "vulkan/vk_platform.h"
#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_core.h"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vulkan_structs.hpp"

namespace rtr::core {

// Buffer helpers
inline std::optional<uint32_t> find_memory_type(
    const vk::PhysicalDeviceMemoryProperties& mem_properties,
    uint32_t type_filter,
    vk::MemoryPropertyFlags properties
) {
    for (uint32_t i = 0; i < mem_properties.memoryTypeCount; i++) {
        if ((type_filter & (1 << i)) && 
            (mem_properties.memoryTypes[i].propertyFlags & properties) == properties) {
            return i;
        }
    }
    return std::nullopt;
}

inline std::optional<std::pair<vk::raii::Buffer, vk::raii::DeviceMemory>> make_buffer_with_memory(
    const vk::raii::Device& device,
    const vk::raii::PhysicalDevice& physical_device,
    vk::DeviceSize size,
    vk::BufferUsageFlags usage,
    vk::MemoryPropertyFlags properties
) {
    vk::BufferCreateInfo buffer_create_info{};
    buffer_create_info.size = size;
    buffer_create_info.usage = usage;
    buffer_create_info.sharingMode = vk::SharingMode::eExclusive;

    vk::raii::Buffer buffer{ device, buffer_create_info };

    vk::MemoryRequirements mem_requirements = buffer.getMemoryRequirements();

    vk::PhysicalDeviceMemoryProperties mem_properties = physical_device.getMemoryProperties();
    if (auto memory_type_index_opt = find_memory_type(
        mem_properties,
        mem_requirements.memoryTypeBits,
        properties
    )) {
        auto memory_type_index = memory_type_index_opt.value();

        vk::MemoryAllocateInfo alloc_info{};
        alloc_info.allocationSize = mem_requirements.size;
        alloc_info.memoryTypeIndex = memory_type_index;

        vk::raii::DeviceMemory buffer_memory{ device, alloc_info };

        buffer.bindMemory(
            *buffer_memory,
            vk::DeviceSize(0)
        );

        return std::make_pair(std::move(buffer), std::move(buffer_memory));
    } else {
        return std::nullopt;
    }
}

inline std::optional<std::tuple<vk::raii::Buffer, vk::raii::DeviceMemory, void*>> make_mapped_buffer_with_memory(
    const vk::raii::Device& device,
    const vk::raii::PhysicalDevice& physical_device,
    vk::DeviceSize size,
    vk::BufferUsageFlags usage,
    vk::MemoryPropertyFlags properties
) {
    if (auto buffer_with_memory_opt = make_buffer_with_memory(
        device,
        physical_device,
        size,
        usage,
        properties | vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent)
    ) {
        auto [buffer, buffer_memory] = std::move(buffer_with_memory_opt.value());
        void* mapped_ptr = buffer_memory.mapMemory(0, size);
        return std::make_tuple(
            std::move(buffer),
            std::move(buffer_memory),
            mapped_ptr
        );
    } else {
        return std::nullopt;
    }
}


// 辅助函数：创建 Image 并绑定内存
inline std::optional<std::pair<vk::raii::Image, vk::raii::DeviceMemory>> make_image_with_memory(
    const vk::raii::Device& device,
    const vk::raii::PhysicalDevice& physical_device,
    uint32_t width,
    uint32_t height,
    vk::Format format,
    vk::ImageTiling tiling,
    vk::ImageUsageFlags usage,
    vk::MemoryPropertyFlags properties
) {
    // 1. 设置 Image 创建信息
    vk::ImageCreateInfo image_info{};
    image_info.imageType = vk::ImageType::e2D;
    image_info.extent.width = width;
    image_info.extent.height = height;
    image_info.extent.depth = 1;
    image_info.mipLevels = 1;
    image_info.arrayLayers = 1;
    image_info.format = format;
    image_info.tiling = tiling;           // 关键点：Linear vs Optimal
    image_info.initialLayout = vk::ImageLayout::eUndefined; // 初始状态不可用
    image_info.usage = usage;
    image_info.samples = vk::SampleCountFlagBits::e1;
    image_info.sharingMode = vk::SharingMode::eExclusive;

    // 创建 Image 句柄 (此时还没显存)
    vk::raii::Image image(device, image_info);

    // 2. 获取内存需求
    vk::MemoryRequirements mem_requirements = image.getMemoryRequirements();

    // 3. 查找并分配内存
    vk::PhysicalDeviceMemoryProperties mem_properties = physical_device.getMemoryProperties();
    auto memory_type_index_opt = find_memory_type(
        mem_properties,
        mem_requirements.memoryTypeBits,
        properties
    );

    if (!memory_type_index_opt) {
        return std::nullopt;
    }

    vk::MemoryAllocateInfo alloc_info{};
    alloc_info.allocationSize = mem_requirements.size;
    alloc_info.memoryTypeIndex = memory_type_index_opt.value();

    vk::raii::DeviceMemory image_memory(device, alloc_info);

    // 4. 绑定内存到 Image
    image.bindMemory(*image_memory, 0);

    return std::make_pair(std::move(image), std::move(image_memory));
}

template<typename Mapper>
inline void map_memory(
    const vk::raii::DeviceMemory& memory,
    vk::DeviceSize size,
    vk::DeviceSize offset,
    Mapper&& mapper,
    vk::MemoryMapFlags flags = vk::MemoryMapFlags{}
) {
    void* data = memory.mapMemory(
        offset,
        size,
        flags
    );
    mapper(data);
    memory.unmapMemory();
}

// Context helpers
inline bool is_instance_extensions_supported(
    const std::vector<vk::ExtensionProperties>& instance_extension_properties,
    const std::vector<std::string>& required_extensions
) {
    for (const auto& required_ext : required_extensions) {
        if (std::ranges::none_of(instance_extension_properties, 
            [&](const vk::ExtensionProperties& ext) {
                return required_ext == ext.extensionName;
            })) {
            return false;
        }
    }
    return true;
}

inline bool is_layers_supported(
    const std::vector<vk::LayerProperties>& available_layer_properties,
    const std::vector<std::string>& required_layers
) {
    for (const auto& required_layer : required_layers) {
        if (std::ranges::none_of(available_layer_properties, 
            [&](const vk::LayerProperties& layer) {
                return required_layer == layer.layerName;
            })) {
            return false;
        }
    }
    return true;
}

inline std::optional<std::pair<vk::raii::Context, vk::raii::Instance>> make_instance(
    const std::vector<std::string>& required_layers,
    const std::vector<std::string>& required_extensions,
    const vk::ApplicationInfo& app_info
) {
    vk::raii::Context context{};

    auto ext_prop = context.enumerateInstanceExtensionProperties();
    if (!is_instance_extensions_supported(
        ext_prop, 
        required_extensions
    )) {
        std::cout << "Not all required extensions are supported:" << std::endl;
        std::cout << "Required extensions:" << std::endl;
        for (const auto& ext : required_extensions) {
            std::cout << "  " << ext << std::endl;
        }
        std::cout << std::endl;
        std::cout << "Available extensions:" << std::endl;
        for (const auto& ext : ext_prop) {
            std::cout << "  " << ext.extensionName << std::endl;
        }
        return std::nullopt;
    }

    auto layer_prop = context.enumerateInstanceLayerProperties();
    if (!is_layers_supported(
        layer_prop,
        required_layers
    )) {
        std::cout << "Not all required layers are supported:" << std::endl;
        return std::nullopt;
    }

    vk::InstanceCreateInfo instance_info{};
#if defined (__APPLE__)
    instance_info.flags = vk::InstanceCreateFlagBits::eEnumeratePortabilityKHR;
#endif
    instance_info.pApplicationInfo = &app_info; 
    std::vector<const char*> extension_names;
    std::ranges::transform(
        required_extensions, 
        std::back_inserter(extension_names),
        [](const std::string& ext) { return ext.c_str(); }
    );
    instance_info.enabledExtensionCount = static_cast<uint32_t>(extension_names.size());
    instance_info.ppEnabledExtensionNames = extension_names.data();
    std::vector<const char*> layer_names;
    std::ranges::transform(
        required_layers, 
        std::back_inserter(layer_names),
        [](const std::string& layer) { return layer.c_str(); }
    );
    instance_info.enabledLayerCount = static_cast<uint32_t>(layer_names.size());
    instance_info.ppEnabledLayerNames = layer_names.data();
    vk::raii::Instance instance{ context, instance_info };
    return std::make_pair(std::move(context), std::move(instance));
}

inline VKAPI_ATTR vk::Bool32 VKAPI_CALL debug_callback(vk::DebugUtilsMessageSeverityFlagBitsEXT severity, vk::DebugUtilsMessageTypeFlagsEXT type, const vk::DebugUtilsMessengerCallbackDataEXT* pCallbackData, void*) {
    if (severity == vk::DebugUtilsMessageSeverityFlagBitsEXT::eError || 
        severity == vk::DebugUtilsMessageSeverityFlagBitsEXT::eWarning
    ) {
        std::cerr << "validation layer: type " << to_string(type) << " msg: " << pCallbackData->pMessage << std::endl;
    }
    return vk::False;
}

inline vk::raii::DebugUtilsMessengerEXT create_debug_messenger(const vk::raii::Instance& instance) {
    vk::DebugUtilsMessageSeverityFlagsEXT severity_flags( 
        vk::DebugUtilsMessageSeverityFlagBitsEXT::eVerbose | 
        vk::DebugUtilsMessageSeverityFlagBitsEXT::eWarning | 
        vk::DebugUtilsMessageSeverityFlagBitsEXT::eError 
    );  

    vk::DebugUtilsMessageTypeFlagsEXT    messageType_flags( 
        vk::DebugUtilsMessageTypeFlagBitsEXT::eGeneral | 
        vk::DebugUtilsMessageTypeFlagBitsEXT::ePerformance | 
        vk::DebugUtilsMessageTypeFlagBitsEXT::eValidation 
    );

    vk::DebugUtilsMessengerCreateInfoEXT debugUtils_messenger_createInfo_EXT{};
    debugUtils_messenger_createInfo_EXT.messageSeverity = severity_flags;
    debugUtils_messenger_createInfo_EXT.messageType = messageType_flags;
    debugUtils_messenger_createInfo_EXT.pfnUserCallback = &debug_callback;
    return vk::raii::DebugUtilsMessengerEXT(instance, debugUtils_messenger_createInfo_EXT);
}

// Device helpers
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

} // namespace rtr::core
