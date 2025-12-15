#pragma once

#include <algorithm>
#include <iostream>
#include <optional>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vk_platform.h"
#include "vulkan/vulkan_structs.hpp"

namespace rtr {

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

inline std::optional<vk::raii::DebugUtilsMessengerEXT> create_debug_messenger(
    const vk::raii::Instance& instance
) {
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

}