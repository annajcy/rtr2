#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <optional>
#include <iostream>

#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vk_platform.h"
#include "vulkan/vulkan_structs.hpp"

#include "window.hpp"

namespace rtr::core {

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

class Context {
private:
    Window* m_window{};
    vk::raii::Context m_context{};
    vk::raii::Instance m_instance{nullptr};
    vk::raii::SurfaceKHR m_surface{nullptr};
    vk::raii::DebugUtilsMessengerEXT m_debug_messenger{nullptr};

#if !defined(NDEBUG)
    bool m_is_validation_layers_enabled{ true }; 
#else
    bool m_is_validation_layers_enabled{ false }; 
#endif

    std::vector<std::string> m_instance_layers{};
    std::vector<std::string> m_instance_extensions{
#if defined(__APPLE__)
        vk::KHRPortabilityEnumerationExtensionName,
#endif
        "VK_EXT_surface_maintenance1",
        "VK_KHR_get_surface_capabilities2"
    };

private:
    void create_instance() {
        auto window_extensions = m_window->required_extensions();
        m_instance_extensions.insert(
            m_instance_extensions.end(),
            window_extensions.begin(),
            window_extensions.end()
        );
        if (m_is_validation_layers_enabled) {
            m_instance_layers.push_back("VK_LAYER_KHRONOS_validation");
            m_instance_extensions.push_back(vk::EXTDebugUtilsExtensionName);
        }

        uint32_t application_version{VK_MAKE_VERSION(1, 0, 0)};
        
        vk::ApplicationInfo app_info{};
        app_info.pApplicationName = m_window->title().c_str();
        app_info.applicationVersion = application_version;
        app_info.pEngineName = m_window->title().c_str();
        app_info.engineVersion = application_version;
        app_info.apiVersion = vk::ApiVersion14;
        auto instance_result = make_instance(
            m_instance_layers,
            m_instance_extensions,
            app_info
        );

        if (!instance_result.has_value()) {
            throw std::runtime_error("Failed to create Vulkan instance.");
        }

        auto instance_handles = std::move(instance_result.value());
        m_context = std::move(instance_handles.first);
        m_instance = std::move(instance_handles.second);
    }

    void create_surface() {
        if (auto surface_handle = m_window->create_vk_surface(m_instance)) {
            m_surface = vk::raii::SurfaceKHR{m_instance, surface_handle.value()};
        } else {
            throw std::runtime_error("Failed to create Vulkan surface.");
        }
    }

    void create_debug_messenger() {
        if (m_is_validation_layers_enabled) {
            m_debug_messenger = rtr::core::create_debug_messenger(m_instance);
        }
    }
    
public:
    Context(Window* window) : m_window(window) {
        create_instance();
        create_surface();
        create_debug_messenger();
    }

    const Window* window() const {
        return m_window;
    }

    const vk::raii::Instance& instance() const {
        return m_instance;
    }

    const vk::raii::SurfaceKHR& surface() const {
        return m_surface;
    }

    const vk::raii::Context& context() const {
        return m_context;
    }

    bool is_validation_layers_enabled() const {
        return m_is_validation_layers_enabled;
    }

    const std::vector<std::string>& instance_layers() const {
        return m_instance_layers;
    }

    const std::vector<std::string>& instance_extensions() const {
        return m_instance_extensions;
    }

};

};