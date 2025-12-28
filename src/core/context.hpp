#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <optional>
#include <iostream>

#include "common.hpp"
#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vk_platform.h"
#include "vulkan/vulkan_structs.hpp"

#include "window.hpp"

namespace rtr::core {

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
