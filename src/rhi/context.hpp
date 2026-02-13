#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rtr/rhi/common.hpp"
#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vulkan_structs.hpp"

namespace rtr::rhi {

struct ContextCreateInfo {
    std::string app_name{"RTR"};
    std::vector<std::string> instance_extensions{};
    std::function<std::optional<VkSurfaceKHR>(const vk::raii::Instance&)> surface_creator{};
//    std::function<std::pair<int, int>()> framebuffer_size{};
#if !defined(NDEBUG)
    bool enable_validation_layers{true};
#else
    bool enable_validation_layers{false};
#endif
};

class Context {
private:
    ContextCreateInfo m_create_info{};

    vk::raii::Context m_context{};
    vk::raii::Instance m_instance{nullptr};
    vk::raii::SurfaceKHR m_surface{nullptr};
    vk::raii::DebugUtilsMessengerEXT m_debug_messenger{nullptr};

    bool m_is_validation_layers_enabled{false};

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
        m_instance_extensions.insert(
            m_instance_extensions.end(),
            m_create_info.instance_extensions.begin(),
            m_create_info.instance_extensions.end()
        );
        if (m_is_validation_layers_enabled) {
            m_instance_layers.push_back("VK_LAYER_KHRONOS_validation");
            m_instance_extensions.push_back(vk::EXTDebugUtilsExtensionName);
        }

        uint32_t application_version{VK_MAKE_VERSION(1, 0, 0)};

        vk::ApplicationInfo app_info{};
        app_info.pApplicationName = m_create_info.app_name.c_str();
        app_info.applicationVersion = application_version;
        app_info.pEngineName = m_create_info.app_name.c_str();
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
        if (!m_create_info.surface_creator) {
            throw std::runtime_error("Missing surface factory callback in ContextCreateInfo.");
        }
        if (auto surface_handle = m_create_info.surface_creator(m_instance)) {
            m_surface = vk::raii::SurfaceKHR{m_instance, surface_handle.value()};
        } else {
            throw std::runtime_error("Failed to create Vulkan surface.");
        }
    }

    void create_debug_messenger() {
        if (m_is_validation_layers_enabled) {
            m_debug_messenger = rtr::rhi::create_debug_messenger(m_instance);
        }
    }

public:
    explicit Context(ContextCreateInfo create_info)
        : m_create_info(std::move(create_info)),
          m_is_validation_layers_enabled(m_create_info.enable_validation_layers) {
        create_instance();
        create_surface();
        create_debug_messenger();
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

    // std::pair<int, int> framebuffer_size() const {
    //     if (!m_create_info.framebuffer_size) {
    //         throw std::runtime_error("Missing framebuffer size callback in ContextCreateInfo.");
    //     }
    //     return m_create_info.framebuffer_size();
    // }
};

} // namespace rtr::rhi
