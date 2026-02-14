#pragma once

#include "context.hpp"
#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vulkan_structs.hpp"

#include <functional>
#include <optional>
#include <vector>
#include <algorithm>

#include "rtr/utils/log.hpp"

namespace rtr::rhi {

namespace detail {
    template<typename T>
    struct FeatureChecker;

    template<>
    struct FeatureChecker<vk::PhysicalDeviceFeatures> {
        static bool check(const vk::PhysicalDeviceFeatures& required, const vk::PhysicalDeviceFeatures& supported) {
            if (required.samplerAnisotropy && !supported.samplerAnisotropy) return false;
            if (required.shaderStorageImageReadWithoutFormat && !supported.shaderStorageImageReadWithoutFormat) return false;
            if (required.shaderStorageImageWriteWithoutFormat && !supported.shaderStorageImageWriteWithoutFormat) return false;
            return true;
        }
    };

    template<>
    struct FeatureChecker<vk::PhysicalDeviceVulkan11Features> {
        static bool check(const vk::PhysicalDeviceVulkan11Features& required, const vk::PhysicalDeviceVulkan11Features& supported) {
            if (required.shaderDrawParameters && !supported.shaderDrawParameters) return false;
            return true;
        }
    };

    template<>
    struct FeatureChecker<vk::PhysicalDeviceVulkan13Features> {
        static bool check(const vk::PhysicalDeviceVulkan13Features& required, const vk::PhysicalDeviceVulkan13Features& supported) {
            if (required.synchronization2 && !supported.synchronization2) return false;
            if (required.dynamicRendering && !supported.dynamicRendering) return false;
            return true;
        }
    };

    template<>
    struct FeatureChecker<vk::PhysicalDeviceFeatures2> {
        static bool check(const vk::PhysicalDeviceFeatures2& required, const vk::PhysicalDeviceFeatures2& supported) {
            return FeatureChecker<vk::PhysicalDeviceFeatures>::check(required.features, supported.features);
        }
    };

    template<>
    struct FeatureChecker<vk::PhysicalDeviceSynchronization2Features> {
        static bool check(const vk::PhysicalDeviceSynchronization2Features& required, const vk::PhysicalDeviceSynchronization2Features& supported) {
            if (required.synchronization2 && !supported.synchronization2) return false;
            return true;
        }
    };

    template<>
    struct FeatureChecker<vk::PhysicalDeviceDynamicRenderingFeatures> {
        static bool check(const vk::PhysicalDeviceDynamicRenderingFeatures& required, const vk::PhysicalDeviceDynamicRenderingFeatures& supported) {
            if (required.dynamicRendering && !supported.dynamicRendering) return false;
            return true;
        }
    };

    template<>
    struct FeatureChecker<vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT> {
        static bool check(const vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT& required, const vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT& supported) {
            if (required.extendedDynamicState && !supported.extendedDynamicState) return false;
            return true;
        }
    };

    template<>
    struct FeatureChecker<vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT> {
        static bool check(const vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT& required, const vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT& supported) {
            if (required.swapchainMaintenance1 && !supported.swapchainMaintenance1) return false;
            return true;
        }
    };
}

class PhysicalDeviceSelector {
public:
    struct Selection {
        vk::raii::PhysicalDevice physical_device = nullptr;
        uint32_t queue_family_index = 0;
    };

private:
const vk::raii::Instance& m_instance;
    const vk::raii::SurfaceKHR* m_surface = nullptr;
    uint32_t m_required_api_version = 0;
    std::vector<std::string> m_required_extensions;
    std::optional<vk::PhysicalDeviceType> m_required_type;
    vk::QueueFlags m_required_queue_flags;
    std::vector<std::function<bool(const vk::raii::PhysicalDevice&)>> m_feature_checkers;
    std::vector<std::function<bool(const vk::raii::PhysicalDevice&)>> m_custom_checkers;

public:

    explicit PhysicalDeviceSelector(const vk::raii::Instance& instance) 
        : m_instance(instance) {}

    PhysicalDeviceSelector& set_surface(const vk::raii::SurfaceKHR& surface) {
        m_surface = &surface;
        return *this;
    }

    PhysicalDeviceSelector& require_api_version(uint32_t version) {
        m_required_api_version = version;
        return *this;
    }

    PhysicalDeviceSelector& require_extensions(const std::vector<std::string>& extensions) {
        m_required_extensions = extensions;
        return *this;
    }

    PhysicalDeviceSelector& require_gpu_type(vk::PhysicalDeviceType type) {
        m_required_type = type;
        return *this;
    }

    PhysicalDeviceSelector& require_queue_flags(vk::QueueFlags flags) {
        m_required_queue_flags = flags;
        return *this;
    }

    PhysicalDeviceSelector& require_custom_check(std::function<bool(const vk::raii::PhysicalDevice&)> check) {
        m_custom_checkers.push_back(check);
        return *this;
    }

    template<typename... Features>
    PhysicalDeviceSelector& require_features(const vk::StructureChain<Features...>& required_features) {
        m_feature_checkers.push_back([required_features](const vk::raii::PhysicalDevice& device) {
            auto supported_features = device.getFeatures2<Features...>();
            return (detail::FeatureChecker<Features>::check(
                required_features.template get<Features>(),
                supported_features.template get<Features>()
            ) && ...);
        });
        return *this;
    }
    
    std::optional<Selection> select() const {
        auto logger = utils::get_logger("rhi.device");
        vk::raii::PhysicalDevices devices(m_instance);
        for (const auto& device : devices) {
            if (check_device(device)) {
                auto queue_idx = find_queue_family(device);
                if (queue_idx) {
                    return Selection{ device, *queue_idx };
                }
                const std::string device_name = device.getProperties().deviceName.data();
                logger->debug(
                    "Device '{}' rejected: no compatible queue family for required flags.",
                    device_name
                );
            }
        }
        return std::nullopt;
    }

private:
    bool check_device(const vk::raii::PhysicalDevice& device) const {
        auto logger = utils::get_logger("rhi.device");
        auto properties = device.getProperties();
        const std::string device_name = properties.deviceName.data();
        
        // Check api version
        if (properties.apiVersion < m_required_api_version) {
            logger->debug("Device '{}' rejected: API version too low.", device_name);
            return false;
        }

        // Check gpu type
        if (m_required_type && properties.deviceType != *m_required_type) {
            logger->debug("Device '{}' rejected: GPU type does not match requirement.", device_name);
            return false;
        }

        // Check extensions
        if (!m_required_extensions.empty()) {
            auto available_extensions = device.enumerateDeviceExtensionProperties();
            for (const auto& req : m_required_extensions) {
                bool found = std::ranges::any_of(available_extensions, [&](const auto& ext) {
                    return req == ext.extensionName;
                });
                if (!found) {
                    logger->debug("Device '{}' rejected: missing extension '{}'.", device_name, req);
                    return false;
                }
            }
        }

        for (const auto& checker : m_feature_checkers) {
            if (!checker(device)) {
                logger->debug("Device '{}' rejected: required feature chain not supported.", device_name);
                return false;
            }
        }

        for (const auto& checker : m_custom_checkers) {
            if (!checker(device)) {
                logger->debug("Device '{}' rejected: custom checker failed.", device_name);
                return false;
            }
        }

        return true;
    }

    std::optional<uint32_t> find_queue_family(const vk::raii::PhysicalDevice& device) const {
        auto queue_families = device.getQueueFamilyProperties();
        for (uint32_t i = 0; i < queue_families.size(); ++i) {
            const auto& props = queue_families[i];
            
            if ((props.queueFlags & m_required_queue_flags) != m_required_queue_flags) continue;
            
            if (m_surface && !device.getSurfaceSupportKHR(i, *m_surface)) continue;
            
            return i;
        }
        return std::nullopt;
    }
};

class Device {
private:
    Context* m_context{}; 
    vk::raii::PhysicalDevice m_physical_device{nullptr};
    vk::raii::Device m_device{nullptr};
    vk::raii::Queue m_queue{nullptr};
    uint32_t m_queue_family_index{0};

    std::vector<std::string> m_device_extensions = {
        vk::KHRSwapchainExtensionName,
        vk::KHRSpirv14ExtensionName,
        vk::KHRSynchronization2ExtensionName,
        vk::KHRCreateRenderpass2ExtensionName,
        "VK_EXT_swapchain_maintenance1",
#if defined(__APPLE__)
        "VK_KHR_portability_subset",
        "VK_KHR_dynamic_rendering"
#endif
    };

#if defined(__APPLE__)
    using DeviceFeatureChainType = vk::StructureChain<
        vk::PhysicalDeviceFeatures2,
        vk::PhysicalDeviceDynamicRenderingFeatures,
        vk::PhysicalDeviceSynchronization2Features,
        vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT,
        vk::PhysicalDeviceVulkan11Features,
        vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT>;

    std::function<DeviceFeatureChainType()> m_device_feature_chain_generator = []() {
        vk::PhysicalDeviceFeatures2 physical_device_features2{};
        physical_device_features2.features.samplerAnisotropy = true;
        physical_device_features2.features.shaderStorageImageReadWithoutFormat = true;
        physical_device_features2.features.shaderStorageImageWriteWithoutFormat = true;

        vk::PhysicalDeviceDynamicRenderingFeatures dynamic_rendering_features{};
        dynamic_rendering_features.dynamicRendering = true;

        vk::PhysicalDeviceSynchronization2Features synchronization2_features{};
        synchronization2_features.synchronization2 = true;

        vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT extended_dynamic_state_features{};
        extended_dynamic_state_features.extendedDynamicState = true;

        vk::PhysicalDeviceVulkan11Features vulkan11_features{};
        vulkan11_features.shaderDrawParameters = true;

        vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT swapchain_maintenance_features{};
        swapchain_maintenance_features.swapchainMaintenance1 = true;

        return DeviceFeatureChainType{
            physical_device_features2,
            dynamic_rendering_features,
            synchronization2_features,
            extended_dynamic_state_features,
            vulkan11_features,
            swapchain_maintenance_features
        };
    };
#else
    using DeviceFeatureChainType = vk::StructureChain<
        vk::PhysicalDeviceFeatures2,
        vk::PhysicalDeviceVulkan11Features,
        vk::PhysicalDeviceVulkan13Features,
        vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT,
        vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT>;

    std::function<DeviceFeatureChainType()> m_device_feature_chain_generator = []() {

        vk::PhysicalDeviceFeatures2 physical_device_features2{};
        physical_device_features2.features.samplerAnisotropy = true;
        physical_device_features2.features.shaderStorageImageReadWithoutFormat = true;
        physical_device_features2.features.shaderStorageImageWriteWithoutFormat = true;

        vk::PhysicalDeviceVulkan13Features vulkan13_features{};
        vulkan13_features.dynamicRendering = true;
        vulkan13_features.synchronization2 = true;

        vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT extended_dynamic_state_features{};
        extended_dynamic_state_features.extendedDynamicState = true;

        vk::PhysicalDeviceVulkan11Features vulkan11_features{};
        vulkan11_features.shaderDrawParameters = true;

        vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT swapchain_maintenance_features{};
        swapchain_maintenance_features.swapchainMaintenance1 = true;

        return DeviceFeatureChainType{
            physical_device_features2,
            vulkan11_features,
            vulkan13_features,
            extended_dynamic_state_features,
            swapchain_maintenance_features
        };
    };
#endif

private:
    void select_physical_device(const vk::raii::Instance& instance, const vk::raii::SurfaceKHR& surface) {
        auto logger = utils::get_logger("rhi.device");
        PhysicalDeviceSelector selector(instance);
        auto result = selector
            .set_surface(surface)
            .require_api_version(
#if defined(__APPLE__)
                vk::ApiVersion12
#else
                vk::ApiVersion13
#endif
            )
            .require_extensions(m_device_extensions)
            .require_queue_flags(vk::QueueFlagBits::eGraphics | vk::QueueFlagBits::eCompute | vk::QueueFlagBits::eTransfer)
            .require_features(m_device_feature_chain_generator())
            .select();

        if (result) {
            m_physical_device = std::move(result->physical_device);
            m_queue_family_index = result->queue_family_index;
            const std::string device_name = m_physical_device.getProperties().deviceName.data();
            logger->info(
                "Physical device selected: '{}' (queue_family_index={})",
                device_name,
                m_queue_family_index
            );
        } else {
            logger->error("Failed to find suitable physical device.");
            throw std::runtime_error("Failed to find suitable physical device");
        }
    }

    void create_logical_device() {
        auto logger = utils::get_logger("rhi.device");
        auto device_result = make_device(
            m_physical_device,
            m_device_extensions,
            m_device_feature_chain_generator(),
            m_queue_family_index
        );

        if (!device_result.has_value()) {
            logger->error("Failed to create logical device.");
            throw std::runtime_error("Failed to create logical device.");
        }

        m_device = std::move(device_result.value());
        logger->info("Logical device created.");
    }

    void create_queue() {
        m_queue = m_device.getQueue(m_queue_family_index, 0);
    }

public:
    Device(Context *context) : m_context(context) {
        select_physical_device(context->instance(), context->surface());
        create_logical_device();
        create_queue();
        utils::get_logger("rhi.device")->info("Graphics queue created.");
    }

    const vk::raii::PhysicalDevice& physical_device() const { return m_physical_device; }
    const vk::raii::Device& device() const { return m_device; }
    const vk::raii::Queue& queue() const { return m_queue; }
    uint32_t queue_family_index() const { return m_queue_family_index; }

    void wait_idle() const {
        m_device.waitIdle();
    }
};

} // namespace rtr::rhi
