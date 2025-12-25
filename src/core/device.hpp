#pragma once

#include "context.hpp"
#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vulkan_structs.hpp"

#include <functional>
#include <iostream>
#include <optional>
#include <vector>
#include <algorithm>

namespace rtr::core {

namespace detail {
    template<typename T>
    struct FeatureChecker;

    template<>
    struct FeatureChecker<vk::PhysicalDeviceFeatures> {
        static bool check(const vk::PhysicalDeviceFeatures& required, const vk::PhysicalDeviceFeatures& supported) {
            if (required.robustBufferAccess && !supported.robustBufferAccess) return false;
            if (required.fullDrawIndexUint32 && !supported.fullDrawIndexUint32) return false;
            if (required.imageCubeArray && !supported.imageCubeArray) return false;
            if (required.independentBlend && !supported.independentBlend) return false;
            if (required.geometryShader && !supported.geometryShader) return false;
            if (required.tessellationShader && !supported.tessellationShader) return false;
            if (required.sampleRateShading && !supported.sampleRateShading) return false;
            if (required.dualSrcBlend && !supported.dualSrcBlend) return false;
            if (required.logicOp && !supported.logicOp) return false;
            if (required.multiDrawIndirect && !supported.multiDrawIndirect) return false;
            if (required.drawIndirectFirstInstance && !supported.drawIndirectFirstInstance) return false;
            if (required.depthClamp && !supported.depthClamp) return false;
            if (required.depthBiasClamp && !supported.depthBiasClamp) return false;
            if (required.fillModeNonSolid && !supported.fillModeNonSolid) return false;
            if (required.depthBounds && !supported.depthBounds) return false;
            if (required.wideLines && !supported.wideLines) return false;
            if (required.largePoints && !supported.largePoints) return false;
            if (required.alphaToOne && !supported.alphaToOne) return false;
            if (required.multiViewport && !supported.multiViewport) return false;
            if (required.samplerAnisotropy && !supported.samplerAnisotropy) return false;
            if (required.textureCompressionETC2 && !supported.textureCompressionETC2) return false;
            if (required.textureCompressionASTC_LDR && !supported.textureCompressionASTC_LDR) return false;
            if (required.textureCompressionBC && !supported.textureCompressionBC) return false;
            if (required.occlusionQueryPrecise && !supported.occlusionQueryPrecise) return false;
            if (required.pipelineStatisticsQuery && !supported.pipelineStatisticsQuery) return false;
            if (required.vertexPipelineStoresAndAtomics && !supported.vertexPipelineStoresAndAtomics) return false;
            if (required.fragmentStoresAndAtomics && !supported.fragmentStoresAndAtomics) return false;
            if (required.shaderTessellationAndGeometryPointSize && !supported.shaderTessellationAndGeometryPointSize) return false;
            if (required.shaderImageGatherExtended && !supported.shaderImageGatherExtended) return false;
            if (required.shaderStorageImageExtendedFormats && !supported.shaderStorageImageExtendedFormats) return false;
            if (required.shaderStorageImageMultisample && !supported.shaderStorageImageMultisample) return false;
            if (required.shaderStorageImageReadWithoutFormat && !supported.shaderStorageImageReadWithoutFormat) return false;
            if (required.shaderStorageImageWriteWithoutFormat && !supported.shaderStorageImageWriteWithoutFormat) return false;
            if (required.shaderUniformBufferArrayDynamicIndexing && !supported.shaderUniformBufferArrayDynamicIndexing) return false;
            if (required.shaderSampledImageArrayDynamicIndexing && !supported.shaderSampledImageArrayDynamicIndexing) return false;
            if (required.shaderStorageBufferArrayDynamicIndexing && !supported.shaderStorageBufferArrayDynamicIndexing) return false;
            if (required.shaderStorageImageArrayDynamicIndexing && !supported.shaderStorageImageArrayDynamicIndexing) return false;
            if (required.shaderClipDistance && !supported.shaderClipDistance) return false;
            if (required.shaderCullDistance && !supported.shaderCullDistance) return false;
            if (required.shaderFloat64 && !supported.shaderFloat64) return false;
            if (required.shaderInt64 && !supported.shaderInt64) return false;
            if (required.shaderInt16 && !supported.shaderInt16) return false;
            if (required.shaderResourceResidency && !supported.shaderResourceResidency) return false;
            if (required.shaderResourceMinLod && !supported.shaderResourceMinLod) return false;
            if (required.sparseBinding && !supported.sparseBinding) return false;
            if (required.sparseResidencyBuffer && !supported.sparseResidencyBuffer) return false;
            if (required.sparseResidencyImage2D && !supported.sparseResidencyImage2D) return false;
            if (required.sparseResidencyImage3D && !supported.sparseResidencyImage3D) return false;
            if (required.sparseResidency2Samples && !supported.sparseResidency2Samples) return false;
            if (required.sparseResidency4Samples && !supported.sparseResidency4Samples) return false;
            if (required.sparseResidency8Samples && !supported.sparseResidency8Samples) return false;
            if (required.sparseResidency16Samples && !supported.sparseResidency16Samples) return false;
            if (required.sparseResidencyAliased && !supported.sparseResidencyAliased) return false;
            if (required.variableMultisampleRate && !supported.variableMultisampleRate) return false;
            if (required.inheritedQueries && !supported.inheritedQueries) return false;
            return true;
        }
    };

    template<>
    struct FeatureChecker<vk::PhysicalDeviceVulkan11Features> {
        static bool check(const vk::PhysicalDeviceVulkan11Features& required, const vk::PhysicalDeviceVulkan11Features& supported) {
            if (required.storageBuffer16BitAccess && !supported.storageBuffer16BitAccess) return false;
            if (required.uniformAndStorageBuffer16BitAccess && !supported.uniformAndStorageBuffer16BitAccess) return false;
            if (required.storagePushConstant16 && !supported.storagePushConstant16) return false;
            if (required.storageInputOutput16 && !supported.storageInputOutput16) return false;
            if (required.multiview && !supported.multiview) return false;
            if (required.multiviewGeometryShader && !supported.multiviewGeometryShader) return false;
            if (required.multiviewTessellationShader && !supported.multiviewTessellationShader) return false;
            if (required.variablePointersStorageBuffer && !supported.variablePointersStorageBuffer) return false;
            if (required.variablePointers && !supported.variablePointers) return false;
            if (required.protectedMemory && !supported.protectedMemory) return false;
            if (required.samplerYcbcrConversion && !supported.samplerYcbcrConversion) return false;
            if (required.shaderDrawParameters && !supported.shaderDrawParameters) return false;
            return true;
        }
    };

    template<>
    struct FeatureChecker<vk::PhysicalDeviceVulkan13Features> {
        static bool check(const vk::PhysicalDeviceVulkan13Features& required, const vk::PhysicalDeviceVulkan13Features& supported) {
            if (required.robustImageAccess && !supported.robustImageAccess) return false;
            if (required.inlineUniformBlock && !supported.inlineUniformBlock) return false;
            if (required.descriptorBindingInlineUniformBlockUpdateAfterBind && !supported.descriptorBindingInlineUniformBlockUpdateAfterBind) return false;
            if (required.pipelineCreationCacheControl && !supported.pipelineCreationCacheControl) return false;
            if (required.privateData && !supported.privateData) return false;
            if (required.shaderDemoteToHelperInvocation && !supported.shaderDemoteToHelperInvocation) return false;
            if (required.shaderTerminateInvocation && !supported.shaderTerminateInvocation) return false;
            if (required.subgroupSizeControl && !supported.subgroupSizeControl) return false;
            if (required.computeFullSubgroups && !supported.computeFullSubgroups) return false;
            if (required.synchronization2 && !supported.synchronization2) return false;
            if (required.textureCompressionASTC_HDR && !supported.textureCompressionASTC_HDR) return false;
            if (required.shaderZeroInitializeWorkgroupMemory && !supported.shaderZeroInitializeWorkgroupMemory) return false;
            if (required.dynamicRendering && !supported.dynamicRendering) return false;
            if (required.shaderIntegerDotProduct && !supported.shaderIntegerDotProduct) return false;
            if (required.maintenance4 && !supported.maintenance4) return false;
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
            return (!required.synchronization2 || supported.synchronization2);
        }
    };

    template<>
    struct FeatureChecker<vk::PhysicalDeviceDynamicRenderingFeatures> {
        static bool check(const vk::PhysicalDeviceDynamicRenderingFeatures& required, const vk::PhysicalDeviceDynamicRenderingFeatures& supported) {
            return (!required.dynamicRendering || supported.dynamicRendering);
        }
    };

    template<>
    struct FeatureChecker<vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT> {
        static bool check(const vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT& required, const vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT& supported) {
            return (!required.extendedDynamicState || supported.extendedDynamicState);
        }
    };

    template<>
    struct FeatureChecker<vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT> {
        static bool check(const vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT& required, const vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT& supported) {
            return (!required.swapchainMaintenance1 || supported.swapchainMaintenance1);
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
        vk::raii::PhysicalDevices devices(m_instance);
        for (const auto& device : devices) {
            if (check_device(device)) {
                auto queue_idx = find_queue_family(device);
                if (queue_idx) {
                    return Selection{ device, *queue_idx };
                }
            }
        }
        return std::nullopt;
    }

private:
    bool check_device(const vk::raii::PhysicalDevice& device) const {
        auto properties = device.getProperties();
        
        // Check api version
        if (properties.apiVersion < m_required_api_version) {
            std::cout << "Device " << properties.deviceName << " API version too low." << std::endl;
            return false;
        }

        // Check gpu type
        if (m_required_type && properties.deviceType != *m_required_type) {
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
                    std::cout << "Device " << properties.deviceName << " missing extension: " << req << std::endl;
                    return false;
                }
            }
        }

        for (const auto& checker : m_feature_checkers) {
            if (!checker(device)) return false;
        }

        for (const auto& checker : m_custom_checkers) {
            if (!checker(device)) return false;
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

        vk::PhysicalDeviceFeatures2 physical_device_features2{};

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
        vk::PhysicalDeviceVulkan13Features vulkan13_features{};
        vulkan13_features.dynamicRendering = true;
        vulkan13_features.synchronization2 = true;

        vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT extended_dynamic_state_features{};
        extended_dynamic_state_features.extendedDynamicState = true;

        vk::PhysicalDeviceVulkan11Features vulkan11_features{};
        vulkan11_features.shaderDrawParameters = true;

        vk::PhysicalDeviceSwapchainMaintenance1FeaturesEXT swapchain_maintenance_features{};
        swapchain_maintenance_features.swapchainMaintenance1 = true;

        vk::PhysicalDeviceFeatures2 physical_device_features2{};

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
            std::cout << "Physical device selected: " << m_physical_device.getProperties().deviceName << std::endl;
        } else {
            throw std::runtime_error("Failed to find suitable physical device");
        }
    }

    void create_logical_device() {
        auto device_result = make_device(
            m_physical_device,
            m_device_extensions,
            m_device_feature_chain_generator(),
            m_queue_family_index
        );

        if (!device_result.has_value()) {
            throw std::runtime_error("Failed to create logical device.");
        }

        m_device = std::move(device_result.value());
    }

    void create_queue() {
        m_queue = m_device.getQueue(m_queue_family_index, 0);
    }

public:
    Device(Context *context) : m_context(context) {
        select_physical_device(context->instance(), context->surface());
        create_logical_device();
        create_queue();
    }

    const vk::raii::PhysicalDevice& physical_device() const { return m_physical_device; }
    const vk::raii::Device& device() const { return m_device; }
    const vk::raii::Queue& queue() const { return m_queue; }
    uint32_t queue_family_index() const { return m_queue_family_index; }

    void waitIdle() const {
        m_device.waitIdle();
    }

    const Context* context() const {
        return m_context;
    }
};

}
