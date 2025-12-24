#pragma once

#include <algorithm>
#include <functional>
#include <iostream>
#include <vector>

#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vulkan_structs.hpp"

namespace rtr {

class VKPhysicalDevicePickerRuleBase {
public:
    virtual bool check(const vk::raii::PhysicalDevice& device) const = 0;
    virtual ~VKPhysicalDevicePickerRuleBase() = default;
};

class VKPhysicalDevicePickerCustomRule : public VKPhysicalDevicePickerRuleBase {
private: 
    std::function<bool(vk::raii::PhysicalDevice)> m_checker;

public:
    VKPhysicalDevicePickerCustomRule(std::function<bool(vk::raii::PhysicalDevice)> checker) : m_checker(checker) {}

    bool check(const vk::raii::PhysicalDevice& device) const override {
        return m_checker(device);
    }
};

class VKPhysicalDevicePickerDeviceExtensionRule : public VKPhysicalDevicePickerRuleBase {
private:
    std::vector<std::string> m_required_device_extensions;

public:
    VKPhysicalDevicePickerDeviceExtensionRule(const std::vector<std::string>& required_device_extensions) : m_required_device_extensions(required_device_extensions) {}

    bool check(const vk::raii::PhysicalDevice& device) const override {
        auto availableDeviceExtensions = device.enumerateDeviceExtensionProperties();
        for (const auto& required_ext : m_required_device_extensions) {
            if (std::ranges::none_of(availableDeviceExtensions, 
                [&](const vk::ExtensionProperties& ext) {
                    return required_ext == ext.extensionName;
                })) {
                std::cout << "Device " << device.getProperties().deviceName << " does not support required extension " << required_ext << std::endl;
                std::cout << "Available extensions are: " << std::endl;
                for (const auto& ext : availableDeviceExtensions) {
                    std::cout << "  " << ext.extensionName << std::endl;
                }
                std::cout << std::endl;
                return false;
            }
        }
        return true;
    }
};

class VKPhysicalDevicePickerGpuRule : public VKPhysicalDevicePickerRuleBase {
private:
    vk::PhysicalDeviceType m_device_type;
    
public:
    VKPhysicalDevicePickerGpuRule(vk::PhysicalDeviceType device_type) : m_device_type(device_type) {}

    bool check(const vk::raii::PhysicalDevice& device) const override {
        auto properties = device.getProperties();
        auto result = properties.deviceType == m_device_type;
        if (!result) {
            std::cout << "Device " << properties.deviceName << " is of type " 
                      << vk::to_string(properties.deviceType) 
                      << ", required type is " 
                      << vk::to_string(m_device_type) 
                      << std::endl;
        }
        return result;
    }
};

class VKPhysicalDevicePickerQueueCheckerBase {
public:
    virtual bool check(
        int index, 
        vk::QueueFamilyProperties properties, 
        vk::raii::PhysicalDevice device
    ) const = 0;

    virtual ~VKPhysicalDevicePickerQueueCheckerBase() = default;
};

class VKPhysicalDevicePickerQueueCustomChecker : public VKPhysicalDevicePickerQueueCheckerBase {
protected:
    std::function<bool(int, vk::QueueFamilyProperties, vk::raii::PhysicalDevice)> m_checker;

public:
    VKPhysicalDevicePickerQueueCustomChecker(std::function<bool(int, vk::QueueFamilyProperties, vk::raii::PhysicalDevice)> checker) : m_checker(checker) {}

    bool check(
        int index, 
        vk::QueueFamilyProperties properties, 
        vk::raii::PhysicalDevice device
    ) const override {
        return m_checker(index, properties, device);
    }

};

class VKPhysicalDevicePickerQueuePresentChecker : public VKPhysicalDevicePickerQueueCheckerBase {
protected:
    const vk::raii::SurfaceKHR& m_surface;

public:
    VKPhysicalDevicePickerQueuePresentChecker(const vk::raii::SurfaceKHR& surface) : m_surface(surface) {}

    bool check(int index, vk::QueueFamilyProperties properties, vk::raii::PhysicalDevice device) const override {
        auto result = device.getSurfaceSupportKHR(index, *m_surface);
        if (!result) {
            std::cout << "Queue family index " << index << " does not support presentation to the given surface." << std::endl;
        }
        return result;
    }
};

class VKPhysicalDevicePickerQueueBitsChecker : public VKPhysicalDevicePickerQueueCheckerBase {
protected:
    vk::QueueFlags m_queue_flags;

public:
    VKPhysicalDevicePickerQueueBitsChecker(vk::QueueFlags queue_flags) : m_queue_flags(queue_flags) {}

    bool check(int index, vk::QueueFamilyProperties properties, vk::raii::PhysicalDevice device) const override {
        auto result = (properties.queueFlags & m_queue_flags) != vk::QueueFlags{};
        if (!result) {
            std::cout << "Queue family index " << index << " does not support required queue flags: " << vk::to_string(m_queue_flags) << std::endl;
        }
        return result;
    }
};

class VKPhysicalDevicePickerQueueRule {
protected:
    int& m_queue_index;
    std::vector<std::reference_wrapper<const VKPhysicalDevicePickerQueueCheckerBase>> m_checkers;

public:
    VKPhysicalDevicePickerQueueRule(
        int& queue_index, 
        const std::vector<std::reference_wrapper<const VKPhysicalDevicePickerQueueCheckerBase>>& checkers
    ) : m_queue_index(queue_index), m_checkers(checkers) {
    }

    int& queue_index() {
        return m_queue_index;
    }

    const int& queue_index() const {
        return m_queue_index;
    }

    bool check(const vk::raii::PhysicalDevice& device) const {
        auto queue_families = device.getQueueFamilyProperties();
        for (int i = 0; i < queue_families.size(); ++i) {
            bool all_passed = std::all_of(
                m_checkers.begin(), 
                m_checkers.end(), 
                [&](const auto& checker) {
                    return checker.get().check(i, queue_families[i], device);
                    //            ^^^^^ 添加 .get()
                }
            );
            if (all_passed) {
                m_queue_index = i;
                return true;
            }
        }
        m_queue_index = -1;
        return false;
    }
};

class VKPhysicalDevicePickerApiVersionRule : public VKPhysicalDevicePickerRuleBase {
private:
    uint32_t m_api_version;
public:
    VKPhysicalDevicePickerApiVersionRule(uint32_t api_version) : m_api_version(api_version) {}

    bool check(const vk::raii::PhysicalDevice& device) const override {
        auto properties = device.getProperties();
        auto result = properties.apiVersion >= m_api_version;
        if (!result) {
            std::cout << "Device " << properties.deviceName << " at api version " 
                    << VK_VERSION_MAJOR(properties.apiVersion) << "."
                    << VK_VERSION_MINOR(properties.apiVersion) << "."
                    << VK_VERSION_PATCH(properties.apiVersion) << " does not support required API version "
                    << VK_VERSION_MAJOR(m_api_version) << "."
                    << VK_VERSION_MINOR(m_api_version) << "."
                    << VK_VERSION_PATCH(m_api_version) << std::endl;
        }
        return result;
    }
};

template<typename Feature>
class VKPhysicalDeviceFeatureChecker {
private:
    std::function<bool(const Feature&)> m_checker;

public:
    VKPhysicalDeviceFeatureChecker(std::function<bool(const Feature&)> checker) : m_checker(checker) {}

    template<typename Func>
    VKPhysicalDeviceFeatureChecker(Func&& checker) 
    requires (!std::is_same_v<
        std::remove_cvref_t<Func>, 
        VKPhysicalDeviceFeatureChecker>
    ) : m_checker(std::forward<Func>(checker)) {}

    bool check(const Feature& feature) const {
        return m_checker(feature);
    }
};

namespace detail {
    template<typename T>
    struct FeatureChecker {
        static bool check(const T& required, const T& supported) {
            struct Header {
                vk::StructureType sType;
                void* pNext;
            };
            constexpr size_t offset = sizeof(Header);
            
            const uint8_t* req_ptr = reinterpret_cast<const uint8_t*>(&required);
            const uint8_t* sup_ptr = reinterpret_cast<const uint8_t*>(&supported);
            
            const VkBool32* req_bools = reinterpret_cast<const VkBool32*>(req_ptr + offset);
            const VkBool32* sup_bools = reinterpret_cast<const VkBool32*>(sup_ptr + offset);
            
            size_t num_bools = (sizeof(T) - offset) / sizeof(VkBool32);
            
            for (size_t i = 0; i < num_bools; ++i) {
                if (req_bools[i] && !sup_bools[i]) {
                    return false;
                }
            }
            return true;
        }
    };

    template<>
    struct FeatureChecker<vk::PhysicalDeviceFeatures> {
        static bool check(const vk::PhysicalDeviceFeatures& required, const vk::PhysicalDeviceFeatures& supported) {
            const VkBool32* req_bools = reinterpret_cast<const VkBool32*>(&required);
            const VkBool32* sup_bools = reinterpret_cast<const VkBool32*>(&supported);
            size_t num_bools = sizeof(vk::PhysicalDeviceFeatures) / sizeof(VkBool32);
            
            for (size_t i = 0; i < num_bools; ++i) {
                if (req_bools[i] && !sup_bools[i]) {
                    return false;
                }
            }
            return true;
        }
    };

    template<>
    struct FeatureChecker<vk::PhysicalDeviceFeatures2> {
        static bool check(const vk::PhysicalDeviceFeatures2& required, const vk::PhysicalDeviceFeatures2& supported) {
            return FeatureChecker<vk::PhysicalDeviceFeatures>::check(required.features, supported.features);
        }
    };
}

// New StructureChain-based feature rule
template<typename... Features>
class VKPhysicalDevicePickerFeatureRule : public VKPhysicalDevicePickerRuleBase {
private:
    std::function<bool(const vk::StructureChain<Features...>&)> m_checker;

public:
    // Constructor that takes a checker function operating on the entire StructureChain
    VKPhysicalDevicePickerFeatureRule(
        std::function<bool(const vk::StructureChain<Features...>&)> checker
    ) : m_checker(checker) {}

    // Template constructor for lambda/function objects
    template<typename Func>
    VKPhysicalDevicePickerFeatureRule(Func&& checker) 
    requires (!std::is_same_v<
        std::remove_cvref_t<Func>, 
        VKPhysicalDevicePickerFeatureRule>
    ) : m_checker(std::forward<Func>(checker)) {}

    VKPhysicalDevicePickerFeatureRule(
        const vk::StructureChain<Features...>& feature_chain
    ) {
        m_checker = [feature_chain](const vk::StructureChain<Features...>& device_feature_chain) {
            return (detail::FeatureChecker<Features>::check(
                feature_chain.template get<Features>(), 
                device_feature_chain.template get<Features>()
            ) && ...);
        };
    }

    bool check(const vk::raii::PhysicalDevice& device) const override {
        // Query the device features using the StructureChain types
        auto features = device.getFeatures2<Features...>();
        // Call the user-provided checker with the feature chain
        return m_checker(features);
    }
};

template<typename... Rules>
inline std::optional<vk::raii::PhysicalDevice> pick_physical_device(
    const std::vector<vk::raii::PhysicalDevice>& candidate_physical_devices,
    Rules... rules
) {
    for (const auto& device : candidate_physical_devices) {
        bool all_passed = (rules.check(device) && ...);
        if (all_passed) {
            return device;
        }
    }
    return std::nullopt;
}

inline std::optional<vk::raii::PhysicalDevice> pick_physical_device(
    const std::vector<vk::raii::PhysicalDevice>& candidate_physical_devices,
    const std::vector<std::reference_wrapper<const VKPhysicalDevicePickerRuleBase>>& rules
) {
    for (const auto& device : candidate_physical_devices) {
        bool all_passed = std::all_of(rules.begin(), rules.end(), [&](const auto& rule) {
            return rule.get().check(device);
        });
        if (all_passed) {
            return device;
        }
    }
    return std::nullopt;
}

}