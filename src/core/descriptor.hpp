#pragma once

#include "core/context.hpp"
#include "device.hpp"
#include "vulkan/vulkan_raii.hpp"
#include <vector>
#include <unordered_map>

namespace rtr::core {

// ============================================================================
// DescriptorSetLayout
// ============================================================================

class DescriptorSetLayout {
private:
    Device* m_device;
    vk::raii::DescriptorSetLayout m_layout{nullptr};
    std::vector<vk::DescriptorSetLayoutBinding> m_bindings;

public:
    class Builder {
    private:
        std::vector<vk::DescriptorSetLayoutBinding> m_bindings;

    public:
        Builder& add_binding(
            uint32_t binding,
            vk::DescriptorType type,
            vk::ShaderStageFlags stages,
            uint32_t count = 1
        ) {
            vk::DescriptorSetLayoutBinding layout_binding{};
            layout_binding.binding = binding;
            layout_binding.descriptorType = type;
            layout_binding.descriptorCount = count;
            layout_binding.stageFlags = stages;
            layout_binding.pImmutableSamplers = nullptr;
            
            m_bindings.push_back(layout_binding);
            return *this;
        }

        DescriptorSetLayout build(Device* device) {
            return DescriptorSetLayout(device, m_bindings);
        }
    };

    DescriptorSetLayout(Device* device, const std::vector<vk::DescriptorSetLayoutBinding>& bindings)
        : m_device(device), m_bindings(bindings) {
        vk::DescriptorSetLayoutCreateInfo create_info{};
        create_info.bindingCount = static_cast<uint32_t>(bindings.size());
        create_info.pBindings = bindings.data();

        m_layout = vk::raii::DescriptorSetLayout(device->device(), create_info);
    }

    const Device* device() const { return m_device; }
    const vk::raii::DescriptorSetLayout& layout() const { return m_layout; }
    const std::vector<vk::DescriptorSetLayoutBinding>& bindings() const { return m_bindings; }
};

// ============================================================================
// DescriptorPool
// ============================================================================

class DescriptorPool {
private:
    Device* m_device;
    vk::raii::DescriptorPool m_pool{nullptr};

public:
    class Builder {
    private:
        std::unordered_map<vk::DescriptorType, uint32_t> m_descriptor_counts;
        uint32_t m_max_sets = 0;
        vk::DescriptorPoolCreateFlags m_flags = {};

    public:
        Builder& add_pool_size(vk::DescriptorType type, uint32_t count) {
            m_descriptor_counts[type] += count;
            return *this;
        }

        // 自动根据 layout 计算所需的描述符数量
        Builder& add_layout(const DescriptorSetLayout& layout, uint32_t set_count) {
            // 遍历 layout 的所有 binding
            for (const auto& binding : layout.bindings()) {
                // 累加该类型描述符的总需求量
                m_descriptor_counts[binding.descriptorType] += 
                    binding.descriptorCount * set_count;
            }
            // 累加总的描述符集数量
            m_max_sets += set_count;
            return *this;
        }

        Builder& set_max_sets(uint32_t max_sets) {
            m_max_sets = max_sets;
            return *this;
        }

        Builder& set_flags(vk::DescriptorPoolCreateFlags flags) {
            m_flags = flags;
            return *this;
        }

        DescriptorPool build(Device* device) {
            std::vector<vk::DescriptorPoolSize> pool_sizes;
            pool_sizes.reserve(m_descriptor_counts.size());
            for (const auto& [type, count] : m_descriptor_counts) {
                pool_sizes.push_back({type, count});
            }
            return DescriptorPool(device, pool_sizes, m_max_sets, m_flags);
        }
    };

    DescriptorPool(
        Device* device,
        const std::vector<vk::DescriptorPoolSize>& pool_sizes,
        uint32_t max_sets,
        vk::DescriptorPoolCreateFlags flags = {}
    ) : m_device(device) {
        vk::DescriptorPoolCreateInfo pool_info{};
        pool_info.poolSizeCount = static_cast<uint32_t>(pool_sizes.size());
        pool_info.pPoolSizes = pool_sizes.data();
        pool_info.maxSets = max_sets;
        pool_info.flags = flags;

        m_pool = vk::raii::DescriptorPool(device->device(), pool_info);
    }

    vk::raii::DescriptorSet allocate(const DescriptorSetLayout& layout) {
        vk::DescriptorSetAllocateInfo alloc_info{};
        alloc_info.descriptorPool = *m_pool;
        alloc_info.descriptorSetCount = 1;
        alloc_info.pSetLayouts = &*layout.layout();

        auto sets = m_device->device().allocateDescriptorSets(alloc_info);
        return std::move(sets.front());
    }

    std::vector<vk::raii::DescriptorSet> allocate_multiple(
        const DescriptorSetLayout& layout, 
        uint32_t count
    ) {
        std::vector<vk::DescriptorSetLayout> layouts(count, *layout.layout());
        
        vk::DescriptorSetAllocateInfo alloc_info{};
        alloc_info.descriptorPool = *m_pool;
        alloc_info.descriptorSetCount = count;
        alloc_info.pSetLayouts = layouts.data();

        return m_device->device().allocateDescriptorSets(alloc_info);
    }

    const vk::raii::DescriptorPool& pool() const { return m_pool; }
    const Device* device() const { return m_device; }
};

// ============================================================================
// DescriptorWriter
// ============================================================================

class DescriptorWriter {
private:
    std::vector<vk::WriteDescriptorSet> m_writes;
    std::vector<vk::DescriptorBufferInfo> m_buffer_infos;
    std::vector<vk::DescriptorImageInfo> m_image_infos;

public:
    DescriptorWriter& write_buffer(
        uint32_t binding,
        vk::Buffer buffer,
        vk::DeviceSize offset,
        vk::DeviceSize range,
        vk::DescriptorType type = vk::DescriptorType::eUniformBuffer,
        uint32_t array_element = 0
    ) {
        vk::DescriptorBufferInfo buffer_info{};
        buffer_info.buffer = buffer;
        buffer_info.offset = offset;
        buffer_info.range = range;
        m_buffer_infos.push_back(buffer_info);

        vk::WriteDescriptorSet write{};
        write.dstBinding = binding;
        write.dstArrayElement = array_element;
        write.descriptorType = type;
        write.descriptorCount = 1;
        write.pBufferInfo = &m_buffer_infos.back();
        m_writes.push_back(write);

        return *this;
    }

    DescriptorWriter& write_combined_image(
        uint32_t binding,
        vk::ImageView image_view,
        vk::Sampler sampler,
        vk::ImageLayout layout,
        uint32_t array_element = 0
    ) {
        vk::DescriptorImageInfo image_info{};
        image_info.imageView = image_view;
        image_info.sampler = sampler;
        image_info.imageLayout = layout;
        m_image_infos.push_back(image_info);

        vk::WriteDescriptorSet write{};
        write.dstBinding = binding;
        write.dstArrayElement = array_element;
        write.descriptorType = vk::DescriptorType::eCombinedImageSampler;
        write.descriptorCount = 1;
        write.pImageInfo = &m_image_infos.back();
        m_writes.push_back(write);

        return *this;
    }

    DescriptorWriter& write_image(
        uint32_t binding,
        vk::ImageView image_view,
        vk::ImageLayout layout,
        uint32_t array_element = 0
    ) {
        vk::DescriptorImageInfo image_info{};
        image_info.imageView = image_view;
        image_info.imageLayout = layout;
        // sampler is VK_NULL_HANDLE for eSampledImage
        m_image_infos.push_back(image_info);

        vk::WriteDescriptorSet write{};
        write.dstBinding = binding;
        write.dstArrayElement = array_element;
        write.descriptorType = vk::DescriptorType::eSampledImage;
        write.descriptorCount = 1;
        write.pImageInfo = &m_image_infos.back();
        m_writes.push_back(write);

        return *this;
    }

    DescriptorWriter& write_sampler(
        uint32_t binding,
        vk::Sampler sampler,
        uint32_t array_element = 0
    ) {
        vk::DescriptorImageInfo image_info{};
        image_info.sampler = sampler;
        m_image_infos.push_back(image_info);

        vk::WriteDescriptorSet write{};
        write.dstBinding = binding;
        write.dstArrayElement = array_element;
        write.descriptorType = vk::DescriptorType::eSampler;
        write.descriptorCount = 1;
        write.pImageInfo = &m_image_infos.back();
        m_writes.push_back(write);

        return *this;
    }

    // ========================================================================
    // Array versions for batch updates
    // ========================================================================

    DescriptorWriter& write_buffer_array(
        uint32_t binding,
        const std::vector<vk::Buffer>& buffers,
        vk::DeviceSize offset,
        vk::DeviceSize range,
        vk::DescriptorType type = vk::DescriptorType::eUniformBuffer,
        uint32_t first_array_element = 0
    ) {
        size_t start_index = m_buffer_infos.size();
        
        for (const auto& buffer : buffers) {
            vk::DescriptorBufferInfo buffer_info{};
            buffer_info.buffer = buffer;
            buffer_info.offset = offset;
            buffer_info.range = range;
            m_buffer_infos.push_back(buffer_info);
        }

        vk::WriteDescriptorSet write{};
        write.dstBinding = binding;
        write.dstArrayElement = first_array_element;
        write.descriptorType = type;
        write.descriptorCount = static_cast<uint32_t>(buffers.size());
        write.pBufferInfo = &m_buffer_infos[start_index];
        m_writes.push_back(write);

        return *this;
    }

    DescriptorWriter& write_combined_image_array(
        uint32_t binding,
        const std::vector<vk::ImageView>& image_views,
        vk::Sampler sampler,
        vk::ImageLayout layout,
        uint32_t first_array_element = 0
    ) {
        size_t start_index = m_image_infos.size();
        
        for (const auto& image_view : image_views) {
            vk::DescriptorImageInfo image_info{};
            image_info.imageView = image_view;
            image_info.sampler = sampler;
            image_info.imageLayout = layout;
            m_image_infos.push_back(image_info);
        }

        vk::WriteDescriptorSet write{};
        write.dstBinding = binding;
        write.dstArrayElement = first_array_element;
        write.descriptorType = vk::DescriptorType::eCombinedImageSampler;
        write.descriptorCount = static_cast<uint32_t>(image_views.size());
        write.pImageInfo = &m_image_infos[start_index];
        m_writes.push_back(write);

        return *this;
    }

    DescriptorWriter& write_image_array(
        uint32_t binding,
        const std::vector<vk::ImageView>& image_views,
        vk::ImageLayout layout,
        uint32_t first_array_element = 0
    ) {
        size_t start_index = m_image_infos.size();
        
        for (const auto& image_view : image_views) {
            vk::DescriptorImageInfo image_info{};
            image_info.imageView = image_view;
            image_info.imageLayout = layout;
            // sampler is VK_NULL_HANDLE for eSampledImage
            m_image_infos.push_back(image_info);
        }

        vk::WriteDescriptorSet write{};
        write.dstBinding = binding;
        write.dstArrayElement = first_array_element;
        write.descriptorType = vk::DescriptorType::eSampledImage;
        write.descriptorCount = static_cast<uint32_t>(image_views.size());
        write.pImageInfo = &m_image_infos[start_index];
        m_writes.push_back(write);

        return *this;
    }

    DescriptorWriter& write_sampler_array(
        uint32_t binding,
        const std::vector<vk::Sampler>& samplers,
        uint32_t first_array_element = 0
    ) {
        size_t start_index = m_image_infos.size();
        
        for (const auto& sampler : samplers) {
            vk::DescriptorImageInfo image_info{};
            image_info.sampler = sampler;
            m_image_infos.push_back(image_info);
        }

        vk::WriteDescriptorSet write{};
        write.dstBinding = binding;
        write.dstArrayElement = first_array_element;
        write.descriptorType = vk::DescriptorType::eSampler;
        write.descriptorCount = static_cast<uint32_t>(samplers.size());
        write.pImageInfo = &m_image_infos[start_index];
        m_writes.push_back(write);

        return *this;
    }

    void update(Device* device, vk::DescriptorSet set) {
        // Set the descriptor set for all writes
        for (auto& write : m_writes) {
            write.dstSet = set;
        }

        device->device().updateDescriptorSets(m_writes, nullptr);

        // Clear for reuse
        clear();
    }

    void clear() {
        m_writes.clear();
        m_buffer_infos.clear();
        m_image_infos.clear();
    }

    const std::vector<vk::WriteDescriptorSet>& writes() const {
        return m_writes;
    }

    const std::vector<vk::DescriptorBufferInfo>& buffer_infos() const {
        return m_buffer_infos;
    }

    const std::vector<vk::DescriptorImageInfo>& image_infos() const {
        return m_image_infos;
    }
};

} // namespace rtr::core
