#pragma once

#include "device.hpp"
#include "vulkan/vulkan_raii.hpp"

#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace rtr::rhi {

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

inline std::string to_string(const DescriptorSetLayout& layout) {
    std::ostringstream oss;
    oss << "DescriptorSetLayout:\n";
    oss << "  Bindings (" << layout.bindings().size() << "):" << "\n";

    for (const auto& binding : layout.bindings()) {
        oss << "    [" << binding.binding << "] ";
        oss << "Type: " << vk::to_string(binding.descriptorType);
        oss << ", Count: " << binding.descriptorCount;
        oss << ", Stages: " << vk::to_string(binding.stageFlags);
        oss << "\n";
    }

    return oss.str();
}

// ============================================================================
// DescriptorPool
// ============================================================================

class DescriptorPool {
private:
    Device* m_device;
    vk::raii::DescriptorPool m_pool{nullptr};
    std::vector<vk::DescriptorPoolSize> m_pool_sizes;
    uint32_t m_max_sets = 0;
    vk::DescriptorPoolCreateFlags m_flags = {};

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

        Builder& add_layout(const DescriptorSetLayout& layout, uint32_t set_count) {
            for (const auto& binding : layout.bindings()) {
                m_descriptor_counts[binding.descriptorType] += binding.descriptorCount * set_count;
            }
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
    ) : m_device(device), m_pool_sizes(pool_sizes), m_max_sets(max_sets), m_flags(flags) {
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

    std::vector<vk::raii::DescriptorSet> allocate_multiple(const DescriptorSetLayout& layout, uint32_t count) {
        std::vector<vk::DescriptorSetLayout> layouts(count, *layout.layout());

        vk::DescriptorSetAllocateInfo alloc_info{};
        alloc_info.descriptorPool = *m_pool;
        alloc_info.descriptorSetCount = count;
        alloc_info.pSetLayouts = layouts.data();

        return m_device->device().allocateDescriptorSets(alloc_info);
    }

    const vk::raii::DescriptorPool& pool() const { return m_pool; }
    const std::vector<vk::DescriptorPoolSize>& pool_sizes() const { return m_pool_sizes; }
    uint32_t max_sets() const { return m_max_sets; }
    vk::DescriptorPoolCreateFlags flags() const { return m_flags; }
};

inline std::string to_string(const DescriptorPool& descriptor_pool) {
    std::ostringstream oss;
    oss << "DescriptorPool:\n";
    oss << "  Max Sets: " << descriptor_pool.max_sets() << "\n";
    oss << "  Flags: " << vk::to_string(descriptor_pool.flags()) << "\n";
    oss << "  Pool Sizes (" << descriptor_pool.pool_sizes().size() << "):" << "\n";

    for (const auto& pool_size : descriptor_pool.pool_sizes()) {
        oss << "    " << vk::to_string(pool_size.type);
        oss << ": " << pool_size.descriptorCount << " descriptors\n";
    }

    return oss.str();
}

// ============================================================================
// DescriptorWriter
// ============================================================================

class DescriptorWriter {
private:
    enum class InfoKind {
        eNone,
        eBuffer,
        eImage
    };

    struct PendingWrite {
        uint32_t dst_binding = 0;
        uint32_t dst_array_element = 0;
        vk::DescriptorType descriptor_type = vk::DescriptorType::eUniformBuffer;
        uint32_t descriptor_count = 0;
        InfoKind info_kind = InfoKind::eNone;
        size_t info_offset = 0;
    };

    std::vector<PendingWrite> m_pending_writes;
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

        const size_t info_offset = m_buffer_infos.size();
        m_buffer_infos.push_back(buffer_info);

        m_pending_writes.push_back(PendingWrite{
            .dst_binding = binding,
            .dst_array_element = array_element,
            .descriptor_type = type,
            .descriptor_count = 1,
            .info_kind = InfoKind::eBuffer,
            .info_offset = info_offset
        });

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

        const size_t info_offset = m_image_infos.size();
        m_image_infos.push_back(image_info);

        m_pending_writes.push_back(PendingWrite{
            .dst_binding = binding,
            .dst_array_element = array_element,
            .descriptor_type = vk::DescriptorType::eCombinedImageSampler,
            .descriptor_count = 1,
            .info_kind = InfoKind::eImage,
            .info_offset = info_offset
        });

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

        const size_t info_offset = m_image_infos.size();
        m_image_infos.push_back(image_info);

        m_pending_writes.push_back(PendingWrite{
            .dst_binding = binding,
            .dst_array_element = array_element,
            .descriptor_type = vk::DescriptorType::eSampledImage,
            .descriptor_count = 1,
            .info_kind = InfoKind::eImage,
            .info_offset = info_offset
        });

        return *this;
    }

    DescriptorWriter& write_storage_image(
        uint32_t binding,
        vk::ImageView image_view,
        vk::ImageLayout layout,
        uint32_t array_element = 0
    ) {
        vk::DescriptorImageInfo image_info{};
        image_info.imageView = image_view;
        image_info.imageLayout = layout;

        const size_t info_offset = m_image_infos.size();
        m_image_infos.push_back(image_info);

        m_pending_writes.push_back(PendingWrite{
            .dst_binding = binding,
            .dst_array_element = array_element,
            .descriptor_type = vk::DescriptorType::eStorageImage,
            .descriptor_count = 1,
            .info_kind = InfoKind::eImage,
            .info_offset = info_offset
        });

        return *this;
    }

    DescriptorWriter& write_sampler(
        uint32_t binding,
        vk::Sampler sampler,
        uint32_t array_element = 0
    ) {
        vk::DescriptorImageInfo image_info{};
        image_info.sampler = sampler;

        const size_t info_offset = m_image_infos.size();
        m_image_infos.push_back(image_info);

        m_pending_writes.push_back(PendingWrite{
            .dst_binding = binding,
            .dst_array_element = array_element,
            .descriptor_type = vk::DescriptorType::eSampler,
            .descriptor_count = 1,
            .info_kind = InfoKind::eImage,
            .info_offset = info_offset
        });

        return *this;
    }

    DescriptorWriter& write_buffer_array(
        uint32_t binding,
        const std::vector<vk::Buffer>& buffers,
        vk::DeviceSize offset,
        vk::DeviceSize range,
        vk::DescriptorType type = vk::DescriptorType::eUniformBuffer,
        uint32_t first_array_element = 0
    ) {
        const size_t start_index = m_buffer_infos.size();

        for (const auto& buffer : buffers) {
            vk::DescriptorBufferInfo buffer_info{};
            buffer_info.buffer = buffer;
            buffer_info.offset = offset;
            buffer_info.range = range;
            m_buffer_infos.push_back(buffer_info);
        }

        m_pending_writes.push_back(PendingWrite{
            .dst_binding = binding,
            .dst_array_element = first_array_element,
            .descriptor_type = type,
            .descriptor_count = static_cast<uint32_t>(buffers.size()),
            .info_kind = InfoKind::eBuffer,
            .info_offset = start_index
        });

        return *this;
    }

    DescriptorWriter& write_combined_image_array(
        uint32_t binding,
        const std::vector<vk::ImageView>& image_views,
        vk::Sampler sampler,
        vk::ImageLayout layout,
        uint32_t first_array_element = 0
    ) {
        const size_t start_index = m_image_infos.size();

        for (const auto& image_view : image_views) {
            vk::DescriptorImageInfo image_info{};
            image_info.imageView = image_view;
            image_info.sampler = sampler;
            image_info.imageLayout = layout;
            m_image_infos.push_back(image_info);
        }

        m_pending_writes.push_back(PendingWrite{
            .dst_binding = binding,
            .dst_array_element = first_array_element,
            .descriptor_type = vk::DescriptorType::eCombinedImageSampler,
            .descriptor_count = static_cast<uint32_t>(image_views.size()),
            .info_kind = InfoKind::eImage,
            .info_offset = start_index
        });

        return *this;
    }

    DescriptorWriter& write_image_array(
        uint32_t binding,
        const std::vector<vk::ImageView>& image_views,
        vk::ImageLayout layout,
        uint32_t first_array_element = 0
    ) {
        const size_t start_index = m_image_infos.size();

        for (const auto& image_view : image_views) {
            vk::DescriptorImageInfo image_info{};
            image_info.imageView = image_view;
            image_info.imageLayout = layout;
            m_image_infos.push_back(image_info);
        }

        m_pending_writes.push_back(PendingWrite{
            .dst_binding = binding,
            .dst_array_element = first_array_element,
            .descriptor_type = vk::DescriptorType::eSampledImage,
            .descriptor_count = static_cast<uint32_t>(image_views.size()),
            .info_kind = InfoKind::eImage,
            .info_offset = start_index
        });

        return *this;
    }

    DescriptorWriter& write_storage_image_array(
        uint32_t binding,
        const std::vector<vk::ImageView>& image_views,
        vk::ImageLayout layout,
        uint32_t first_array_element = 0
    ) {
        const size_t start_index = m_image_infos.size();

        for (const auto& image_view : image_views) {
            vk::DescriptorImageInfo image_info{};
            image_info.imageView = image_view;
            image_info.imageLayout = layout;
            m_image_infos.push_back(image_info);
        }

        m_pending_writes.push_back(PendingWrite{
            .dst_binding = binding,
            .dst_array_element = first_array_element,
            .descriptor_type = vk::DescriptorType::eStorageImage,
            .descriptor_count = static_cast<uint32_t>(image_views.size()),
            .info_kind = InfoKind::eImage,
            .info_offset = start_index
        });

        return *this;
    }

    DescriptorWriter& write_sampler_array(
        uint32_t binding,
        const std::vector<vk::Sampler>& samplers,
        uint32_t first_array_element = 0
    ) {
        const size_t start_index = m_image_infos.size();

        for (const auto& sampler : samplers) {
            vk::DescriptorImageInfo image_info{};
            image_info.sampler = sampler;
            m_image_infos.push_back(image_info);
        }

        m_pending_writes.push_back(PendingWrite{
            .dst_binding = binding,
            .dst_array_element = first_array_element,
            .descriptor_type = vk::DescriptorType::eSampler,
            .descriptor_count = static_cast<uint32_t>(samplers.size()),
            .info_kind = InfoKind::eImage,
            .info_offset = start_index
        });

        return *this;
    }

    void update(Device* device, vk::DescriptorSet set) {
        std::vector<vk::WriteDescriptorSet> writes;
        writes.reserve(m_pending_writes.size());

        for (const auto& pending : m_pending_writes) {
            vk::WriteDescriptorSet write{};
            write.dstSet = set;
            write.dstBinding = pending.dst_binding;
            write.dstArrayElement = pending.dst_array_element;
            write.descriptorType = pending.descriptor_type;
            write.descriptorCount = pending.descriptor_count;

            if (pending.info_kind == InfoKind::eBuffer) {
                write.pBufferInfo = m_buffer_infos.data() + pending.info_offset;
            } else if (pending.info_kind == InfoKind::eImage) {
                write.pImageInfo = m_image_infos.data() + pending.info_offset;
            }

            writes.push_back(write);
        }

        device->device().updateDescriptorSets(writes, nullptr);
        clear();
    }

    void clear() {
        m_pending_writes.clear();
        m_buffer_infos.clear();
        m_image_infos.clear();
    }

    size_t write_count() const { return m_pending_writes.size(); }
    const std::vector<vk::DescriptorBufferInfo>& buffer_infos() const { return m_buffer_infos; }
    const std::vector<vk::DescriptorImageInfo>& image_infos() const { return m_image_infos; }
};

inline std::string to_string(const DescriptorWriter& writer) {
    std::ostringstream oss;
    oss << "DescriptorWriter:\n";
    oss << "  Write Operations: " << writer.write_count() << "\n";
    oss << "  Buffer Infos: " << writer.buffer_infos().size() << "\n";
    oss << "  Image Infos: " << writer.image_infos().size() << "\n";
    return oss.str();
}

} // namespace rtr::rhi
