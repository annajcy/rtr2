#pragma once

#include "device.hpp"
#include "vulkan/vulkan_raii.hpp"
#include <vector>
#include <unordered_map>
#include <string>
#include <sstream>

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

 // 打印布局信息
inline std::string to_string(const DescriptorSetLayout& layout) {
    std::ostringstream oss;
    oss << "DescriptorSetLayout:\n";
    oss << "  Bindings (" << layout.bindings().size() << "):\n";
    
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
    std::vector<vk::DescriptorPoolSize> m_pool_sizes;  // 保存 pool sizes 用于打印
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
    const std::vector<vk::DescriptorPoolSize>& pool_sizes() const { return m_pool_sizes; }
    uint32_t max_sets() const { return m_max_sets; }
    vk::DescriptorPoolCreateFlags flags() const { return m_flags; }
    const Device* device() const { return m_device; }
};

  // 打印 Pool 信息
inline std::string to_string(const DescriptorPool& descriptor_pool) {
    std::ostringstream oss;
    oss << "DescriptorPool:\n";
    oss << "  Max Sets: " << descriptor_pool.max_sets() << "\n";
    oss << "  Flags: " << vk::to_string(descriptor_pool.flags()) << "\n";
    oss << "  Pool Sizes (" << descriptor_pool.pool_sizes().size() << "):\n";
    
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

 // 打印 Writer 信息
inline std::string to_string(const DescriptorWriter& writer) {
    std::ostringstream oss;
    oss << "DescriptorWriter:\n";
    oss << "  Write Operations: " << writer.writes().size() << "\n";
    oss << "  Buffer Infos: " << writer.buffer_infos().size() << "\n";
    oss << "  Image Infos: " << writer.image_infos().size() << "\n";
    
    for (size_t i = 0; i < writer.writes().size(); ++i) {
        const auto& write = writer.writes()[i];
        oss << "  [" << i << "] Binding " << write.dstBinding;
        oss << ", Array[" << write.dstArrayElement << "]";
        oss << ", Type: " << vk::to_string(write.descriptorType);
        oss << ", Count: " << write.descriptorCount << "\n";
    }
    
    return oss.str();
}

class DescriptorSystem {
public:
    // 描述符集的配置信息
    struct SetConfig {
        std::unique_ptr<DescriptorSetLayout> layout;
        uint32_t set_index;  // 在 pipeline layout 中的 set 索引
        uint32_t count;      // 需要分配的描述符集数量
        
        SetConfig(DescriptorSetLayout::Builder builder, 
                 Device* device, uint32_t idx, uint32_t cnt)
            : layout(std::make_unique<DescriptorSetLayout>(builder.build(device)))
            , set_index(idx)
            , count(cnt) {}
    };

    // 描述符集句柄，封装实际的 vk::raii::DescriptorSet
    struct SetHandle {
        uint32_t set_index;
        uint32_t array_index;
        
        bool operator==(const SetHandle& other) const {
            return set_index == other.set_index && array_index == other.array_index;
        }
    };

public:
    class Builder {
    private:
        Device* m_device;
        std::unordered_map<std::string, SetConfig> m_configs;
        vk::DescriptorPoolCreateFlags m_pool_flags = vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet;

    public:
        explicit Builder(Device* device) : m_device(device) {}

        // 添加一个描述符集配置
        Builder& add_set(
            const std::string& name,
            uint32_t set_index,
            uint32_t count,
            std::function<void(DescriptorSetLayout::Builder&)> configure_layout
        ) {
            DescriptorSetLayout::Builder layout_builder;
            configure_layout(layout_builder);
            
            m_configs.emplace(
                name,
                SetConfig(std::move(layout_builder), m_device, set_index, count)
            );
            return *this;
        }

        Builder& set_pool_flags(vk::DescriptorPoolCreateFlags flags) {
            m_pool_flags = flags;
            return *this;
        }

        // 未来：从 Slang 反射创建
        // Builder& add_set_from_reflection(
        //     const std::string& name,
        //     uint32_t set_index,
        //     uint32_t count,
        //     const slang::IModule* module
        // ) {
        //     // TODO: 实现反射逻辑
        //     return *this;
        // }

        DescriptorSystem build() {
            return DescriptorSystem(m_device, std::move(m_configs), m_pool_flags);
        }
    };

private:
    Device* m_device;
    std::unordered_map<std::string, SetConfig> m_set_configs;
    std::unique_ptr<DescriptorPool> m_pool;
    
    // 存储分配的描述符集：set_name -> vector of descriptor sets
    std::unordered_map<std::string, std::vector<vk::raii::DescriptorSet>> m_allocated_sets;

public:
    DescriptorSystem(
        Device* device,
        std::unordered_map<std::string, SetConfig> configs,
        vk::DescriptorPoolCreateFlags pool_flags
    ) : m_device(device), m_set_configs(std::move(configs)) {
        create_pool(pool_flags);
        allocate_all_sets();
    }

    Device* device() const { return m_device; }
    const std::unordered_map<std::string, SetConfig>& set_configs() const { return m_set_configs; }
    const std::unordered_map<std::string, std::vector<vk::raii::DescriptorSet>>& allocated_sets() const { return m_allocated_sets; }
    const DescriptorPool& pool() const { return *m_pool; }

    // 获取描述符集布局（用于创建 pipeline layout）
    const DescriptorSetLayout& get_layout(const std::string& set_name) const {
        return *m_set_configs.at(set_name).layout;
    }

    // 获取所有布局（按 set_index 排序）
    std::vector<vk::DescriptorSetLayout> get_all_layouts() const {
        std::vector<std::pair<uint32_t, vk::DescriptorSetLayout>> layouts;
        
        for (const auto& [name, config] : m_set_configs) {
            layouts.emplace_back(config.set_index, *config.layout->layout());
        }
        
        // 按 set_index 排序
        std::sort(layouts.begin(), layouts.end(),
                 [](const auto& a, const auto& b) { return a.first < b.first; });
        
        std::vector<vk::DescriptorSetLayout> result;
        result.reserve(layouts.size());
        for (const auto& [idx, layout] : layouts) {
            result.push_back(layout);
        }
        return result;
    }

    // 获取描述符集
    const vk::raii::DescriptorSet& get_set(const std::string& set_name, uint32_t index = 0) const {
        const auto& sets = m_allocated_sets.at(set_name);
        return sets.at(index);
    }

    // 获取所有描述符集（某个类型的）
    const std::vector<vk::raii::DescriptorSet>& get_sets(const std::string& set_name) const {
        return m_allocated_sets.at(set_name);
    }

    // 更新描述符集
    void update_set(
        const std::string& set_name,
        uint32_t index,
        std::function<void(DescriptorWriter&)> write_fn
    ) {
        DescriptorWriter writer;
        write_fn(writer);
        writer.update(m_device, *m_allocated_sets.at(set_name).at(index));
    }

    // 批量更新某类型的所有描述符集
    DescriptorSystem& update_set(
        const std::string& set_name,
        std::function<void(DescriptorWriter&, uint32_t index)> write_fn
    ) {
        auto& sets = m_allocated_sets.at(set_name);
        for (uint32_t i = 0; i < sets.size(); ++i) {
            DescriptorWriter writer;
            write_fn(writer, i);
            writer.update(m_device, *sets[i]);
        }
        return *this;
    }

    // 获取 set 的数量
    int get_set_count(const std::string& set_name) const {
        return m_allocated_sets.at(set_name).size();
    }

    struct PipelineLayoutInfo {
        vk::PipelineLayoutCreateInfo info{};
        std::vector<vk::DescriptorSetLayout> set_layouts;
    };

    static PipelineLayoutInfo make_pipeline_layout_info(
        const DescriptorSystem& system,
        std::span<const vk::PushConstantRange> push_constants = {}
    ) {
        PipelineLayoutInfo result{};
        result.set_layouts = system.get_all_layouts();
        result.info.setLayoutCount = static_cast<uint32_t>(result.set_layouts.size());
        result.info.pSetLayouts = result.set_layouts.data();
        result.info.pushConstantRangeCount = static_cast<uint32_t>(push_constants.size());
        result.info.pPushConstantRanges = push_constants.data();
        return result;
    }

private:
    void create_pool(vk::DescriptorPoolCreateFlags flags) {
        DescriptorPool::Builder pool_builder;
        
        // 根据所有 set config 自动计算 pool 大小
        for (const auto& [name, config] : m_set_configs) {
            pool_builder.add_layout(*config.layout, config.count);
        }
        
        pool_builder.set_flags(flags);
        m_pool = std::make_unique<DescriptorPool>(pool_builder.build(m_device));
    }

    void allocate_all_sets() {
        for (const auto& [name, config] : m_set_configs) {
            auto sets = m_pool->allocate_multiple(*config.layout, config.count);
            m_allocated_sets.emplace(name, std::move(sets));
        }
    }
};


// 打印整个 DescriptorSystem 的信息
inline std::string to_string(const DescriptorSystem& descriptor_system) {
    std::ostringstream oss;
    oss << "DescriptorSystem:\n";
    oss << "  Total Set Types: " << descriptor_system.set_configs().size() << "\n\n";
    
    // 按 set_index 排序输出
    std::vector<std::pair<std::string, const DescriptorSystem::SetConfig*>> sorted_configs;
    for (const auto& [name, config] : descriptor_system.set_configs()) {
        sorted_configs.emplace_back(name, &config);
    }
    std::sort(sorted_configs.begin(), sorted_configs.end(),
                [](const auto& a, const auto& b) { 
                    return a.second->set_index < b.second->set_index; 
                });
    
    for (const auto& [name, config] : sorted_configs) {
        oss << "  Set[" << config->set_index << "] \"" << name << "\":\n";
        oss << "    Allocated Count: " << config->count << "\n";
        oss << "    Layout:\n";
        
        for (const auto& binding : config->layout->bindings()) {
            oss << "      [" << binding.binding << "] ";
            oss << vk::to_string(binding.descriptorType);
            oss << " x" << binding.descriptorCount;
            oss << " (" << vk::to_string(binding.stageFlags) << ")\n";
        }
        oss << "\n";
    }
    
    return oss.str();
}

} // namespace rtr::core
