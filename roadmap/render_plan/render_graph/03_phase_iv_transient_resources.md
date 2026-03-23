# Phase IV: Transient 资源管理（~0.5 天）

## 目标

Graph 中通过 `create_texture()` / `create_buffer()` 声明的 transient 资源由 graph 自动创建 Vulkan 对象，并在不再需要时释放。跨帧缓存避免每帧重新分配。

## 为什么需要 Transient 资源管理

SRT 管线中间资源很多：

```
G-Buffer RT0/RT1/RT2 (RGBA16F × 3)     → 只在当帧使用
G-Buffer Depth (D32F)                    → 当帧 + 下帧 temporal reuse
Shadow Map (D32F × 4 cascade)            → 只在当帧使用
HDR Color (RGBA16F)                      → 只在当帧使用
Reservoir Buffer (current/previous)      → 跨帧双缓冲
Motion Vector (RG16F)                    → 当帧 + 下帧
Denoise history (RGBA16F)                → 跨帧
TAA history (RGBA16F)                    → 跨帧
```

手动管理这些资源的创建/销毁/resize 极其繁琐。Graph 应该自动处理。

## 资源缓存池 — `render_graph_resource.hpp`

### 设计思路

每帧重建 graph 但**不重新分配 Vulkan 资源**。用 key-based 缓存池：

```
key = (name, width, height, format, usage_flags)
↓
如果缓存池中已存在匹配的资源 → 复用
如果不存在 → 创建新资源
如果 resize → 标记旧资源为 retired → 创建新资源
```

### 资源缓存

```cpp
// Image 缓存 key
struct ImageCacheKey {
    std::string      name;
    uint32_t         width;
    uint32_t         height;
    uint32_t         depth;
    uint32_t         mip_levels;
    vk::Format       format;
    vk::ImageType    type;
    vk::ImageUsageFlags usage;

    bool operator==(const ImageCacheKey&) const = default;
    // hash 实现省略
};

// Buffer 缓存 key
struct BufferCacheKey {
    std::string          name;
    vk::DeviceSize       size;
    vk::BufferUsageFlags usage;

    bool operator==(const BufferCacheKey&) const = default;
};

class TransientResourcePool {
    rhi::Device& device_;

    // 活跃资源：当前帧正在使用
    std::unordered_map<ImageCacheKey, rhi::Image>   image_cache_;
    std::unordered_map<BufferCacheKey, rhi::Buffer>  buffer_cache_;

    // 已退休资源：等待 GPU 完成后销毁（参考 SceneTargetController 模式）
    struct RetiredResource {
        uint64_t frame_generation;  // 退休时的帧号
        std::variant<rhi::Image, rhi::Buffer> resource;
    };
    std::vector<RetiredResource> retired_;

public:
    // 获取或创建 image
    rhi::Image& acquire_image(const ImageCacheKey& key) {
        auto it = image_cache_.find(key);
        if (it != image_cache_.end()) {
            return it->second;
        }

        // 创建新 image
        auto image = rhi::Image(device_,
            key.width, key.height, key.format,
            vk::ImageTiling::eOptimal,
            key.usage,
            vk::MemoryPropertyFlagBits::eDeviceLocal,
            format_to_aspect(key.format),
            key.mip_levels > 1);

        auto [inserted, _] = image_cache_.emplace(key, std::move(image));
        return inserted->second;
    }

    // 获取或创建 buffer
    rhi::Buffer& acquire_buffer(const BufferCacheKey& key) {
        auto it = buffer_cache_.find(key);
        if (it != buffer_cache_.end()) {
            return it->second;
        }

        auto buffer = rhi::Buffer::create_device_local_buffer(
            device_, key.size, key.usage);

        auto [inserted, _] = buffer_cache_.emplace(key, std::move(buffer));
        return inserted->second;
    }

    // 每帧开始时调用：清理已完成帧的退休资源
    void collect_garbage(uint64_t completed_frame) {
        std::erase_if(retired_, [&](const RetiredResource& r) {
            return r.frame_generation <= completed_frame;
        });
    }

    // resize 时：旧资源退休，新资源创建
    void invalidate(const ImageCacheKey& old_key, uint64_t current_frame) {
        auto it = image_cache_.find(old_key);
        if (it != image_cache_.end()) {
            retired_.push_back({current_frame, std::move(it->second)});
            image_cache_.erase(it);
        }
    }
};
```

### 集成到 RenderGraph

```cpp
class RenderGraph {
    TransientResourcePool& resource_pool_;  // 跨帧持久

    void allocate_transient_resources() {
        for (auto& state : images_) {
            if (state.is_imported) continue;

            ImageCacheKey key{
                .name       = std::string(state.desc.name),
                .width      = state.desc.width,
                .height     = state.desc.height,
                .depth      = state.desc.depth,
                .mip_levels = state.desc.mip_levels,
                .format     = state.desc.format,
                .type       = state.desc.type,
                .usage      = state.accumulated_usage,
            };

            rhi::Image& image = resource_pool_.acquire_image(key);
            state.imported_image = &image;  // 统一通过指针访问
        }

        for (auto& state : buffers_) {
            if (state.is_imported) continue;

            BufferCacheKey key{
                .name  = std::string(state.desc.name),
                .size  = state.desc.size,
                .usage = state.accumulated_usage,
            };

            rhi::Buffer& buffer = resource_pool_.acquire_buffer(key);
            state.imported_buffer = &buffer;
        }
    }
};
```

## Resize 处理

窗口 resize 时，所有尺寸与 swapchain 相关的 transient 资源需要重新创建：

```cpp
void RenderGraph::on_resize(vk::Extent2D new_extent, uint64_t frame_gen) {
    // 遍历所有 transient image，如果尺寸不匹配则 invalidate
    // 下次 acquire 时会自动创建新尺寸的资源
    for (auto& state : images_) {
        if (state.is_imported) continue;
        if (state.desc.width != new_extent.width ||
            state.desc.height != new_extent.height) {
            ImageCacheKey old_key = make_key(state);
            resource_pool_.invalidate(old_key, frame_gen);
        }
    }
}
```

实际上，由于每帧重建 graph，resize 自然处理：
- 下一帧 `create_texture()` 传入新尺寸 → key 不匹配 → 创建新资源
- 旧资源在缓存池中无人引用 → 可以在 GC 时清理

### 自动 GC 策略

```cpp
// 每帧开始时：
void TransientResourcePool::begin_frame(uint64_t current_frame) {
    // 清理 2 帧前退休的资源（kFramesInFlight = 2）
    collect_garbage(current_frame - rhi::kFramesInFlight);

    // 标记本帧未被 acquire 的缓存资源为"冷"
    // 连续 N 帧未使用的资源可以主动释放
    // （可选优化，初期不做）
}
```

## 持久资源 vs Transient 资源

| 类型 | 创建方式 | 生命周期 | 示例 |
|------|----------|----------|------|
| **Imported** | `import_image()` | 用户管理 | swapchain, SDF cascade, probe atlas |
| **Transient (帧内)** | `create_texture()` | graph 帧内创建，缓存跨帧 | G-Buffer RT, HDR color |
| **Persistent** | `import_image()` | 用户管理，跨帧读写 | reservoir (双缓冲), TAA history |

Persistent 资源（如 reservoir previous 帧 buffer）由用户在 pipeline 层管理并 import 到 graph，
graph 不负责其生命周期，只负责 barrier。

## Phase IV 完成标准

- [ ] `TransientResourcePool` 按 key 缓存 image 和 buffer
- [ ] 首帧创建，后续帧复用（零分配）
- [ ] Resize 后旧资源正确退休，新资源正确创建
- [ ] Retired 资源在 GPU 完成后安全销毁（kFramesInFlight 延迟）
- [ ] Usage flags 从 graph 声明自动推导（用户不需手动指定）
- [ ] `RenderContext::get_image()` 正确返回 transient 资源的 Vulkan 对象
- [ ] 内存占用合理：未使用的缓存资源不无限增长
