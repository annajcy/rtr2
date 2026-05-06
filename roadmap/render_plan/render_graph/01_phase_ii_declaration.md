# Phase II: 资源声明与依赖推导（~0.5 天）

## 目标

实现 PassBuilder API，让用户在 add_pass() 的 lambda 中声明资源读写关系，graph 从声明中推导依赖边。

## PassBuilder — `render_graph_builder.hpp`

```cpp
class PassBuilder {
    PassNode& node_;        // 正在构建的 pass
    RenderGraph& graph_;    // 所属 graph

public:
    // ═══════════════════════════════════════════
    //  Image 读写声明
    // ═══════════════════════════════════════════

    // 作为 color attachment 写入
    void write_color(ImageHandle h, uint32_t index = 0) {
        node_.image_usages.push_back({h, ImageAccess::ColorAttachmentWrite});
    }

    // 作为 depth attachment 写入
    void write_depth(ImageHandle h) {
        node_.image_usages.push_back({h, ImageAccess::DepthAttachmentWrite});
    }

    // 作为 depth attachment 只读（shadow map 采样等）
    void read_depth(ImageHandle h) {
        node_.image_usages.push_back({h, ImageAccess::DepthAttachmentRead});
    }

    // 在 fragment shader 中采样（deferred lighting 读 G-Buffer）
    void read_texture(ImageHandle h) {
        auto access = (node_.type == PassType::Compute)
            ? ImageAccess::SampledCompute
            : ImageAccess::SampledGraphics;
        node_.image_usages.push_back({h, access});
    }

    // Compute shader storage image
    void read_storage_image(ImageHandle h) {
        node_.image_usages.push_back({h, ImageAccess::StorageImageRead});
    }

    void write_storage_image(ImageHandle h) {
        node_.image_usages.push_back({h, ImageAccess::StorageImageWrite});
    }

    void readwrite_storage_image(ImageHandle h) {
        node_.image_usages.push_back({h, ImageAccess::StorageImageReadWrite});
    }

    // Transfer (blit)
    void read_transfer(ImageHandle h) {
        node_.image_usages.push_back({h, ImageAccess::TransferSrc});
    }

    void write_transfer(ImageHandle h) {
        node_.image_usages.push_back({h, ImageAccess::TransferDst});
    }

    // Present (swapchain)
    void present(ImageHandle h) {
        node_.image_usages.push_back({h, ImageAccess::Present});
    }

    // ═══════════════════════════════════════════
    //  Buffer 读写声明
    // ═══════════════════════════════════════════

    void read_uniform_buffer(BufferHandle h) {
        node_.buffer_usages.push_back({h, BufferAccess::UniformRead});
    }

    void read_storage_buffer(BufferHandle h) {
        node_.buffer_usages.push_back({h, BufferAccess::StorageRead});
    }

    void write_storage_buffer(BufferHandle h) {
        node_.buffer_usages.push_back({h, BufferAccess::StorageWrite});
    }

    void readwrite_storage_buffer(BufferHandle h) {
        node_.buffer_usages.push_back({h, BufferAccess::StorageReadWrite});
    }

    void read_vertex_buffer(BufferHandle h) {
        node_.buffer_usages.push_back({h, BufferAccess::VertexBuffer});
    }

    void read_index_buffer(BufferHandle h) {
        node_.buffer_usages.push_back({h, BufferAccess::IndexBuffer});
    }

    void read_indirect_buffer(BufferHandle h) {
        node_.buffer_usages.push_back({h, BufferAccess::IndirectBuffer});
    }
};
```

## 依赖推导规则

依赖关系基于**资源的读写先后顺序**推导：

```
规则 1: Write-After-Read (WAR)
  Pass A 读资源 R，Pass B 写资源 R → B 依赖 A（A 必须在 B 前完成读取）

规则 2: Read-After-Write (RAW)
  Pass A 写资源 R，Pass B 读资源 R → B 依赖 A（A 必须在 B 前完成写入）

规则 3: Write-After-Write (WAW)
  Pass A 写资源 R，Pass B 写资源 R → B 依赖 A（写顺序不能乱）

规则 4: Read-After-Read (RAR)
  Pass A 读资源 R，Pass B 读资源 R → 无依赖（多个 pass 可以同时读）
```

### 依赖推导算法

```cpp
struct DependencyEdge {
    uint32_t from_pass;  // 必须先执行的 pass
    uint32_t to_pass;    // 后执行的 pass
};

// 对每个资源，记录最后一次写入的 pass 和所有读取 pass
struct ResourceAccessHistory {
    std::optional<uint32_t> last_writer;
    std::vector<uint32_t>   readers_since_last_write;
};

std::vector<DependencyEdge> derive_dependencies(
    const std::vector<PassNode>& passes
) {
    // 为每个 image/buffer 维护 access history
    std::unordered_map<uint32_t, ResourceAccessHistory> image_history;
    std::unordered_map<uint32_t, ResourceAccessHistory> buffer_history;

    std::vector<DependencyEdge> edges;

    for (uint32_t pass_idx = 0; pass_idx < passes.size(); pass_idx++) {
        const auto& pass = passes[pass_idx];

        for (const auto& usage : pass.image_usages) {
            auto& history = image_history[usage.handle.index];
            auto info = image_access_info(usage.access);

            if (info.writes) {
                // 写操作：依赖最后一个 writer (WAW) 和所有 reader (WAR)
                if (history.last_writer.has_value()) {
                    edges.push_back({history.last_writer.value(), pass_idx});
                }
                for (auto reader : history.readers_since_last_write) {
                    edges.push_back({reader, pass_idx});
                }
                history.last_writer = pass_idx;
                history.readers_since_last_write.clear();
            } else {
                // 读操作：依赖最后一个 writer (RAW)
                if (history.last_writer.has_value()) {
                    edges.push_back({history.last_writer.value(), pass_idx});
                }
                history.readers_since_last_write.push_back(pass_idx);
            }
        }

        // buffer 同理
        for (const auto& usage : pass.buffer_usages) {
            // ... 相同逻辑
        }
    }

    return edges;
}
```

### Usage Flags 自动推导

在用户声明资源使用时，同步累积 `vk::ImageUsageFlags`：

```cpp
void accumulate_image_usage(ImageState& state, ImageAccess access) {
    switch (access) {
    case ImageAccess::ColorAttachmentWrite:
        state.accumulated_usage |= vk::ImageUsageFlagBits::eColorAttachment;
        break;
    case ImageAccess::DepthAttachmentWrite:
    case ImageAccess::DepthAttachmentRead:
        state.accumulated_usage |= vk::ImageUsageFlagBits::eDepthStencilAttachment;
        break;
    case ImageAccess::SampledGraphics:
    case ImageAccess::SampledCompute:
        state.accumulated_usage |= vk::ImageUsageFlagBits::eSampled;
        break;
    case ImageAccess::StorageImageRead:
    case ImageAccess::StorageImageWrite:
    case ImageAccess::StorageImageReadWrite:
        state.accumulated_usage |= vk::ImageUsageFlagBits::eStorage;
        break;
    case ImageAccess::TransferSrc:
        state.accumulated_usage |= vk::ImageUsageFlagBits::eTransferSrc;
        break;
    case ImageAccess::TransferDst:
        state.accumulated_usage |= vk::ImageUsageFlagBits::eTransferDst;
        break;
    case ImageAccess::Present:
        // swapchain image，无需额外 usage flag
        break;
    }
}
```

transient 资源在 `compile()` 时根据 `accumulated_usage` 创建 Vulkan image/buffer。

## RenderGraph 构建 API

```cpp
class RenderGraph {
    rhi::Device& device_;
    FrameContext& frame_ctx_;

    std::vector<ImageState>  images_;
    std::vector<BufferState> buffers_;
    std::vector<PassNode>    passes_;

public:
    // 创建 transient image（graph 管理生命周期）
    ImageHandle create_texture(std::string_view name, const ImageDesc& desc) {
        ImageHandle h{static_cast<uint32_t>(images_.size())};
        images_.push_back({.desc = desc, .is_imported = false});
        return h;
    }

    // 创建 transient buffer
    BufferHandle create_buffer(std::string_view name, const BufferDesc& desc) {
        BufferHandle h{static_cast<uint32_t>(buffers_.size())};
        buffers_.push_back({.desc = desc, .is_imported = false});
        return h;
    }

    // 导入外部 image（已存在，不由 graph 管理生命周期）
    ImageHandle import_image(std::string_view name, rhi::Image& image,
                             vk::ImageLayout initial_layout = vk::ImageLayout::eUndefined) {
        ImageHandle h{static_cast<uint32_t>(images_.size())};
        images_.push_back({
            .current_layout = initial_layout,
            .is_imported = true,
            .imported_image = &image
        });
        return h;
    }

    // 导入 swapchain image（特殊：只有 vk::Image，无 rhi::Image 包装）
    ImageHandle import_swapchain(std::string_view name, vk::Image vk_image,
                                 vk::Format format, vk::Extent2D extent) {
        ImageHandle h{static_cast<uint32_t>(images_.size())};
        images_.push_back({
            .desc = {.name = name, .width = extent.width, .height = extent.height, .format = format},
            .current_layout = vk::ImageLayout::eUndefined,
            .is_imported = true,
            .imported_vk_image = vk_image
        });
        return h;
    }

    // 导入外部 buffer
    BufferHandle import_buffer(std::string_view name, rhi::Buffer& buffer) {
        BufferHandle h{static_cast<uint32_t>(buffers_.size())};
        buffers_.push_back({.is_imported = true, .imported_buffer = &buffer});
        return h;
    }

    // 添加 pass
    template<typename SetupFn>
    void add_pass(std::string_view name, PassType type, SetupFn&& setup) {
        PassNode node;
        node.name = std::string(name);
        node.type = type;

        PassBuilder builder(node, *this);
        node.execute_fn = setup(builder);

        passes_.push_back(std::move(node));
    }

    void compile_and_execute();
};
```

## Phase II 完成标准

- [ ] `PassBuilder` 提供完整的读写声明 API
- [ ] `read_texture()` 根据 pass type 自动选择 `SampledGraphics` / `SampledCompute`
- [ ] 依赖推导正确处理 RAW / WAR / WAW / RAR 四种关系
- [ ] Usage flags 自动从 pass 声明累积
- [ ] `RenderGraph` 的 `create_texture` / `import_image` / `add_pass` API 可用
- [ ] 单元测试：3 pass 线性链 → 正确推导 2 条依赖边
- [ ] 单元测试：2 pass 同时读 → 无依赖边（RAR）
- [ ] 单元测试：1 write + 2 read → 2 条 RAW 边
