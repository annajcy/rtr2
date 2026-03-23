# Phase I: 核心数据结构（~0.5 天）

## 目标

定义 render graph 的所有类型：资源句柄、资源描述、pass 节点、资源使用声明。

## 资源句柄与描述 — `render_graph_types.hpp`

### 资源句柄

```cpp
// 轻量句柄 —— 只是一个索引，可以 copy/compare
struct ImageHandle {
    uint32_t index{UINT32_MAX};
    bool valid() const { return index != UINT32_MAX; }
    bool operator==(const ImageHandle&) const = default;
};

struct BufferHandle {
    uint32_t index{UINT32_MAX};
    bool valid() const { return index != UINT32_MAX; }
    bool operator==(const BufferHandle&) const = default;
};
```

> **为什么分 ImageHandle / BufferHandle？**
> Image 有 layout 状态需要追踪，Buffer 没有。合并成一个 ResourceHandle 会让 barrier 推导代码充满 if-else。
> 分开后类型安全，编译器帮你检查 `read_texture()` 不会误传 buffer。

### 资源使用方式

```cpp
// Image 在一个 pass 中的使用方式
enum class ImageAccess : uint32_t {
    ColorAttachmentWrite,       // 作为 color attachment 写入
    DepthAttachmentWrite,       // 作为 depth attachment 写入
    DepthAttachmentRead,        // 作为 depth attachment 只读（depth test, no write）
    SampledGraphics,            // fragment shader 中采样
    SampledCompute,             // compute shader 中采样
    StorageImageRead,           // compute shader storage image 读
    StorageImageWrite,          // compute shader storage image 写
    StorageImageReadWrite,      // compute shader storage image 读写
    TransferSrc,                // blit / copy 源
    TransferDst,                // blit / copy 目的
    Present,                    // swapchain present
};

// Buffer 在一个 pass 中的使用方式
enum class BufferAccess : uint32_t {
    UniformRead,                // uniform buffer 读
    StorageRead,                // storage buffer 读
    StorageWrite,               // storage buffer 写
    StorageReadWrite,           // storage buffer 读写
    TransferSrc,                // copy 源
    TransferDst,                // copy 目的
    VertexBuffer,               // vertex input
    IndexBuffer,                // index input
    IndirectBuffer,             // indirect dispatch/draw
};
```

### 从 Access 推导 Vulkan 参数

```cpp
// 编译时映射表：ImageAccess → (layout, stage, access)
struct ImageAccessInfo {
    vk::ImageLayout         layout;
    vk::PipelineStageFlags2 stage;
    vk::AccessFlags2        access;
    bool                    writes;     // 是否产生写操作
};

constexpr ImageAccessInfo image_access_info(ImageAccess access) {
    switch (access) {
    case ImageAccess::ColorAttachmentWrite:
        return {vk::ImageLayout::eColorAttachmentOptimal,
                vk::PipelineStageFlagBits2::eColorAttachmentOutput,
                vk::AccessFlagBits2::eColorAttachmentWrite,
                true};
    case ImageAccess::DepthAttachmentWrite:
        return {vk::ImageLayout::eDepthAttachmentOptimal,
                vk::PipelineStageFlagBits2::eEarlyFragmentTests |
                vk::PipelineStageFlagBits2::eLateFragmentTests,
                vk::AccessFlagBits2::eDepthStencilAttachmentWrite,
                true};
    case ImageAccess::DepthAttachmentRead:
        return {vk::ImageLayout::eDepthReadOnlyOptimal,
                vk::PipelineStageFlagBits2::eEarlyFragmentTests,
                vk::AccessFlagBits2::eDepthStencilAttachmentRead,
                false};
    case ImageAccess::SampledGraphics:
        return {vk::ImageLayout::eShaderReadOnlyOptimal,
                vk::PipelineStageFlagBits2::eFragmentShader,
                vk::AccessFlagBits2::eShaderSampledRead,
                false};
    case ImageAccess::SampledCompute:
        return {vk::ImageLayout::eShaderReadOnlyOptimal,
                vk::PipelineStageFlagBits2::eComputeShader,
                vk::AccessFlagBits2::eShaderSampledRead,
                false};
    case ImageAccess::StorageImageRead:
        return {vk::ImageLayout::eGeneral,
                vk::PipelineStageFlagBits2::eComputeShader,
                vk::AccessFlagBits2::eShaderStorageRead,
                false};
    case ImageAccess::StorageImageWrite:
        return {vk::ImageLayout::eGeneral,
                vk::PipelineStageFlagBits2::eComputeShader,
                vk::AccessFlagBits2::eShaderStorageWrite,
                true};
    case ImageAccess::StorageImageReadWrite:
        return {vk::ImageLayout::eGeneral,
                vk::PipelineStageFlagBits2::eComputeShader,
                vk::AccessFlagBits2::eShaderStorageRead |
                vk::AccessFlagBits2::eShaderStorageWrite,
                true};
    case ImageAccess::TransferSrc:
        return {vk::ImageLayout::eTransferSrcOptimal,
                vk::PipelineStageFlagBits2::eTransfer,
                vk::AccessFlagBits2::eTransferRead,
                false};
    case ImageAccess::TransferDst:
        return {vk::ImageLayout::eTransferDstOptimal,
                vk::PipelineStageFlagBits2::eTransfer,
                vk::AccessFlagBits2::eTransferWrite,
                true};
    case ImageAccess::Present:
        return {vk::ImageLayout::ePresentSrcKHR,
                vk::PipelineStageFlagBits2::eBottomOfPipe,
                vk::AccessFlagBits2::eNone,
                false};
    }
}

// 同理 buffer_access_info()
struct BufferAccessInfo {
    vk::PipelineStageFlags2 stage;
    vk::AccessFlags2        access;
    bool                    writes;
};

constexpr BufferAccessInfo buffer_access_info(BufferAccess access);
```

### 资源描述（用于 transient 资源创建）

```cpp
struct ImageDesc {
    std::string_view name;
    uint32_t         width{0};
    uint32_t         height{0};
    uint32_t         depth{1};              // 3D texture 用
    uint32_t         mip_levels{1};
    uint32_t         array_layers{1};
    vk::Format       format{vk::Format::eUndefined};
    vk::ImageType    type{vk::ImageType::e2D};
    vk::SampleCountFlagBits samples{vk::SampleCountFlagBits::e1};

    // usage flags 由 graph 从 pass 声明中自动推导，用户不需要指定
    // 例如：如果有 pass 做 ColorAttachmentWrite，自动加 eColorAttachment
    // 如果有 pass 做 SampledCompute，自动加 eSampled
};

struct BufferDesc {
    std::string_view     name;
    vk::DeviceSize       size{0};
    // usage flags 同样自动推导
};
```

### Pass 节点

```cpp
enum class PassType {
    Graphics,       // 光栅化 pass
    Compute,        // compute dispatch
    Transfer,       // blit / copy
};

// 单个 pass 对某个资源的使用记录
struct ImageUsageRecord {
    ImageHandle handle;
    ImageAccess access;
};

struct BufferUsageRecord {
    BufferHandle handle;
    BufferAccess access;
};

// Pass 执行上下文（传给用户 lambda）
struct RenderContext {
    const vk::raii::CommandBuffer& cmd;
    uint32_t                       frame_index;
    vk::Extent2D                   render_extent;

    // 获取 transient 资源的实际 Vulkan 对象
    const rhi::Image&  get_image(ImageHandle h) const;
    const rhi::Buffer& get_buffer(BufferHandle h) const;

    // 获取 image 当前 layout（barrier 已执行后的状态）
    vk::ImageLayout current_layout(ImageHandle h) const;
};

// 用户 pass 回调类型
using PassExecuteFn = std::function<void(RenderContext&)>;

// Pass 节点（graph 内部数据）
struct PassNode {
    std::string                      name;
    PassType                         type;
    std::vector<ImageUsageRecord>    image_usages;
    std::vector<BufferUsageRecord>   buffer_usages;
    PassExecuteFn                    execute_fn;
    uint32_t                         order{0};   // 拓扑排序后的执行顺序
};
```

### 内部资源状态

```cpp
// Graph 追踪每个 image 的当前状态
struct ImageState {
    ImageDesc                desc;
    vk::ImageLayout          current_layout{vk::ImageLayout::eUndefined};
    vk::PipelineStageFlags2  last_stage{vk::PipelineStageFlagBits2::eTopOfPipe};
    vk::AccessFlags2         last_access{vk::AccessFlagBits2::eNone};
    bool                     last_was_write{false};
    vk::ImageUsageFlags      accumulated_usage{};  // 自动推导的 usage flags

    // 导入的外部资源 vs transient
    bool                     is_imported{false};
    rhi::Image*              imported_image{nullptr};     // 外部资源
    vk::Image                imported_vk_image{};         // 外部 vk::Image（swapchain 用）
};

struct BufferState {
    BufferDesc               desc;
    vk::PipelineStageFlags2  last_stage{vk::PipelineStageFlagBits2::eTopOfPipe};
    vk::AccessFlags2         last_access{vk::AccessFlagBits2::eNone};
    bool                     last_was_write{false};
    vk::BufferUsageFlags     accumulated_usage{};

    bool                     is_imported{false};
    rhi::Buffer*             imported_buffer{nullptr};
};
```

## 为什么每帧重建图

```
方案 A: 编译一次，缓存图结构
  + 零编译开销
  - 不能动态增删 pass（SDF 更新按需执行、debug 可视化按需开关）
  - resize 时需要重编译

方案 B: 每帧重建图
  + pass 可以动态开关（if (sdf_dirty_) graph.add_pass(...)）
  + resize 自然处理
  - 每帧有构建 + 编译开销
  - 但只是填结构体 + 拓扑排序 → < 0.1ms，可忽略
```

选择**方案 B**：每帧重建。构建开销可忽略（~20 个 pass × ~50 个资源使用 = 几百次结构体赋值）。

## Phase I 完成标准

- [ ] `ImageHandle` / `BufferHandle` 句柄类型定义
- [ ] `ImageAccess` / `BufferAccess` 枚举完整覆盖 SRT 所有使用场景
- [ ] `image_access_info()` / `buffer_access_info()` 正确映射到 Vulkan 参数
- [ ] `ImageDesc` / `BufferDesc` 资源描述结构
- [ ] `PassNode` 包含名字、类型、资源使用列表、执行回调
- [ ] `ImageState` / `BufferState` 追踪当前同步状态
- [ ] `RenderContext` 提供 cmd + 资源访问接口
- [ ] 所有类型 header-only，零运行时开销
