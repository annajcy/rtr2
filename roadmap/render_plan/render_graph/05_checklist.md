# Render Graph 实施检查清单

## Phase I: 核心数据结构（Day 0 上午）

### 类型定义
- [ ] `ImageHandle` / `BufferHandle` — 轻量句柄（uint32_t index）
- [ ] `ImageAccess` 枚举 — 11 种 image 使用方式
- [ ] `BufferAccess` 枚举 — 9 种 buffer 使用方式
- [ ] `image_access_info()` — ImageAccess → (layout, stage, access, writes)
- [ ] `buffer_access_info()` — BufferAccess → (stage, access, writes)
- [ ] `ImageDesc` / `BufferDesc` — transient 资源描述
- [ ] `PassNode` — pass 名字 + 类型 + 资源使用列表 + 执行回调
- [ ] `ImageState` / `BufferState` — 运行时同步状态追踪
- [ ] `RenderContext` — pass 执行上下文（cmd + 资源访问）

### 验证
- [ ] 所有 ImageAccess 映射到正确的 Vulkan layout/stage/access
- [ ] writes 标志正确（ColorAttachmentWrite=true, SampledCompute=false 等）

## Phase II: 资源声明与依赖推导（Day 0 下午）

### PassBuilder API
- [ ] `write_color()` / `write_depth()` — attachment 写入
- [ ] `read_depth()` — depth 只读
- [ ] `read_texture()` — 自动区分 Graphics/Compute stage
- [ ] `read_storage_image()` / `write_storage_image()` / `readwrite_storage_image()`
- [ ] `read_transfer()` / `write_transfer()` / `present()`
- [ ] `read_uniform_buffer()` / `read_storage_buffer()` / `write_storage_buffer()`
- [ ] `readwrite_storage_buffer()` / `read_vertex_buffer()` / `read_index_buffer()`

### RenderGraph API
- [ ] `create_texture()` — transient image 声明
- [ ] `create_buffer()` — transient buffer 声明
- [ ] `import_image()` — 外部 image 导入（含初始 layout）
- [ ] `import_swapchain()` — swapchain image 导入
- [ ] `import_buffer()` — 外部 buffer 导入
- [ ] `add_pass()` — 模板方法，接收 setup lambda

### 依赖推导
- [ ] `derive_dependencies()` — 从 pass 声明推导依赖边
- [ ] RAW (read-after-write) 正确
- [ ] WAR (write-after-read) 正确
- [ ] WAW (write-after-write) 正确
- [ ] RAR (read-after-read) 无冗余边
- [ ] Usage flags 自动累积（`accumulated_usage`）

### 单元测试
- [ ] 3 pass 线性链 → 2 条边
- [ ] 2 pass 并行读 → 0 条边
- [ ] 1 writer + 2 readers → 2 条 RAW 边
- [ ] 菱形依赖 (A→B,C→D) → 正确

## Phase III: 图编译与执行（Day 1 上午）

### 拓扑排序
- [ ] Kahn's algorithm 实现
- [ ] 环检测 + assert
- [ ] 同入度 pass 保持声明顺序
- [ ] 20 pass 排序 < 1μs

### Barrier 推导
- [ ] `compute_barriers()` — 逐 pass 对比当前状态 vs 要求状态
- [ ] Image layout transition 正确
- [ ] Buffer memory barrier 正确
- [ ] RAR 同 layout 跳过（无冗余 barrier）
- [ ] 多 barrier 合并为单次 `pipelineBarrier2()` 调用

### 执行
- [ ] `GraphExecutor::execute()` — barrier → pass → barrier → pass → ...
- [ ] Debug label (`beginDebugUtilsLabelEXT` / `endDebugUtilsLabelEXT`)
- [ ] `compile_and_execute()` 一键完成

### 验证
- [ ] Vulkan 验证层零同步报错
- [ ] RenderDoc 中 pass label 可见

## Phase IV: Transient 资源管理（Day 1 下午）

### 资源池
- [ ] `TransientResourcePool` — key-based 缓存
- [ ] `ImageCacheKey` — (name, width, height, format, usage) 匹配
- [ ] `BufferCacheKey` — (name, size, usage) 匹配
- [ ] `acquire_image()` / `acquire_buffer()` — 首次创建，后续复用
- [ ] Resize 后旧资源退休（`invalidate()`）
- [ ] 退休资源延迟 kFramesInFlight 帧后销毁

### 集成
- [ ] `allocate_transient_resources()` 在 compile 时调用
- [ ] Usage flags 从 graph 声明自动填入 key
- [ ] `RenderContext::get_image()` 返回正确的 Vulkan 对象

## Phase V: 集成与迁移（Day 2 上午）

### ForwardPipeline 迁移
- [ ] 用 graph 声明 2 pass（Forward + Present）
- [ ] 移除手写 barrier 代码
- [ ] 视觉输出与迁移前一致
- [ ] 验证层零报错
- [ ] 帧时间增长 < 0.1ms

### SRT 骨架
- [ ] 15 pass 声明编译通过
- [ ] 拓扑排序结果合理
- [ ] 动态 pass 正确处理
- [ ] DOT 可视化输出

## 与其他 Plan 的执行顺序

```
材质系统 Phase A-D
    ↓
Render Graph Phase I-V  ← 本 plan（~2.5 天）
    ↓
SRT Phase 0 (G-Buffer + PBR + Shadow)
    ↓
SRT Phase 1-4 (SDF, SDFGI, ReSTIR, Post)
```

Render Graph 在材质系统之后、SRT 之前完成。
SRT 从第一天就在 graph 上构建，不需要回头迁移。
