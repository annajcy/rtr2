# RTR2 轻量 Render Graph 计划

## 背景

### 当前同步管理方式

现有管线（ForwardPipeline / ShaderToyPipeline）的 barrier 完全手写：

```cpp
// ForwardPass::do_execute() 中：
vk::ImageMemoryBarrier2 to_color{};
to_color.srcStageMask  = vk::PipelineStageFlagBits2::eTopOfPipe;
to_color.dstStageMask  = vk::PipelineStageFlagBits2::eColorAttachmentOutput;
to_color.srcAccessMask = vk::AccessFlagBits2::eNone;
to_color.dstAccessMask = vk::AccessFlagBits2::eColorAttachmentWrite;
to_color.oldLayout     = resources.color.layout;
to_color.newLayout     = vk::ImageLayout::eColorAttachmentOptimal;
// ... 20 行设置 subresourceRange ...
cmd.pipelineBarrier2(to_render_dep);
```

每个 pass 手动设置 src/dst stage、access、layout → **每个 pass 约 30-50 行 barrier 样板代码**。

现有系统只有 2-3 个 pass，手写勉强可行。SRT 管线有 **15+ pass、20+ 资源**，手写 barrier：
- 约 500-800 行纯 barrier 代码
- 改一个 pass 的读写关系需要同时更新上下游所有 barrier
- 错误只能靠验证层发现，且定位困难

### 已有的可复用基础设施

| 组件 | 位置 | 复用方式 |
|------|------|----------|
| `TrackedImage` / `FrameTrackedImage` | `render_resource_state.hpp` | → 升级为 render graph 的 resource handle |
| `RenderPass<TResource>` | `render_pass.hpp` | → 保留，graph pass 内部仍可使用 |
| `pipelineBarrier2()` | Vulkan 1.3 Synchronization2 | → graph 生成 barrier 的底层 API |
| `Image::LayoutTransitionConfig` | `texture.hpp` | → graph barrier 推导的参考模板 |
| `SceneTargetController` | `scene_target_controller.hpp` | → transient resource 生命周期管理参考 |
| `FrameScheduler` | `frame_scheduler.hpp` | → graph 运行在单帧 command buffer 内 |
| `DescriptorWriter` | `descriptor.hpp` | → pass 内部绑定描述符不变 |

## 目标

构建**最小可用的 render graph**，解决 SRT 管线的 barrier 自动化问题：

1. Pass 声明读写资源 → **自动推导并插入 barrier**
2. Transient resource 由 graph 自动创建/销毁
3. DAG 拓扑排序保证执行顺序正确
4. 与现有 `RenderPass<T>` / `FrameScheduler` 无缝集成

## 不做什么

- 资源内存别名（aliasing / memory pooling）
- 多 queue 异步 compute 调度
- 图编译优化 / pass 合并（pass merging）
- 动态 pass 裁剪（dead-pass elimination）
- Render pass 对象创建（用 dynamic rendering）
- 跨帧图缓存（每帧重建图，但 resource 可跨帧持久）

## 总体策略

```
Phase I:   核心数据结构        (~0.5 天)  — ResourceHandle, PassNode, DAG
Phase II:  资源声明与依赖推导   (~0.5 天)  — read/write 声明, barrier 推导
Phase III: 图编译与执行        (~0.5 天)  — 拓扑排序, barrier 插入, 执行
Phase IV:  Transient 资源管理   (~0.5 天)  — 自动创建/销毁 transient image/buffer
Phase V:   集成与迁移           (~0.5 天)  — 迁移 ForwardPipeline, SRT 骨架验证
```

总计约 2.5 个工作日。

## 架构总览

```
                  用户 API
                     │
          ┌──────────┴──────────┐
          │    RenderGraph      │
          │  (per-frame 构建)    │
          └──────────┬──────────┘
                     │ add_pass() / create_texture() / import()
          ┌──────────┴──────────┐
          │   Graph Builder     │
          │  (收集 pass + 资源)  │
          └──────────┬──────────┘
                     │ compile()
          ┌──────────┴──────────┐
          │   Graph Compiler    │
          │  (拓扑排序+barrier)  │
          └──────────┬──────────┘
                     │ execute()
          ┌──────────┴──────────┐
          │  Graph Executor     │
          │  (录制 cmd buffer)   │
          └──────────┬──────────┘
                     │
              vk::CommandBuffer
```

## 目录结构

```
src/rtr/system/render/graph/
├── render_graph.hpp            # 主入口：RenderGraph 类
├── render_graph_types.hpp      # ResourceHandle, ResourceUsage, PassNode 等类型
├── render_graph_builder.hpp    # PassBuilder：pass 内声明读写资源
├── render_graph_compiler.hpp   # 拓扑排序 + barrier 推导
├── render_graph_executor.hpp   # 按序执行 pass + 插入 barrier
└── render_graph_resource.hpp   # Transient resource 创建与管理
```

## 文件依赖图

```
render_graph_types.hpp
  <- render_graph_builder.hpp
  <- render_graph_resource.hpp
       <- render_graph_compiler.hpp
            <- render_graph_executor.hpp
                 <- render_graph.hpp (facade)
```

## 使用示例（目标 API）

```cpp
void SRTPipeline::render(FrameContext& ctx) {
    RenderGraph graph(device_, ctx);

    // 导入外部资源（swapchain image）
    auto swapchain = graph.import_image("swapchain", ctx.swapchain_image(), ...);

    // 声明 transient 资源（graph 自动创建）
    auto gbuffer_albedo = graph.create_texture("gbuffer_albedo", {
        .width = extent.width, .height = extent.height,
        .format = vk::Format::eR16G16B16A16Sfloat,
        .usage = ImageUsage::ColorAttachment | ImageUsage::Sampled
    });
    auto gbuffer_normal = graph.create_texture("gbuffer_normal", { ... });
    auto gbuffer_depth  = graph.create_texture("gbuffer_depth", { ... });
    auto shadow_map     = graph.create_texture("shadow_map", { ... });
    auto reservoir_buf  = graph.create_buffer("reservoir", { ... });
    auto hdr_color      = graph.create_texture("hdr_color", { ... });

    // G-Buffer pass
    graph.add_pass("GBuffer", PassType::Graphics, [&](PassBuilder& b) {
        b.write_color(gbuffer_albedo);
        b.write_color(gbuffer_normal);
        b.write_depth(gbuffer_depth);
        return [=](RenderContext& rc) {
            // 在这里做实际的渲染：bind pipeline, bindDescriptorSets, draw
            // rc.cmd 是 vk::CommandBuffer
            // barrier 已经在 pass 执行前自动插入
        };
    });

    // Shadow pass
    graph.add_pass("Shadow", PassType::Graphics, [&](PassBuilder& b) {
        b.write_depth(shadow_map);
        return [=](RenderContext& rc) { /* shadow rendering */ };
    });

    // SDFGI Trace（compute）
    graph.add_pass("SDFGI_Trace", PassType::Compute, [&](PassBuilder& b) {
        b.read_texture(gbuffer_albedo);
        b.read_texture(gbuffer_normal);
        b.write_storage_image(sdfgi_probe_atlas);
        return [=](RenderContext& rc) { /* compute dispatch */ };
    });

    // ReSTIR Initial（compute）
    graph.add_pass("ReSTIR_Initial", PassType::Compute, [&](PassBuilder& b) {
        b.read_texture(gbuffer_albedo);
        b.read_texture(gbuffer_normal);
        b.read_texture(gbuffer_depth);
        b.write_storage_buffer(reservoir_buf);
        return [=](RenderContext& rc) { /* compute dispatch */ };
    });

    // ReSTIR Shade
    graph.add_pass("ReSTIR_Shade", PassType::Compute, [&](PassBuilder& b) {
        b.read_storage_buffer(reservoir_buf);
        b.read_texture(gbuffer_albedo);
        b.read_texture(shadow_map);
        b.read_texture(sdfgi_probe_atlas);
        b.write_storage_image(hdr_color);
        return [=](RenderContext& rc) { /* final shading */ };
    });

    // Present
    graph.add_pass("Present", PassType::Transfer, [&](PassBuilder& b) {
        b.read_transfer(hdr_color);
        b.write_transfer(swapchain);
        return [=](RenderContext& rc) { /* blit */ };
    });

    // 一键编译 + 执行
    graph.compile_and_execute();
}
```

每帧重建图（构建开销极低：只是填结构体），但 transient 资源可跨帧缓存避免重复分配。
