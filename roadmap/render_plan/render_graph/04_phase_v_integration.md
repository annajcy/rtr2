# Phase V: 集成与迁移（~0.5 天）

## 目标

将现有 ForwardPipeline 迁移到 render graph，验证零回归；为 SRT pipeline 提供 graph 骨架。

## ForwardPipeline 迁移

### 当前结构

```cpp
// 现有 ForwardPipeline::render() 大致流程：
void render(FrameContext& ctx) {
    // 手动 barrier: color → attachment
    // 手动 barrier: depth → attachment
    forward_pass_.execute(ctx, resources);  // draw
    // 手动 barrier: color → transferSrc, swapchain → transferDst
    present_pass_.execute(ctx, resources);  // blit
    // 手动 barrier: swapchain → colorAttachment
}
```

### 迁移后

```cpp
void ForwardPipeline::render(FrameContext& ctx) {
    RenderGraph graph(device_, resource_pool_, ctx);

    auto swapchain = graph.import_swapchain("swapchain",
        ctx.swapchain_image(), m_color_format, m_swapchain_extent);

    auto color = graph.create_texture("offscreen_color", {
        .width = m_swapchain_extent.width,
        .height = m_swapchain_extent.height,
        .format = m_color_format,
    });
    auto depth = graph.create_texture("depth", {
        .width = m_swapchain_extent.width,
        .height = m_swapchain_extent.height,
        .format = m_depth_format,
    });

    graph.add_pass("Forward", PassType::Graphics, [&](PassBuilder& b) {
        b.write_color(color);
        b.write_depth(depth);
        return [this, color, depth](RenderContext& rc) {
            auto& cmd = rc.cmd;
            // 现有 ForwardPass 的渲染逻辑
            // beginRendering → bindPipeline → draw → endRendering
            // barrier 已自动处理，不需要手写
        };
    });

    graph.add_pass("Present", PassType::Transfer, [&](PassBuilder& b) {
        b.read_transfer(color);
        b.write_transfer(swapchain);
        return [this, color, swapchain](RenderContext& rc) {
            auto& cmd = rc.cmd;
            // blit offscreen → swapchain
        };
    });

    graph.compile_and_execute();
}
```

### 迁移验证

- 视觉输出与迁移前完全一致
- 验证层零报错
- 帧时间无明显增长（< 0.1ms graph 开销）

## SRT Pipeline 骨架

验证 graph 能表达 SRT 的完整 pass DAG：

```cpp
void SRTPipeline::render(FrameContext& ctx) {
    RenderGraph graph(device_, resource_pool_, ctx);

    // ─── 外部资源 ───
    auto swapchain = graph.import_swapchain("swapchain", ...);
    auto sdf_volume = graph.import_image("sdf_cascade", sdf_cascade_.volume(), ...);
    auto probe_atlas = graph.import_image("probe_atlas", sdfgi_.irradiance_atlas(), ...);
    auto reservoir_prev = graph.import_buffer("reservoir_prev", reservoir_previous_);

    // ─── Transient 资源 ───
    auto gbuf_albedo   = graph.create_texture("gbuf_albedo",   {extent, R16G16B16A16Sfloat});
    auto gbuf_normal   = graph.create_texture("gbuf_normal",   {extent, R16G16B16A16Sfloat});
    auto gbuf_position = graph.create_texture("gbuf_position", {extent, R32G32B32A32Sfloat});
    auto gbuf_depth    = graph.create_texture("gbuf_depth",    {extent, D32Sfloat});
    auto motion_vec    = graph.create_texture("motion_vec",    {extent, R16G16Sfloat});
    auto shadow_map    = graph.create_texture("shadow_map",    {2048, 2048, D32Sfloat});
    auto reservoir_cur = graph.create_buffer("reservoir_cur",  {reservoir_size});
    auto hdr_color     = graph.create_texture("hdr_color",     {extent, R16G16B16A16Sfloat});
    auto denoised      = graph.create_texture("denoised",      {extent, R16G16B16A16Sfloat});
    auto taa_output    = graph.create_texture("taa_output",    {extent, R16G16B16A16Sfloat});
    auto ldr_output    = graph.create_texture("ldr_output",    {extent, R8G8B8A8Unorm});

    // ─── Pass 声明 ───

    graph.add_pass("GBuffer", PassType::Graphics, [&](PassBuilder& b) {
        b.write_color(gbuf_albedo);
        b.write_color(gbuf_normal);
        b.write_color(gbuf_position);
        b.write_color(motion_vec);
        b.write_depth(gbuf_depth);
        return [=](RenderContext& rc) { /* ... */ };
    });

    graph.add_pass("Shadow", PassType::Graphics, [&](PassBuilder& b) {
        b.write_depth(shadow_map);
        return [=](RenderContext& rc) { /* ... */ };
    });

    if (sdf_needs_update_) {
        graph.add_pass("SDF_Update", PassType::Compute, [&](PassBuilder& b) {
            b.write_storage_image(sdf_volume);
            return [=](RenderContext& rc) { /* ... */ };
        });
    }

    graph.add_pass("SDFGI_Trace", PassType::Compute, [&](PassBuilder& b) {
        b.read_texture(sdf_volume);
        b.read_texture(gbuf_albedo);
        b.read_texture(gbuf_normal);
        b.write_storage_image(probe_atlas);
        return [=](RenderContext& rc) { /* ... */ };
    });

    graph.add_pass("SDFGI_Update", PassType::Compute, [&](PassBuilder& b) {
        b.readwrite_storage_image(probe_atlas);
        return [=](RenderContext& rc) { /* ... */ };
    });

    graph.add_pass("ReSTIR_Initial", PassType::Compute, [&](PassBuilder& b) {
        b.read_texture(gbuf_albedo);
        b.read_texture(gbuf_normal);
        b.read_texture(gbuf_depth);
        b.write_storage_buffer(reservoir_cur);
        return [=](RenderContext& rc) { /* ... */ };
    });

    graph.add_pass("ReSTIR_Temporal", PassType::Compute, [&](PassBuilder& b) {
        b.readwrite_storage_buffer(reservoir_cur);
        b.read_storage_buffer(reservoir_prev);
        b.read_texture(motion_vec);
        return [=](RenderContext& rc) { /* ... */ };
    });

    graph.add_pass("ReSTIR_Spatial", PassType::Compute, [&](PassBuilder& b) {
        b.readwrite_storage_buffer(reservoir_cur);
        b.read_texture(gbuf_normal);
        b.read_texture(gbuf_depth);
        return [=](RenderContext& rc) { /* ... */ };
    });

    graph.add_pass("ReSTIR_Shade", PassType::Compute, [&](PassBuilder& b) {
        b.read_storage_buffer(reservoir_cur);
        b.read_texture(gbuf_albedo);
        b.read_texture(gbuf_normal);
        b.read_texture(shadow_map);
        b.read_texture(probe_atlas);
        b.read_texture(sdf_volume);
        b.write_storage_image(hdr_color);
        return [=](RenderContext& rc) { /* ... */ };
    });

    graph.add_pass("Denoise", PassType::Compute, [&](PassBuilder& b) {
        b.read_texture(hdr_color);
        b.read_texture(gbuf_normal);
        b.read_texture(gbuf_depth);
        b.read_texture(motion_vec);
        b.write_storage_image(denoised);
        return [=](RenderContext& rc) { /* ... */ };
    });

    graph.add_pass("TAA", PassType::Compute, [&](PassBuilder& b) {
        b.read_texture(denoised);
        b.read_texture(motion_vec);
        b.write_storage_image(taa_output);
        return [=](RenderContext& rc) { /* ... */ };
    });

    graph.add_pass("ToneMap", PassType::Compute, [&](PassBuilder& b) {
        b.read_texture(taa_output);
        b.write_storage_image(ldr_output);
        return [=](RenderContext& rc) { /* ... */ };
    });

    graph.add_pass("Present", PassType::Transfer, [&](PassBuilder& b) {
        b.read_transfer(ldr_output);
        b.write_transfer(swapchain);
        return [=](RenderContext& rc) { /* blit */ };
    });

    graph.compile_and_execute();
}
```

### 验证

- Graph 成功编译 15 个 pass，无环
- 拓扑排序结果合理（GBuffer/Shadow 在前，Present 在最后）
- `sdf_needs_update_ = false` 时 SDF_Update pass 不执行（动态 pass）
- 验证层零报错
- 所有中间资源通过 graph 自动创建，无手动 `rhi::Image` 构造

## 与现有 RenderPass<T> 的兼容

现有 `RenderPass<TResource>` 模板仍然可以在 graph pass 内部使用：

```cpp
graph.add_pass("Forward", PassType::Graphics, [&](PassBuilder& b) {
    b.write_color(color);
    b.write_depth(depth);
    return [this](RenderContext& rc) {
        // graph 已经插入 barrier，资源状态正确
        // 现有 ForwardPass 不需要内部 barrier 了
        forward_pass_.execute_without_barriers(rc, resources);
    };
});
```

可以渐进迁移：先用 graph 管 barrier，内部 pass 逻辑保持不变。

## Graph 可视化（debug 工具）

```cpp
// 输出 Graphviz DOT 格式，用于调试依赖关系
std::string RenderGraph::dump_dot() const {
    std::ostringstream ss;
    ss << "digraph RenderGraph {\n";
    ss << "  rankdir=LR;\n";

    for (uint32_t i = 0; i < passes_.size(); i++) {
        ss << "  pass_" << i << " [label=\"" << passes_[i].name << "\"];\n";
    }

    for (const auto& e : edges_) {
        ss << "  pass_" << e.from_pass << " -> pass_" << e.to_pass << ";\n";
    }

    ss << "}\n";
    return ss.str();
}
```

使用：`dot -Tpng graph.dot -o graph.png` 可视化 DAG。

## Phase V 完成标准

- [ ] ForwardPipeline 迁移到 render graph，视觉输出一致
- [ ] 迁移后验证层零报错
- [ ] 迁移后帧时间增长 < 0.1ms
- [ ] SRT 骨架 15 pass 声明编译通过，拓扑排序正确
- [ ] 动态 pass（`if (cond) add_pass()`）正确处理
- [ ] Graph DOT 输出可视化 DAG
- [ ] 现有 `RenderPass<T>` 可在 graph pass 内部复用
