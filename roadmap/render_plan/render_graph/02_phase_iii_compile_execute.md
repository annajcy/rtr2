# Phase III: 图编译与执行（~0.5 天）

## 目标

实现拓扑排序确定 pass 执行顺序，在 pass 之间自动插入 Vulkan barrier。

## 拓扑排序 — `render_graph_compiler.hpp`

### 算法：Kahn's Algorithm

```cpp
// 输入：pass 列表 + 依赖边
// 输出：pass 执行顺序（拓扑排序后的索引数组）
std::vector<uint32_t> topological_sort(
    uint32_t pass_count,
    const std::vector<DependencyEdge>& edges
) {
    // 建邻接表 + 入度数组
    std::vector<std::vector<uint32_t>> adj(pass_count);
    std::vector<uint32_t> in_degree(pass_count, 0);

    for (const auto& e : edges) {
        adj[e.from_pass].push_back(e.to_pass);
        in_degree[e.to_pass]++;
    }

    // 入度为 0 的节点入队
    std::queue<uint32_t> q;
    for (uint32_t i = 0; i < pass_count; i++) {
        if (in_degree[i] == 0) q.push(i);
    }

    std::vector<uint32_t> order;
    order.reserve(pass_count);

    while (!q.empty()) {
        uint32_t u = q.front(); q.pop();
        order.push_back(u);
        for (uint32_t v : adj[u]) {
            if (--in_degree[v] == 0) q.push(v);
        }
    }

    // 检测环
    if (order.size() != pass_count) {
        // 环依赖 → bug，assert / 报错
        assert(false && "Render graph has cyclic dependencies");
    }

    return order;
}
```

> **为什么用 Kahn 而不是 DFS？**
> Kahn 产生的顺序更符合用户的 add_pass 声明顺序（BFS 性质），
> 且天然检测环。20 个 pass 的拓扑排序耗时 < 1μs。

### 保持用户声明顺序的稳定性

当多个 pass 拥有相同入度时，按 add_pass 的声明顺序排序（用 priority_queue 或稳定排序）。
这保证了用户对 pass 顺序的直觉不被打破：

```cpp
// 如果用户写了：
graph.add_pass("A", ...);  // idx 0
graph.add_pass("B", ...);  // idx 1
// A 和 B 无依赖时，保持 A 在 B 前执行
```

## Barrier 推导 — `render_graph_compiler.hpp`

### 核心逻辑

在拓扑排序确定的执行顺序中，对每个 pass **执行前**检查它使用的每个资源：
当前资源状态（上一个 pass 留下的 layout / stage / access）是否与本 pass 要求的一致。

```cpp
struct BarrierBatch {
    std::vector<vk::ImageMemoryBarrier2>  image_barriers;
    std::vector<vk::BufferMemoryBarrier2> buffer_barriers;
};

// 为每个 pass 计算需要在其前面插入的 barrier
std::vector<BarrierBatch> compute_barriers(
    const std::vector<uint32_t>& execution_order,
    const std::vector<PassNode>& passes,
    std::vector<ImageState>& images,
    std::vector<BufferState>& buffers
) {
    std::vector<BarrierBatch> barriers(passes.size());

    for (uint32_t exec_idx = 0; exec_idx < execution_order.size(); exec_idx++) {
        uint32_t pass_idx = execution_order[exec_idx];
        const auto& pass = passes[pass_idx];
        auto& batch = barriers[pass_idx];

        // Image barriers
        for (const auto& usage : pass.image_usages) {
            auto& state = images[usage.handle.index];
            auto required = image_access_info(usage.access);

            bool needs_barrier = false;

            // 情况 1: layout 不同 → 必须 transition
            if (state.current_layout != required.layout) {
                needs_barrier = true;
            }
            // 情况 2: 上一次是写，这次是读/写 → 需要 memory barrier
            else if (state.last_was_write) {
                needs_barrier = true;
            }
            // 情况 3: 这次是写 → 需要 execution barrier（即使 layout 相同）
            else if (required.writes) {
                needs_barrier = true;
            }
            // 情况 4: RAR 同 layout → 不需要 barrier
            // （但如果跨 stage 可能需要 execution dependency —— 保守起见也加）

            if (needs_barrier) {
                vk::ImageMemoryBarrier2 b{};
                b.srcStageMask  = state.last_stage;
                b.srcAccessMask = state.last_access;
                b.dstStageMask  = required.stage;
                b.dstAccessMask = required.access;
                b.oldLayout     = state.current_layout;
                b.newLayout     = required.layout;
                b.image         = get_vk_image(state);
                b.subresourceRange = make_full_subresource_range(state);
                batch.image_barriers.push_back(b);
            }

            // 更新状态
            state.current_layout = required.layout;
            state.last_stage     = required.stage;
            state.last_access    = required.access;
            state.last_was_write = required.writes;
        }

        // Buffer barriers（逻辑类似，无 layout）
        for (const auto& usage : pass.buffer_usages) {
            auto& state = buffers[usage.handle.index];
            auto required = buffer_access_info(usage.access);

            bool needs_barrier = state.last_was_write || required.writes;

            if (needs_barrier) {
                vk::BufferMemoryBarrier2 b{};
                b.srcStageMask  = state.last_stage;
                b.srcAccessMask = state.last_access;
                b.dstStageMask  = required.stage;
                b.dstAccessMask = required.access;
                b.buffer        = get_vk_buffer(state);
                b.offset        = 0;
                b.size          = VK_WHOLE_SIZE;
                batch.buffer_barriers.push_back(b);
            }

            state.last_stage     = required.stage;
            state.last_access    = required.access;
            state.last_was_write = required.writes;
        }
    }

    return barriers;
}
```

### Barrier 合并优化

相邻 pass 之间如果有多个 barrier，合并到一个 `pipelineBarrier2()` 调用：

```cpp
void emit_barriers(const vk::raii::CommandBuffer& cmd, const BarrierBatch& batch) {
    if (batch.image_barriers.empty() && batch.buffer_barriers.empty()) return;

    vk::DependencyInfo dep{};
    dep.imageMemoryBarrierCount  = static_cast<uint32_t>(batch.image_barriers.size());
    dep.pImageMemoryBarriers     = batch.image_barriers.data();
    dep.bufferMemoryBarrierCount = static_cast<uint32_t>(batch.buffer_barriers.size());
    dep.pBufferMemoryBarriers    = batch.buffer_barriers.data();

    cmd.pipelineBarrier2(dep);
}
```

## 图执行 — `render_graph_executor.hpp`

```cpp
class GraphExecutor {
public:
    void execute(
        const vk::raii::CommandBuffer& cmd,
        const std::vector<uint32_t>& execution_order,
        const std::vector<PassNode>& passes,
        const std::vector<BarrierBatch>& barriers,
        RenderContext& ctx
    ) {
        for (uint32_t exec_idx = 0; exec_idx < execution_order.size(); exec_idx++) {
            uint32_t pass_idx = execution_order[exec_idx];
            const auto& pass = passes[pass_idx];

            // 1. 插入 barrier
            emit_barriers(cmd, barriers[pass_idx]);

            // 2. 可选：debug marker
            #ifndef NDEBUG
            vk::DebugUtilsLabelEXT label{};
            label.pLabelName = pass.name.c_str();
            cmd.beginDebugUtilsLabelEXT(label);
            #endif

            // 3. 执行用户 pass
            pass.execute_fn(ctx);

            #ifndef NDEBUG
            cmd.endDebugUtilsLabelEXT();
            #endif
        }
    }
};
```

### Debug 标注

每个 pass 用 `vk::DebugUtilsLabelEXT` 包裹，这样在 RenderDoc / GPU 调试器中可以看到：

```
[GBuffer]
  barrier: gbuffer_albedo undefined → colorAttachment
  barrier: gbuffer_depth undefined → depthAttachment
  draw calls...
[Shadow]
  barrier: shadow_map undefined → depthAttachment
  draw calls...
[ReSTIR_Initial]
  barrier: gbuffer_albedo colorAttachment → shaderReadOnly
  compute dispatch...
```

## compile_and_execute 整合

```cpp
void RenderGraph::compile_and_execute() {
    // 1. 推导依赖边
    auto edges = derive_dependencies(passes_);

    // 2. 拓扑排序
    auto order = topological_sort(passes_.size(), edges);

    // 3. 创建 transient 资源（Phase IV）
    allocate_transient_resources();

    // 4. 计算 barrier
    auto barriers = compute_barriers(order, passes_, images_, buffers_);

    // 5. 构建执行上下文
    RenderContext ctx{
        .cmd = frame_ctx_.cmd().command_buffer(),
        .frame_index = frame_ctx_.frame_index(),
        .render_extent = frame_ctx_.render_extent(),
        // ... resource accessors
    };

    // 6. 执行
    GraphExecutor executor;
    executor.execute(ctx.cmd, order, passes_, barriers, ctx);
}
```

## 验证层友好性

正确推导 barrier 后，Vulkan 验证层（`VK_LAYER_KHRONOS_validation`）应该不再报告：
- `VUID-VkImageMemoryBarrier2-oldLayout-...`（layout mismatch）
- `VUID-vkCmdDraw-...-hazard`（同步 hazard）
- `SYNC-HAZARD-WRITE-AFTER-READ` / `READ-AFTER-WRITE`

如果还有报告，说明 barrier 推导有 bug。验证层是 render graph 正确性的最终裁判。

## Phase III 完成标准

- [ ] Kahn 拓扑排序正确排列所有 pass
- [ ] 环检测报告清晰错误信息
- [ ] 同入度 pass 保持声明顺序（稳定性）
- [ ] Barrier 推导覆盖 RAW / WAR / WAW 场景
- [ ] RAR 无冗余 barrier（同 layout、同 stage 时跳过）
- [ ] Image layout transition 正确（undefined → attachment → sampled → ...）
- [ ] Buffer memory barrier 正确（write → read 插 barrier）
- [ ] 多 barrier 合并为单次 `pipelineBarrier2()` 调用
- [ ] Debug label 在 RenderDoc 中可见
- [ ] 3 pass 链（GBuffer → Lighting → Present）全流程正确，验证层零报错
