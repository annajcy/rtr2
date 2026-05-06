# `collision_candidates.hpp`

`src/rtr/system/physics/ipc/contact/collision_candidates.hpp` 在 `CollisionMesh` 之上实现了第一层 broad phase。

当前目标是正确性和稳定行为，不是加速。因此候选生成采用 brute force，但它已经把后续 barrier / CCD 真正依赖的 primitive 组合和过滤规则定下来了。

## 候选类型

这个文件定义了两种 primitive-pair 记录：

```cpp
struct PTCandidate {
    std::size_t point_vertex_idx;
    std::size_t triangle_idx;
};

struct EECandidate {
    std::size_t edge_a_idx;
    std::size_t edge_b_idx;
};
```

以及一个聚合容器：

```cpp
struct CollisionCandidates {
    std::vector<PTCandidate> pt_candidates;
    std::vector<EECandidate> ee_candidates;
};
```

这些候选只保存 `CollisionMesh` 里的 primitive index。ownership 和坐标读取逻辑仍然留在 mesh 上，不在这里重复。

## Day 2 Broad Phase 范围

当前实现刻意只生成第一版 tet-vs-static-obstacle 管线需要的组合：

1. deformable point vs obstacle triangle
2. obstacle point vs deformable triangle
3. deformable edge vs obstacle edge

这样已经足够覆盖 vertex-face 和 edge-edge 接近关系，同时又不会过早引入 self-contact 或 obstacle-obstacle 路径。

## 稳定的枚举顺序

主入口是：

```cpp
CollisionCandidates build_collision_candidates(
    const CollisionMesh& mesh,
    const IPCState& state,
    const BroadPhaseConfig& config = {}
);
```

循环顺序是刻意固定的：

1. deformable vertices × obstacle triangles
2. obstacle vertices × deformable triangles
3. deformable edges × obstacle edges

这些循环都遍历 `CollisionMesh` 中预先整理好的 primitive index 列表，因此同一个 mesh 的输出顺序是稳定的。这对测试和调试都很重要。

## 过滤规则

Broad phase 不能把所有笛卡尔积 pair 都吐出去，它必须去掉拓扑上无效或当前阶段不支持的情况。

### Point-Triangle

一个 PT pair 会在以下情况下被丢弃：

- 这个 point 本来就是该 triangle 的顶点
- point 和 triangle 属于同一个 body
- 两侧同为 deformable，或同为 obstacle

第一条规则避免平凡的自关联，后两条规则则是当前 Day 2 接触范围的显式编码。

### Edge-Edge

一个 EE pair 会在以下情况下被丢弃：

- 两条边属于同一个 body
- 它们共享任意端点
- 两条边来自同一类 side

共享端点规则的作用，是排除那些拓扑上本来就相邻、而不是几何上相互独立的边对。

## 可选的 AABB 预过滤

`BroadPhaseConfig` 暴露了一个可选的保守预过滤开关：

```cpp
struct BroadPhaseConfig {
    bool enable_aabb_prefilter{false};
    double aabb_padding{0.0};
};
```

启用后：

- PT 只有在 point AABB 和 triangle AABB 重叠时才保留
- EE 只有在两条边的 AABB 重叠时才保留

padding 会在三个坐标轴上统一扩张包围盒：

$$
\text{AABB}_{\text{padded}} = [x_{\min} - p,\ x_{\max} + p]
$$

这个预过滤仍然是保守的：它可以保留过多候选，但在 padding 选得合理时，不应该误删真正可能发生的接触。

## 为什么接口里已经带上 `IPCState`

虽然默认 Day 2 路径完全可以是纯 brute force，这个函数仍然一开始就接收 `IPCState`，因为任何空间过滤都依赖 deformable 顶点的当前坐标。

这样一来，后续如果调用方想打开：

- 带 padding 的当前步 AABB 过滤
- 未来基于运动的保守剔除
- Newton 或 line search 内部的 candidate 重建

就不需要再改这层 API。

## 在系统中的作用

`CollisionCandidates` 是当前 contact 预处理阶段的最终产物。

后续模块会以两种方式消费它：

- barrier 装配会把每个 candidate 转成局部 PT 或 EE distance input
- CCD 会在搜索方向上的试探状态下，复用同一套 candidate topology 做安全判定

最关键的架构点在于：candidate generation 仍然是 topology-centric 的。它不计算 barrier energy，也不决定最终 active contact，它只负责定义“哪些 primitive pair 值得往下传”。
