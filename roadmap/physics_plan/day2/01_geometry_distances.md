# Phase 1: 几何距离模块

## 目标

在当前仓库里补齐 Day 2 接触系统的数学地基：实现 **距离平方 + 解析 gradient + 解析 Hessian**。

目标函数只做四类局部原语：

1. Point-Point (PP)
2. Point-Edge (PE)
3. Point-Triangle (PT)
4. Edge-Edge (EE)

其中真正会被接触层直接用到的是 PT 和 EE，但考虑到：

- PT 会退化到 PE / PP
- EE 会退化到 PE / PP
- 有限差分测试需要可拆的最小内核

所以仓库里应该把 PP 和 PE 也作为一等公民实现出来，而不是只把它们埋在 `PT/EE` 内部。

---

## 这一步为什么要先做、而且要做“解析导数”

当前仓库的能量与 solver 已经全部是解析梯度/Hessian 路线：

- `InertialEnergy`
- `GravityEnergy`
- `material_energy_variant`
- `Newton solver`

Day 2 如果在几何距离层退回到数值导数，会直接带来三个问题：

1. barrier Hessian 质量差，Newton 收敛会明显退化
2. 运行时成本太高，尤其是 PT/EE 每对 12 DOF
3. 很难定位错误来自距离本身还是 barrier 链式法则

结论很明确：

- 运行时实现必须是解析 gradient + Hessian
- 有限差分只出现在测试里

---

## 与当前仓库接口的衔接要求

这一步虽然是纯数学模块，但需要为后续 `contact/` 和 `ccd/` 做接口预留。

### 约束 1: geometry 层只接收局部输入，不接收 `IPCState`

不要让 geometry 直接依赖 `IPCSystem` 或 `CollisionMesh`。这一层应该是纯局部函数：

```cpp
PointTriangleDistance::compute(input)
EdgeEdgeDistance::compute(input)
```

后续无论坐标来自：

- tet 顶点 DOF
- obstacle 顶点位置
- 临时 trial state `x + alpha * dx`

都能直接复用。

这里和 Day 1 的 energy 模块一样，推荐把“输入参数打包”作为正式接口，而不是让每个调用点都手写长参数列表。

### 约束 2: 返回结果要能直接喂给 barrier

barrier 层需要的不只是标量距离，还需要：

- `distance_squared`
- `gradient`
- `hessian`

因此 geometry 结果结构不要做得过于抽象，直接给固定维度矩阵最省事。

### 约束 3: 局部 DOF 排列必须固定

后续 barrier 层会把局部 12 DOF 装配到全局稀疏矩阵，局部排列必须在文档和实现里统一。

---

## 建议的文件布局

推荐按“最小公共内核 -> 组合原语”的顺序落文件。

| 文件 | 作用 |
|------|------|
| `src/rtr/system/physics/ipc/geometry/distance_concept.hpp` | distance kernel 的 compile-time 接口约束 |
| `src/rtr/system/physics/ipc/geometry/point_point_distance.hpp` | PP 距离平方、gradient、Hessian |
| `src/rtr/system/physics/ipc/geometry/point_edge_distance.hpp` | PE 距离平方、gradient、Hessian |
| `src/rtr/system/physics/ipc/geometry/point_triangle_distance.hpp` | PT 区域分类与组合 |
| `src/rtr/system/physics/ipc/geometry/edge_edge_distance.hpp` | EE 区域分类、平行退化与组合 |

如果实现中发现一些辅助函数会重复，可以再补一个非常薄的公共头，例如：

- `geometry/distance_common.hpp`

但不要一开始就拆很多 util 文件。当前仓库的规模还不需要。

---

## 需要引入 `distance concept`

Day 1 里已经有 [`energy_concept.hpp`](/Users/jinceyang/Desktop/codebase/graphics/rtr2/src/rtr/system/physics/ipc/energy/energy_concept.hpp)，它的价值不是“为了抽象而抽象”，而是把所有 energy term 的最小接口稳定下来：

- 每个模块都自带 `Input`
- 都提供统一的 `compute_*`
- solver/assembly 可以按统一协议消费

Day 2 的距离核也有完全相同的问题：

- barrier 会消费 PT / EE，也会间接依赖 PP / PE
- CCD 会复用同一批距离核做最小距离或安全裕度判断
- 测试层希望对不同距离核复用一套 FD helper

如果不先把接口定下来，后面很容易出现：

- PT 返回 `result`
- EE 返回 tuple
- PE 用自由函数
- PP 用另一套命名

最后 barrier 和 CCD 都要写分支适配。

所以 Day 2 应该像 energy 一样，引入一个 geometry 层的 concept，把 distance kernel 的外形统一下来。

### 推荐文件

```text
src/rtr/system/physics/ipc/geometry/distance_concept.hpp
```

### 推荐 concept 形状

建议每个距离核都暴露：

- `Input`
- `Result`
- `static Result compute(const Input&)`

于是 concept 可以写成：

```cpp
template <typename T>
concept Distance = requires(const typename T::Input& input,
                            const typename T::Result& result) {
    typename T::Input;
    typename T::Result;
    { T::compute(input) } -> std::same_as<typename T::Result>;
    { result.distance_squared } -> std::convertible_to<double>;
    result.gradient;
    result.hessian;
};
```

这个 concept 不强行约束维度，因为：

- PP 是 6
- PE 是 9
- PT / EE 是 12

concept 的职责只是保证接口统一，不负责表达具体维数。

### 为什么这里用 `Input + Result + compute(...)`

这是对 Day 1 `Energy` 风格的直接延续，但也更适合 geometry：

1. `Input` 让 PT/EE 这类多点原语不必靠长参数列表传递
2. `Result` 让 barrier/CCD/test 都能消费同一份返回结构
3. `compute(...)` 让不同距离核可以被 template helper 统一调用

### 建议的使用方式

每个具体距离核定义成一个 type，而不是只提供裸函数。例如：

```cpp
struct PointPointDistance {
    struct Input {
        Eigen::Vector3d p0;
        Eigen::Vector3d p1;
    };

    using Result = PointPointDistanceResult;

    static Result compute(const Input& input);
};
```

PE / PT / EE 同理。

如果你仍然希望保留简洁调用方式，可以额外提供一个薄包装：

```cpp
inline PointPointDistanceResult point_point_distance(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1
) {
    return PointPointDistance::compute({.p0 = p0, .p1 = p1});
}
```

但这个包装只是语法糖，正式协议仍然以 concept 为准。

### 这个 concept 在 Day 2 的直接收益

1. Barrier 层可以写通用 helper

```cpp
template <Distance D>
double compute_barrier_energy_for_pair(const typename D::Input& input, double dhat_squared);
```

2. FD 测试可以抽成统一模板

```cpp
template <Distance D>
void expect_distance_gradient_matches_fd(const typename D::Input& input);
```

3. CCD 可以统一复用距离核，而不用关心每个模块是函数还是类

### 与 `Energy` concept 的差别

| | `Energy` | `Distance` |
|---|---|---|
| 作用空间 | 全局 DOF / 稀疏装配 | 局部原语 / 稠密局部导数 |
| 输入 | 每种 energy 自定义 `Input` | 每种距离核自定义 `Input` |
| 输出 | `energy + gradient + hessian_triplets` 分开提供 | 单个 `Result` 一次性返回 |
| Hessian 形式 | 全局 sparse triplets | 局部 dense matrix |
| 消费方 | `IPCSystem` / Newton solver | barrier / CCD / FD tests |

geometry 层不需要照抄 energy 层的三函数式接口，因为距离核天然是“一个局部求值单元”，一次返回完整结果更合适。

---

## 推荐的结果结构

固定维度返回值最适合当前仓库风格。建议：

```cpp
struct PointPointDistanceResult {
    double distance_squared{0.0};
    Eigen::Matrix<double, 6, 1> gradient{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Matrix<double, 6, 6> hessian{Eigen::Matrix<double, 6, 6>::Zero()};
};

struct PointEdgeDistanceResult {
    double distance_squared{0.0};
    Eigen::Matrix<double, 9, 1> gradient{Eigen::Matrix<double, 9, 1>::Zero()};
    Eigen::Matrix<double, 9, 9> hessian{Eigen::Matrix<double, 9, 9>::Zero()};
};

struct PointTriangleDistanceResult {
    double distance_squared{0.0};
    Eigen::Matrix<double, 12, 1> gradient{Eigen::Matrix<double, 12, 1>::Zero()};
    Eigen::Matrix<double, 12, 12> hessian{Eigen::Matrix<double, 12, 12>::Zero()};
};

struct EdgeEdgeDistanceResult {
    double distance_squared{0.0};
    Eigen::Matrix<double, 12, 1> gradient{Eigen::Matrix<double, 12, 1>::Zero()};
    Eigen::Matrix<double, 12, 12> hessian{Eigen::Matrix<double, 12, 12>::Zero()};
};
```

如果调试价值比较高，可以额外返回分类信息，例如：

```cpp
enum class PointTriangleRegion { Face, Edge01, Edge02, Edge12, Vertex0, Vertex1, Vertex2 };
enum class EdgeEdgeRegion { InteriorInterior, EndpointOnEdge, ParallelDegenerate };
```

这不是运行必需，但会极大降低测试定位成本，建议保留。

如果走 `distance concept` 路线，那么这些 `Result` 结构应作为各 distance kernel 的标准返回类型，而不是只作为内部细节。

### 推荐的 `Input` 结构

```cpp
struct PointPointDistance {
    struct Input {
        Eigen::Vector3d p0;
        Eigen::Vector3d p1;
    };
    using Result = PointPointDistanceResult;
    static Result compute(const Input& input);
};

struct PointEdgeDistance {
    struct Input {
        Eigen::Vector3d p;
        Eigen::Vector3d e0;
        Eigen::Vector3d e1;
    };
    using Result = PointEdgeDistanceResult;
    static Result compute(const Input& input);
};

struct PointTriangleDistance {
    struct Input {
        Eigen::Vector3d p;
        Eigen::Vector3d t0;
        Eigen::Vector3d t1;
        Eigen::Vector3d t2;
    };
    using Result = PointTriangleDistanceResult;
    static Result compute(const Input& input);
};

struct EdgeEdgeDistance {
    struct Input {
        Eigen::Vector3d ea0;
        Eigen::Vector3d ea1;
        Eigen::Vector3d eb0;
        Eigen::Vector3d eb1;
    };
    using Result = EdgeEdgeDistanceResult;
    static Result compute(const Input& input);
};
```

这样 `barrier_potential.hpp`、`collision_free_stepsize.hpp`、FD test helper 都能用统一协议处理它们。

---

## 局部 DOF 排列

### PP

```text
[p0_x, p0_y, p0_z, p1_x, p1_y, p1_z]
```

### PE

```text
[p_x, p_y, p_z, e0_x, e0_y, e0_z, e1_x, e1_y, e1_z]
```

### PT

```text
[p_x, p_y, p_z, t0_x, t0_y, t0_z, t1_x, t1_y, t1_z, t2_x, t2_y, t2_z]
```

### EE

```text
[ea0_x, ea0_y, ea0_z, ea1_x, ea1_y, ea1_z, eb0_x, eb0_y, eb0_z, eb1_x, eb1_y, eb1_z]
```

后续 barrier 装配、有限差分测试、局部到全局映射都必须严格沿用这个顺序。

---

## 具体实现策略

### 1. Point-Point (PP)

这是最简单也最重要的基准函数。

定义：

$$
s = \|p_0 - p_1\|^2
$$

性质：

- gradient 是线性的
- Hessian 是常量
- 不存在区域分类

这部分必须写得极稳，因为：

- PE/PT/EE 退化时都会落到这里
- 它非常适合当有限差分测试模板

实现要求：

- 不需要任何 epsilon 分支
- 除了基础输入合法性之外，不要引入特殊逻辑

---

### 2. Point-Edge (PE)

定义：

给点 `p` 和线段 `(e0, e1)`，先投影到无穷直线，再把参数裁剪到 `[0, 1]`。

记：

$$
u = e_1 - e_0,\quad
\alpha = \frac{(p - e_0)\cdot u}{u\cdot u}
$$

然后：

- `alpha <= 0` 时退化到 `PP(p, e0)`
- `alpha >= 1` 时退化到 `PP(p, e1)`
- `0 < alpha < 1` 时是线段内部 PE

### 这里要注意的仓库级数值约束

Day 2 后续会把这个函数用于：

- PT 的边区域
- EE 的端点退化
- CCD 的最小距离评估

所以建议：

- 对退化边 `|u|^2 <= eps` 直接走 `PP(e0)` 或抛异常
- 推荐把“零长度边不合法”写成显式保护

不要默默返回 0，这会把 bug 带进 barrier。

---

### 3. Point-Triangle (PT)

PT 是 Day 2 最关键的距离函数。

### 数学分类

点到三角形最近点总落在 7 个区域之一：

| 区域 | 处理方式 |
|------|----------|
| 面内部 | face interior PT |
| 边 `t0-t1` | 退化到 PE |
| 边 `t0-t2` | 退化到 PE |
| 边 `t1-t2` | 退化到 PE |
| 顶点 `t0` | 退化到 PP |
| 顶点 `t1` | 退化到 PP |
| 顶点 `t2` | 退化到 PP |

### 不建议的实现方式

不要把所有公式硬塞进一个大函数里，最后同时处理：

- 投影
- 区域分类
- 导数推导
- Hessian

那样测试一旦挂掉会很难定位。

### 建议的实现路径

分两层：

1. 先做“分类”
2. 再做“调用对应子核并嵌入 12 DOF”

也就是说，`point_triangle_distance.hpp` 不一定需要自己推完所有区域的导数。更现实的方式是：

- face interior：写 PT-face 专用解析公式
- edge 区域：调用 `PE`
- vertex 区域：调用 `PP`

然后用固定 embedding 把 6/9 维结果扩成 12 维。

### face interior case 的建议

实现 face interior 时，建议直接基于点到平面的平方距离公式，但避免显式单位法向量归一化带来的复杂导数链。

更稳妥的方向是：

- 以三角形仿射参数或未归一化法向量形式写出 `s`
- 再整理出解析 gradient/Hessian

总之目标是：

- 公式可测
- 数值上不依赖 `1 / |n|` 的脆弱分支

### 三角形退化

当前仓库后续会从 tet 表面和 obstacle 三角网格读三角形。理论上这些三角形不该退化，但现实里不能假设永远成立。

建议：

- 若三角形面积过小，显式退化到 3 条边的最小 PE
- 不要继续走 face interior 公式

这样能把障碍物 mesh 质量问题隔离在 geometry 层。

---

### 4. Edge-Edge (EE)

EE 是另一类 barrier/CCD 的核心原语。

### 数学分类

两条线段的最近点分三类：

1. 两条边的内部点互相最近
2. 某个端点到另一条边最近，退化到 PE
3. 极端退化时再落到 PP

### 平行或近似平行是 Day 2 的重点风险

对当前仓库来说，EE 最大风险不是一般位置，而是：

- 平行边
- 几乎平行边
- 很短边

因为这些情况一旦处理不好，后面的 barrier Hessian 很容易炸掉。

建议处理策略：

```cpp
const double cross_norm_sq = (ea1 - ea0).cross(eb1 - eb0).squaredNorm();
if (cross_norm_sq < kParallelThreshold) {
    // 退化到若干 PE/PP 的最小值
}
```

`kParallelThreshold` 先保守一些，推荐量级：

- `1e-20` 到 `1e-16`

具体阈值可以在测试里校准，但必须有这条分支。

### 实现结构建议

和 PT 一样，建议把 EE 写成：

1. 参数分类
2. interior-interior 专用公式
3. 端点退化调用 PE/PP

不要从一开始就追求“一条闭式公式覆盖所有情况”。

---

## 结果嵌入方式

因为 PT/EE 都会调用 PP/PE，所以必须明确“如何把低维导数嵌回高维局部向量”。

例如 PT 的 `Edge01` 情况：

- 真实参与的局部变量是 `[p, t0, t1]`
- 结果来自 9 维 `PE`
- 需要嵌回 12 维 `[p, t0, t1, t2]`

建议文档里明确写出 embedding 规则，代码里做成小 helper，例如：

```cpp
template <typename DerivedVec, typename DerivedMat>
void embed_pe_into_pt(
    const DerivedVec& pe_grad,
    const DerivedMat& pe_hess,
    int tri_vertex_a,
    int tri_vertex_b,
    Eigen::Matrix<double, 12, 1>& pt_grad,
    Eigen::Matrix<double, 12, 12>& pt_hess
);
```

EE 退化到 PE/PP 时同理。

这样做的好处是：

- 组合关系清楚
- 测试失败时更容易排查是“子核错了”还是“嵌入错了”

---

## Hessian 处理原则

距离函数自己的 Hessian 不要求 PSD。

这一点要在实现里坚持住，不要提前在 geometry 层做正定化。理由是：

1. 几何 Hessian 本身只是二阶导，不承担优化稳定性责任
2. barrier Hessian 的最终形式是

$$
b''(s)\nabla s \nabla s^T + b'(s)\nabla^2 s
$$

真正需要 PSD projection 的是 barrier 层，不是 geometry 层

所以 geometry 层的职责只有：

- 返回正确解析 Hessian
- 保证数值有限

---

## 建议的异常/退化策略

为了与当前仓库风格一致，建议 geometry 层对明显非法输入采取“早失败”。

### 可以直接抛异常的情况

- 零长度边
- 非有限输入坐标
- 明显退化且无法稳健降级的构型

### 可以做数值降级的情况

- PT 的退化三角形 -> 若干 PE 的最小值
- EE 的近似平行 -> 若干 PE/PP 的最小值

原则是：

- 可以明确给出几何替代意义的，做降级
- 没有清晰几何意义的，直接报错

---

## 测试策略

当前仓库已经大量使用 `gtest` 做纯头文件模块测试。Day 2 geometry 也应完全沿用这一风格。

### 建议测试文件

| 文件 | 内容 |
|------|------|
| `test/system/physics/ipc/geometry/point_point_distance_test.cpp` | PP 的解析导数与 FD |
| `test/system/physics/ipc/geometry/point_edge_distance_test.cpp` | PE 内部区、端点区、退化边 |
| `test/system/physics/ipc/geometry/point_triangle_distance_test.cpp` | PT 七区域覆盖 |
| `test/system/physics/ipc/geometry/edge_edge_distance_test.cpp` | EE 内部区、端点退化、平行退化 |

### 有限差分建议

梯度：

$$
\frac{f(x + h e_i) - f(x - h e_i)}{2h}
$$

Hessian：

- 对解析 gradient 再做中心差分

推荐参数：

- `h = 1e-7`
- 相对误差目标 `1e-5`
- 个别退化/近平行情形可以适当放宽到 `1e-4`

### PT 必测样例

至少覆盖：

1. face interior
2. edge `01`
3. edge `02`
4. edge `12`
5. vertex `0`
6. vertex `1`
7. vertex `2`
8. 退化三角形

### EE 必测样例

至少覆盖：

1. interior-interior
2. endpoint-edge
3. endpoint-endpoint
4. 平行
5. 几乎平行
6. 短边

### 额外建议

测试里最好额外检查：

- gradient/Hessian 全部 finite
- Hessian 近似对称

这比只比 FD 更能尽早发现装配错误。

---

## 与后续 Phase 的接口约定

Phase 1 完成后，Phase 3 barrier 层应能直接这样消费 geometry：

```cpp
const auto result = point_triangle_distance(p, t0, t1, t2);
const double s = result.distance_squared;
const auto& grad_s = result.gradient;
const auto& hess_s = result.hessian;
```

Phase 5 CCD 层也应该能够直接复用同一批距离函数做：

- 最小距离评估
- 是否仍有安全裕度判断

这就是为什么 geometry 层必须保持纯局部、纯坐标、无系统依赖。

---

## Phase 1 的完成标准

做到下面这些，这一阶段才算真正结束：

- `geometry/` 目录建立完成
- PP / PE / PT / EE 四个距离函数都可独立调用
- 每个函数都返回解析 gradient + Hessian
- PT 的区域分类正确
- EE 的平行退化路径正确
- 所有测试通过有限差分校验
- 所有结果在测试样例中保持 finite

如果这些条件没满足，就不要急着进入 barrier。Day 2 的后续质量几乎完全取决于这一层是否扎实。
