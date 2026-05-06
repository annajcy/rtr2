# IPC Geometry 总览

`geometry/` 目录承载未来 IPC 接触系统要用到的局部几何距离核。

这一层的职责刻意收得很窄：

- 只接收局部 primitive 坐标
- 只返回局部 dense 的 `distance_squared + gradient + hessian`
- 不依赖 `IPCState`、`IPCSystem`、body 元数据或 candidate 容器

这样同一批距离核就可以被 barrier、CCD 和有限差分测试复用。

## 当前模块

`src/rtr/system/physics/ipc/geometry/` 当前包含：

- `distance_common.hpp`：共享结果类型、finite 检查、embedding helper 和二阶 AutoDiff 求值
- `distance_concept.hpp`：distance kernel 的编译期接口约束
- `point_point_distance.hpp`：点点距离平方
- `point_edge_distance.hpp`：点线段距离平方与端点退化
- `point_triangle_distance.hpp`：点三角形区域分类与组合求值
- `edge_edge_distance.hpp`：边边区域分类、平行退化与组合求值

## 局部 DOF 排列

所有距离核都使用固定的 point-major 局部排列：

| 距离核 | 局部 DOF |
|---|---|
| PP | `[p0_x, p0_y, p0_z, p1_x, p1_y, p1_z]` |
| PE | `[p_x, p_y, p_z, e0_x, e0_y, e0_z, e1_x, e1_y, e1_z]` |
| PT | `[p_x, p_y, p_z, t0_x, t0_y, t0_z, t1_x, t1_y, t1_z, t2_x, t2_y, t2_z]` |
| EE | `[ea0_x, ea0_y, ea0_z, ea1_x, ea1_y, ea1_z, eb0_x, eb0_y, eb0_z, eb1_x, eb1_y, eb1_z]` |

这个顺序是模块契约的一部分。后续 barrier 装配、candidate wrapper 与有限差分测试都必须严格沿用同一顺序。

## 在系统中的位置

`geometry/` 位于未来 contact 管线的底层：

```text
局部坐标
    -> geometry distance kernels
    -> 局部 dense distance / gradient / hessian
    -> barrier/contact 装配
    -> CCD 安全步长判断

同一批 kernel
    -> finite-difference tests
```

更具体地说：

- `geometry` 只负责单个 primitive pair 的局部求值
- 未来 `contact/barrier` 会消费 `distance_squared`、`gradient` 和 `hessian`
- 未来 `ccd` 会复用同一批 kernel，在 trial step 上评估局部分离距离
- tests 通过统一接口对解析导数做有限差分校验

## 为什么当前实现采用这种结构

当前实现把三件事组合在一起：

1. **显式区域分类**
   - PT 和 EE 先决定当前激活的是哪一个几何区域
   - 这样分支逻辑更容易读、也更容易测

2. **子核复用 + embedding**
   - PT 的边区和顶点区复用 PE / PP
   - EE 的 fallback 区域复用 PE
   - 低维导数结果再嵌回父级 12 DOF 布局

3. **对 smooth expression 使用二阶 AutoDiff**
   - 一旦 active region 确定，局部光滑表达式就用嵌套的 `Eigen::AutoDiffScalar` 求值
   - 这样运行时拿到的是解析 gradient/Hessian，而不是数值差分

这和“把 PT/EE 所有二阶导硬塞进一个大闭式函数”不同。当前设计用少量抽象换来了更好的可维护性：

- 区域分类错误和导数错误可以分开排查
- 复用的子核在 PT / EE 中保持一致
- tests 可以对整套 `Distance` 接口统一做验证

## 与后续 IPC 层的关系

`geometry/` 本身不是完整接触系统，而是后续模块的数学地基：

- barrier/contact 会把局部距离平方转成局部 barrier 能量，再装配进全局稀疏系统
- CCD 会在 trial 坐标上调用同一批 kernel 来约束安全步长
- solver 仍然负责 globalization、PSD 处理和稀疏装配

这里有一个重要边界：

- `geometry/` 返回的是距离表达式本身的真实局部二阶导
- 它**不会**做 PSD projection，也不会替 solver 做稳定化
- 任何正定化或正则化都应该留在 barrier / solver 层
