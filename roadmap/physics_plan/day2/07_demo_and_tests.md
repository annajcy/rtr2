# Phase 7: Demo 与测试

## 目标

端到端验证 obstacle contact 工作正确：tet body 在地面上停住，不穿透。

---

## Demo：tet block 落到地面

### 文件

```
examples/editor/ipc_ground_contact_editor.cpp
```

### 场景描述

- 一个 3×3×3 tet block，初始位于 $y = 1.5$
- 一个 ground plane obstacle，位于 $y = 0$
- 无固定顶点（全部自由）
- 重力 $g = -9.81$
- block 在重力下自由落体，落到地面后被 barrier 挡住

### 与 Day 1 demo 的区别

| 对比项 | Day 1 demo | Day 2 demo |
|--------|-----------|-----------|
| 约束方式 | `fixed_vertices`（固定一端） | 无固定顶点，靠 barrier 挡住 |
| 地面 | 纯视觉，不参与物理 | `ObstacleBody`，参与接触检测 |
| 接触 | 无 | ground plane contact |
| 行为 | cantilever 下垂 | 自由落体 → 接触 → 停住 |

### 初始化流程

```text
1. runtime + editor + scene + camera + light（照抄 Day 1）
2. 生成 tet block：generate_tet_block(3, 3, 3, 0.3, origin)
3. 不设 fixed_vertices（全部自由）
4. 注册 tet body 到 ipc_system
5. 生成 ground plane obstacle：generate_ground_plane(10.0, 0.0)
6. 注册 obstacle body 到 ipc_system
7. ipc_system.initialize()
8. 创建渲染用的 DeformableMeshComponent + IPCTetComponent
9. （可选）创建 ground visual mesh
10. runtime.run()
```

### IPCConfig

```cpp
ipc::IPCConfig config{
    .dt = 1.0 / 60.0,
    .gravity = Eigen::Vector3d{0.0, -9.81, 0.0},
    .solver_params = {
        .max_iterations = 50,
        .gradient_tolerance = 1e-5,
    },
    .dhat_squared = 0.01,        // dhat ≈ 0.1
    .barrier_stiffness = 1e4,    // κ
    .enable_contact = true,
};
```

### 预期行为

1. 前 1-2 秒：block 在重力下加速下落
2. block 接近地面（$y \approx 0.1$）时：barrier 开始生效，block 减速
3. block 最终停在地面附近（$y \approx 0$），略有弹跳后稳定
4. 长时间运行不穿透、不 NaN

### 可能遇到的问题及调参

| 问题 | 原因 | 调参 |
|------|------|------|
| block 穿透地面 | $\kappa$ 太小或 CCD 没生效 | 增大 `barrier_stiffness`；检查 CCD 是否约束了 line search |
| block 弹飞 | $\kappa$ 太大 | 减小 `barrier_stiffness` |
| Newton 不收敛 | barrier Hessian 条件数差 | 增大 `NewtonSolverParams::min_regularization`；减小 `barrier_stiffness` |
| block 悬浮在空中 | `dhat` 太大 | 减小 `dhat_squared` |
| NaN 出现 | 距离函数退化 | 检查 distance 实现中的除零保护 |

---

## 可选 Demo 2：tet block 落到球上

如果 ground plane demo 顺利，可以加一个球体障碍物的 demo：

- block 从上方落下
- 球体固定在 $(0, 0.5, 0)$，半径 0.5
- block 应该在球面上滑动/弹开（无 friction 的情况下会滑走）

这个 demo 验证的是非平面接触（曲面法向量各不同）。

---

## 强制测试清单

### 几何距离 FD 测试

| 测试 | 通过标准 |
|------|---------|
| PP distance gradient fd | 相对误差 < $10^{-5}$ |
| PP distance Hessian fd | 相对误差 < $10^{-5}$ |
| PE distance gradient fd | 相对误差 < $10^{-5}$ |
| PE distance Hessian fd | 相对误差 < $10^{-5}$ |
| PT distance gradient fd（多区域） | 相对误差 < $10^{-5}$ |
| PT distance Hessian fd（多区域） | 相对误差 < $10^{-5}$ |
| EE distance gradient fd | 相对误差 < $10^{-5}$ |
| EE distance Hessian fd（含平行） | 相对误差 < $10^{-5}$ |

### Barrier FD 测试

| 测试 | 通过标准 |
|------|---------|
| scalar barrier gradient fd | 相对误差 < $10^{-5}$ |
| scalar barrier Hessian fd | 相对误差 < $10^{-5}$ |
| single PT barrier energy gradient fd | 相对误差 < $10^{-4}$ |

### 系统级 Smoke 测试

| 测试 | 通过标准 |
|------|---------|
| tet block 落到地面不穿透 | 最终 $y_{min} > -0.01$ |
| 多步运行无 NaN | 100 步无异常 |
| 无接触情况下行为不变 | `enable_contact=false` 结果与 Day 1 一致 |
| barrier energy > 0 when close | 距离 < dhat 时 barrier 贡献非零 |

---

## CMake 修改

```cmake
# examples/CMakeLists.txt
add_executable(ipc_ground_contact_editor editor/ipc_ground_contact_editor.cpp)
target_link_libraries(ipc_ground_contact_editor PRIVATE rtr::runtime rtr::editor)
```

测试文件注册到 `test/CMakeLists.txt`。

---

## Day 2 结束时应明确回答的问题

1. 几何距离函数在所有区域的梯度/Hessian 是否正确？（FD 检查通过？）
2. barrier 在 $d \to 0$ 时是否真的 $\to +\infty$？在 $d = \hat{d}$ 时是否 $C^1$ 连接到零？
3. CCD 安全步长是否真正约束了 line search？
4. tet block 能否在地面上停住？
5. 长时间运行（100+ 步）是否稳定？
6. obstacle body 添加/删除是否正确触发 collision mesh 重建？
7. Day 3 加 self-contact 时该往哪里插代码？

## Day 2 结束后的输出物

- [ ] 编译通过的 geometry + contact + CCD 模块
- [ ] 通过 FD 检查的距离和 barrier 测试
- [ ] 1 个 contact demo 可运行
- [ ] 已知问题列表
- [ ] Day 3 第一优先级事项
