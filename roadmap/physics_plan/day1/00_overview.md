# Day 1 Overview

## 目标

跑通无接触 tet FEM 隐式求解闭环：

**`TetBody -> implicit energy solve -> updated positions/velocities -> scene write-back -> next step`**

## Demo Target

一个顶部固定一小块顶点的程序生成 tet block（3x3x3），在重力下自然下垂并表现出弹性变形。

## 不做什么

- cloth / ShellBody
- contact / barrier / CCD
- friction
- self-contact
- 高性能 broad phase
- 完整 hyperelastic 材料库
- scene 级集成（不接入 PhysicsSystem 主循环）

## 交付物

1. 编译通过的 IPC 代码骨架
2. 3 个单元测试通过
3. 1 个 smoke test 验证多步求解稳定
4. 可选：headless demo 入口

## 执行顺序

```text
Phase -1: 清理 Physics 目录结构          (~1 hr)
Phase  0: Eigen 引入与构建验证           (~30 min)
Phase  1: 核心数据结构                   (~1.5 hr)
Phase  2: 能量模块                       (~2 hr)
Phase  3: Newton solver + line search    (~1.5 hr)
Phase  4: IPCSystem 组装                 (~1 hr)
Phase  5: 测试与验证                     (~1.5 hr)
```

详见各阶段文件。

Phase -1 必须最先做：把 collision 合并进 rigid_body，删除 cloth/fem/common/coupling，新建 ipc 目录。这样后续所有代码直接写入干净的目录结构。

## 清理后的 Physics 目录结构

```text
src/rtr/system/physics/
├── physics_system.hpp
├── normal_recompute.hpp
├── rigid_body/              # 保留 + collision 合并
│   ├── collision/           # 从 physics/collision/ 移入
│   └── ...
└── ipc/                     # 新建
    ├── core/
    ├── model/
    ├── energy/
    └── solver/
```

## 文件依赖图

```
ipc_state.hpp
  <- ipc_body.hpp
       <- tet_body.hpp
       <- obstacle_body.hpp (占位)
  <- tet_material_model.hpp
       <- tet_fixed_corotated_energy.hpp
  <- inertial_energy.hpp
  <- gravity_energy.hpp
  <- line_search.hpp
       <- newton_solver.hpp
            <- ipc_system.hpp
```

## 理论→代码对照表

| 课程笔记 | Section | Day 1 代码文件 | 对应内容 |
|----------|---------|---------------|----------|
| Lec 1 | §4.3 | `inertial_energy.hpp` | $E_I$ 的定义和导数 |
| Lec 1 | §4.3 | `gravity_energy.hpp` | 独立重力势能项 |
| Lec 1 | §4.4 | `line_search.hpp` | Backtracking line search |
| Lec 1 | §4.4 | `newton_solver.hpp` | Projected Newton 主循环 |
| Lec 2 | §3.5-3.7 | `tet_elastic_assembler.hpp` | 局部→全局装配 |
| Lec 3 | §4 | `newton_solver.hpp` | Dirichlet DOF elimination |
| Lec 7 | §1.4 | `tet_elastic_assembler.hpp` | $F = D_s D_m^{-1}$ |
| Lec 7 | §3 | `tet_fixed_corotated_energy.hpp` | SVD / polar decomposition |
| Lec 7 | §5.2 | `tet_fixed_corotated_energy.hpp` | Fixed corotated $\Psi$ |
| Lec 7 | §5.5 | `tet_material_model.hpp` | Lame 参数 |
| Lec 8 | §2 | `tet_fixed_corotated_energy.hpp` | PK1 stress $P$ |
| Lec 8 | §4-5 | `tet_fixed_corotated_energy.hpp` | Stress derivative / Hessian |
| Lec 10 | §2-3 | `ipc_state.hpp` | FEM 离散 → 全局 DOF 向量 |
| Lec 10 | §3 | `tet_body.hpp` | Shape functions = 重心坐标 |
| Lec 10 | §4 | `tet_body.hpp` + `ipc_system.hpp` | Mass lumping |
| Lec 10 | §5 | `tet_elastic_assembler.hpp` | $f^{int} = -V_e P \nabla N_a$ |

课程笔记根目录：`/Users/jinceyang/Desktop/cg_course/physics_based_simulation/`

参考代码：`solid-sim-tutorial/6_inv_free/`（Neo-Hookean FEM 完整实现）

### 推荐复习顺序（1.5 小时）

1. **Lec 1 §4.3**（15 min）— 优化重构，$E(x)$ 两项的物理意义
2. **Lec 7 §1.4**（10 min）— $F = D_s D_m^{-1}$ 的推导和直觉
3. **Lec 7 §5.2**（10 min）— Fixed corotated 公式
4. **Lec 8 §2**（10 min）— PK1 stress 定义
5. **Lec 10 §5.1-5.5**（15 min）— $F$ 在单元内是常量，节点力 $= -V_e P \nabla N_a$
6. **Lec 3 §4**（15 min）— DOF elimination 实现方式
7. **Lec 1 §4.4**（10 min）— Backtracking line search
8. **Lec 2 §3.5-3.7**（10 min）— 局部→全局稀疏装配

---

## 成功标准

- `IPCState` 统一管理所有 DOF
- `TetBody` rest shape 预计算正确（Dm, inv(Dm), volume）
- 总能量装配路径打通（inertial + gravity + elastic）
- Newton solver 调用 energy/gradient/Hessian 并收敛
- Dirichlet 消元正确处理固定顶点
- 多步运行不出 NaN
- 明天加 contact 时知道往哪插 barrier
