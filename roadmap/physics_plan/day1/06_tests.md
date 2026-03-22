# Phase 5: 测试与验证

## 测试注册

在 `test/CMakeLists.txt` 中新增（需要 link Eigen）：

```cmake
# ipc
rtr_add_test(
    test_ipc_state
    system/physics/ipc/ipc_state_test.cpp
    Eigen3::Eigen
)
rtr_add_test(
    test_ipc_tet_body
    system/physics/ipc/tet_body_test.cpp
    Eigen3::Eigen
)
rtr_add_test(
    test_ipc_tet_smoke
    system/physics/ipc/solver/ipc_tet_smoke_test.cpp
    Eigen3::Eigen
    spdlog::spdlog
)
```

---

## Test 1: `ipc_state_test.cpp`

```
namespace rtr::system::physics::ipc::test

TEST(IPCStateTest, ResizeAndDofCount)
  - resize(4) -> dof_count() == 12, vertex_count() == 4
  - x, v, mass_diag 大小正确

TEST(IPCStateTest, PositionSegment)
  - 设 x = [0,1,2, 3,4,5, 6,7,8, 9,10,11]
  - position(0) == [0,1,2]
  - position(2) == [6,7,8]

TEST(IPCStateTest, ResizeZero)
  - resize(0) -> dof_count() == 0
```

---

## Test 2: `tet_body_test.cpp`

```
TEST(TetBodyTest, SingleTetPrecompute)
  - 构造一个正四面体（已知坐标）
  - 调 precompute()
  - 验证 Dm_inv 正确（与手算对比）
  - 验证 rest_volume 正确（正四面体体积公式）

TEST(TetBodyTest, DegenerateTetThrows)
  - 四个共面点
  - precompute() 应抛异常或 volume <= 0 检查

TEST(TetBodyTest, TetBlockGeneration)
  - generate_tet_block(2,2,2, 1.0, origin)
  - vertex_count == 27
  - tet_count == 48 (2*2*2*6)
  - 所有 rest_volumes > 0
  - precompute 不抛异常

TEST(TetBodyTest, FixedVerticesMask)
  - 3x3x3 block
  - 标记 y >= 2.0 的顶点为 fixed
  - 验证 fixed_vertices 数量正确
```

---

## Test 3: `ipc_tet_smoke_test.cpp`

这是 Day 1 最关键的测试，验证完整闭环。

```
TEST(IPCTetSmokeTest, SingleStepNoNaN)
  - 构造 3x3x3 tet block
  - 顶部固定
  - IPCSystem::initialize()
  - IPCSystem::step() 一次
  - 验证 state.x 不含 NaN
  - 验证 fixed vertices 位置不变
  - 验证 free vertices 位置有变化（重力方向下移）

TEST(IPCTetSmokeTest, MultiStepStable)
  - 同上设置
  - 连续 step 10 次
  - 每步检查无 NaN
  - 每步检查 fixed vertices 不动
  - 最终 free vertices 有明显下移

TEST(IPCTetSmokeTest, ZeroGravityStationary)
  - gravity = (0,0,0)
  - 不固定任何顶点
  - step 5 次
  - 顶点应基本不动（静止状态是能量最小值）

TEST(IPCTetSmokeTest, EnergyDecreases)
  - 给一些顶点初始速度
  - 观察能量（通过 solver log 或直接算）
  - 多步后系统倾向于静止（惯性能主导的阻尼效果）
```

---

## 可选 Test 4: 有限差分检查

如果时间允许，做最高价值的数值验证：

```
TEST(TetEnergyFDTest, GradientMatchesFD)
  - 构造单 tet，随机微小扰动 F
  - 计算 analytic PK1
  - 计算 FD gradient (中心差分)
  - EXPECT_NEAR 每个分量，tolerance ~1e-5

TEST(InertialEnergyFDTest, GradientMatchesFD)
  - 随机 x, x_hat, mass_diag
  - analytic gradient vs FD
  - tolerance ~1e-8 (二次型，应该精确)
```

这两个 FD 测试能抓住 90% 的导数 bug，强烈建议 Day 1 做。

---

## 测试目录结构

```
test/system/physics/ipc/
  ipc_state_test.cpp
  tet_body_test.cpp
  solver/
    ipc_tet_smoke_test.cpp
```

## 验收总览

| 测试 | 最低要求 | 优先级 |
|------|---------|--------|
| ipc_state_test | 必须通过 | P0 |
| tet_body_test | 必须通过 | P0 |
| ipc_tet_smoke_test (SingleStep) | 必须通过 | P0 |
| ipc_tet_smoke_test (MultiStep) | 必须通过 | P0 |
| tet_energy_fd_test | 强烈推荐 | P1 |
| inertial_energy_fd_test | 推荐 | P1 |
