# 刚体动力学理论与实现 (Rigid Body Dynamics)

本文档结合基础理论，说明当前代码库中的刚体模拟实现。这里描述的是现在的 `PhysicsSystem -> RigidBodyWorld -> collision pair generation` 结构，而不是旧版 `PhysicsWorld` / Leapfrog 设计。

## 一、 刚体动力学基础 (Introduction)

**核心目标：**
物理引擎模拟的核心是随着时间推移，不断更新刚体集合的状态变量 $s^{[k]} = \{v, x, \omega, q\}$。

**运动拆解与方程表示：**
由于刚体不会发生形变，其在空间中的所有复杂运动都可以被完美拆分为两部分：平移（Translation） 和 旋转（Rotation）。
刚体上任意一点的当前世界坐标可以表示为 $x + Rr_i$，即先通过局部坐标 $r_i$ 和旋转矩阵 $R$ 进行旋转计算，再加上刚体质心的平移位置 $x$。在引擎中，物体的 `position` 就是此处的质心平移 $x$，而 `orientation` (四元数) 则对应这里的旋转矩阵 $R$。 

## 二、 平移运动与数值积分 (Translational Motion)

平移运动的状态变量包含物理位置 $x$ 和线性速度 $v$（详见代码 `TranslationState`）。

**数值积分方法对比：**
位置和速度的更新本质上是在计算积分，即估算速度/加速度曲线下方的面积：
- **显式欧拉 (Explicit Euler)**：一阶精度，使用时间段起点 $t^{[0]}$ 的速度作为矩形高度。截断误差为 $O(\Delta t^2)$。
- **隐式欧拉 (Implicit Euler)**：一阶精度，使用时间段终点 $t^{[1]}$ 的速度作为矩形高度。截断误差同样为 $O(\Delta t^2)$。
- **中点法 (Mid-point)**：二阶精度，使用时间段中点 $t^{[0.5]}$ 的速度作为高度。其一阶导数带来的误差可以相互抵消，最终截断误差缩小至 $O(\Delta t^3)$。

**半隐式欧拉积分 (Semi-implicit Euler) 代码实现：**
为了在实现复杂度和稳定性之间取得平衡，当前引擎在 `RigidBodyWorld::integrate_body` 中采用半隐式欧拉更新：

```cpp
const auto acc = rb.state().inverse_mass() * rb.state().forces.accumulated_force;
rb.state().translation.linear_velocity += acc * delta_seconds;
rb.state().translation.linear_velocity *= rb.linear_decay();
rb.state().translation.position += rb.state().translation.linear_velocity * delta_seconds;
```

这里的顺序很关键：先更新速度，再用更新后的速度推进位置，这就是标准的 semi-implicit Euler。

**常见受力计算 (Force Computation)：**
- **重力 (Gravity Force)**：$f_{gravity} = M\mathbf{g}$。如果 `use_gravity()` 打开，系统会在积分前把重力加到力累加器中。
- **外力累加 (Force Accumulation)**：用户代码或框架逻辑施加的外力进入 `state.forces.accumulated_force`，再通过逆质量转成线加速度。

## 三、 旋转运动与四元数 (Rotational Motion)

旋转运动的状态变量包含四元数 $q$（表示朝向）和角速度 $\omega$（方向代表旋转轴，大小代表旋转速率）（详见代码 `RotationalState`）。

**为什么选择四元数？**
- **矩阵 (Matrix)**：有 9 个元素但只有 3 个自由度，存在严重冗余，不直观，且很难定义其时间导数。
- **欧拉角 (Euler Angles)**：虽然直观，但在某些轴向对齐时会导致自由度丢失，即万向节死锁（Gimbal Lock），同样难以定义时间导数。
- **四元数 (Quaternion)**：使用四个数字即可无死角地表示 3D 旋转，计算高效，是诸如 Unity 等游戏引擎的内置首选表示法。表示绕轴 $v$ 旋转 $\theta$ 角度的四元数公式：$q = [\cos\frac{\theta}{2} \quad \mathbf{v}]$（其中向量部分的长度为 $\sin\frac{\theta}{2}$）。
引擎代码中使用 `pbpt::math::Quat` 来执行极高效率的旋转更新计算。

**旋转动力学的核心物理量：**
- **力矩 (Torque, $\tau$)**：相当于平移运动中的力（Force），即代码中的 `state.forces.accumulated_torque`。计算公式为受力点的世界力臂向量和力的叉乘：$\tau_i = (Rr_i) \times f_i$。
- **惯性张量 (Inertia, $I$)**：相当于平移运动中的质量（Mass）。由于物体在旋转，其世界坐标下的惯性张量需要根据当前的旋转矩阵 $R$ 实时更新：$I^{-1} = R I_{ref}^{-1} R^T$。代码里刚体存储的是自身的局部逆惯性张量 (`m_inverse_inertia_tensor_ref`)。
  在系统中封装在 `RigidBodyWorld::inverse_inertia_tensor_world`：
  ```cpp
  const auto rotation_matrix = body.state().rotation.orientation.to_mat3();
  return rotation_matrix * body.inverse_inertia_tensor_ref() * pbpt::math::transpose(rotation_matrix);
  ```

## 四、 完整的刚体更新循环 (Simulation Implementation)

在 `RigidBodyWorld::step` 中，每一帧通过给定时间步长 $\Delta t$，按如下顺序更新状态集合 $s = \{v, x, \omega, q\}$：

1. **积分 Dynamic 刚体**
   对每个 awake 的 Dynamic 刚体，系统会：
   - 在需要时加入重力，
   - 根据累计外力积分线速度，
   - 施加线速度衰减，
   - 用更新后的线速度推进位置，
   - 计算世界空间逆惯性张量，
   - 根据累计力矩积分角速度，
   - 施加角速度衰减，
   - 推进四元数朝向并重新归一化。
   
2. **构建 Contact Snapshot**
   系统遍历 collider 对，跳过同 body 和 static-static 组合，将 collider 转成 `WorldCollider`，并调用 `collision/` 中对应的 `ContactPairTrait<...>::generate(...)`。

3. **把几何接触提升为求解接触**
   有效的 `ContactResult` 会先转成 rigid-body 模块内部的 `Contact`，再转成 `SolverContact`。这一阶段会计算：
   - body id 和 collider id，
   - 接触力臂 `r_a`、`r_b`，
   - 世界空间逆惯性张量，
   - 法线与切线方向，
   - 法向/切向 effective mass，
   - restitution bias，
   - 当前帧内初始化为 0 的累计冲量。

4. **速度阶段：Projected Gauss-Seidel**
   系统对同一帧 snapshot 迭代 `velocity_iterations` 次。对每个 contact：
   - 求法向冲量，
   - 以累计形式更新并 clamp 到非负，
   - 将实际增量冲量施加到两个刚体，
   - 再求切向冲量，
   - 将摩擦冲量幅值限制在 `mu * normal_impulse_sum` 内。

5. **位置阶段：穿透修正**
   系统再执行 `position_iterations` 轮位置修正，使用 snapshot 中保存的法线和穿透深度来缓解 interpenetration。这个阶段不会在同一帧内重建接触几何。

6. **清空外力**
   求解结束后，每个 rigid body 都会清空力和力矩累加器，为下一帧的外部驱动做准备。

## 五、 旋转更新细节

当前朝向更新使用标准的四元数导数形式：

```cpp
const auto angular_velocity_quat = pbpt::math::Quat(
    0.0f,
    rb.state().rotation.angular_velocity.x(),
    rb.state().rotation.angular_velocity.y(),
    rb.state().rotation.angular_velocity.z());
rb.state().rotation.orientation +=
    0.5f * delta_seconds * angular_velocity_quat * rb.state().rotation.orientation;
rb.state().rotation.orientation = pbpt::math::normalize(rb.state().rotation.orientation);
```

这不是当前代码中的“半步长角速度”积分，而是直接的 semi-implicit 更新，并通过归一化抑制数值误差。

## 六、 `collision` 与求解器的边界

当前刚体求解器有意持有耦合侧类型：

- `rigid_body/collider.hpp` 定义带 `RigidBodyID` 的 body-attached collider。
- `rigid_body/contact.hpp` 定义 `Contact` 和 `SolverContact`。
- `collision/contact.hpp` 只保留 `ContactResult` 和 `ContactPairTrait`。

这样做可以让碰撞检测保持可复用，同时保证 solver-facing state 保留在 rigid-body 模块内部。
