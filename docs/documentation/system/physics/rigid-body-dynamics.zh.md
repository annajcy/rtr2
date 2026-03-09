# 刚体动力学理论与实现 (Rigid Body Dynamics)

本文档将结合刚体动力学基础理论，逐步剖析其实际在代码引擎中的实现与其物理意义。

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

**蛙跳法 / 半隐式积分 (Leapfrog / Semi-implicit Integration) 代码实现：**
为了在计算性能和稳定性之间取得平衡，物理引擎常采用这种方法（中点法的应用范例）。这体现在 `PhysicsWorld::integrate_forces_and_drift`：

```cpp
// 1. 半步长速度的初始化与缓存（即 v^{[-0.5]} -> v^{[0.5]} 的预测）
if (!body.linear_half_step_initialized()) {
    body.initialize_half_step_linear_velocity(
        state.translation.linear_velocity + acceleration * (0.5f * delta_seconds));
}

// 2. 隐式更新位置：x^{[1]} = x^{[0]} + \Delta t v^{[0.5]}
state.translation.position += body.half_step_linear_velocity() * delta_seconds;

// 3. 显式更新半步长速度：v^{[0.5]} 的累加
body.half_step_linear_velocity() += acceleration * delta_seconds;
```
*(注：这里 `half_step_linear_velocity` 实际上就是由于积分错开半个步长的中点速度估算：先显式更新速度 $v^{[1]} = v^{[0]} + \Delta t M^{-1} f^{[0]}$，然后用最新的速度来隐式更新位置 $x^{[1]} = x^{[0]} + \Delta t v^{[1]}$，这等同于 $v^{[0.5]} = v^{[-0.5]} + \Delta t M^{-1} f^{[0]}$ 以及 $x^{[1]} = x^{[0]} + \Delta t v^{[0.5]}$)*

**常见受力计算 (Force Computation)：**
- **重力 (Gravity Force)**：$f_{gravity}^{[0]} = M\mathbf{g}$。在代码中直接体现为：`const pbpt::math::Vec3 gravity_force = body.use_gravity() ? (m_gravity * state.mass) : pbpt::math::Vec3(0.0f);`
- **合力与阻力 (Drag Force)**：理论公式为 $f_{drag}^{[0]} = -\sigma v^{[0]}$，在引擎中由用户自定义的阻力等合力会直接进入 `state.forces.accumulated_force`，二者共同算出加速度 `acceleration = total_force / state.mass`。（注：在实际工程中，由于阻力会降低速度，有时也会直接通过衰减系数使速度衰减：$v^{[1]} = a v^{[0]}$）

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
  在系统中封装在 `PhysicsWorld::inverse_inertia_tensor_world`：
  ```cpp
  const auto rotation_matrix = body.state().rotation.orientation.to_mat3();
  return rotation_matrix * body.inverse_inertia_tensor_ref() * pbpt::math::transpose(rotation_matrix);
  ```

## 四、 完整的刚体更新循环 (Simulation Implementation)

在物理引擎底层的 `PhysicsWorld::tick` 中，每一帧通过给定的时间步长 $\Delta t$，按照如下顺序对状态集合 $s = \{v, x, \omega, q\}$ 进行了完整更新：

1. **预处理与力、力矩累加 (Force Integration & Drift)**
   将四元数 $q^{[0]}$ 转换为旋转矩阵 $R^{[0]}$。根据局部惯性张量 $I_{ref}$ 计算当前的世界惯性张量（即前文的 $I^{[0]} \leftarrow R^{[0]}I_{ref}(R^{[0]})^T$ 操作）。收集所有外力 $f_i^{[0]}$ 并累加合力 $f^{[0]}$，转换为平移加速度。计算偏心受力产生的力矩并累加总力矩 $\tau^{[0]} \leftarrow \sum (R^{[0]}r_i) \times f_i^{[0]}$ 计算角加速度。
   
2. **平移状态更新半隐式更新 (Translational Update)**
   根据前述的蛙跳法，推进 `half_step_linear_velocity` 并用于隐式更新下一帧的位置 `position`。

3. **旋转状态半隐式更新 (Rotational Update)**
   类似平移状态，代码同样采用半步长角速度来积分更新旋转朝向四元数：
   ```cpp
   // 更新朝向四元数（四元数增量导数乘法积分）
   state.rotation.orientation = integrate_orientation(state.rotation.orientation, body.half_step_angular_velocity(), delta_seconds);
   // 更新半步长角加速度
   body.half_step_angular_velocity() += angular_acceleration * delta_seconds;
   ```
   其中 `integrate_orientation` 严格实现了公式 $q^{[1]} \leftarrow q^{[0]} + \left[0 \quad \frac{\Delta t}{2}\omega^{[1]}\right] \times q^{[0]}$（使用四元数乘法）：
   ```cpp
   const pbpt::math::Quat delta = (omega_quat * orientation) * (0.5f * delta_seconds);
   return pbpt::math::normalize(...); // 为了抵消浮点误差累加进行重归一化
   ```

4. **碰撞点处理与微调 (Solve Contacts)**
   积分完成后，可能引起物体相互嵌入。此时调用 `solve_contacts` 计算各碰撞点的冲量(Impulse)，直接用于修改刚体的相对半步长速度进行物理弹开，并通过位移校正减缓持续穿透的问题（Penetration Adjustment）。

5. **速度的复苏化同步 (Update Observable Velocities)**
   执行 `update_observable_velocities`。由于物理世界的积分推演错开了半个时间步长，为了让游戏逻辑层观测到的线性速度与角速度能够匹配目前的位置，$v$ 与 $\omega$ 会被再次补偿回落到整点节拍上。
