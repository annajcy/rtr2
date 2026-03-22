# `tet_material_model_concept.hpp`

`src/rtr/system/physics/ipc/energy/material_model/tet_material_model_concept.hpp` 定义了 tet 材料模型的编译期 concept。

## 设计思路：concept + 模板，不用虚函数

`compute_energy` / `compute_pk1` / `compute_hessian` 在每次 Newton 迭代中被**每个 tet** 调用——最热的内循环。需要编译器能 inline。

| 方案 | inline | 侵入性 | 错误提示 |
|------|--------|--------|----------|
| 虚函数 | 不能 | 需继承基类 | 运行时 |
| CRTP | 能 | 需继承 `Base<Derived>` | 编译期但难读 |
| **concept** | 能 | **零侵入** | 编译期，清晰 |

材料类型不需要继承任何基类，只要满足接口约束即可。

## `TetMaterialModel` concept

```cpp
template <typename M>
concept TetMaterialModel = requires(const M& material,
                                    const Eigen::Matrix3d& F,
                                    double rest_volume,
                                    double youngs_modulus,
                                    double poisson_ratio) {
    { material.compute_energy(F, rest_volume, youngs_modulus, poisson_ratio) }
        -> std::convertible_to<double>;
    { material.compute_pk1(F, rest_volume, youngs_modulus, poisson_ratio) }
        -> std::convertible_to<Eigen::Matrix3d>;
    { material.compute_hessian(F, rest_volume, youngs_modulus, poisson_ratio) }
        -> std::convertible_to<Eigen::Matrix<double, 9, 9>>;
};
```

## 三个方法的含义

| 方法 | 数学 | 返回 | 说明 |
|------|------|------|------|
| `compute_energy` | $V_e \cdot \Psi(F)$ | `double` | 单元总能量（密度 × rest volume） |
| `compute_pk1` | $P = \partial\Psi/\partial F$ | `Matrix3d` | PK1 stress（**不含** $V_e$） |
| `compute_hessian` | $\partial^2\Psi/\partial F^2$ | `Matrix<9,9>` | F-space Hessian（$F$ 展平为 9 维） |

注意 `compute_energy` 返回的已含 $V_e$，但 `compute_pk1` 和 `compute_hessian` 不含——$V_e$ 在装配层 `MaterialEnergy` 中乘入。

## 参数说明

| 参数 | 类型 | 含义 |
|------|------|------|
| `F` | `Matrix3d` | 形变梯度，$F = D_s D_m^{-1}$ |
| `rest_volume` | `double` | 参考构型体积 $V_e = \|\det(D_m)\| / 6$ |
| `youngs_modulus` | `double` | 杨氏模量 $E$（刚度） |
| `poisson_ratio` | `double` | 泊松比 $\nu \in (-1, 0.5)$ |

材料模型只关心"给定 $F$，返回能量/应力/Hessian"。$F$ 的构建（从全局 DOF 提取 $D_s$）和结果的映射（PK1 → 节点力）都属于装配层 `MaterialEnergy` 的职责。

## 当前实现

唯一的具体材料：`FixedCorotatedMaterial`（见 `tet_fixed_corotated.hpp`）。

```cpp
static_assert(TetMaterialModel<FixedCorotatedMaterial>);
```

## 扩展

添加新材料（如 Neo-Hookean、StVK）只需定义一个提供三个 const 方法的 struct，不需要继承、注册或修改任何已有代码。

如果未来需要运行时选材料（编辑器下拉菜单），可以用 `std::variant<FixedCorotated, NeoHookean, ...>` + `std::visit`，内层仍保持模板 inline。
