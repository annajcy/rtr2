# `tet_material_variant.hpp`

[`tet_material_variant.hpp`](/Users/jinceyang/Desktop/codebase/graphics/rtr2/src/rtr/system/physics/ipc/energy/material_model/tet_material_variant.hpp) 定义了 `TetBody` 当前使用的运行时材料容器。

## 作用

现在材料模型放在每个 body 自己身上，而不是整个 IPCSystem 只持有一个全局材料。当前类型是：

```cpp
using TetMaterialVariant = std::variant<
    FixedCorotatedMaterial
>;
```

这意味着：

- 后面不同 tet body 可以挂不同本构模型
- `IPCSystem` 不需要把材料类型写死成一个全局实例
- dispatch 在 body 级别用 `std::visit` 完成，而真正的 per-tet 内循环仍然保持模板化

## 当前范围

目前 variant 里只有 `FixedCorotatedMaterial` 一种类型，所以它本质上还是一个为后续扩展准备好的兼容封装。
