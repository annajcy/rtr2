# `tet_material_variant.hpp`

[`tet_material_variant.hpp`](/Users/jinceyang/Desktop/codebase/graphics/rtr2/src/rtr/system/physics/ipc/energy/material_model/tet_material_variant.hpp) defines the runtime material container used by `TetBody`.

## Role

Material behavior now lives on each body instead of on the whole IPC system. The current type is:

```cpp
using TetMaterialVariant = std::variant<
    FixedCorotatedMaterial
>;
```

That means:

- different tet bodies can carry different constitutive models later
- `IPCSystem` no longer needs to hard-code one global material instance
- dispatch happens per body through `std::visit`, while the per-tet inner loop remains template-based

## Current Scope

At the moment the variant contains only `FixedCorotatedMaterial`, so the runtime cost is effectively just a forward-compatible wrapper around the existing model.
