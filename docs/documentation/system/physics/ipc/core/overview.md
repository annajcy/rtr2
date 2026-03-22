# IPC Core Overview

The `core/` directory contains solver-facing state that is independent of any single tet body.

Current contents:

- `src/rtr/system/physics/ipc/core/ipc_state.hpp`: the global `3N` state vector used by the future FEM/IPC solver

This layer intentionally does not own:

- tet connectivity
- material parameters
- boundary-condition metadata
- render-mesh conversion logic

Those belong in `model/`. The purpose of `core/` is to expose a compact, optimizer-friendly representation of the unknowns.
