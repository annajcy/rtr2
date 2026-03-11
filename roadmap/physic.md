# Physics System Roadmap

## Goal

Build the follow-up GAMES103 labs on top of the current RTR2 framework by extending the existing rigid-body pipeline into a layered physics system:

- Lab 2: cloth simulation
- Lab 3: tetrahedral FEM soft body
- Lab 4: shallow-wave water and coupling

The right strategy is not to rewrite the current rigid-body system. RTR2 already has:

- fixed-step runtime loop
- `PhysicsWorld` for rigid bodies and collisions
- `Component::on_fixed_update()` for custom simulation updates
- scene-to-physics and physics-to-scene sync

So the roadmap should treat cloth, FEM, and water as new simulation modules that run alongside the current rigid-body world.

## Existing Integration Points

Useful framework anchors:

- `src/rtr/app/app_runtime.hpp`
- `src/rtr/framework/component/component.hpp`
- `src/rtr/framework/core/game_object.hpp`
- `src/rtr/framework/integration/physics/scene_physics_sync.hpp`
- `examples/games103_lab/lab1_angry_bunny/lab1_angry_bunny.cpp`

Design implication:

- keep `PhysicsWorld` focused on rigid-body dynamics and collision
- add new simulation systems for deformables and fluids
- update these systems in fixed tick
- write results back to scene transforms or dynamic mesh vertices

## Architecture Direction

Recommended code layout:

- `src/rtr/system/simulation/common/`
- `src/rtr/system/simulation/cloth/`
- `src/rtr/system/simulation/fem/`
- `src/rtr/system/simulation/water/`
- `src/rtr/framework/component/simulation/`
- `examples/games103_lab/lab2_cloth/`
- `examples/games103_lab/lab3_bouncy_house/`
- `examples/games103_lab/lab4_pool_ripples/`

Recommended ownership split:

- rigid bodies, colliders, impulses: existing `PhysicsWorld`
- particle/constraint states: cloth module
- tetrahedral mesh and elastic force states: FEM module
- 2D height field and coupling masks: water module
- render-facing mesh sync: common simulation utilities

## Phase 0: Foundation

Objective:

Create a reusable simulation scaffold before implementing individual labs.

Tasks:

- define a common fixed-step simulation update pattern
- add dynamic mesh update utilities for deforming surfaces/volumes
- add lightweight debug visualization for particles, edges, tetrahedra, and grids
- separate simulation data from render mesh data
- decide how each simulation component stores topology and state

Deliverables:

- reusable simulation base utilities
- one minimal deformable-mesh example that updates vertices every fixed tick

Why this matters:

Lab 2, 3, and 4 all need stable fixed-step updates and mesh write-back. If this is hacked per lab, later coupling work will become messy.

## Phase 1: Lab 2 Cloth

Objective:

Implement cloth first, starting with PBD, then optionally adding the implicit solver backend.

Recommended order:

1. particle and edge topology
2. pinned vertices
3. gravity and damping
4. PBD strain limiting
5. sphere collision
6. implicit cloth solver
7. Chebyshev acceleration

Why start with PBD:

- faster to get running
- builds reusable particle-system infrastructure
- good for validating fixed points, constraints, and collision projection

Core features:

- cloth mesh topology generation
- per-vertex position and velocity
- edge rest lengths
- pinned constraints
- multiple solver iterations per fixed step
- sphere collision projection and velocity correction

Suggested modules:

- `cloth_state`
- `cloth_topology`
- `cloth_pbd_solver`
- `cloth_implicit_solver`
- `cloth_mesh_sync`

Exit criteria:

- fixed points remain stable
- cloth does not stretch excessively
- sphere collision is robust
- simulation remains stable at the project fixed time step

## Phase 2: Lab 3 FEM Soft Body

Objective:

Extend the system from surface particles to tetrahedral elastic bodies.

Core features:

- tetrahedral topology
- reference edge matrix `Dm`
- cached `inv(Dm)` per tet
- deformation gradient `F`
- Green strain
- StVK stress
- elastic force accumulation to tet vertices
- explicit integration
- floor contact with friction
- Laplacian smoothing on velocities

Implementation order:

1. single tetrahedron test
2. edge-matrix builder and `inv(Dm)` cache
3. force computation for one tet
4. force assembly for full mesh
5. explicit integration and damping
6. floor collision and friction
7. velocity smoothing
8. house-model integration

Suggested modules:

- `tet_mesh`
- `fem_state`
- `stvk_material`
- `fem_force_assembler`
- `fem_collision`
- `velocity_smoothing`

Exit criteria:

- single tetrahedron behaves correctly under deformation
- house mesh falls and deforms under gravity
- floor contact is stable
- smoothing visibly reduces high-frequency jitter

## Phase 3: Lab 4 Shallow-Wave Water

Objective:

Add a grid-based surface solver instead of a particle or tetrahedral solver.

Important design note:

Do not try to force water into the same data model as cloth or FEM. Water here is a height-field simulation and should stay separate.

Core features:

- load mesh vertex heights into a height field
- random droplet injection with volume preservation
- repeated shallow-wave substeps
- Neumann boundary conditions
- mesh normal recomputation

Suggested modules:

- `height_field_2d`
- `shallow_wave_solver`
- `water_disturbance`
- `water_mesh_sync`

Implementation order:

1. map mesh vertices to height grid
2. implement `h`, `old_h`, and `new_h`
3. add random drops on key input
4. preserve total volume locally
5. run 8 shallow-wave iterations per frame/update step
6. update mesh normals

Exit criteria:

- pressing `r` creates stable disturbances
- waves propagate and reflect correctly
- boundary behavior matches Neumann conditions
- surface normals update correctly for rendering

## Phase 4: Coupling

Objective:

Connect the new water solver with the existing rigid-body system.

### 4.1 One-Way Coupling

Blocks affect water.

Tasks:

- compute contact mask from block occupancy
- compute `low_h`
- solve masked Poisson equation
- feed corrected height response back to water

Why first:

This matches the lab requirement and keeps the dependency direction simple.

Exit criteria:

- moving blocks depress the surface correctly
- ripples are generated around occupied cells

### 4.2 Two-Way Coupling

Water affects rigid bodies.

Tasks:

- derive forces from water response
- integrate force and torque onto rigid blocks
- feed them into existing rigid-body dynamics

Main difficulty:

Once blocks rotate, occupancy and target water height are no longer simple position checks. This likely needs bounding volume tests plus ray or sampling queries.

Exit criteria:

- blocks disturb water
- water response produces plausible rigid-body reaction

## Cross-Cutting Engineering Priorities

These should be done early and reused everywhere:

- fixed-step determinism where practical
- simulation reset support
- stable parameter exposure for damping, stiffness, solver iterations, and substeps
- mesh-state visualization for debugging
- small deterministic examples before full scenes

Recommended debug assets:

- particle/edge overlay for cloth
- tetra wireframe or tet inspector for FEM
- grid occupancy and mask debug view for water coupling

## Milestone Plan

### Milestone A

Simulation scaffold plus one deformable mesh update path.

### Milestone B

Lab 2 PBD cloth fully working.

### Milestone C

Implicit cloth solver added as an alternative backend.

### Milestone D

Lab 3 FEM house simulation working on tet mesh.

### Milestone E

Lab 4 ripple simulation working on a height field.

### Milestone F

Lab 4 one-way coupling with rigid blocks.

### Milestone G

Optional two-way coupling from water back to rigid bodies.

## Recommended Execution Order

If the goal is to finish the labs efficiently:

1. PBD cloth
2. FEM soft body
3. shallow-wave water
4. one-way coupling
5. implicit cloth
6. two-way coupling

If the goal is to build a stronger reusable physics system:

1. foundation scaffold
2. PBD cloth
3. FEM soft body
4. water solver
5. coupling
6. optional advanced solvers and accelerations

## Final Guidance

The main technical principle for this roadmap is:

- do not over-unify the math
- do unify the runtime scaffolding

Cloth, FEM, and shallow-wave water are different numerical models. They should share:

- fixed-step scheduling
- component integration style
- mesh synchronization
- debugging and parameter plumbing

But they should not be forced into one fake generic solver abstraction too early.
