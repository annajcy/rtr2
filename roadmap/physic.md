# Physics System Roadmap

## Goal

Build the follow-up GAMES103 labs on top of the current RTR2 framework by extending the existing rigid-body pipeline into a physics system with explicit world ownership:

- Lab 2: cloth simulation
- Lab 3: tetrahedral FEM soft body
- Lab 4: shallow-wave water and coupling

The right strategy is not to rewrite the current rigid-body system. RTR2 already has:

- fixed-step runtime loop
- `PhysicsWorld` for rigid bodies and collisions
- `Component::on_fixed_update()` for custom simulation updates
- scene-to-physics and physics-to-scene sync

So the roadmap should evolve `PhysicsSystem` into a coordinator that explicitly owns concrete worlds such as `RigidBodyWorld`, `ClothWorld`, `FemWorld`, and `WaterWorld`, instead of introducing a fake generic solver layer.

## Existing Integration Points

Useful framework anchors:

- `src/rtr/app/app_runtime.hpp`
- `src/rtr/framework/component/component.hpp`
- `src/rtr/framework/core/game_object.hpp`
- `src/rtr/framework/integration/physics/scene_physics_sync.hpp`
- `examples/games103_lab/lab1_angry_bunny/lab1_angry_bunny.cpp`

Design implication:

- keep `PhysicsWorld` focused on rigid-body dynamics and collision
- let `PhysicsSystem` coordinate multiple concrete simulation worlds in fixed tick
- add cloth, FEM, and water incrementally as separate world implementations
- write results back to scene transforms or dynamic mesh vertices as needed

## Architecture Direction

Recommended code layout:

- `src/rtr/system/physics/cloth/`
- `src/rtr/system/physics/fem/`
- `src/rtr/system/physics/water/`
- `src/rtr/framework/component/physics/cloth/`
- `src/rtr/framework/component/physics/fem/`
- `src/rtr/framework/component/physics/water/`
- `examples/games103_lab/lab2_cloth/`
- `examples/games103_lab/lab3_bouncy_house/`
- `examples/games103_lab/lab4_pool_ripples/`

Recommended ownership split:

- rigid bodies, colliders, impulses: existing `PhysicsWorld`
- cloth particles, constraints, and solver state: `ClothWorld`
- tetrahedral mesh and elastic force state: `FemWorld`
- 2D height field and coupling masks: `WaterWorld`
- `PhysicsSystem`: fixed-step coordinator for all worlds
- `framework/component/physics/*`: optional scene-facing helpers added only when integration becomes necessary

Important design rule:

`PhysicsSystem` owns separate worlds for rigid bodies, cloth, FEM, and water. They are coordinated in the fixed-step loop, but they should not be forced behind a generic simulation abstraction unless concrete duplication later justifies shared utilities.

## Phase 1: Lab 2 Cloth

Objective:

Implement cloth first. Start by establishing a cloth-specific deformable-mesh path, then build the actual cloth solver on top of it. Begin with PBD, and optionally add the implicit solver backend later.

Recommended order:

1. cloth mesh resource and deformable vertex write-back path
2. particle and edge topology
3. pinned vertices
4. gravity and damping
5. PBD strain limiting
6. sphere collision
7. implicit cloth solver
8. Chebyshev acceleration

Why start with cloth:

- it is the first target lab
- it validates deformable mesh write-back in the smallest useful context
- PBD is faster to get running and is good for validating fixed points, constraints, and collision projection

Core features:

- cloth mesh topology generation
- cloth-to-render mesh write-back each fixed tick
- per-vertex position and velocity
- edge rest lengths
- pinned constraints
- multiple solver iterations per fixed step
- sphere collision projection and velocity correction

Suggested modules:

- `cloth_world`
- `cloth_state`
- `cloth_topology`
- `cloth_pbd_solver`
- `cloth_implicit_solver`
- `cloth_mesh_sync`

Exit criteria:

- a minimal cloth-owned deformable mesh updates vertices every fixed tick
- fixed points remain stable
- cloth does not stretch excessively
- sphere collision is robust
- simulation remains stable at the project fixed time step

## Phase 2: Lab 3 FEM Soft Body

Objective:

Extend the system from cloth surface particles to tetrahedral elastic bodies.

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

- `fem_world`
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

Do not try to force water into the same data model as cloth or FEM. Water here is a height-field simulation and should stay separate as its own world implementation.

Core features:

- load mesh vertex heights into a height field
- random droplet injection with volume preservation
- repeated shallow-wave substeps
- Neumann boundary conditions
- mesh normal recomputation

Suggested modules:

- `water_world`
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

These should be done early and reused where they are clearly needed:

- fixed-step determinism where practical
- simulation reset support
- stable parameter exposure for damping, stiffness, solver iterations, and substeps
- small deterministic examples before full scenes
- add mesh/debug visualization only when diagnosis becomes necessary

Recommended debug progression:

- cloth particle or edge overlay first if cloth debugging needs it
- tetra wireframe or tet inspector later when FEM work starts
- grid occupancy and mask views later when water coupling work starts

## Milestone Plan

### Milestone A

Cloth deformable mesh path updates vertices every fixed tick.

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

1. cloth deformable mesh path + PBD cloth
2. FEM soft body
3. shallow-wave water
4. one-way coupling
5. implicit cloth
6. two-way coupling

If the goal is to build a stronger physics system without over-generalizing:

1. cloth deformable mesh path + PBD cloth
2. FEM soft body
3. water solver
4. coupling
5. optional advanced solvers and accelerations

## Final Guidance

The main technical principle for this roadmap is:

- do not over-unify the math
- do not over-generalize the architecture too early

Cloth, FEM, and shallow-wave water are different numerical models. They may selectively share:

- fixed-step scheduling
- scene integration style
- mesh synchronization helpers
- debugging and parameter plumbing when the duplication is real

But the default design should prefer explicit world ownership, with shared utilities added only after concrete duplication appears.
