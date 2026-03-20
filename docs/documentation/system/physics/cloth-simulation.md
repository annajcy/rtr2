# Cloth Simulation

The authoritative version of this page is currently the Chinese document `cloth-simulation.zh.md`.

This English page exists to keep the bilingual MkDocs structure aligned.

## Current Cloth Runtime

RTR2 currently implements an explicit mass-spring cloth solver with:

- edge springs built from unique mesh edges,
- bend springs built from interior-edge opposite vertices,
- semi-implicit Euler integration,
- substeps,
- spring damping and global velocity damping,
- pinned vertices,
- deformable mesh vertex and normal write-back.

Not implemented:

- cloth collision,
- self-collision,
- cloth-rigid coupling,
- implicit cloth,
- PBD / XPBD.
