# IPC Energy Overview

The `energy/` directory is reserved for the future scalar objective terms used by the deformable solver.

Expected responsibilities include:

- inertial energy
- elastic energy per tet
- gravity
- contact barrier terms
- future gradient and Hessian helpers that are tightly coupled to one energy family

The directory exists now so the IPC documentation tree matches the source tree, but it does not yet contain implementation headers.
