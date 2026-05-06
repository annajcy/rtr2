# IPC Solver Overview

The `solver/` directory contains the current nonlinear solve utilities for IPC state updates.

## Current Files

- `line_search.hpp`
  Armijo backtracking line search for energy descent along a proposed search direction.
- `newton_solver.hpp`
  A minimal Newton solver that works on global energy / gradient / Hessian callbacks, supports per-DOF Dirichlet masks, and uses sparse reduced solves.

## Scope

The current solver layer is responsible for:

- Newton steps in global DOF space
- line search
- convergence checks
- reduced solves under per-DOF fixed masks

It does not yet include:

- PSD projection
- CCD step clipping
- contact-aware filtering
- friction solve orchestration
