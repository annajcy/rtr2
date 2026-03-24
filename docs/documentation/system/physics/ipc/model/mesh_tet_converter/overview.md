# Mesh Tet Converter Overview

`src/rtr/system/physics/ipc/model/mesh_tet_converter/` contains the bridge between surface-mesh data and tet-volume data.

Current responsibilities:

- `mesh_to_tet.hpp`: tetrahedralize or convert render-facing mesh data into tet-body input
- `tet_to_mesh.hpp`: extract or update a renderable surface mesh from tet state

This directory is the geometry-conversion boundary between IPC simulation data and renderable scene data.
