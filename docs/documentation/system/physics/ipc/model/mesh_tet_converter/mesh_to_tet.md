# Mesh To Tet

`src/rtr/system/physics/ipc/model/mesh_tet_converter/mesh_to_tet.hpp` is the mesh-to-tetrahedral-body conversion entry used by the IPC model layer.

It turns surface-oriented mesh input into tet simulation input, typically for building `TetGeometry` and `TetBody` data that can enter the IPC pipeline.

For the fuller algorithm and design notes, also see the legacy overview page at [`../mesh_to_tet.md`](/Users/jinceyang/Desktop/codebase/graphics/rtr2/docs/documentation/system/physics/ipc/model/mesh_to_tet.md).
