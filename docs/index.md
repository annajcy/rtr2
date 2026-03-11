# RTR2

RTR2 is a modern real-time 3D rendering engine written in C++23, using Vulkan and Slang shaders.

## Highlights

- Real-time rendering framework with a Vulkan-based RHI
- Editor integration with ImGui
- Offline path tracing integration via PBPT
- Test suite with unit and GPU integration tests

## Repository layout

- `src/rtr/`: engine and runtime source
- `shaders/`: Slang shader programs
- `examples/`: runnable sample applications
- `test/`: GoogleTest-based test suite

Continue to [Getting Started](getting-started.md).

See [Headless MCP](headless-mcp.md) for the local HTTP MCP workflow.

See [Camera and Coordinate Conventions](documentation/system/render/camera-coordinate-conventions.md) for the canonical matrix and camera-space rules shared by PBPT and RTR2.

See [API Documentation](api/index.md) for generated Doxygen references.
