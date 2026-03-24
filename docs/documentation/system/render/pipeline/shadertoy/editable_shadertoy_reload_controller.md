# Editable ShaderToy Reload Controller

`src/rtr/system/render/pipeline/shadertoy/editable_shadertoy_reload_controller.hpp` contains the state machine that decides when an editable ShaderToy program should be recompiled.

It tracks:

- the requested source path
- whether auto reload is enabled
- whether an explicit reload was requested
- the last compiled path and file timestamp
- the public reload state reported to UI/runtime code

This controller does not build pipelines directly. It only resolves source paths, compares timestamps, and records success/failure state around compilation attempts.
