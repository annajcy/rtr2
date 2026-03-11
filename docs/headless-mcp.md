# Headless MCP

This document explains how to use RTR2's headless MCP service for scene authoring and PBPT offline rendering.

## Overview

RTR2 exposes a local HTTP MCP endpoint:

```text
http://127.0.0.1:8000/mcp
```

The MCP service is designed for local use only:

- The outer MCP transport is `streamable-http`
- The service listens on `127.0.0.1` only
- The C++ helper remains a private subprocess behind the Python MCP server
- No editor window or real-time renderer is started

## Build

Install Python dependencies and build the helper:

```bash
uv sync
cmake --preset conan-debug
cmake --build --preset conan-debug --target rtr_mcp_bridge
```

If the helper binary is not in the default build search paths, set:

```bash
export RTR_MCP_BRIDGE_BIN=/absolute/path/to/rtr_mcp_bridge
```

## Start The Service

Run:

```bash
uv run rtr-mcp
```

After startup, connect your MCP client to:

```text
http://127.0.0.1:8000/mcp
```

## Client Setup

### Claude Desktop

Claude Desktop uses the remote MCP connector flow rather than `claude_desktop_config.json` for HTTP servers.

Use:

1. Open `Settings`
2. Open `Connectors`
3. Add a custom remote MCP connector
4. Enter:

```text
http://127.0.0.1:8000/mcp
```

Notes:

- Claude Desktop's official remote MCP flow uses `Settings > Connectors`
- Remote MCP support depends on your Claude plan and app support
- For HTTP servers, do not expect `claude_desktop_config.json` to be the source of truth

### VS Code

VS Code supports remote MCP servers through `mcp.json`.

You can configure RTR2 in either of these locations:

- Workspace: `.vscode/mcp.json`
- User profile: run `MCP: Open User Configuration`

Example:

```json
{
  "servers": {
    "rtr2": {
      "type": "http",
      "url": "http://127.0.0.1:8000/mcp"
    }
  }
}
```

You can also use `MCP: Add Server` from the Command Palette and choose a remote HTTP server.

After saving the config:

- Start or restart the server from the `mcp.json` editor actions or `MCP: List Servers`
- Accept the trust prompt the first time VS Code starts the server

### Cursor

Cursor supports MCP servers configured by URL. Add a global config at `~/.cursor/mcp.json` or a project-local config
at `.cursor/mcp.json`:

```json
{
  "mcpServers": {
    "rtr2": {
      "url": "http://127.0.0.1:8000/mcp"
    }
  }
}
```

After saving the file, reopen Cursor or refresh MCP servers from settings.

### Cherry Studio

Cherry Studio's official documentation currently documents manual MCP setup with `STDIO`. It does not clearly document
HTTP remote MCP setup in the guide referenced for this project.

If your Cherry Studio version exposes a remote or URL-based MCP server type in:

`Settings -> MCP Server -> Add Server`

then fill the server URL with:

```text
http://127.0.0.1:8000/mcp
```

If your version only offers `STDIO`, HTTP MCP may not be available in that UI yet. In that case, use a client with
documented URL-based MCP support, such as Cursor, or keep a separate stdio wrapper for Cherry Studio.

## What It Can Do

The service currently exposes 8 tools:

- `project_info`: Return helper version, defaults, and supported node types
- `session_create`: Create a new headless RTR2 session
- `session_reset`: Reset one session and create a new empty scene
- `scene_inspect`: Inspect the active scene and exportability checks
- `scene_replace`: Replace the active scene from a declarative `SceneSpec`
- `scene_import_pbpt`: Import a Mitsuba-style PBPT XML scene
- `scene_export_pbpt`: Export the current scene to PBPT XML
- `offline_render`: Start, inspect, or cancel PBPT offline rendering

## Minimal Workflow

Typical usage looks like this:

1. Call `session_create`
2. Call `scene_replace` or `scene_import_pbpt`
3. Call `scene_inspect`
4. Call `scene_export_pbpt`
5. Call `offline_render` with `action="start"`
6. Poll `offline_render` with `action="status"`

## End-To-End MCP Example

If you want to call RTR2 directly over raw HTTP instead of using an MCP SDK, the full flow looks like this.

All requests go to:

```text
POST http://127.0.0.1:8000/mcp
```

Use these headers:

```text
content-type: application/json
accept: application/json, text/event-stream
mcp-protocol-version: 2025-11-25
```

Important:

- After `initialize`, preserve the `mcp-session-id` response header
- Send that same `mcp-session-id` header on every later request
- The examples below focus on request JSON; actual tool responses may also include `content` and `structuredContent`

### 1. Initialize

Request:

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "initialize",
  "params": {
    "protocolVersion": "2025-11-25",
    "capabilities": {},
    "clientInfo": {
      "name": "rtr2-demo-client",
      "version": "0.1.0"
    }
  }
}
```

### 2. Send initialized notification

Request:

```json
{
  "jsonrpc": "2.0",
  "method": "notifications/initialized"
}
```

### 3. List tools

Request:

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "method": "tools/list",
  "params": {}
}
```

### 4. Create a session

Request:

```json
{
  "jsonrpc": "2.0",
  "id": 3,
  "method": "tools/call",
  "params": {
    "name": "session_create",
    "arguments": {
      "resource_root_dir": "/absolute/path/to/rtr2/assets"
    }
  }
}
```

Key response field to keep:

```json
{
  "session_id": "session_1"
}
```

### 5. Replace the scene

Request:

```json
{
  "jsonrpc": "2.0",
  "id": 4,
  "method": "tools/call",
  "params": {
    "name": "scene_replace",
    "arguments": {
      "session_id": "session_1",
      "scene_spec": {
        "scene_name": "mcp_demo",
        "nodes": [
          {
            "type": "perspective_camera",
            "name": "camera",
            "active": true,
            "fov_degrees": 50.0,
            "near": 0.1,
            "far": 200.0,
            "transform": {
              "position": {"x": 0.0, "y": 1.0, "z": 6.0},
              "rotation_quat": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
              "scale": {"x": 1.0, "y": 1.0, "z": 1.0}
            }
          },
          {
            "type": "mesh_object",
            "name": "bunny",
            "mesh_path": "models/stanford_bunny.obj",
            "base_color": {"x": 0.9, "y": 0.8, "z": 0.7, "w": 1.0},
            "pbpt_mesh": true
          },
          {
            "type": "emissive_mesh_object",
            "name": "light_quad",
            "mesh_path": "models/colored_quad.obj",
            "base_color": {"x": 1.0, "y": 1.0, "z": 1.0, "w": 1.0},
            "radiance_spectrum": [
              {"lambda_nm": 400.0, "value": 4.0},
              {"lambda_nm": 500.0, "value": 4.0},
              {"lambda_nm": 600.0, "value": 4.0},
              {"lambda_nm": 700.0, "value": 4.0}
            ]
          }
        ]
      }
    }
  }
}
```

### 6. Inspect the scene

Request:

```json
{
  "jsonrpc": "2.0",
  "id": 5,
  "method": "tools/call",
  "params": {
    "name": "scene_inspect",
    "arguments": {
      "session_id": "session_1"
    }
  }
}
```

Check that the response says:

- `export_checks.can_export_pbpt == true`
- `export_checks.can_start_offline_render == true`

### 7. Export PBPT XML

Request:

```json
{
  "jsonrpc": "2.0",
  "id": 6,
  "method": "tools/call",
  "params": {
    "name": "scene_export_pbpt",
    "arguments": {
      "session_id": "session_1",
      "scene_xml_path": "/tmp/rtr_mcp/runtime_scene.xml",
      "film_width": 512,
      "film_height": 512,
      "spp": 16
    }
  }
}
```

### 8. Start offline render

Request:

```json
{
  "jsonrpc": "2.0",
  "id": 7,
  "method": "tools/call",
  "params": {
    "name": "offline_render",
    "arguments": {
      "session_id": "session_1",
      "action": "start",
      "scene_xml_path": "/tmp/rtr_mcp/runtime_scene.xml",
      "output_exr_path": "/tmp/rtr_mcp/runtime_output.exr",
      "spp": 16,
      "film_width": 512,
      "film_height": 512
    }
  }
}
```

Typical response fields:

```json
{
  "job_id": "job_1",
  "state": "Running",
  "progress_01": 0.0,
  "is_running": true
}
```

### 9. Poll render status

Request:

```json
{
  "jsonrpc": "2.0",
  "id": 8,
  "method": "tools/call",
  "params": {
    "name": "offline_render",
    "arguments": {
      "session_id": "session_1",
      "action": "status"
    }
  }
}
```

Poll until:

```json
{
  "state": "Succeeded",
  "is_running": false,
  "progress_01": 1.0
}
```

At that point the final image should exist at `/tmp/rtr_mcp/runtime_output.exr`.

## Minimal SceneSpec

`scene_replace` accepts a `scene_spec` object. Version 1 supports:

- `perspective_camera`
- `mesh_object`
- `emissive_mesh_object`

Example:

```json
{
  "scene_name": "mcp_demo",
  "nodes": [
    {
      "type": "perspective_camera",
      "name": "camera",
      "active": true,
      "fov_degrees": 50.0,
      "near": 0.1,
      "far": 200.0,
      "transform": {
        "position": {"x": 0.0, "y": 1.0, "z": 6.0},
        "rotation_quat": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
        "scale": {"x": 1.0, "y": 1.0, "z": 1.0}
      }
    },
    {
      "type": "mesh_object",
      "name": "bunny",
      "mesh_path": "models/stanford_bunny.obj",
      "base_color": {"x": 0.9, "y": 0.8, "z": 0.7, "w": 1.0},
      "pbpt_mesh": true
    },
    {
      "type": "emissive_mesh_object",
      "name": "light_quad",
      "mesh_path": "models/colored_quad.obj",
      "base_color": {"x": 1.0, "y": 1.0, "z": 1.0, "w": 1.0},
      "radiance_spectrum": [
        {"lambda_nm": 400.0, "value": 4.0},
        {"lambda_nm": 500.0, "value": 4.0},
        {"lambda_nm": 600.0, "value": 4.0},
        {"lambda_nm": 700.0, "value": 4.0}
      ]
    }
  ]
}
```

## Offline Render

Example `offline_render` start payload:

```json
{
  "session_id": "session_1",
  "action": "start",
  "scene_xml_path": "/tmp/rtr_mcp/runtime_scene.xml",
  "output_exr_path": "/tmp/rtr_mcp/runtime_output.exr",
  "spp": 16,
  "film_width": 512,
  "film_height": 512
}
```

Then poll:

```json
{
  "session_id": "session_1",
  "action": "status"
}
```

The status result includes:

- `state`
- `progress_01`
- `message`
- `is_running`

## Logs And Output

- The HTTP MCP server can print normal logs to your terminal
- The helper keeps its protocol output private so render progress does not corrupt responses
- Helper logs are written to `./output/logs/rtr_mcp_bridge.log`

## Troubleshooting

If the server does not start:

- Check whether `127.0.0.1:8000` is already in use
- Confirm `rtr_mcp_bridge` was built successfully
- Set `RTR_MCP_BRIDGE_BIN` if the helper is outside the default search paths

If a render fails:

- Call `scene_inspect` and check `export_checks`
- Make sure the scene has exactly one active perspective camera
- Make sure the scene has at least one exportable PBPT mesh
- Make sure the scene has at least one PBPT area emitter before starting offline rendering
