# Headless MCP

本文档介绍如何使用 RTR2 的 headless MCP 服务完成场景搭建和 PBPT 离线渲染。

## 概览

RTR2 对外暴露一个本地 HTTP MCP 端点：

```text
http://127.0.0.1:8000/mcp
```

当前服务只面向本机使用：

- 外层 MCP transport 使用 `streamable-http`
- 服务只监听 `127.0.0.1`
- C++ helper 仍然作为 Python MCP server 后面的私有子进程存在
- 不会启动 editor 窗口，也不会启动实时渲染器

## 构建

先安装 Python 依赖并构建 helper：

```bash
uv sync
cmake --preset conan-debug
cmake --build --preset conan-debug --target rtr_mcp_bridge
```

如果 helper 不在默认搜索路径中，可以手动指定：

```bash
export RTR_MCP_BRIDGE_BIN=/absolute/path/to/rtr_mcp_bridge
```

## 启动服务

运行：

```bash
uv run rtr-mcp
```

启动后，将你的 MCP 客户端连接到：

```text
http://127.0.0.1:8000/mcp
```

## 客户端配置

### Claude Desktop

Claude Desktop 对 HTTP MCP 使用的是 remote connector 流程，而不是 `claude_desktop_config.json`。

使用方式：

1. 打开 `Settings`
2. 进入 `Connectors`
3. 添加一个自定义远程 MCP connector
4. 填入：

```text
http://127.0.0.1:8000/mcp
```

注意：

- Claude Desktop 官方远程 MCP 配置入口是 `Settings > Connectors`
- Remote MCP 是否可用取决于你的 Claude 计划和客户端支持
- 对于 HTTP 服务器，不要再把 `claude_desktop_config.json` 当成配置入口

### VS Code

VS Code 现在通过 `mcp.json` 配置远程 MCP 服务。

你可以把 RTR2 配置在这两个位置之一：

- 工作区：`.vscode/mcp.json`
- 用户级：执行 `MCP: Open User Configuration`

示例：

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

你也可以在命令面板里执行 `MCP: Add Server`，然后选择远程 HTTP server。

保存配置后：

- 在 `mcp.json` 编辑器的内联操作，或者 `MCP: List Servers` 里启动或重启该 server
- 第一次启动时接受 VS Code 的信任提示

### Cursor

Cursor 支持通过 URL 配置 MCP 服务。你可以在全局 `~/.cursor/mcp.json` 或项目内 `.cursor/mcp.json` 中写入：

```json
{
  "mcpServers": {
    "rtr2": {
      "url": "http://127.0.0.1:8000/mcp"
    }
  }
}
```

保存后，重新打开 Cursor，或者在设置里刷新 MCP Servers。

### Cherry Studio

Cherry Studio 官方文档目前明确写到的仍是 `STDIO` 手动配置；我没有在当前官方说明里看到清晰的 HTTP remote MCP 配置写法。

如果你的 Cherry Studio 版本在：

`Settings -> MCP Server -> Add Server`

里已经提供了 URL / Remote / HTTP 类型，那么把服务地址填写为：

```text
http://127.0.0.1:8000/mcp
```

如果你的版本目前只有 `STDIO`，那说明这个界面里可能还不能直接接 HTTP MCP。此时更稳妥的做法是先使用
Cursor 这类官方明确支持 URL 配置的客户端，或者额外保留一个给 Cherry Studio 用的 stdio 包装层。

## 可用能力

当前服务提供 8 个工具：

- `project_info`：返回 helper 版本、默认配置和支持的节点类型
- `session_create`：创建一个新的 headless RTR2 session
- `session_reset`：重置指定 session 并创建一个空场景
- `scene_inspect`：检查当前 active scene 和导出前置条件
- `scene_replace`：用声明式 `SceneSpec` 全量替换当前场景
- `scene_import_pbpt`：导入 Mitsuba 风格的 PBPT XML 场景
- `scene_export_pbpt`：将当前场景导出为 PBPT XML
- `offline_render`：启动、查询或取消 PBPT 离线渲染

## 最小工作流

典型使用流程如下：

1. 调用 `session_create`
2. 调用 `scene_replace` 或 `scene_import_pbpt`
3. 调用 `scene_inspect`
4. 调用 `scene_export_pbpt`
5. 调用 `offline_render`，其中 `action="start"`
6. 轮询 `offline_render`，其中 `action="status"`

## 从 0 到出图的完整 MCP 示例

如果你不用 MCP SDK，而是想直接用原始 HTTP 请求调用 RTR2，那么完整流程如下。

所有请求都发到：

```text
POST http://127.0.0.1:8000/mcp
```

请求头建议带上：

```text
content-type: application/json
accept: application/json, text/event-stream
mcp-protocol-version: 2025-11-25
```

注意：

- `initialize` 之后，要保存服务端返回的 `mcp-session-id` 响应头
- 后续每一个请求都要继续带上同一个 `mcp-session-id`
- 下面主要展示“请求 JSON”；实际返回里通常还会有 `content` 和 `structuredContent`

### 1. 初始化

请求：

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

### 2. 发送 initialized 通知

请求：

```json
{
  "jsonrpc": "2.0",
  "method": "notifications/initialized"
}
```

### 3. 列出工具

请求：

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "method": "tools/list",
  "params": {}
}
```

### 4. 创建 session

请求：

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

这个响应里你最关心的是：

```json
{
  "session_id": "session_1"
}
```

### 5. 全量替换场景

请求：

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

### 6. 检查场景是否合法

请求：

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

重点检查返回里的：

- `export_checks.can_export_pbpt == true`
- `export_checks.can_start_offline_render == true`

### 7. 导出 PBPT XML

请求：

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

### 8. 启动离线渲染

请求：

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

典型返回里会有这些字段：

```json
{
  "job_id": "job_1",
  "state": "Running",
  "progress_01": 0.0,
  "is_running": true
}
```

### 9. 轮询渲染状态

请求：

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

持续轮询，直到返回类似：

```json
{
  "state": "Succeeded",
  "is_running": false,
  "progress_01": 1.0
}
```

这时最终图片就应该已经生成在 `/tmp/rtr_mcp/runtime_output.exr`。

## 最小 SceneSpec

`scene_replace` 接受一个 `scene_spec` 对象。v1 目前支持：

- `perspective_camera`
- `mesh_object`
- `emissive_mesh_object`

示例：

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

## 离线渲染

`offline_render` 的启动请求示例：

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

然后使用下面的请求轮询状态：

```json
{
  "session_id": "session_1",
  "action": "status"
}
```

状态结果中会包含：

- `state`
- `progress_01`
- `message`
- `is_running`

## 日志与输出

- 顶层 HTTP MCP server 可以正常向终端输出日志
- helper 会把自己的协议输出与普通输出隔离，避免渲染进度污染响应
- helper 日志写入 `./output/logs/rtr_mcp_bridge.log`

## 常见问题

如果服务无法启动：

- 检查 `127.0.0.1:8000` 是否已被占用
- 确认 `rtr_mcp_bridge` 已成功构建
- 如果 helper 不在默认路径中，设置 `RTR_MCP_BRIDGE_BIN`

如果渲染失败：

- 先调用 `scene_inspect`，查看 `export_checks`
- 确保场景中恰好有一个 active perspective camera
- 确保场景里至少有一个可导出的 PBPT mesh
- 启动离线渲染前，确保场景里至少有一个 PBPT area emitter
