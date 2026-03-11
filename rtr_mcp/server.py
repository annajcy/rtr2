from __future__ import annotations

from typing import Any, Protocol

from mcp.server.fastmcp import FastMCP

from .bridge_client import BridgeClient


TOOL_NAMES = (
    "project_info",
    "session_create",
    "session_reset",
    "scene_inspect",
    "scene_replace",
    "scene_import_pbpt",
    "scene_export_pbpt",
    "offline_render",
)

MCP_HOST = "127.0.0.1"
MCP_PORT = 8000
MCP_PATH = "/mcp"


class BridgeProtocol(Protocol):
    def call(self, method: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        ...


def register_tools(mcp: Any, bridge: BridgeProtocol) -> None:
    @mcp.tool()
    def project_info() -> dict[str, Any]:
        """Return helper version, supported node types, and default render settings."""
        return bridge.call("project_info")

    @mcp.tool()
    def session_create(resource_root_dir: str | None = None) -> dict[str, Any]:
        """Create a new headless RTR2 session."""
        params: dict[str, Any] = {}
        if resource_root_dir:
            params["resource_root_dir"] = resource_root_dir
        return bridge.call("session_create", params)

    @mcp.tool()
    def session_reset(session_id: str, scene_name: str = "main_scene") -> dict[str, Any]:
        """Reset one session and create a new empty active scene."""
        return bridge.call(
            "session_reset",
            {
                "session_id": session_id,
                "scene_name": scene_name,
            },
        )

    @mcp.tool()
    def scene_inspect(session_id: str) -> dict[str, Any]:
        """Inspect the active scene and exportability checks."""
        return bridge.call("scene_inspect", {"session_id": session_id})

    @mcp.tool()
    def scene_replace(session_id: str, scene_spec: dict[str, Any]) -> dict[str, Any]:
        """Replace the active scene from a declarative SceneSpec."""
        return bridge.call(
            "scene_replace",
            {
                "session_id": session_id,
                "scene_spec": scene_spec,
            },
        )

    @mcp.tool()
    def scene_import_pbpt(session_id: str, scene_xml_path: str, mode: str = "replace") -> dict[str, Any]:
        """Import a PBPT/Mitsuba-style XML scene into the active session."""
        return bridge.call(
            "scene_import_pbpt",
            {
                "session_id": session_id,
                "scene_xml_path": scene_xml_path,
                "mode": mode,
            },
        )

    @mcp.tool()
    def scene_export_pbpt(
        session_id: str,
        scene_xml_path: str,
        film_width: int = 0,
        film_height: int = 0,
        spp: int = 16,
    ) -> dict[str, Any]:
        """Export the active scene to PBPT XML without starting a render."""
        return bridge.call(
            "scene_export_pbpt",
            {
                "session_id": session_id,
                "scene_xml_path": scene_xml_path,
                "film_width": film_width,
                "film_height": film_height,
                "spp": spp,
            },
        )

    @mcp.tool()
    def offline_render(
        session_id: str,
        action: str,
        scene_xml_path: str = "",
        output_exr_path: str = "",
        spp: int = 16,
        film_width: int = 0,
        film_height: int = 0,
    ) -> dict[str, Any]:
        """Start, inspect, or cancel PBPT offline rendering for the active scene."""
        return bridge.call(
            "offline_render",
            {
                "session_id": session_id,
                "action": action,
                "scene_xml_path": scene_xml_path,
                "output_exr_path": output_exr_path,
                "spp": spp,
                "film_width": film_width,
                "film_height": film_height,
            },
        )


def build_server(bridge: BridgeProtocol | None = None) -> FastMCP:
    mcp = FastMCP(
        "RTR2 Headless MCP",
        host=MCP_HOST,
        port=MCP_PORT,
        streamable_http_path=MCP_PATH,
    )
    register_tools(mcp, bridge or BridgeClient())
    return mcp


def main() -> None:
    build_server().run(transport="streamable-http")
