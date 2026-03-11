from __future__ import annotations

import anyio
import os
from pathlib import Path
import subprocess
import time

from mcp.client.session import ClientSession
from mcp.client.streamable_http import streamable_http_client


MCP_URL = "http://127.0.0.1:8000/mcp"
FILM_WIDTH = 64
FILM_HEIGHT = 64
SPP = 32


async def _probe_server_once() -> None:
    async with streamable_http_client(MCP_URL) as (read_stream, write_stream, _):
        async with ClientSession(read_stream, write_stream) as session:
            await session.initialize()


async def _start_server_if_needed(repo_root: Path) -> subprocess.Popen[str] | None:
    try:
        await _probe_server_once()
        return None
    except Exception:  # noqa: BLE001
        pass

    env = os.environ.copy()
    env.setdefault("PYTHONUNBUFFERED", "1")
    process = subprocess.Popen(
        ["uv", "run", "rtr-mcp"],
        cwd=repo_root,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=env,
    )
    deadline = time.time() + 10.0
    while time.time() < deadline:
        if process.poll() is not None:
            stderr = process.stderr.read() if process.stderr is not None else ""
            raise RuntimeError(f"Failed to start local MCP server: {stderr}")
        try:
            await _probe_server_once()
            return process
        except Exception:  # noqa: BLE001
            await anyio.sleep(0.1)
    process.terminate()
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5.0)
    raise RuntimeError("Timed out while waiting for the local MCP server to become ready.")


def _stop_server(process: subprocess.Popen[str] | None) -> None:
    if process is None:
        return
    process.terminate()
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5.0)


async def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    assets_dir = repo_root / "assets"
    output_dir = repo_root / "output"
    output_dir.mkdir(parents=True, exist_ok=True)

    cbox_scene_xml = assets_dir / "pbpt_scene" / "cbox" / "cbox.xml"
    runtime_scene_xml = output_dir / "http_smoke_scene.xml"
    runtime_output_exr = output_dir / "http_smoke_render.exr"

    process = await _start_server_if_needed(repo_root)
    try:
        async with streamable_http_client(MCP_URL) as (read_stream, write_stream, _):
            async with ClientSession(read_stream, write_stream) as session:
                await session.initialize()

                session_created = await session.call_tool(
                    "session_create",
                    {"resource_root_dir": str(assets_dir)},
                )
                session_id = session_created.structuredContent["session_id"]

                await session.call_tool(
                    "scene_import_pbpt",
                    {
                        "session_id": session_id,
                        "scene_xml_path": str(cbox_scene_xml),
                        "mode": "replace",
                    },
                )

                inspect_result = await session.call_tool("scene_inspect", {"session_id": session_id})
                export_checks = inspect_result.structuredContent["export_checks"]
                if not export_checks["can_start_offline_render"]:
                    raise RuntimeError(f"Scene is not renderable: {export_checks}")

                await session.call_tool(
                    "scene_export_pbpt",
                    {
                        "session_id": session_id,
                        "scene_xml_path": str(runtime_scene_xml),
                        "film_width": FILM_WIDTH,
                        "film_height": FILM_HEIGHT,
                        "spp": SPP,
                    },
                )

                started = await session.call_tool(
                    "offline_render",
                    {
                        "session_id": session_id,
                        "action": "start",
                        "scene_xml_path": str(runtime_scene_xml),
                        "output_exr_path": str(runtime_output_exr),
                        "film_width": FILM_WIDTH,
                        "film_height": FILM_HEIGHT,
                        "spp": SPP,
                    },
                )
                print(f"Started render job: {started.structuredContent.get('job_id', '<none>')}")

                deadline = time.time() + 20.0
                while True:
                    status = await session.call_tool(
                        "offline_render",
                        {
                            "session_id": session_id,
                            "action": "status",
                        },
                    )
                    state = status.structuredContent["state"]
                    progress = status.structuredContent["progress_01"]
                    print(f"Render state={state} progress={progress:.3f}")
                    if state == "Succeeded":
                        break
                    if state in {"Failed", "Canceled"}:
                        raise RuntimeError(f"Offline render ended in state {state}: {status.structuredContent}")
                    if time.time() >= deadline:
                        raise RuntimeError(f"Timed out while waiting for offline render: {status.structuredContent}")
                    await anyio.sleep(0.2)
    finally:
        _stop_server(process)

    print(f"Scene XML: {runtime_scene_xml}")
    print(f"Output EXR: {runtime_output_exr}")


if __name__ == "__main__":
    anyio.run(main)
