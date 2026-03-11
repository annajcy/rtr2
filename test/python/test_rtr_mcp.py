from __future__ import annotations

import anyio
import os
import stat
import subprocess
import sys
import tempfile
import textwrap
import time
import unittest
from pathlib import Path
from typing import Any
from unittest import mock

from mcp.client.session import ClientSession
from mcp.client.streamable_http import streamable_http_client
from rtr_mcp.bridge_client import BridgeClient, BridgeClientError, BridgeResponseError
from rtr_mcp.server import MCP_HOST, MCP_PATH, MCP_PORT, TOOL_NAMES, build_server, main, register_tools


class FakeMCP:
    def __init__(self) -> None:
        self.tools: dict[str, Any] = {}

    def tool(self, *args: Any, **kwargs: Any) -> Any:
        del args, kwargs

        def decorator(func: Any) -> Any:
            self.tools[func.__name__] = func
            return func

        return decorator


class FakeBridge:
    def __init__(self) -> None:
        self.calls: list[tuple[str, dict[str, Any] | None]] = []

    def call(self, method: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        self.calls.append((method, params))
        return {"method": method, "params": params or {}}


class ServerRegistrationTest(unittest.TestCase):
    def test_register_tools_exposes_expected_names(self) -> None:
        fake_mcp = FakeMCP()
        fake_bridge = FakeBridge()

        register_tools(fake_mcp, fake_bridge)

        self.assertEqual(tuple(fake_mcp.tools.keys()), TOOL_NAMES)
        result = fake_mcp.tools["scene_export_pbpt"](
            session_id="session_1",
            scene_xml_path="/tmp/out.xml",
            film_width=64,
            film_height=64,
            spp=2,
        )
        self.assertEqual(result["method"], "scene_export_pbpt")
        self.assertEqual(
            fake_bridge.calls[-1],
            (
                "scene_export_pbpt",
                {
                    "session_id": "session_1",
                    "scene_xml_path": "/tmp/out.xml",
                    "film_width": 64,
                    "film_height": 64,
                    "spp": 2,
                },
            ),
        )

    def test_build_server_pins_streamable_http_settings(self) -> None:
        server = build_server(FakeBridge())
        self.assertEqual(server.settings.host, MCP_HOST)
        self.assertEqual(server.settings.port, MCP_PORT)
        self.assertEqual(server.settings.streamable_http_path, MCP_PATH)

    def test_main_runs_streamable_http_transport(self) -> None:
        fake_server = mock.Mock()
        with mock.patch("rtr_mcp.server.build_server", return_value=fake_server):
            main()
        fake_server.run.assert_called_once_with(transport="streamable-http")


class BridgeClientTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.script_path = Path(self.temp_dir.name) / "mock_bridge.py"
        self.script_path.write_text(
            textwrap.dedent(
                """\
                import json
                import sys
                import time

                mode = sys.argv[1]
                for raw_line in sys.stdin:
                    request = json.loads(raw_line)
                    if mode == "ok":
                        response = {
                            "id": request["id"],
                            "ok": True,
                            "error_code": "",
                            "message": "ok",
                            "data": {"echo": request["method"], "params": request["params"]},
                        }
                        print(json.dumps(response), flush=True)
                    elif mode == "error":
                        response = {
                            "id": request["id"],
                            "ok": False,
                            "error_code": "render_failed",
                            "message": "backend failed",
                            "data": {},
                        }
                        print(json.dumps(response), flush=True)
                    elif mode == "timeout":
                        time.sleep(2.0)
                    elif mode == "crash":
                        raise SystemExit(7)
                """
            ),
            encoding="utf-8",
        )
        os.chmod(self.script_path, os.stat(self.script_path).st_mode | stat.S_IEXEC)

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def make_client(self, mode: str, timeout: float = 0.3) -> BridgeClient:
        return BridgeClient(
            helper_command=[sys.executable, str(self.script_path), mode],
            timeout_seconds=timeout,
        )

    def test_bridge_client_returns_data_payload(self) -> None:
        client = self.make_client("ok")
        try:
            data = client.call("scene_inspect", {"session_id": "session_1"})
        finally:
            client.close()
        self.assertEqual(data["echo"], "scene_inspect")
        self.assertEqual(data["params"]["session_id"], "session_1")

    def test_bridge_client_raises_structured_error(self) -> None:
        client = self.make_client("error")
        try:
            with self.assertRaises(BridgeResponseError) as ctx:
                client.call("offline_render", {"action": "start"})
        finally:
            client.close()
        self.assertEqual(ctx.exception.error_code, "render_failed")
        self.assertIn("backend failed", str(ctx.exception))

    def test_bridge_client_raises_timeout_error(self) -> None:
        client = self.make_client("timeout", timeout=0.1)
        try:
            with self.assertRaises(BridgeClientError) as ctx:
                client.call("project_info")
        finally:
            client.close()
        self.assertIn("timed out", str(ctx.exception))

    def test_bridge_client_reports_process_exit(self) -> None:
        client = self.make_client("crash")
        try:
            with self.assertRaises(BridgeClientError) as ctx:
                client.call("project_info")
        finally:
            client.close()
        self.assertIn("Bridge exited", str(ctx.exception))


class HttpServerSmokeTest(unittest.TestCase):
    def test_http_server_serves_tools_over_streamable_http(self) -> None:
        process = self._start_server_if_needed()
        try:
            self._wait_until_ready(process)
            result = anyio.run(self._call_project_info_and_session_create)
            self.assertIn("project_info", result["tool_names"])
            self.assertEqual(result["session_prefix"], "session_")
        finally:
            self._stop_server(process)

    def test_http_server_renders_imported_cbox_scene(self) -> None:
        process = self._start_server_if_needed()
        try:
            self._wait_until_ready(process)
            result = anyio.run(self._render_imported_cbox_scene)
            self.assertEqual(result["state"], "Succeeded")
            self.assertTrue(result["scene_xml_exists"])
            self.assertTrue(result["output_exr_exists"])
            self.assertGreater(result["shape_count"], 0)
            self.assertTrue(result["can_start_offline_render"])
        finally:
            self._stop_server(process)

    def _start_server_if_needed(self) -> subprocess.Popen[str] | None:
        try:
            anyio.run(self._probe_server_once)
            return None
        except Exception:  # noqa: BLE001
            pass

        command = ["uv", "run", "rtr-mcp"]
        env = os.environ.copy()
        env.setdefault("PYTHONUNBUFFERED", "1")
        return subprocess.Popen(
            command,
            cwd=Path(__file__).resolve().parents[2],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=env,
        )

    def _stop_server(self, process: subprocess.Popen[str] | None) -> None:
        if process is None:
            return
        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5.0)

    def _wait_until_ready(self, process: subprocess.Popen[str] | None) -> None:
        deadline = time.time() + 10.0
        last_error = ""
        while time.time() < deadline:
            if process is not None and process.poll() is not None:
                stderr = process.stderr.read() if process.stderr is not None else ""
                raise AssertionError(f"HTTP MCP server exited early: {stderr}")
            try:
                anyio.run(self._probe_server_once)
                return
            except Exception as exc:  # noqa: BLE001
                last_error = str(exc)
                time.sleep(0.1)
        raise AssertionError(f"HTTP MCP server did not become ready: {last_error}")

    async def _probe_server_once(self) -> None:
        async with streamable_http_client(f"http://{MCP_HOST}:{MCP_PORT}{MCP_PATH}") as (read_stream, write_stream, _):
            async with ClientSession(read_stream, write_stream) as session:
                await session.initialize()

    async def _call_project_info_and_session_create(self) -> dict[str, Any]:
        async with streamable_http_client(f"http://{MCP_HOST}:{MCP_PORT}{MCP_PATH}") as (read_stream, write_stream, _):
            async with ClientSession(read_stream, write_stream) as session:
                await session.initialize()
                tools = await session.list_tools()
                session_created = await session.call_tool("session_create", {"resource_root_dir": str(Path("assets").resolve())})
                return {
                    "tool_names": [tool.name for tool in tools.tools],
                    "session_prefix": session_created.structuredContent["session_id"][:8],
                }

    async def _render_imported_cbox_scene(self) -> dict[str, Any]:
        repo_root = Path(__file__).resolve().parents[2]
        assets_dir = repo_root / "assets"
        cbox_scene_xml = assets_dir / "pbpt_scene" / "cbox" / "cbox.xml"

        with tempfile.TemporaryDirectory() as temp_dir_str:
            temp_dir = Path(temp_dir_str)
            scene_xml_path = temp_dir / "runtime_scene.xml"
            output_exr_path = temp_dir / "runtime_output.exr"

            async with streamable_http_client(f"http://{MCP_HOST}:{MCP_PORT}{MCP_PATH}") as (read_stream, write_stream, _):
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

                    inspected = await session.call_tool("scene_inspect", {"session_id": session_id})
                    export_checks = inspected.structuredContent["export_checks"]

                    exported = await session.call_tool(
                        "scene_export_pbpt",
                        {
                            "session_id": session_id,
                            "scene_xml_path": str(scene_xml_path),
                            "film_width": 64,
                            "film_height": 64,
                            "spp": 1,
                        },
                    )

                    started = await session.call_tool(
                        "offline_render",
                        {
                            "session_id": session_id,
                            "action": "start",
                            "scene_xml_path": str(scene_xml_path),
                            "output_exr_path": str(output_exr_path),
                            "film_width": 64,
                            "film_height": 64,
                            "spp": 1,
                        },
                    )
                    status = started.structuredContent

                    deadline = time.time() + 20.0
                    while status["state"] == "Running" and time.time() < deadline:
                        await anyio.sleep(0.2)
                        polled = await session.call_tool(
                            "offline_render",
                            {
                                "session_id": session_id,
                                "action": "status",
                            },
                        )
                        status = polled.structuredContent

                    if status["state"] == "Running":
                        self.fail(f"Offline render did not finish before timeout: {status}")

                    return {
                        "state": status["state"],
                        "scene_xml_exists": scene_xml_path.exists(),
                        "output_exr_exists": output_exr_path.exists(),
                        "shape_count": exported.structuredContent["shape_count"],
                        "can_start_offline_render": export_checks["can_start_offline_render"],
                    }


if __name__ == "__main__":
    unittest.main()
