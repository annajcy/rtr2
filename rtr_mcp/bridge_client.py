from __future__ import annotations

import atexit
import json
import os
import queue
import shutil
import subprocess
import threading
from collections import deque
from pathlib import Path
from typing import Any, Sequence


class BridgeClientError(RuntimeError):
    pass


class BridgeResponseError(BridgeClientError):
    def __init__(self, error_code: str, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code


class BridgeClient:
    def __init__(
        self,
        helper_path: str | Path | None = None,
        *,
        helper_command: Sequence[str] | None = None,
        timeout_seconds: float = 30.0,
        process_factory: Any = subprocess.Popen,
    ) -> None:
        self._helper_path = Path(helper_path).expanduser() if helper_path is not None else None
        self._helper_command = list(helper_command) if helper_command is not None else None
        self._timeout_seconds = timeout_seconds
        self._process_factory = process_factory
        self._process: subprocess.Popen[str] | None = None
        self._response_queue: queue.Queue[dict[str, Any]] | None = None
        self._stderr_tail: deque[str] = deque(maxlen=50)
        self._stdout_thread: threading.Thread | None = None
        self._stderr_thread: threading.Thread | None = None
        self._request_lock = threading.Lock()
        self._next_request_id = 1
        atexit.register(self.close)

    def close(self) -> None:
        process = self._process
        self._process = None
        self._response_queue = None
        if process is None:
            return
        if process.stdin is not None and not process.stdin.closed:
            process.stdin.close()
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        if process.stdout is not None and not process.stdout.closed:
            process.stdout.close()
        if process.stderr is not None and not process.stderr.closed:
            process.stderr.close()

    def call(self, method: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        response = self.invoke(method, params)
        if not response.get("ok", False):
            raise BridgeResponseError(response.get("error_code", "bridge_error"), response.get("message", "Bridge error"))
        data = response.get("data")
        if not isinstance(data, dict):
            raise BridgeClientError("Bridge returned non-object data payload.")
        return data

    def invoke(self, method: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        with self._request_lock:
            self._ensure_started()
            assert self._process is not None
            assert self._response_queue is not None
            assert self._process.stdin is not None

            request_id = self._next_request_id
            self._next_request_id += 1

            payload = {"id": request_id, "method": method, "params": params or {}}
            try:
                self._process.stdin.write(json.dumps(payload) + "\n")
                self._process.stdin.flush()
            except BrokenPipeError as exc:
                raise BridgeClientError(self._bridge_failure_message("Bridge stdin closed unexpectedly.")) from exc

            response = self._wait_for_response()

            if response.get("id") != request_id:
                raise BridgeClientError(self._bridge_failure_message("Bridge response id mismatch."))
            return response

    def _ensure_started(self) -> None:
        if self._process is not None and self._process.poll() is None:
            return

        command = self._resolve_command()
        process = self._process_factory(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self._process = process
        self._response_queue = queue.Queue()
        self._stderr_tail.clear()

        assert process.stdout is not None
        assert process.stderr is not None

        self._stdout_thread = threading.Thread(target=self._stdout_reader, args=(process.stdout,), daemon=True)
        self._stderr_thread = threading.Thread(target=self._stderr_reader, args=(process.stderr,), daemon=True)
        self._stdout_thread.start()
        self._stderr_thread.start()

    def _resolve_command(self) -> list[str]:
        if self._helper_command is not None:
            return list(self._helper_command)

        env_override = os.environ.get("RTR_MCP_BRIDGE_BIN")
        if env_override:
            return [env_override]

        if self._helper_path is not None:
            return [str(self._helper_path)]

        helper_name = "rtr_mcp_bridge.exe" if os.name == "nt" else "rtr_mcp_bridge"
        found = shutil.which(helper_name)
        if found:
            return [found]

        project_root = Path(__file__).resolve().parents[1]
        build_root = project_root / "build"
        direct_candidates = [
            build_root / "Debug" / "src" / helper_name,
            build_root / "Release" / "src" / helper_name,
            build_root / "src" / helper_name,
        ]
        for candidate in direct_candidates:
            if candidate.is_file():
                return [str(candidate)]

        for candidate in build_root.rglob(helper_name):
            if candidate.is_file():
                return [str(candidate)]

        raise BridgeClientError(
            "Could not find 'rtr_mcp_bridge'. Build it first with "
            "'cmake --build --preset conan-debug --target rtr_mcp_bridge' "
            "or set RTR_MCP_BRIDGE_BIN."
        )

    def _stdout_reader(self, pipe: Any) -> None:
        assert self._response_queue is not None
        for raw_line in pipe:
            line = raw_line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError as exc:
                self._response_queue.put(
                    {
                        "id": None,
                        "ok": False,
                        "error_code": "bridge_protocol_error",
                        "message": self._bridge_failure_message(f"Invalid JSON from bridge: {exc}"),
                        "data": {},
                    }
                )
                return
            self._response_queue.put(payload)

    def _wait_for_response(self) -> dict[str, Any]:
        assert self._response_queue is not None
        assert self._process is not None

        remaining = self._timeout_seconds
        step = min(0.1, self._timeout_seconds)

        while remaining > 0:
            try:
                return self._response_queue.get(timeout=step)
            except queue.Empty:
                if self._process.poll() is not None:
                    raise BridgeClientError(self._bridge_failure_message("Bridge exited before sending a response."))
                remaining -= step

        raise BridgeClientError(self._bridge_failure_message("Bridge response timed out."))

    def _stderr_reader(self, pipe: Any) -> None:
        for raw_line in pipe:
            line = raw_line.rstrip()
            if line:
                self._stderr_tail.append(line)

    def _bridge_failure_message(self, prefix: str) -> str:
        if self._process is not None and self._process.poll() is not None:
            prefix = f"{prefix} Bridge exited with code {self._process.returncode}."
        if self._stderr_tail:
            prefix = f"{prefix} stderr tail: {' | '.join(self._stderr_tail)}"
        return prefix
