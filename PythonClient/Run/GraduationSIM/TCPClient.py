from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any, Dict, List, Optional


JsonDict = Dict[str, Any]


class TCPClient:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 9000,
        timeout: float = 10.0,
        auto_connect: bool = True,
        auto_reconnect: bool = True,
        reconnect_delay_s: float = 0.2,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)
        self.auto_reconnect = bool(auto_reconnect)
        self.reconnect_delay_s = float(reconnect_delay_s)
        self._socket: Optional[socket.socket] = None
        self._recv_buffer = b""
        self._lock = threading.RLock()
        if auto_connect:
            self.connect()

    @property
    def connected(self) -> bool:
        return self._socket is not None

    def is_connected(self) -> bool:
        return self.connected

    def _dispose_socket(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            except Exception:
                pass
        self._socket = None
        self._recv_buffer = b""

    def connect(self) -> bool:
        with self._lock:
            self._dispose_socket()
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(self.timeout)
                sock.connect((self.host, self.port))
            except Exception as exc:
                print(f"[ERROR] failed to connect to {self.host}:{self.port}: {exc}")
                self._dispose_socket()
                return False
            self._socket = sock
            self._recv_buffer = b""
            print(f"[INFO] connected to simulator at {self.host}:{self.port}")
            return True

    def close(self) -> None:
        with self._lock:
            self._dispose_socket()

    disconnect = close

    def _extract_json_from_buffer(self) -> Optional[Any]:
        if not self._recv_buffer:
            return None

        try:
            text = self._recv_buffer.decode("utf-8")
        except UnicodeDecodeError:
            return None

        stripped = text.lstrip()
        if not stripped:
            self._recv_buffer = b""
            return None

        leading_chars = len(text) - len(stripped)
        decoder = json.JSONDecoder()
        try:
            obj, index = decoder.raw_decode(stripped)
        except json.JSONDecodeError:
            return None

        consumed_chars = leading_chars + index
        consumed_bytes = len(text[:consumed_chars].encode("utf-8"))
        self._recv_buffer = self._recv_buffer[consumed_bytes:].lstrip()
        return obj

    def _recv_json_locked(self) -> JsonDict:
        if self._socket is None:
            return {"status": "error", "message": "Not connected"}

        while True:
            parsed = self._extract_json_from_buffer()
            if isinstance(parsed, dict):
                return parsed
            if parsed is not None:
                return {
                    "status": "error",
                    "message": "Response is not a JSON object",
                    "raw": parsed,
                }

            chunk = self._socket.recv(4096)
            if not chunk:
                break
            self._recv_buffer += chunk

        raw = ""
        if self._recv_buffer:
            raw = self._recv_buffer.decode("utf-8", errors="replace").strip()
            self._recv_buffer = b""
        if raw:
            return {"status": "error", "message": "Incomplete JSON response", "raw": raw}
        return {"status": "error", "message": "Empty response"}

    def request(self, payload: JsonDict) -> JsonDict:
        message = json.dumps(payload, ensure_ascii=False) + "\n"
        max_attempts = 2 if self.auto_reconnect else 1
        last_error = "Communication failed"

        for attempt in range(max_attempts):
            with self._lock:
                if not self.connected and not self.connect():
                    last_error = f"Unable to connect to {self.host}:{self.port}"
                    continue
                try:
                    assert self._socket is not None
                    self._socket.sendall(message.encode("utf-8"))
                    return self._recv_json_locked()
                except Exception as exc:
                    last_error = str(exc)
                    self._dispose_socket()

            if attempt + 1 < max_attempts:
                print(f"[WARN] communication failed, retrying: {last_error}")
                time.sleep(self.reconnect_delay_s)

        return {"status": "error", "message": last_error}

    def send_message(self, payload: JsonDict) -> JsonDict:
        return self.request(payload)

    def ping(self) -> JsonDict:
        return self.request({"ping": {}})

    def sim_pause(self) -> JsonDict:
        return self.request({"sim_pause": {}})

    def sim_resume(self) -> JsonDict:
        return self.request({"sim_resume": {}})

    def sim_step(self, steps: int = 1, dt: float = 0.01) -> JsonDict:
        return self.request({"sim_step": {"steps": int(max(1, steps)), "dt": float(dt)}})

    def sim_reset(self) -> JsonDict:
        return self.request({"sim_reset": {}})

    def get_agent_list(self) -> JsonDict:
        return self.request({"get_agent_list": {}})

    def get_agents(self) -> List[str]:
        response = self.get_agent_list()
        agents = response.get("agents", []) if isinstance(response, dict) else []
        return [str(actor_id) for actor_id in agents if isinstance(actor_id, str) and actor_id]

    def get_agents_detail(self):
        response = self.get_agent_list()
        detail = response.get("agents_detail", []) if isinstance(response, dict) else []
        return detail if isinstance(detail, list) else []
