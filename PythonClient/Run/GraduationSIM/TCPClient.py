from __future__ import annotations

import json
import socket
import time
from typing import Any, Dict, List, Optional

JsonDict = Dict[str, Any]

class TCPClient:
    def __init__(
        self,
        host="127.0.0.1",
        port=9000,
        timeout=10.0,
        auto_connect=True,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self._socket = None
        self.receive_buffer = b""
        if auto_connect:
            self.connect()

    # 判断是否连接
    def is_connected(self) -> bool:
        return self._socket is not None

    # 尝试连接
    def connect(self) -> bool:
        self.disconnect()
        try:
            sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        except OSError as exc:
            print(f"[ERROR] failed to connect to {self.host}:{self.port}: {exc}")
            self.disconnect()
            return False

        self._socket = sock
        self.receive_buffer = b""
        print(f"[INFO] connected to {self.host}:{self.port}")
        return True

    # 关闭连接
    def disconnect(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
        self._socket = None
        self.receive_buffer = b""

    # 接收一行数据
    def receive_one_message(self) -> Optional[str]:
        if self._socket is None:
            return None

        deadline = time.time() + self.timeout
        while True:
            newline_index = self.receive_buffer.find(b"\n")
            if newline_index >= 0:
                message = self.receive_buffer[:newline_index]
                self.receive_buffer = self.receive_buffer[newline_index + 1 :]
                text = message.decode("utf-8", errors="replace").strip()
                if text:
                    return text
                continue

            if time.time() >= deadline:
                raise TimeoutError("Response timeout")

            chunk = self._socket.recv(4096)
            if not chunk:
                return None
            self.receive_buffer += chunk

    # 接收消息
    def receive_message(self) -> JsonDict:
        raw = self.receive_one_message()
        if raw is None:
            return {"status": "error", "message": "Connection closed"}

        try:
            value = json.loads(raw)
        except json.JSONDecodeError:
            return {"status": "error", "message": "JSON decode failed", "raw": raw}

        if isinstance(value, dict):
            return value
        return {"status": "error", "message": "Response is not a JSON object", "raw": value}

    # 发送消息
    def send_message(self, payload: JsonDict) -> JsonDict:
        if not isinstance(payload, dict):
            return {"status": "error", "message": "Payload must be a JSON object"}

        message = json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"
        attempts = 2

        for attempt in range(attempts):
            if self._socket is None and not self.connect():
                continue
            else:
                try:
                    assert self._socket is not None
                    self._socket.sendall(message.encode("utf-8"))
                    return self.receive_message()
                except (OSError, TimeoutError) as exc:
                    self.disconnect()
            time.sleep(0.2)
        return {"status": "error", "message": "Send message failed"}
