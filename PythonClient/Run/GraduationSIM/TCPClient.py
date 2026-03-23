from __future__ import annotations

import json
import socket
import time
from typing import Any

JsonDict = dict[str, Any]


class TCPClient:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 9000,
        timeout: float = 10.0,
        auto_connect: bool = True,
        retry_attempts: int = 2,
        retry_delay: float = 0.2,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self.retry_attempts = max(1, int(retry_attempts))
        self.retry_delay = max(0.0, float(retry_delay))
        self._socket: socket.socket | None = None
        self.receive_buffer = b""
        if auto_connect:
            self.connect()

    def is_connected(self) -> bool:
        return self._socket is not None

    def connect(self) -> bool:
        self.disconnect()
        try:
            clientSocket = socket.create_connection((self.host, self.port), timeout=self.timeout)
            clientSocket.settimeout(self.timeout)
        except OSError as error:
            print(f"[ERROR] failed to connect to {self.host}:{self.port}: {error}")
            self.disconnect()
            return False

        self._socket = clientSocket
        self.receive_buffer = b""
        print(f"[INFO] connected to {self.host}:{self.port}")
        return True

    def disconnect(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
        self._socket = None
        self.receive_buffer = b""

    def receive_one_message(self) -> str | None:
        if self._socket is None:
            return None

        deadline = time.monotonic() + self.timeout
        while True:
            newlineIndex = self.receive_buffer.find(b"\n")
            if newlineIndex >= 0:
                message = self.receive_buffer[:newlineIndex]
                self.receive_buffer = self.receive_buffer[newlineIndex + 1 :]
                text = message.decode("utf-8", errors="replace").strip()
                if text:
                    return text
                continue

            if time.monotonic() >= deadline:
                raise TimeoutError("Response timeout")

            try:
                chunk = self._socket.recv(4096)
            except socket.timeout as error:
                raise TimeoutError("Response timeout") from error
            if not chunk:
                return None
            self.receive_buffer += chunk

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

    def send_message(self, payload: JsonDict) -> JsonDict:
        if not isinstance(payload, dict):
            return {"status": "error", "message": "Payload must be a JSON object"}

        message = json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"
        last_error = ""

        for attempt in range(self.retry_attempts):
            if self._socket is None and not self.connect():
                last_error = f"connect failed to {self.host}:{self.port}"
            else:
                try:
                    assert self._socket is not None
                    self._socket.sendall(message.encode("utf-8"))
                    return self.receive_message()
                except (OSError, TimeoutError) as error:
                    last_error = str(error) or error.__class__.__name__
                    self.disconnect()
            if attempt + 1 < self.retry_attempts and self.retry_delay > 0.0:
                time.sleep(self.retry_delay)

        return {"status": "error", "message": f"Send message failed: {last_error}" if last_error else "Send message failed"}
