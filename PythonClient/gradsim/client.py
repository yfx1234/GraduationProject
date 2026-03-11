"""Client entry for GradSim."""

import json
import socket
import threading
import time


class TCPClient:
    """Owns the socket and performs JSON request/response I/O."""

    def __init__(self, host="127.0.0.1", port=9000, timeout=5.0, auto_connect=True):
        self.host = host
        self.ip = host
        self.port = int(port)
        self.timeout = float(timeout)
        self.socket = None
        self._send_lock = threading.Lock()
        self._recv_buffer = b""
        if auto_connect:
            self.connect()

    @property
    def connected(self):
        return self.socket is not None

    def connect(self):
        try:
            if self.socket:
                try:
                    self.socket.close()
                except Exception:
                    pass

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.host, self.port))
            self._recv_buffer = b""
            print(f"Connected to GradSim TCP server at {self.host}:{self.port}")
            return True
        except Exception as exc:
            print(f"Failed to connect: {exc}")
            self.socket = None
            self._recv_buffer = b""
            return False

    def close(self):
        with self._send_lock:
            if self.socket:
                try:
                    self.socket.close()
                finally:
                    self.socket = None
                    self._recv_buffer = b""
                    print("Connection closed.")

    disconnect = close

    def _extract_json_from_buffer(self):
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
            obj, idx = decoder.raw_decode(stripped)
        except json.JSONDecodeError:
            return None

        consumed_chars = leading_chars + idx
        consumed_bytes = len(text[:consumed_chars].encode("utf-8"))
        self._recv_buffer = self._recv_buffer[consumed_bytes:]
        self._recv_buffer = self._recv_buffer.lstrip()
        return obj

    def _recv_json(self):
        if not self.socket:
            return {"status": "error", "message": "Not connected"}

        while True:
            parsed = self._extract_json_from_buffer()
            if isinstance(parsed, dict):
                return parsed
            if parsed is not None:
                return {"status": "error", "message": "Response is not a JSON object", "raw": parsed}

            chunk = self.socket.recv(4096)
            if not chunk:
                break
            self._recv_buffer += chunk

        raw = ""
        if self._recv_buffer:
            try:
                raw = self._recv_buffer.decode("utf-8", errors="replace").strip()
            finally:
                self._recv_buffer = b""

        if raw:
            return {"status": "error", "message": "Incomplete JSON response", "raw": raw}
        return {"status": "error", "message": "Empty response"}

    def request(self, payload):
        with self._send_lock:
            if not self.socket:
                return {"status": "error", "message": "Not connected"}

            try:
                message = json.dumps(payload, ensure_ascii=False) + "\n"
                self.socket.sendall(message.encode("utf-8"))
                return self._recv_json()
            except Exception as exc:
                print(f"Communication error: {exc}")
                time.sleep(0.2)
                self.connect()
                return {"status": "error", "message": str(exc)}

    def send_message(self, payload):
        return self.request(payload)

    def ping(self):
        return self.request({"ping": {}})

    def sim_pause(self):
        return self.request({"sim_pause": {}})

    def sim_resume(self):
        return self.request({"sim_resume": {}})

    def sim_step(self, steps=1, dt=0.01):
        return self.request({"sim_step": {"steps": int(max(1, steps)), "dt": float(dt)}})

    def sim_reset(self):
        return self.request({"sim_reset": {}})

    def get_agent_list(self):
        return self.request({"get_agent_list": {}})

    def get_agents(self):
        response = self.get_agent_list()
        agents = response.get("agents", []) if isinstance(response, dict) else []
        return [str(agent_id) for agent_id in agents if isinstance(agent_id, str) and agent_id]

    def get_agents_detail(self):
        response = self.get_agent_list()
        detail = response.get("agents_detail", []) if isinstance(response, dict) else []
        return detail if isinstance(detail, list) else []


GradSimClient = TCPClient