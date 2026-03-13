"""GradSim TCP 客户端入口。

该模块负责维护底层 Socket 生命周期、发送 JSON 请求、增量解析服务端响应，
并向上层脚本暴露暂停/恢复仿真、查询智能体列表等常用接口。
"""

import json
import socket
import threading
import time


class TCPClient:
    """GradSim 的 TCP 请求/响应客户端。"""

    def __init__(self, host="127.0.0.1", port=9000, timeout=5.0, auto_connect=True):
        """配置目标地址，并按需在构造时立即建立连接。"""
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
        """当前是否已经持有可用的 TCP Socket。"""
        return self.socket is not None

    def connect(self):
        """建立到模拟器 TCP 服务的连接。"""
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
        """关闭连接，并清空所有尚未消费的接收缓存。"""
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
        """尝试从接收缓冲区中解析出一个完整 JSON 对象。"""
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
        # `raw_decode` 可以在同一缓冲区里只消费一个 JSON，适合处理粘包场景。
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
        """接收并返回一个完整 JSON 对象；失败时返回结构化错误。"""
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
        """发送一条请求，并同步等待对应响应。"""
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
        """兼容旧代码的别名，内部仍然直接转发到 ``request``。"""
        return self.request(payload)

    def ping(self):
        """检测服务端是否在线且可以正常应答。"""
        return self.request({"ping": {}})

    def sim_pause(self):
        """暂停 UE 端仿真时钟。"""
        return self.request({"sim_pause": {}})

    def sim_resume(self):
        """恢复 UE 端仿真时钟。"""
        return self.request({"sim_resume": {}})

    def sim_step(self, steps=1, dt=0.01):
        """在暂停状态下按固定步长推进仿真。"""
        return self.request({"sim_step": {"steps": int(max(1, steps)), "dt": float(dt)}})

    def sim_reset(self):
        """请求模拟器执行整体重置。"""
        return self.request({"sim_reset": {}})

    def get_agent_list(self):
        """获取智能体注册表的原始响应。"""
        return self.request({"get_agent_list": {}})

    def get_agents(self):
        """提取并返回智能体 ID 列表。"""
        response = self.get_agent_list()
        agents = response.get("agents", []) if isinstance(response, dict) else []
        return [str(agent_id) for agent_id in agents if isinstance(agent_id, str) and agent_id]

    def get_agents_detail(self):
        """提取并返回更详细的智能体信息列表。"""
        response = self.get_agent_list()
        detail = response.get("agents_detail", []) if isinstance(response, dict) else []
        return detail if isinstance(detail, list) else []


GradSimClient = TCPClient