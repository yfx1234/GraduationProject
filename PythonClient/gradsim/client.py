import base64
import json
import socket
import threading
import time
from typing import Any, Dict, List, Optional


class GradSimClient:
    _IMAGE_TYPE_NAME_MAP = {
        0: "scene",
        1: "depth_planar",
        3: "depth_vis",
        5: "segmentation",
        7: "infrared",
    }

    def __init__(self, ip: str = "127.0.0.1", port: int = 9000, timeout: float = 5.0):
        self.ip = ip
        self.port = int(port)
        self.timeout = float(timeout)
        self.socket: Optional[socket.socket] = None
        self._send_lock = threading.Lock()
        self._recv_buffer = b""
        self.connect()

    def connect(self) -> None:
        try:
            if self.socket:
                try:
                    self.socket.close()
                except Exception:
                    pass

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.ip, self.port))
            self._recv_buffer = b""
            print(f"Connected to GradSim TCP server at {self.ip}:{self.port}")
        except Exception as exc:
            print(f"Failed to connect: {exc}")
            self.socket = None
            self._recv_buffer = b""

    def close(self) -> None:
        with self._send_lock:
            if self.socket:
                try:
                    self.socket.close()
                finally:
                    self.socket = None
                    self._recv_buffer = b""
                    print("Connection closed.")

    def _extract_json_from_buffer(self) -> Optional[Dict[str, Any]]:
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

    def _recv_json(self) -> Dict[str, Any]:
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

    def _send(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._send_lock:
            if not self.socket:
                return {"status": "error", "message": "Not connected"}

            try:
                msg = json.dumps(payload, ensure_ascii=False) + "\n"
                self.socket.sendall(msg.encode("utf-8"))
                return self._recv_json()
            except Exception as exc:
                print(f"Communication error: {exc}")
                time.sleep(0.2)
                self.connect()
                return {"status": "error", "message": str(exc)}

    @staticmethod
    def _unwrap_response(response: Dict[str, Any], field_name: str) -> Dict[str, Any]:
        if isinstance(response, dict):
            field = response.get(field_name)
            if isinstance(field, dict):
                return field
        return response

    @staticmethod
    def _normalize_image_type(image_type: Any) -> str:
        if isinstance(image_type, int):
            return GradSimClient._IMAGE_TYPE_NAME_MAP.get(image_type, "scene")

        text = str(image_type).strip().lower()
        if text.isdigit():
            return GradSimClient._IMAGE_TYPE_NAME_MAP.get(int(text), "scene")

        if text in ("depth", "depthplanar", "depth_planar"):
            return "depth_planar"
        if text in ("depthvis", "depth_vis"):
            return "depth_vis"
        if text in ("seg", "segment"):
            return "segmentation"
        if text in ("ir",):
            return "infrared"
        return text or "scene"

    @staticmethod
    def _normalize_frame(frame: str) -> str:
        if isinstance(frame, str) and frame.lower() == "ned":
            return "ned"
        return "ue"

    def ping(self) -> Dict[str, Any]:
        return self._send({"ping": {}})

    def sim_pause(self) -> Dict[str, Any]:
        return self._send({"sim_pause": {}})

    def sim_resume(self) -> Dict[str, Any]:
        return self._send({"sim_resume": {}})

    def sim_step(self, steps: int = 1, dt: float = 0.01) -> Dict[str, Any]:
        return self._send({"sim_step": {"steps": int(max(1, steps)), "dt": float(dt)}})

    def sim_reset(self) -> Dict[str, Any]:
        return self._send({"sim_reset": {}})

    def get_agent_list(self) -> Dict[str, Any]:
        return self._send({"get_agent_list": {}})

    def get_agents(self) -> List[str]:
        resp = self.get_agent_list()
        agents = resp.get("agents", []) if isinstance(resp, dict) else []
        return [str(x) for x in agents if isinstance(x, str) and x]

    def get_agents_detail(self) -> List[Dict[str, Any]]:
        resp = self.get_agent_list()
        detail = resp.get("agents_detail", []) if isinstance(resp, dict) else []
        return detail if isinstance(detail, list) else []

    def add_actor(
        self,
        classname: str,
        expected_id: Optional[str] = None,
        label: str = "Agent",
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        extra: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "classname": classname,
            "label": label,
            "pose": {
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "roll": float(roll),
                "pitch": float(pitch),
                "yaw": float(yaw),
            },
        }
        if expected_id:
            payload["expected_id"] = expected_id
        if isinstance(extra, dict):
            payload.update(extra)
        response = self._send({"add_actor": payload})
        return self._unwrap_response(response, "add_actor_return")

    def remove_actor(self, actor_id: str) -> Dict[str, Any]:
        response = self._send({"remove_actor": {"actor_id": actor_id}})
        return self._unwrap_response(response, "remove_actor_return")

    def call_actor(self, actor_id: str, function_name: str, parameters=None, expect_return: bool = True) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "actor_id": actor_id,
            "function": function_name,
            "return": bool(expect_return),
        }
        if parameters is not None:
            payload["parameters"] = parameters
        response = self._send({"call_actor": payload})
        return self._unwrap_response(response, "call_actor_return")

    def call_drone(self, function: str, drone_id: str = "drone_0", **kwargs) -> Dict[str, Any]:
        payload = {"function": function, "id": drone_id}
        payload.update(kwargs)
        return self._send({"call_drone": payload})

    def drone_takeoff(self, altitude: float = 3.0, drone_id: str = "drone_0") -> Dict[str, Any]:
        return self.call_drone("takeoff", drone_id=drone_id, altitude=float(altitude))

    def drone_hover(self, drone_id: str = "drone_0") -> Dict[str, Any]:
        return self.call_drone("hover", drone_id=drone_id)

    def drone_land(self, drone_id: str = "drone_0") -> Dict[str, Any]:
        return self.call_drone("land", drone_id=drone_id)

    def drone_move_to(
        self,
        x: float,
        y: float,
        z: float,
        speed: float = 2.0,
        frame: str = "ue",
        drone_id: str = "drone_0",
        yaw_mode: Optional[Dict[str, Any]] = None,
        yaw: Optional[float] = None,
        drivetrain: Optional[str] = None,
    ) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "speed": float(speed),
            "frame": self._normalize_frame(frame),
        }
        if yaw_mode is not None:
            payload["yaw_mode"] = yaw_mode
        if yaw is not None:
            payload["yaw"] = float(yaw)
        if drivetrain is not None:
            payload["drivetrain"] = str(drivetrain)
        return self.call_drone("move_to_position", drone_id=drone_id, **payload)

    def drone_move_by_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        frame: str = "ue",
        drone_id: str = "drone_0",
        yaw_mode: Optional[Dict[str, Any]] = None,
        yaw: Optional[float] = None,
        drivetrain: Optional[str] = None,
    ) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "vx": float(vx),
            "vy": float(vy),
            "vz": float(vz),
            "frame": self._normalize_frame(frame),
        }
        if yaw_mode is not None:
            payload["yaw_mode"] = yaw_mode
        if yaw is not None:
            payload["yaw"] = float(yaw)
        if drivetrain is not None:
            payload["drivetrain"] = str(drivetrain)
        return self.call_drone("move_by_velocity", drone_id=drone_id, **payload)

    def drone_set_camera_angles(self, pitch: float, yaw: float, drone_id: str = "drone_0") -> Dict[str, Any]:
        return self.call_drone("set_camera_angles", drone_id=drone_id, pitch=float(pitch), yaw=float(yaw))

    def drone_state(self, drone_id: str = "drone_0", frame: str = "ue") -> Dict[str, Any]:
        return self._send({"get_drone_state": {"id": drone_id, "frame": self._normalize_frame(frame)}})

    def call_turret(self, function: str, turret_id: str = "turret_0", **kwargs) -> Dict[str, Any]:
        payload = {"function": function, "id": turret_id}
        payload.update(kwargs)
        return self._send({"call_turret": payload})

    def turret_set_angles(self, pitch: float, yaw: float, turret_id: str = "turret_0") -> Dict[str, Any]:
        return self.call_turret("set_angles", turret_id=turret_id, pitch=float(pitch), yaw=float(yaw))

    def turret_fire(self, speed: float = 400.0, turret_id: str = "turret_0") -> Dict[str, Any]:
        return self.call_turret("fire", turret_id=turret_id, speed=float(speed))

    def turret_start_tracking(self, target_id: str, turret_id: str = "turret_0") -> Dict[str, Any]:
        return self.call_turret("start_tracking", turret_id=turret_id, target_id=str(target_id))

    def turret_stop_tracking(self, turret_id: str = "turret_0") -> Dict[str, Any]:
        return self.call_turret("stop_tracking", turret_id=turret_id)

    def turret_reset(self, turret_id: str = "turret_0") -> Dict[str, Any]:
        return self.call_turret("reset", turret_id=turret_id)

    def turret_state(self, turret_id: str = "turret_0") -> Dict[str, Any]:
        return self._send({"get_turret_state": {"id": turret_id}})

    def call_guidance(self, function: str, **kwargs) -> Dict[str, Any]:
        payload = {"function": function}
        payload.update(kwargs)
        return self._send({"call_guidance": payload})

    def guidance_reset(self) -> Dict[str, Any]:
        return self.call_guidance("reset")

    def guidance_auto_intercept(
        self,
        interceptor_id: str,
        target_id: str,
        method: Optional[str] = None,
        speed: Optional[float] = None,
        nav_gain: Optional[float] = None,
        lead_time: Optional[float] = None,
        capture_radius: Optional[float] = None,
        stop_on_capture: bool = True,
    ) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "interceptor_id": interceptor_id,
            "target_id": target_id,
            "stop_on_capture": bool(stop_on_capture),
        }
        if method:
            payload["method"] = method
        if speed is not None:
            payload["speed"] = float(speed)
        if nav_gain is not None:
            payload["nav_gain"] = float(nav_gain)
        if lead_time is not None:
            payload["lead_time"] = float(lead_time)
        if capture_radius is not None:
            payload["capture_radius"] = float(capture_radius)
        return self.call_guidance("auto_intercept", **payload)

    def guidance_state(self) -> Dict[str, Any]:
        return self._send({"get_guidance_state": {}})

    def get_image(self, agent_id: str, image_type: str = "scene", quality: int = 90, max_depth_m: float = 200.0) -> Dict[str, Any]:
        return self._send({
            "get_image": {
                "id": agent_id,
                "image_type": self._normalize_image_type(image_type),
                "quality": int(quality),
                "max_depth_m": float(max_depth_m),
            }
        })

    def get_image_raw(self, agent_id: str, image_type: str = "scene", quality: int = 90, max_depth_m: float = 200.0) -> Optional[Dict[str, Any]]:
        resp = self.get_image(agent_id=agent_id, image_type=image_type, quality=quality, max_depth_m=max_depth_m)
        if isinstance(resp, dict) and resp.get("status") == "ok":
            return resp
        return None

    def get_image_base64(self, agent_id: str, image_type: str = "scene", quality: int = 90, max_depth_m: float = 200.0) -> Optional[str]:
        resp = self.get_image_raw(agent_id=agent_id, image_type=image_type, quality=quality, max_depth_m=max_depth_m)
        if resp:
            return resp.get("data")
        return None

    def get_image_numpy(self, agent_id: str, image_type: str = "scene", quality: int = 90, max_depth_m: float = 200.0):
        data_b64 = self.get_image_base64(agent_id=agent_id, image_type=image_type, quality=quality, max_depth_m=max_depth_m)
        if not data_b64:
            return None

        import cv2
        import numpy as np

        try:
            raw = base64.b64decode(data_b64)
        except Exception:
            return None

        arr = np.frombuffer(raw, dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)
