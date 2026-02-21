"""
SimClient — GraduationProject TCP 客户端封装
所有 UE 仿真交互通过这一个类完成
"""
import socket, json, base64, time
from typing import Optional, Dict, Any, List
import numpy as np

class SimClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 9000, timeout: float = 10.0):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect((host, port))
        print(f"[SimClient] Connected to {host}:{port}")

    def close(self):
        self.sock.close()

    def _send(self, cmd: dict) -> dict:
        try:
            self.sock.send(json.dumps(cmd).encode() + b"\n")
            buf = b""
            while True:
                chunk = self.sock.recv(262144)
                if not chunk:
                    break
                buf += chunk
                try:
                    return json.loads(buf.decode())
                except (json.JSONDecodeError, UnicodeDecodeError):
                    continue
            return json.loads(buf.decode())
        except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError, OSError) as e:
            print(f"[SimClient] 连接断开({e})，重连...")
            self._reconnect()
            return {"status": "error", "message": "reconnected"}

    def _reconnect(self):
        try:
            self.sock.close()
        except:
            pass
        import time
        time.sleep(0.5)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(10.0)
        try:
            self.sock.connect((self.host, self.port))
            print(f"[SimClient] 重连成功")
        except Exception as e:
            print(f"[SimClient] 重连失败: {e}")

    # ---- 基础命令 ----
    def ping(self) -> dict:
        return self._send({"ping": {}})

    def get_agents(self) -> List[str]:
        resp = self._send({"get_agent_list": {}})
        return resp.get("agents", [])

    def sim_pause(self):
        return self._send({"sim_pause": {}})

    def sim_resume(self):
        return self._send({"sim_resume": {}})

    def sim_reset(self):
        return self._send({"sim_reset": {}})

    # ---- Drone ----
    def drone_takeoff(self, altitude: float = 3.0) -> dict:
        return self._send({"call_drone": {"function": "takeoff", "altitude": altitude}})

    def drone_land(self) -> dict:
        return self._send({"call_drone": {"function": "land"}})

    def drone_hover(self) -> dict:
        return self._send({"call_drone": {"function": "hover"}})

    def drone_move_to(self, x: float, y: float, z: float, speed: float = 2.0) -> dict:
        return self._send({"call_drone": {"function": "move_to_position",
                           "x": x, "y": y, "z": z, "speed": speed}})

    def drone_move_by_velocity(self, vx: float, vy: float, vz: float) -> dict:
        return self._send({"call_drone": {"function": "move_by_velocity",
                           "vx": vx, "vy": vy, "vz": vz}})

    def drone_state(self, drone_id: str = "drone_0") -> dict:
        return self._send({"get_drone_state": {"id": drone_id}})

    def drone_position(self, drone_id: str = "drone_0") -> np.ndarray:
        state = self.drone_state(drone_id)
        pos = state.get("position", [0, 0, 0])
        return np.array(pos)

    # ---- Turret ----
    def turret_set_angles(self, pitch: float, yaw: float, turret_id: str = "turret_0") -> dict:
        return self._send({"call_turret": {"function": "set_angles",
                           "id": turret_id, "pitch": pitch, "yaw": yaw}})

    def turret_fire(self, speed: float = 400.0, turret_id: str = "turret_0") -> dict:
        return self._send({"call_turret": {"function": "fire",
                           "id": turret_id, "speed": speed}})

    def turret_start_tracking(self, target_id: str, turret_id: str = "turret_0") -> dict:
        return self._send({"call_turret": {"function": "start_tracking",
                           "id": turret_id, "target_id": target_id}})

    def turret_stop_tracking(self, turret_id: str = "turret_0") -> dict:
        return self._send({"call_turret": {"function": "stop_tracking", "id": turret_id}})

    def turret_state(self, turret_id: str = "turret_0") -> dict:
        return self._send({"get_turret_state": {"id": turret_id}})

    def turret_reset(self, turret_id: str = "turret_0") -> dict:
        return self._send({"call_turret": {"function": "reset", "id": turret_id}})

    # ---- Guidance ----
    def guidance_set_method(self, method: str = "predictive", **kwargs) -> dict:
        cmd = {"function": "set_method", "method": method}
        cmd.update(kwargs)
        return self._send({"call_guidance": cmd})

    def guidance_update_target(self, x: float, y: float, z: float, dt: float = 0.1) -> dict:
        return self._send({"call_guidance": {"function": "update_target",
                           "x": x, "y": y, "z": z, "dt": dt}})

    def guidance_compute_aim(self, turret_id: str = "turret_0", muzzle_speed: float = 400.0) -> dict:
        return self._send({"call_guidance": {"function": "compute_aim",
                           "turret_id": turret_id, "muzzle_speed": muzzle_speed}})

    def guidance_auto_engage(self, turret_id: str = "turret_0", target_id: str = "drone_0",
                             muzzle_speed: float = 400.0, fire: bool = False) -> dict:
        return self._send({"call_guidance": {"function": "auto_engage",
                           "turret_id": turret_id, "target_id": target_id,
                           "muzzle_speed": muzzle_speed, "fire": fire}})

    def guidance_reset(self) -> dict:
        return self._send({"call_guidance": {"function": "reset"}})

    def guidance_state(self) -> dict:
        return self._send({"get_guidance_state": {}})

    def guidance_set_kalman(self, process_noise: float = 1.0, measurement_noise: float = 0.5) -> dict:
        return self._send({"call_guidance": {"function": "set_kalman_params",
                           "process_noise": process_noise,
                           "measurement_noise": measurement_noise}})

    # ---- Image ----
    def get_image_base64(self) -> Optional[str]:
        resp = self._send({"get_image": {}})
        if resp.get("status") == "ok":
            return resp.get("data")
        return None

    def get_image_bytes(self) -> Optional[bytes]:
        b64 = self.get_image_base64()
        if b64:
            return base64.b64decode(b64)
        return None

    def get_image_numpy(self) -> Optional[np.ndarray]:
        """返回 numpy 格式的 BGR 图像（需要 opencv）"""
        import cv2
        img_bytes = self.get_image_bytes()
        if img_bytes:
            arr = np.frombuffer(img_bytes, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return None


if __name__ == "__main__":
    client = SimClient()
    print(client.ping())
    print(client.get_agents())
    client.close()
