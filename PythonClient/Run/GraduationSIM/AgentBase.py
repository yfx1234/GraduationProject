from __future__ import annotations

import base64
import json
import time
from collections.abc import Callable, Iterable
from types import SimpleNamespace as Namespace
from typing import Any

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import numpy as np
except ImportError:
    np = None

JsonDict = dict[str, Any]


def to_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def to_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)


def to_vector3(value: Any, default: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return (to_float(value[0]), to_float(value[1]), to_float(value[2]))
    return default


class Pose:
    __slots__ = ("x", "y", "z", "roll", "pitch", "yaw")

    def __init__(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0) -> None:
        self.x = to_float(x)
        self.y = to_float(y)
        self.z = to_float(z)
        self.roll = to_float(roll)
        self.pitch = to_float(pitch)
        self.yaw = to_float(yaw)

    @classmethod
    def from_any(cls, value=None) -> "Pose":
        if isinstance(value, cls):
            return value
        if isinstance(value, dict):
            return cls(**{key: value.get(key, 0.0) for key in cls.__slots__})
        if isinstance(value, (list, tuple)):
            values = list(value)[:6] + [0.0] * max(0, 6 - len(value))
            return cls(*values[:6])
        return cls()

    def to_initial_dict(self) -> JsonDict:
        return {key: getattr(self, key) for key in self.__slots__}

    def to_dict(self) -> JsonDict:
        return self.to_initial_dict()


class ImagePacket:
    __slots__ = ("width", "height", "data", "image_type")

    def __init__(self, width=0, height=0, data="", image_type="scene") -> None:
        self.width = to_int(width)
        self.height = to_int(height)
        self.data = str(data or "")
        self.image_type = str(image_type or "scene")

    @classmethod
    def from_response(cls, payload: JsonDict | None, image_type: str = "scene") -> "ImagePacket":
        payload = payload or {}
        return cls(
            width=payload.get("width", 0),
            height=payload.get("height", 0),
            data=payload.get("data", ""),
            image_type=payload.get("image_type", image_type),
        )

    def decode_bgr(self):
        if cv2 is None or np is None or not self.data:
            return None
        try:
            raw = base64.b64decode(self.data)
        except Exception:
            return None
        return cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)


def DroneSnapshot(payload: JsonDict | None = None):
    payload = payload or {}
    orientation = payload.get("orientation") if isinstance(payload.get("orientation"), dict) else {}
    return Namespace(
        position=to_vector3(payload.get("position")),
        velocity=to_vector3(payload.get("velocity")),
        roll=to_float(orientation.get("roll")),
        pitch=to_float(orientation.get("pitch")),
        yaw=to_float(orientation.get("yaw")),
        camera_pitch=to_float(payload.get("camera_pitch")),
        camera_yaw=to_float(payload.get("camera_yaw")),
        api_control=bool(payload.get("api_control")),
        raw=dict(payload),
    )


class AgentBase:
    def __init__(self, client, actor_id: str, classname: str | None = None, label: str = "Agent", unit: str = "m") -> None:
        self.client = client
        self.actor_id = str(actor_id)
        self.classname = classname
        self.label = str(label)
        self.unit = str(unit)

    @staticmethod
    def is_ok(response: object) -> bool:
        return isinstance(response, dict) and response.get("status") in {"ok", True}

    @staticmethod
    def wait_response(
        request: Callable[[], JsonDict],
        timeout: float = 5.0,
        interval: float = 0.2,
        accept: Callable[[JsonDict], bool] | None = None,
    ) -> JsonDict:
        deadline = time.time() + timeout
        while time.time() < deadline:
            response = request()
            if isinstance(response, dict) and (accept(response) if accept else bool(response)):
                return response
            time.sleep(interval)
        return {}

    @staticmethod
    def wait_altitude(drone, altitude: float, timeout: float = 20.0, tolerance: float = 0.8) -> bool:
        deadline = time.time() + timeout
        targetAltitude = float(altitude) - float(tolerance)
        while time.time() < deadline:
            state = drone.get_state(frame="ue")
            position = state.get("position", [0.0, 0.0, 0.0]) if isinstance(state, dict) else [0.0, 0.0, 0.0]
            if len(position) >= 3 and to_float(position[2]) >= targetAltitude:
                return True
            time.sleep(0.25)
        return False

    @staticmethod
    def remove_existing_actors(client, actor_ids: Iterable[str], pause_s: float = 0.2) -> None:
        for actorId in actor_ids:
            client.send_message({"remove_actor": {"actor_id": actorId}})
        if pause_s > 0.0:
            time.sleep(pause_s)

    @staticmethod
    def safe_hover(drone) -> None:
        if drone is None:
            return
        try:
            drone.hover()
        except Exception:
            pass

    @staticmethod
    def safe_remove(actor) -> None:
        if actor is None:
            return
        try:
            actor.remove()
        except Exception:
            pass

    @staticmethod
    def disconnect_client(client) -> None:
        if client is None:
            return
        try:
            client.disconnect()
        except Exception:
            pass

    @staticmethod
    def unwrap_response(response: JsonDict, field_name: str) -> JsonDict:
        value = response.get(field_name) if isinstance(response, dict) else None
        return value if isinstance(value, dict) else response

    @staticmethod
    def parse_json_text(value: Any) -> Any:
        if isinstance(value, (dict, list)) or not isinstance(value, str):
            return value
        text = value.strip()
        if not text:
            return {}
        if text[:1] not in "[{":
            return value
        try:
            return json.loads(text)
        except Exception:
            return value

    @staticmethod
    def normalize_frame(frame: str) -> str:
        return "ned" if isinstance(frame, str) and frame.lower() == "ned" else "ue"

    @classmethod
    def normalize_param(cls, value: Any) -> Any:
        if isinstance(value, (int, float, bool, str)):
            return value
        if isinstance(value, Pose):
            return value.to_dict()
        if isinstance(value, tuple):
            value = list(value)
        if isinstance(value, list):
            return [cls.normalize_param(item) for item in value]
        if isinstance(value, dict):
            return {str(key): cls.normalize_param(item) for key, item in value.items()}
        return str(value)

    def create(
        self,
        pose: Pose | None = None,
        classname: str | None = None,
        label: str | None = None,
        extra: JsonDict | None = None,
    ) -> JsonDict:
        actorClassname = classname or self.classname
        if not actorClassname:
            raise ValueError("Type must be set before creating actor")
        payload: JsonDict = {
            "classname": actorClassname,
            "expected_id": self.actor_id,
            "label": label or self.label or "Agent",
            "pose": Pose.from_any(pose).to_initial_dict(),
        }
        payload.update({"unit": self.unit, **(extra or {})})
        response = self.client.send_message({"add_actor": payload})
        response = self.unwrap_response(response, "add_actor_return")
        self.classname = actorClassname
        self.label = str(payload["label"])
        return response

    def remove(self) -> JsonDict:
        return self.unwrap_response(self.client.send_message({"remove_actor": {"actor_id": self.actor_id}}), "remove_actor_return")

    def call_function(
        self,
        function_name: str,
        *args: Any,
        expect_return: bool = False,
        dict_args: JsonDict | None = None,
        parse_return_json: bool = False,
    ) -> JsonDict:
        payload: JsonDict = {"actor_id": self.actor_id, "function": function_name, "return": bool(expect_return)}
        if dict_args is not None:
            payload["parameters"] = self.normalize_param(dict_args)
        elif args:
            payload["parameters"] = [self.normalize_param(arg) for arg in args]
        response = self.unwrap_response(self.client.send_message({"call_actor": payload}), "call_actor_return")
        if not parse_return_json or not self.is_ok(response):
            return response
        parsed = self.parse_json_text(response.get("return", ""))
        if isinstance(parsed, dict):
            return parsed
        return {"status": "ok"} if parsed == {} else {"status": "ok", "return": parsed}

    def get_state(self, frame: str = "ue") -> JsonDict:
        return self.call_function("GetState", expect_return=True, dict_args={"Frame": self.normalize_frame(frame)}, parse_return_json=True)

    def get_image_response(self, image_type: str = "scene", quality: int = 90) -> JsonDict:
        return self.call_function("GetImage", expect_return=True, dict_args={"ImageType": image_type, "Quality": int(quality)}, parse_return_json=True)

    def get_image_packet(self, image_type: str = "scene", quality: int = 90) -> ImagePacket | None:
        response = self.get_image_response(image_type=image_type, quality=quality)
        return None if not self.is_ok(response) else ImagePacket.from_response(response, image_type=image_type)

