from __future__ import annotations

import base64
from dataclasses import dataclass, field
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


def to_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on", "ok"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    return default


def to_vec3(value, default=(0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return (to_float(value[0]), to_float(value[1]), to_float(value[2]))
    return default


@dataclass(slots=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def __post_init__(self) -> None:
        self.x = to_float(self.x)
        self.y = to_float(self.y)
        self.z = to_float(self.z)
        self.roll = to_float(self.roll)
        self.pitch = to_float(self.pitch)
        self.yaw = to_float(self.yaw)

    def position_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def to_initial_dict(self) -> JsonDict:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
        }

    def to_dict(self) -> JsonDict:
        return self.to_initial_dict()


@dataclass(slots=True)
class ImagePacket:
    width: int = 0
    height: int = 0
    data: str = ""
    image_type: str = "scene"

    def __post_init__(self) -> None:
        self.width = to_int(self.width)
        self.height = to_int(self.height)
        self.data = str(self.data or "")
        self.image_type = str(self.image_type or "scene")

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


@dataclass(slots=True)
class DroneSnapshot:
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    camera_pitch: float = 0.0
    camera_yaw: float = 0.0
    api_control: bool = False
    raw: JsonDict = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.position = to_vec3(self.position)
        self.velocity = to_vec3(self.velocity)
        self.roll = to_float(self.roll)
        self.pitch = to_float(self.pitch)
        self.yaw = to_float(self.yaw)
        self.camera_pitch = to_float(self.camera_pitch)
        self.camera_yaw = to_float(self.camera_yaw)
        self.api_control = to_bool(self.api_control)
        self.raw = dict(self.raw or {})

    @classmethod
    def from_state(cls, payload: JsonDict | None) -> "DroneSnapshot":
        payload = payload or {}
        orientation = payload.get("orientation") if isinstance(payload.get("orientation"), dict) else {}
        return cls(
            position=payload.get("position"),
            velocity=payload.get("velocity"),
            roll=orientation.get("roll"),
            pitch=orientation.get("pitch"),
            yaw=orientation.get("yaw"),
            camera_pitch=payload.get("camera_pitch"),
            camera_yaw=payload.get("camera_yaw"),
            api_control=payload.get("api_control"),
            raw=payload,
        )


@dataclass(slots=True)
class AutoLabel:
    valid: bool = False
    class_id: int = 0
    bbox_xyxy: list[int] | None = None
    bbox_yolo: list[float] | None = None
    pixel_count: int = 0
    area_ratio: float = 0.0
    image_w: int = 0
    image_h: int = 0

    def __post_init__(self) -> None:
        self.valid = bool(self.valid)
        self.class_id = to_int(self.class_id)
        self.bbox_xyxy = None if self.bbox_xyxy is None else list(self.bbox_xyxy)
        self.bbox_yolo = None if self.bbox_yolo is None else list(self.bbox_yolo)
        self.pixel_count = to_int(self.pixel_count)
        self.area_ratio = to_float(self.area_ratio)
        self.image_w = to_int(self.image_w)
        self.image_h = to_int(self.image_h)
