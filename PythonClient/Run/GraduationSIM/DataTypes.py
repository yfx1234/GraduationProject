from __future__ import annotations

import base64
from typing import Any

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
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


def to_vec3(
    value: Any,
    default: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[float, float, float]:
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return (to_float(value[0]), to_float(value[1]), to_float(value[2]))
    return default


# 这些类只负责承载协议数据，保持简单直接。
class Pose:
    def __init__(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.roll = float(roll)
        self.pitch = float(pitch)
        self.yaw = float(yaw)

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


class ImagePacket:
    def __init__(
        self,
        width: int = 0,
        height: int = 0,
        data: str = "",
        image_type: str = "scene",
    ) -> None:
        self.width = int(width)
        self.height = int(height)
        self.data = str(data or "")
        self.image_type = str(image_type or "scene")

    @classmethod
    def from_response(cls, payload: JsonDict | None, image_type: str = "scene") -> "ImagePacket":
        payload = payload or {}
        return cls(
            width=to_int(payload.get("width")),
            height=to_int(payload.get("height")),
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


class DetectionCandidate:
    def __init__(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        cx: float,
        cy: float,
        area: float,
        area_ratio: float,
        conf: float,
        class_id: int,
        label: str,
        cx_norm: float = 0.0,
        cy_norm: float = 0.0,
        shadow_penalty: float = 0.0,
        selected: bool = False,
        selection_score: float = 0.0,
        selection_reason: str = "",
    ) -> None:
        self.x1 = float(x1)
        self.y1 = float(y1)
        self.x2 = float(x2)
        self.y2 = float(y2)
        self.cx = float(cx)
        self.cy = float(cy)
        self.area = float(area)
        self.area_ratio = float(area_ratio)
        self.conf = float(conf)
        self.class_id = int(class_id)
        self.label = str(label)
        self.cx_norm = float(cx_norm)
        self.cy_norm = float(cy_norm)
        self.shadow_penalty = float(shadow_penalty)
        self.selected = bool(selected)
        self.selection_score = float(selection_score)
        self.selection_reason = str(selection_reason)


class DetectionResult:
    def __init__(
        self,
        has_detection: bool = False,
        bbox_xyxy: list[float] | None = None,
        cx: float = 0.0,
        cy: float = 0.0,
        area: float = 0.0,
        area_ratio: float = 0.0,
        conf: float = 0.0,
        class_id: int = -1,
        label: str = "",
        image_w: int = 0,
        image_h: int = 0,
        selection_score: float = 0.0,
        selection_reason: str = "",
        candidates: list[DetectionCandidate] | None = None,
    ) -> None:
        self.has_detection = bool(has_detection)
        self.bbox_xyxy = list(bbox_xyxy) if bbox_xyxy is not None else None
        self.cx = float(cx)
        self.cy = float(cy)
        self.area = float(area)
        self.area_ratio = float(area_ratio)
        self.conf = float(conf)
        self.class_id = int(class_id)
        self.label = str(label)
        self.image_w = int(image_w)
        self.image_h = int(image_h)
        self.selection_score = float(selection_score)
        self.selection_reason = str(selection_reason)
        self.candidates = list(candidates or [])

    @classmethod
    def empty(cls, image_w: int = 0, image_h: int = 0) -> "DetectionResult":
        return cls(image_w=image_w, image_h=image_h)

    def __bool__(self) -> bool:
        return self.has_detection or bool(self.candidates)


class GuidanceState:
    def __init__(
        self,
        status: str = "error",
        mode: str = "",
        state: str = "UNKNOWN",
        enabled: bool = False,
        valid: bool = False,
        captured: bool = False,
        detections: int = 0,
        lost_count: int = 0,
        capture_count: int = 0,
        frame: int = 0,
        interceptor_id: str = "",
        target_id: str = "",
        method: str = "",
        target_distance: float = -1.0,
        cmd_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0),
        control: JsonDict | None = None,
        raw: JsonDict | None = None,
    ) -> None:
        self.status = str(status)
        self.mode = str(mode)
        self.state = str(state)
        self.enabled = bool(enabled)
        self.valid = bool(valid)
        self.captured = bool(captured)
        self.detections = int(detections)
        self.lost_count = int(lost_count)
        self.capture_count = int(capture_count)
        self.frame = int(frame)
        self.interceptor_id = str(interceptor_id)
        self.target_id = str(target_id)
        self.method = str(method)
        self.target_distance = float(target_distance)
        self.cmd_velocity = to_vec3(cmd_velocity)
        self.control = dict(control or {})
        self.raw = dict(raw or {})

    @classmethod
    def from_response(cls, payload: JsonDict | None) -> "GuidanceState":
        payload = payload or {}
        control = payload.get("control") if isinstance(payload.get("control"), dict) else {}
        status_value = payload.get("status", "error")
        if status_value is True:
            status = "ok"
        elif status_value is False:
            status = "error"
        else:
            status = str(status_value)
        return cls(
            status=status,
            mode=payload.get("mode", ""),
            state=payload.get("state", "UNKNOWN"),
            enabled=to_bool(payload.get("enabled")),
            valid=to_bool(payload.get("valid")),
            captured=to_bool(payload.get("captured")),
            detections=to_int(payload.get("detections")),
            lost_count=to_int(payload.get("lost_count")),
            capture_count=to_int(payload.get("capture_count")),
            frame=to_int(payload.get("frame")),
            interceptor_id=payload.get("interceptor_id", ""),
            target_id=payload.get("target_id", ""),
            method=payload.get("method", ""),
            target_distance=to_float(payload.get("target_distance"), -1.0),
            cmd_velocity=to_vec3(payload.get("cmd_velocity")),
            control=control,
            raw=payload,
        )


class TrajectoryReference:
    def __init__(self, pose: Pose | None = None, speed: float = 0.0) -> None:
        self.pose = pose if pose is not None else Pose()
        self.speed = float(speed)

    def position_tuple(self) -> tuple[float, float, float]:
        return self.pose.position_tuple()


class DroneSnapshot:
    def __init__(
        self,
        position: tuple[float, float, float] = (0.0, 0.0, 0.0),
        velocity: tuple[float, float, float] = (0.0, 0.0, 0.0),
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        camera_pitch: float = 0.0,
        camera_yaw: float = 0.0,
        api_control: bool = False,
        raw: JsonDict | None = None,
    ) -> None:
        self.position = to_vec3(position)
        self.velocity = to_vec3(velocity)
        self.roll = float(roll)
        self.pitch = float(pitch)
        self.yaw = float(yaw)
        self.camera_pitch = float(camera_pitch)
        self.camera_yaw = float(camera_yaw)
        self.api_control = bool(api_control)
        self.raw = dict(raw or {})

    @classmethod
    def from_state(cls, payload: JsonDict | None) -> "DroneSnapshot":
        payload = payload or {}
        orientation = payload.get("orientation") if isinstance(payload.get("orientation"), dict) else {}
        return cls(
            position=to_vec3(payload.get("position")),
            velocity=to_vec3(payload.get("velocity")),
            roll=to_float(orientation.get("roll")),
            pitch=to_float(orientation.get("pitch")),
            yaw=to_float(orientation.get("yaw")),
            camera_pitch=to_float(payload.get("camera_pitch")),
            camera_yaw=to_float(payload.get("camera_yaw")),
            api_control=to_bool(payload.get("api_control")),
            raw=payload,
        )


class AutoLabel:
    def __init__(
        self,
        valid: bool = False,
        class_id: int = 0,
        bbox_xyxy: list[int] | None = None,
        bbox_yolo: list[float] | None = None,
        pixel_count: int = 0,
        area_ratio: float = 0.0,
        image_w: int = 0,
        image_h: int = 0,
    ) -> None:
        self.valid = bool(valid)
        self.class_id = int(class_id)
        self.bbox_xyxy = list(bbox_xyxy) if bbox_xyxy is not None else None
        self.bbox_yolo = list(bbox_yolo) if bbox_yolo is not None else None
        self.pixel_count = int(pixel_count)
        self.area_ratio = float(area_ratio)
        self.image_w = int(image_w)
        self.image_h = int(image_h)
