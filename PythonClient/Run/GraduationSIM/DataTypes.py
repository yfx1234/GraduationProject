from __future__ import annotations

import base64
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

try:
    import cv2
except ImportError:
    cv2 = None


JsonDict = Dict[str, Any]


def _coerce_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _coerce_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)


def _coerce_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on", "ok"):
        return True
    if text in ("0", "false", "no", "off"):
        return False
    return default


def _coerce_tuple3(value: Any, default: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> Tuple[float, float, float]:
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)) and len(value) >= 3:
        return (_coerce_float(value[0]), _coerce_float(value[1]), _coerce_float(value[2]))
    return default


@dataclass(slots=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    @classmethod
    def from_position_and_yaw(
        cls,
        position: Sequence[float],
        yaw: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
    ) -> "Pose":
        xyz = _coerce_tuple3(position)
        return cls(x=xyz[0], y=xyz[1], z=xyz[2], roll=roll, pitch=pitch, yaw=yaw)

    @classmethod
    def from_dict(cls, payload: Optional[JsonDict]) -> "Pose":
        payload = payload or {}
        return cls(
            x=_coerce_float(payload.get("x")),
            y=_coerce_float(payload.get("y")),
            z=_coerce_float(payload.get("z")),
            roll=_coerce_float(payload.get("roll")),
            pitch=_coerce_float(payload.get("pitch")),
            yaw=_coerce_float(payload.get("yaw")),
        )

    def position_tuple(self) -> Tuple[float, float, float]:
        return (float(self.x), float(self.y), float(self.z))

    def rotation_tuple(self) -> Tuple[float, float, float]:
        return (float(self.roll), float(self.pitch), float(self.yaw))

    def to_spawn_dict(self) -> JsonDict:
        return {
            "x": float(self.x),
            "y": float(self.y),
            "z": float(self.z),
            "roll": float(self.roll),
            "pitch": float(self.pitch),
            "yaw": float(self.yaw),
        }

    def to_dict(self) -> JsonDict:
        return self.to_spawn_dict()


@dataclass(slots=True)
class ConnectionConfig:
    host: str = "127.0.0.1"
    port: int = 9000
    timeout: float = 10.0
    auto_reconnect: bool = True
    reconnect_delay_s: float = 0.2


@dataclass(slots=True)
class DroneConfig:
    actor_id: str
    label: str
    role: str
    spawn_pose: Pose = field(default_factory=Pose)
    altitude: float = 7.0
    classname: Optional[str] = None
    camera_pitch: float = 0.0
    camera_yaw: float = 0.0


@dataclass(slots=True)
class GuidanceConfig:
    actor_id: str = "guidance_0"
    classname: str = "GuidanceActor"
    label: str = "Guidance"
    spawn_pose: Pose = field(default_factory=Pose)


@dataclass(slots=True)
class TrajectoryConfig:
    kind: str = "circle"
    speed: float = 6.0
    turn_rate: float = 0.30
    vertical_amp: float = 0.0
    lead_time: float = 5.0
    radius: Optional[float] = None
    center: Optional[Tuple[float, float, float]] = None
    line_endpoint: Optional[Tuple[float, float, float]] = None
    figure8_radius: Optional[float] = None
    waypoints: List[Pose] = field(default_factory=list)


@dataclass(slots=True)
class VisualConfig:
    method: str = "vision_pid_kalman"
    model_path: Optional[str] = None
    conf: float = 0.35
    iou: float = 0.45
    imgsz: int = 640
    device: Optional[str] = None
    class_id: Optional[int] = 0
    max_det: int = 5
    selection_strategy: str = "heuristic"
    desired_area: Optional[float] = None
    capture_area: Optional[float] = None
    center_tol_x: Optional[float] = None
    center_tol_y: Optional[float] = None
    capture_hold_frames: Optional[int] = None
    lost_to_search_frames: Optional[int] = None
    max_forward_speed: Optional[float] = None
    max_reverse_speed: Optional[float] = None
    max_vertical_speed: Optional[float] = None
    max_yaw_rate_deg: Optional[float] = None
    ram_area_target: Optional[float] = None
    min_ram_speed: Optional[float] = None
    intercept_distance: float = 1.5
    search_cam_yaw_limit_deg: Optional[float] = None
    search_cam_rate_deg: Optional[float] = None
    search_body_yaw_rate_deg: Optional[float] = None
    search_cam_pitch_deg: Optional[float] = None
    search_vz_amp: Optional[float] = None
    use_kalman: Optional[bool] = None
    stop_on_capture: bool = False


@dataclass(slots=True)
class RuntimeConfig:
    hz: float = 12.0
    max_time: float = 120.0
    telemetry_interval: float = 0.5
    show: bool = True
    clean_existing: bool = True
    keep_actors: bool = False
    max_transient_errors: int = 3
    image_type: str = "scene"
    image_quality: int = 82
    max_depth_m: float = 200.0
    window_name: str = "graduation_visual_intercept"


@dataclass(slots=True)
class TaskConfig:
    connection: ConnectionConfig = field(default_factory=ConnectionConfig)
    interceptor: DroneConfig = field(
        default_factory=lambda: DroneConfig(
            actor_id="drone_0",
            label="Interceptor",
            role="interceptor",
            spawn_pose=Pose(0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
            altitude=7.0,
        )
    )
    target: DroneConfig = field(
        default_factory=lambda: DroneConfig(
            actor_id="drone_1",
            label="Target",
            role="target",
            spawn_pose=Pose(35.0, 12.0, 1.0, 0.0, 0.0, 180.0),
            altitude=8.0,
        )
    )
    guidance: GuidanceConfig = field(default_factory=GuidanceConfig)
    trajectory: TrajectoryConfig = field(default_factory=TrajectoryConfig)
    visual: VisualConfig = field(default_factory=VisualConfig)
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)

    def to_dict(self) -> JsonDict:
        return asdict(self)


@dataclass(slots=True)
class ImagePacket:
    width: int = 0
    height: int = 0
    data: str = ""
    image_type: str = "scene"

    @classmethod
    def from_response(cls, payload: Optional[JsonDict], image_type: str = "scene") -> "ImagePacket":
        payload = payload or {}
        return cls(
            width=_coerce_int(payload.get("width")),
            height=_coerce_int(payload.get("height")),
            data=str(payload.get("data", "") or ""),
            image_type=str(payload.get("image_type", image_type) or image_type),
        )

    def decode_bgr(self):
        if cv2 is None or not self.data:
            return None
        try:
            import numpy as np
        except ImportError:
            return None
        try:
            raw = base64.b64decode(self.data)
        except Exception:
            return None
        array = np.frombuffer(raw, dtype=np.uint8)
        return cv2.imdecode(array, cv2.IMREAD_COLOR)


@dataclass(slots=True)
class DetectionCandidate:
    x1: float
    y1: float
    x2: float
    y2: float
    cx: float
    cy: float
    area: float
    area_ratio: float
    conf: float
    class_id: int
    label: str
    cx_norm: float = 0.0
    cy_norm: float = 0.0
    shadow_penalty: float = 0.0
    selected: bool = False
    selection_score: float = 0.0
    selection_reason: str = ""


@dataclass(slots=True)
class DetectionResult:
    has_detection: bool = False
    bbox_xyxy: Optional[List[float]] = None
    cx: float = 0.0
    cy: float = 0.0
    area: float = 0.0
    area_ratio: float = 0.0
    conf: float = 0.0
    class_id: int = -1
    label: str = ""
    image_w: int = 0
    image_h: int = 0
    selection_score: float = 0.0
    selection_reason: str = ""
    candidates: List[DetectionCandidate] = field(default_factory=list)

    @classmethod
    def empty(cls, image_w: int = 0, image_h: int = 0) -> "DetectionResult":
        return cls(image_w=image_w, image_h=image_h)


@dataclass(slots=True)
class GuidanceState:
    status: str = "error"
    state: str = "UNKNOWN"
    valid: bool = False
    captured: bool = False
    detections: int = 0
    lost_count: int = 0
    target_distance: float = -1.0
    cmd_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    control: JsonDict = field(default_factory=dict)
    raw: JsonDict = field(default_factory=dict)

    @classmethod
    def from_response(cls, payload: Optional[JsonDict]) -> "GuidanceState":
        payload = payload or {}
        control = payload.get("control", {})
        if not isinstance(control, dict):
            control = {}
        status_value = payload.get("status", "error")
        if isinstance(status_value, bool):
            status = "ok" if status_value else "error"
        else:
            status = str(status_value)
        return cls(
            status=status,
            state=str(payload.get("state", "UNKNOWN")),
            valid=_coerce_bool(payload.get("valid")),
            captured=_coerce_bool(payload.get("captured")),
            detections=_coerce_int(payload.get("detections")),
            lost_count=_coerce_int(payload.get("lost_count")),
            target_distance=_coerce_float(payload.get("target_distance"), -1.0),
            cmd_velocity=_coerce_tuple3(payload.get("cmd_velocity")),
            control=control,
            raw=payload,
        )


@dataclass(slots=True)
class TrajectoryReference:
    pose: Pose
    speed: float

    def position_tuple(self) -> Tuple[float, float, float]:
        return self.pose.position_tuple()


@dataclass(slots=True)
class DroneSnapshot:
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    camera_pitch: float = 0.0
    camera_yaw: float = 0.0
    api_control: bool = False
    raw: JsonDict = field(default_factory=dict)

    @classmethod
    def from_state(cls, payload: Optional[JsonDict]) -> "DroneSnapshot":
        payload = payload or {}
        orientation = payload.get("orientation", {})
        if not isinstance(orientation, dict):
            orientation = {}
        return cls(
            position=_coerce_tuple3(payload.get("position")),
            velocity=_coerce_tuple3(payload.get("velocity")),
            roll=_coerce_float(orientation.get("roll")),
            pitch=_coerce_float(orientation.get("pitch")),
            yaw=_coerce_float(orientation.get("yaw")),
            camera_pitch=_coerce_float(payload.get("camera_pitch")),
            camera_yaw=_coerce_float(payload.get("camera_yaw")),
            api_control=_coerce_bool(payload.get("api_control")),
            raw=payload,
        )


@dataclass(slots=True)
class AutoLabel:
    valid: bool = False
    class_id: int = 0
    bbox_xyxy: Optional[List[int]] = None
    bbox_yolo: Optional[List[float]] = None
    pixel_count: int = 0
    area_ratio: float = 0.0
    image_w: int = 0
    image_h: int = 0


@dataclass(slots=True)
class CollectionBounds:
    x_min: float = -150.0
    x_max: float = 150.0
    y_min: float = -150.0
    y_max: float = 150.0
    z_min: float = 6.0
    z_max: float = 24.0


@dataclass(slots=True)
class CollectionConfig:
    connection: ConnectionConfig = field(default_factory=ConnectionConfig)
    interceptor: DroneConfig = field(
        default_factory=lambda: DroneConfig(
            actor_id="collect_drone_0",
            label="Collector",
            role="collector",
            spawn_pose=Pose(0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
            altitude=12.0,
            camera_pitch=0.0,
            camera_yaw=0.0,
        )
    )
    target: DroneConfig = field(
        default_factory=lambda: DroneConfig(
            actor_id="collect_drone_1",
            label="Target",
            role="target",
            spawn_pose=Pose(35.0, 12.0, 1.0, 0.0, 0.0, 180.0),
            altitude=12.0,
        )
    )
    output_root: str = "PythonClient/Run/datasets/raw"
    session_name: str = ""
    duration_s: float = 300.0
    max_samples: int = 0
    control_hz: float = 12.0
    sample_hz: float = 4.0
    telemetry_interval_s: float = 1.0
    seed: int = 1234
    clean_existing: bool = True
    keep_actors: bool = False
    show: bool = True
    image_quality: int = 95
    max_depth_m: float = 200.0
    save_segmentation: bool = True
    save_empty_labels: bool = False
    target_segmentation_id: int = 30
    interceptor_segmentation_id: int = 20
    random_bounds: CollectionBounds = field(default_factory=CollectionBounds)
    target_speed_min: float = 6.0
    target_speed_max: float = 18.0
    waypoint_reach_radius: float = 4.0
    waypoint_timeout_s: float = 14.0
    interceptor_max_speed: float = 20.0
    distance_min: float = 4.0
    distance_max: float = 140.0
    relative_altitude_min: float = -12.0
    relative_altitude_max: float = 20.0
    composition_hold_min_s: float = 2.0
    composition_hold_max_s: float = 6.0
    body_yaw_bias_max_deg: float = 18.0
    frame_yaw_bias_max_deg: float = 20.0
    frame_pitch_bias_max_deg: float = 14.0
    camera_yaw_limit_deg: float = 35.0
    camera_pitch_min_deg: float = -35.0
    camera_pitch_max_deg: float = 25.0
    min_mask_pixels: int = 20

    def to_dict(self) -> JsonDict:
        return asdict(self)


@dataclass(slots=True)
class CollectionResult:
    session_dir: str
    exit_reason: str
    saved_frames: int = 0
    labeled_frames: int = 0
    empty_frames: int = 0
    last_response: JsonDict = field(default_factory=dict)

    def to_dict(self) -> JsonDict:
        return {
            "session_dir": self.session_dir,
            "exit_reason": self.exit_reason,
            "saved_frames": self.saved_frames,
            "labeled_frames": self.labeled_frames,
            "empty_frames": self.empty_frames,
            "last_response": self.last_response,
        }


@dataclass(slots=True)
class TaskResult:
    exit_reason: str
    captured: bool = False
    last_response: JsonDict = field(default_factory=dict)

    def to_dict(self) -> JsonDict:
        return {
            "exit_reason": self.exit_reason,
            "captured": self.captured,
            "last_response": self.last_response,
        }
