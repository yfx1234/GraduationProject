from __future__ import annotations

import math
import os
import sys
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace as Namespace

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
DEFAULT_DRONE_CLASS = "/Game/Blueprints/BP_Drone.BP_Drone_C"
RUN_NAME_PREFIX = "drone_detect"
DRONE_HALF_EXTENTS = (0.55, 0.55, 0.22)


@dataclass(frozen=True)
class ResolvedPaths:
    project_root: Path
    run_root: Path
    yolo_root: Path
    dataset_root: Path
    results_root: Path
    weights_root: Path
    runs_root: Path
    detect_runs_root: Path
    ultralytics_source_root: Path
    ultralytics_settings_root: Path


def clamp(value, low, high):
    return max(low, min(high, value))


def resolve_paths(anchor: str | Path | None = None) -> ResolvedPaths:
    anchor_path = Path(anchor or __file__).resolve()
    search_root = anchor_path if anchor_path.is_dir() else anchor_path.parent
    project_root = next(
        (
            candidate
            for candidate in (search_root, *search_root.parents)
            if (candidate / "PythonClient" / "Run").exists() and (candidate / "PythonClient" / "YOLO").exists()
        ),
        None,
    )
    if project_root is None:
        project_root = anchor_path.parents[3] if len(anchor_path.parents) > 3 else anchor_path.parent
    yolo_root = project_root / "PythonClient" / "YOLO"
    runs_root = yolo_root / "runs"
    return ResolvedPaths(
        project_root=project_root,
        run_root=project_root / "PythonClient" / "Run",
        yolo_root=yolo_root,
        dataset_root=yolo_root / "dataset",
        results_root=yolo_root / "results",
        weights_root=yolo_root / "weights",
        runs_root=runs_root,
        detect_runs_root=runs_root / "detect",
        ultralytics_source_root=yolo_root / "ultralytics",
        ultralytics_settings_root=project_root / "PythonClient" / ".ultralytics_config",
    )


def resolve_local_path(raw: str | Path | None, root: Path) -> Path | None:
    if not raw:
        return None
    path = Path(str(raw))
    return path if path.is_absolute() else root / path


def latest_file(paths: Iterable[Path]) -> Path | None:
    latest_path = None
    latest_time = float("-inf")
    for path in paths:
        if not path.is_file():
            continue
        modified_time = path.stat().st_mtime
        if modified_time >= latest_time:
            latest_path = path
            latest_time = modified_time
    return latest_path


def parse_run_number(name: str, prefix: str = RUN_NAME_PREFIX) -> int:
    if not name.startswith(prefix):
        return 0
    suffix = name[len(prefix) :]
    if not suffix:
        return 1
    return int(suffix) if suffix.isdigit() else 0


def next_run_name(paths: ResolvedPaths, prefix: str = RUN_NAME_PREFIX) -> str:
    run_numbers = []
    for root in (paths.runs_root, paths.detect_runs_root):
        if not root.exists():
            continue
        for item in root.iterdir():
            if item.is_dir():
                run_number = parse_run_number(item.name, prefix)
                if run_number > 0:
                    run_numbers.append(run_number)
    next_number = max(run_numbers, default=0) + 1
    return prefix if next_number <= 1 else f"{prefix}{next_number}"


def pick_model_path(paths: ResolvedPaths, configured_model: str | Path | None, error_text: str) -> Path:
    direct_model = resolve_local_path(configured_model, paths.project_root)
    for candidate in (
        direct_model,
        latest_file(paths.runs_root.glob(f"{RUN_NAME_PREFIX}*/weights/best.pt")),
        latest_file(paths.detect_runs_root.glob("*/weights/best.pt")),
        latest_file(paths.weights_root.glob("*.pt")),
    ):
        if candidate and candidate.exists():
            return candidate
    raise RuntimeError(error_text)


def pick_runtime_model_path(paths: ResolvedPaths, configured_model: str | Path | None = None) -> Path:
    return pick_model_path(paths, configured_model, f"no runtime model found under {paths.runs_root} or {paths.detect_runs_root}")


def pick_training_model_path(paths: ResolvedPaths, configured_model: str | Path | None = None) -> Path:
    return pick_model_path(paths, configured_model, f"no training weights found in: {paths.weights_root} or {paths.runs_root}")


def ensure_ultralytics_import_path(project_root: Path) -> None:
    source_root = project_root / "PythonClient" / "YOLO" / "ultralytics"
    settings_root = project_root / "PythonClient" / ".ultralytics_config"
    settings_root.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("YOLO_CONFIG_DIR", str(settings_root))
    os.environ.setdefault("YOLO_AUTOINSTALL", "False")
    os.environ.setdefault("YOLO_VERBOSE", "False")
    source_text = str(source_root)
    if source_text not in sys.path:
        sys.path.insert(0, source_text)


def AutoLabel(valid=False, class_id=0, bbox_xyxy=None, bbox_yolo=None, pixel_count=0, area_ratio=0.0, image_w=0, image_h=0):
    return Namespace(
        valid=bool(valid),
        class_id=int(class_id),
        bbox_xyxy=None if bbox_xyxy is None else list(bbox_xyxy),
        bbox_yolo=None if bbox_yolo is None else list(bbox_yolo),
        pixel_count=int(pixel_count),
        area_ratio=float(area_ratio),
        image_w=int(image_w),
        image_h=int(image_h),
    )


def Detection(box=None, cx=0.0, cy=0.0, area=0.0, ratio=0.0, conf=0.0, class_id=-1, label="", width=0, height=0, has_detection=False, candidates=None):
    return Namespace(
        has_detection=bool(has_detection),
        bbox_xyxy=None if box is None else list(box),
        cx=float(cx),
        cy=float(cy),
        area=float(area),
        area_ratio=float(ratio),
        conf=float(conf),
        class_id=int(class_id),
        label=str(label),
        image_w=int(width),
        image_h=int(height),
        candidates=list(candidates or []),
    )


def dot_product(first, second):
    return float(first[0]) * float(second[0]) + float(first[1]) * float(second[1]) + float(first[2]) * float(second[2])


def rotator_axes(pitch_deg, yaw_deg, roll_deg=0.0):
    pitch = math.radians(float(pitch_deg))
    yaw = math.radians(float(yaw_deg))
    roll = math.radians(float(roll_deg))
    cos_pitch, sin_pitch = math.cos(pitch), math.sin(pitch)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    cos_roll, sin_roll = math.cos(roll), math.sin(roll)
    forward = (cos_pitch * cos_yaw, cos_pitch * sin_yaw, sin_pitch)
    right = (sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw, sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw, -sin_roll * cos_pitch)
    up = (-cos_roll * sin_pitch * cos_yaw - sin_roll * sin_yaw, -cos_roll * sin_pitch * sin_yaw + sin_roll * cos_yaw, cos_roll * cos_pitch)
    return forward, right, up


def project_world_point(camera_pos_cm, camera_rot_deg, fov_deg, image_w, image_h, world_pos_m):
    world_pos_cm = (float(world_pos_m[0]) * 100.0, float(world_pos_m[1]) * 100.0, float(world_pos_m[2]) * 100.0)
    delta = (
        world_pos_cm[0] - float(camera_pos_cm[0]),
        world_pos_cm[1] - float(camera_pos_cm[1]),
        world_pos_cm[2] - float(camera_pos_cm[2]),
    )
    forward, right, up = rotator_axes(camera_rot_deg[0], camera_rot_deg[1], camera_rot_deg[2])
    camera_x = dot_product(delta, right)
    camera_y = dot_product(delta, up)
    camera_z = dot_product(delta, forward)
    if camera_z <= 1e-3:
        return None
    focal = float(image_w) / max(1e-6, 2.0 * math.tan(math.radians(clamp(float(fov_deg), 1.0, 179.0) * 0.5)))
    return (float(image_w) * 0.5 + focal * camera_x / camera_z, float(image_h) * 0.5 - focal * camera_y / camera_z)


def build_auto_label(scene_response, target_state, image_w, image_h):
    camera_pos = scene_response.get("camera_pos", [0.0, 0.0, 0.0])
    camera_rot = scene_response.get("camera_rot", [0.0, 0.0, 0.0])
    camera_fov = float(scene_response.get("fov", 90.0) or 90.0)
    center = project_world_point(camera_pos, camera_rot, camera_fov, image_w, image_h, target_state.position)
    if center is None:
        return AutoLabel(valid=False, image_w=image_w, image_h=image_h)

    forward, right, up = rotator_axes(target_state.pitch, target_state.yaw, target_state.roll)
    half_forward, half_right, half_up = DRONE_HALF_EXTENTS
    points = [center]
    for sign_forward in (-1.0, 1.0):
        for sign_right in (-1.0, 1.0):
            for sign_up in (-1.0, 1.0):
                point = (
                    float(target_state.position[0]) + sign_forward * half_forward * forward[0] + sign_right * half_right * right[0] + sign_up * half_up * up[0],
                    float(target_state.position[1]) + sign_forward * half_forward * forward[1] + sign_right * half_right * right[1] + sign_up * half_up * up[1],
                    float(target_state.position[2]) + sign_forward * half_forward * forward[2] + sign_right * half_right * right[2] + sign_up * half_up * up[2],
                )
                image_point = project_world_point(camera_pos, camera_rot, camera_fov, image_w, image_h, point)
                if image_point is not None:
                    points.append(image_point)

    x_values = [point[0] for point in points]
    y_values = [point[1] for point in points]
    if max(x_values) < 0.0 or max(y_values) < 0.0 or min(x_values) >= float(image_w) or min(y_values) >= float(image_h):
        return AutoLabel(valid=False, image_w=image_w, image_h=image_h)

    left = max(0, int(math.floor(min(x_values) - 4.0)))
    top = max(0, int(math.floor(min(y_values) - 4.0)))
    right_edge = min(int(image_w) - 1, int(math.ceil(max(x_values) + 4.0)))
    bottom = min(int(image_h) - 1, int(math.ceil(max(y_values) + 4.0)))
    box_width = max(1, right_edge - left + 1)
    box_height = max(1, bottom - top + 1)
    area = box_width * box_height
    return AutoLabel(
        valid=True,
        class_id=0,
        bbox_xyxy=[left, top, right_edge, bottom],
        bbox_yolo=[(left + right_edge + 1) * 0.5 / image_w, (top + bottom + 1) * 0.5 / image_h, box_width / image_w, box_height / image_h],
        pixel_count=area,
        area_ratio=float(area) / max(1.0, float(image_w * image_h)),
        image_w=image_w,
        image_h=image_h,
    )


def build_label_text(label):
    if not label.valid or not label.bbox_yolo:
        return ""
    center_x, center_y, box_width, box_height = label.bbox_yolo
    return f"{int(label.class_id)} {center_x:.6f} {center_y:.6f} {box_width:.6f} {box_height:.6f}\n"


def bbox_iou(first_box, second_box):
    if not first_box or not second_box:
        return 0.0
    first_left, first_top, first_right, first_bottom = [float(value) for value in first_box]
    second_left, second_top, second_right, second_bottom = [float(value) for value in second_box]
    inter_left, inter_top = max(first_left, second_left), max(first_top, second_top)
    inter_right, inter_bottom = min(first_right, second_right), min(first_bottom, second_bottom)
    inter_width = max(0.0, inter_right - inter_left + 1.0)
    inter_height = max(0.0, inter_bottom - inter_top + 1.0)
    inter_area = inter_width * inter_height
    if inter_area <= 0.0:
        return 0.0
    first_area = max(0.0, first_right - first_left + 1.0) * max(0.0, first_bottom - first_top + 1.0)
    second_area = max(0.0, second_right - second_left + 1.0) * max(0.0, second_bottom - second_top + 1.0)
    union_area = first_area + second_area - inter_area
    return 0.0 if union_area <= 0.0 else inter_area / union_area


def ensure_label_file(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        path.write_text("", encoding="utf-8")


def list_samples(dataset_root: Path, split: str) -> list[tuple[Path, Path]]:
    images_dir = dataset_root / "images" / split
    labels_dir = dataset_root / "labels" / split
    if not images_dir.exists():
        return []
    return [
        (image_path, labels_dir / f"{image_path.stem}.txt")
        for image_path in sorted(images_dir.iterdir())
        if image_path.is_file() and image_path.suffix.lower() in IMAGE_EXTS
    ]


def move_sample(sample: tuple[Path, Path], dataset_root: Path, target_split: str) -> None:
    image_path, label_path = sample
    target_image_dir = dataset_root / "images" / target_split
    target_label_dir = dataset_root / "labels" / target_split
    target_image_dir.mkdir(parents=True, exist_ok=True)
    target_label_dir.mkdir(parents=True, exist_ok=True)
    image_path.replace(target_image_dir / image_path.name)
    if label_path.exists():
        label_path.replace(target_label_dir / label_path.name)
    else:
        (target_label_dir / label_path.name).write_text("", encoding="utf-8")


def write_data_yaml(dataset_root: Path, use_train_as_val: bool = False) -> Path:
    yaml_path = dataset_root / "data.yaml"
    yaml_path.write_text(
        "path: " + dataset_root.as_posix() + "\n"
        "train: images/train\n"
        f"val: {'images/train' if use_train_as_val else 'images/val'}\n\n"
        "nc: 1\n"
        "names:\n"
        "  0: drone\n",
        encoding="utf-8",
    )
    return yaml_path


def ensure_dataset_layout(root: Path) -> None:
    for split in ("train", "val"):
        (root / "images" / split).mkdir(parents=True, exist_ok=True)
        (root / "labels" / split).mkdir(parents=True, exist_ok=True)
    write_data_yaml(root)


def next_index(root: Path) -> int:
    next_value = -1
    for split in ("train", "val"):
        for path in (root / "images" / split).glob("*.*"):
            if path.suffix.lower() in IMAGE_EXTS and path.stem.isdigit():
                next_value = max(next_value, int(path.stem))
    return next_value + 1


def count_images(root: Path) -> dict[str, int]:
    return {
        split: sum(1 for path in (root / "images" / split).glob("*.*") if path.suffix.lower() in IMAGE_EXTS)
        for split in ("train", "val")
    }


def choose_split(counts: dict[str, int], val_ratio: float) -> str:
    ratio = max(0.0, min(float(val_ratio), 0.95))
    if ratio <= 0.0:
        split = "train"
    else:
        total_after_save = counts["train"] + counts["val"] + 1
        desired_val = int(total_after_save * ratio + 0.5)
        if total_after_save > 1:
            desired_val = min(desired_val, total_after_save - 1)
        split = "val" if counts["val"] < desired_val else "train"
    counts[split] += 1
    return split


def prepare_dataset(dataset_root: Path) -> tuple[Path, int, int]:
    ensure_dataset_layout(dataset_root)
    train_samples = list_samples(dataset_root, "train")
    val_samples = list_samples(dataset_root, "val")
    if not train_samples and val_samples:
        move_sample(val_samples[0], dataset_root, "train")
    train_samples = list_samples(dataset_root, "train")
    val_samples = list_samples(dataset_root, "val")
    if not val_samples and len(train_samples) > 1:
        move_sample(train_samples[-1], dataset_root, "val")
    for split in ("train", "val"):
        for _, label_path in list_samples(dataset_root, split):
            ensure_label_file(label_path)
    train_count = len(list_samples(dataset_root, "train"))
    val_count = len(list_samples(dataset_root, "val"))
    if train_count <= 0:
        raise RuntimeError(f"dataset has no training images: {dataset_root}")
    return write_data_yaml(dataset_root, use_train_as_val=(val_count <= 0)), train_count, val_count


def scan_split(dataset_root: Path, split: str) -> dict[str, float | int]:
    stats: dict[str, float | int] = {
        "images": 0,
        "labeled_images": 0,
        "empty_labels": 0,
        "missing_labels": 0,
        "invalid_labels": 0,
        "boxes": 0,
        "min_area_ratio": 0.0,
        "max_area_ratio": 0.0,
        "mean_area_ratio": 0.0,
    }
    area_ratios = []
    for _, label_path in list_samples(dataset_root, split):
        stats["images"] += 1
        if not label_path.exists():
            stats["missing_labels"] += 1
            ensure_label_file(label_path)
            stats["empty_labels"] += 1
            continue
        text = label_path.read_text(encoding="utf-8").strip()
        if not text:
            stats["empty_labels"] += 1
            continue
        valid_image = False
        for raw_line in text.splitlines():
            parts = raw_line.strip().split()
            if len(parts) != 5:
                stats["invalid_labels"] += 1
                continue
            try:
                _, _, _, box_width, box_height = [float(item) for item in parts]
            except ValueError:
                stats["invalid_labels"] += 1
                continue
            if box_width <= 0.0 or box_height <= 0.0 or box_width > 1.0 or box_height > 1.0:
                stats["invalid_labels"] += 1
                continue
            valid_image = True
            stats["boxes"] += 1
            area_ratios.append(box_width * box_height)
        if valid_image:
            stats["labeled_images"] += 1
    if area_ratios:
        stats["min_area_ratio"] = min(area_ratios)
        stats["max_area_ratio"] = max(area_ratios)
        stats["mean_area_ratio"] = sum(area_ratios) / len(area_ratios)
    return stats


def scan_dataset(dataset_root: Path) -> dict[str, object]:
    train_stats = scan_split(dataset_root, "train")
    val_stats = scan_split(dataset_root, "val")
    return {"train": train_stats, "val": val_stats, "total_boxes": int(train_stats["boxes"] + val_stats["boxes"])}


class YoloDetector:
    def __init__(self, model_path=None, conf=0.35, iou=0.45, imgsz=640, device=None, class_id=0, max_det=5) -> None:
        self.paths = resolve_paths(__file__)
        self.model_path = Path(model_path) if model_path else self.paths.weights_root / "yolo26n.pt"
        self.conf = float(conf)
        self.iou = float(iou)
        self.imgsz = int(imgsz)
        self.device = None if device in (None, "", "auto") else str(device)
        self.class_id = None if class_id is None else int(class_id)
        self.max_det = max(1, int(max_det))
        ensure_ultralytics_import_path(self.paths.project_root)
        from ultralytics import YOLO

        self.model = YOLO(str(self.model_path))
        names = getattr(self.model, "names", {})
        self.names = names if isinstance(names, dict) else {}

    def detect_bgr(self, frame):
        if frame is None:
            return Detection()
        classes = None if self.class_id is None else [self.class_id]
        results = self.model.predict(
            source=frame,
            conf=self.conf,
            iou=self.iou,
            imgsz=self.imgsz,
            device=self.device,
            classes=classes,
            max_det=self.max_det,
            save=False,
            verbose=False,
        )
        if not results:
            return Detection(width=frame.shape[1], height=frame.shape[0])
        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) <= 0:
            return Detection(width=frame.shape[1], height=frame.shape[0])
        image_height, image_width = frame.shape[:2]
        confidences = boxes.conf.detach().cpu().numpy()
        boxes_xyxy = boxes.xyxy.detach().cpu().numpy()
        class_array = None if getattr(boxes, "cls", None) is None else boxes.cls.detach().cpu().numpy()
        image_area = max(1.0, float(image_width * image_height))
        candidates = []
        for index, item in enumerate(boxes_xyxy):
            left, top, right_edge, bottom = [float(value) for value in item.tolist()]
            box_width = max(0.0, right_edge - left)
            box_height = max(0.0, bottom - top)
            class_id = int(class_array[index]) if class_array is not None else -1
            candidates.append(
                Detection(
                    box=[left, top, right_edge, bottom],
                    cx=0.5 * (left + right_edge),
                    cy=0.5 * (top + bottom),
                    area=box_width * box_height,
                    ratio=(box_width * box_height) / image_area,
                    conf=float(confidences[index]),
                    class_id=class_id,
                    label=self.names.get(class_id, str(class_id)),
                    width=image_width,
                    height=image_height,
                )
            )
        selected = max(candidates, key=lambda item: (item.conf, item.area), default=None)
        if selected is None:
            return Detection(width=image_width, height=image_height)
        return Detection(
            box=selected.bbox_xyxy,
            cx=selected.cx,
            cy=selected.cy,
            area=selected.area,
            ratio=selected.area_ratio,
            conf=selected.conf,
            class_id=selected.class_id,
            label=selected.label,
            width=image_width,
            height=image_height,
            has_detection=True,
            candidates=candidates,
        )


__all__ = [
    "AutoLabel",
    "DEFAULT_DRONE_CLASS",
    "Detection",
    "ResolvedPaths",
    "YoloDetector",
    "bbox_iou",
    "build_auto_label",
    "build_label_text",
    "choose_split",
    "count_images",
    "ensure_dataset_layout",
    "ensure_ultralytics_import_path",
    "latest_file",
    "next_index",
    "next_run_name",
    "parse_run_number",
    "pick_runtime_model_path",
    "pick_training_model_path",
    "prepare_dataset",
    "resolve_local_path",
    "resolve_paths",
    "scan_dataset",
]
