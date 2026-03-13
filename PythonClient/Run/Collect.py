from __future__ import annotations

import argparse
import base64
import json
import math
import random
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import numpy as np
except ImportError:
    np = None

from GraduationSIM import (
    DEFAULT_DRONE_CLASS,
    AgentBase,
    AgentDrone,
    CollectionBounds,
    CollectionConfig,
    CollectionResult,
    ConnectionConfig,
    DroneConfig,
    DroneSnapshot,
    Pose,
    TCPClient,
)


@dataclass(slots=True)
class _Waypoint:
    position: Tuple[float, float, float]
    speed: float
    yaw_deg: float
    expires_at: float


@dataclass(slots=True)
class _Composition:
    desired_distance: float
    azimuth_deg: float
    relative_altitude: float
    body_yaw_bias_deg: float
    frame_yaw_bias_deg: float
    frame_pitch_bias_deg: float
    expires_at: float


def _ok(response) -> bool:
    return AgentBase.is_ok(response)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _wrap_deg(angle_deg: float) -> float:
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def _distance(a, b) -> float:
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    dz = float(a[2]) - float(b[2])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _distance_bins(distance_min: float, distance_max: float, count: int = 5):
    lo = max(1.0, float(distance_min))
    hi = max(lo + 1.0, float(distance_max))
    if count < 2:
        return [(lo, hi)]
    ratio = hi / lo
    edges = [lo]
    for index in range(1, count):
        edges.append(lo * (ratio ** (index / float(count))))
    edges.append(hi)
    return [(edges[i], max(edges[i] + 0.5, edges[i + 1])) for i in range(len(edges) - 1)]


def _segmentation_color_bgr(segmentation_id: int):
    seg_id = max(0, min(255, int(segmentation_id)))
    if seg_id == 0:
        return (0, 0, 0)
    red = max(16, (seg_id * 37 + 13) % 255)
    green = max(16, (seg_id * 73 + 47) % 255)
    blue = max(16, (seg_id * 109 + 91) % 255)
    return (blue, green, red)


def _resolve_path(root: Path, path_text: str) -> Path:
    path = Path(path_text)
    if not path.is_absolute():
        path = root / path
    return path.resolve()


def _snapshot(drone: AgentDrone) -> DroneSnapshot:
    response = drone.get_state(frame="ue")
    return DroneSnapshot.from_state(response if isinstance(response, dict) else {})


def _wait_for_altitude(drone: AgentDrone, altitude: float, timeout_s: float = 20.0, tolerance_m: float = 0.8) -> bool:
    deadline = time.time() + max(1.0, timeout_s)
    while time.time() < deadline:
        if _snapshot(drone).position[2] >= float(altitude) - tolerance_m:
            return True
        time.sleep(0.25)
    return False


def _ensure_removed(client: TCPClient, actor_id: str) -> None:
    response = AgentBase.unwrap_response(client.request({"remove_actor": {"actor_id": actor_id}}), "remove_actor_return")
    if _ok(response):
        print(f"[INFO] removed existing actor: {actor_id}")


def _spawn_drone(drone: AgentDrone, drone_config) -> bool:
    drone.mission_role = drone_config.role
    drone.label = drone_config.label
    primary_class = drone_config.classname or DEFAULT_DRONE_CLASS
    response = drone.create_raw(pose=drone_config.spawn_pose, classname=primary_class, label=drone_config.label)
    if _ok(response):
        return True
    if primary_class != "DronePawn":
        fallback = drone.create_raw(pose=drone_config.spawn_pose, classname="DronePawn", label=drone_config.label)
        if _ok(fallback):
            print("[WARN] fallback class 'DronePawn' used")
            return True
    print(f"[ERROR] spawn failed for {drone.actor_id}: {response}")
    return False


def _sample_waypoint(config: CollectionConfig, rng: random.Random, current_position, now_s: float) -> _Waypoint:
    bounds = config.random_bounds
    position = (
        rng.uniform(bounds.x_min, bounds.x_max),
        rng.uniform(bounds.y_min, bounds.y_max),
        rng.uniform(bounds.z_min, bounds.z_max),
    )
    speed = rng.uniform(config.target_speed_min, config.target_speed_max)
    dx = position[0] - float(current_position[0])
    dy = position[1] - float(current_position[1])
    yaw_deg = math.degrees(math.atan2(dy, dx)) if (abs(dx) + abs(dy)) > 1e-6 else 0.0
    return _Waypoint(position=position, speed=speed, yaw_deg=yaw_deg, expires_at=now_s + max(2.0, float(config.waypoint_timeout_s)))


def _sample_composition(config: CollectionConfig, rng: random.Random, bins, pending_bins, now_s: float) -> _Composition:
    if not pending_bins:
        pending_bins.extend(range(len(bins)))
        rng.shuffle(pending_bins)
    lo, hi = bins[pending_bins.pop()]
    desired_distance = rng.uniform(lo, hi)
    near_scale = 1.0 if desired_distance >= 10.0 else max(0.35, desired_distance / 10.0)
    return _Composition(
        desired_distance=desired_distance,
        azimuth_deg=rng.uniform(-160.0, 160.0),
        relative_altitude=rng.uniform(config.relative_altitude_min, config.relative_altitude_max),
        body_yaw_bias_deg=rng.uniform(-config.body_yaw_bias_max_deg, config.body_yaw_bias_max_deg) * near_scale,
        frame_yaw_bias_deg=rng.uniform(-config.frame_yaw_bias_max_deg, config.frame_yaw_bias_max_deg) * near_scale,
        frame_pitch_bias_deg=rng.uniform(-config.frame_pitch_bias_max_deg, config.frame_pitch_bias_max_deg) * near_scale,
        expires_at=now_s + rng.uniform(config.composition_hold_min_s, config.composition_hold_max_s),
    )


def _build_follow_command(config: CollectionConfig, composition: _Composition, target: DroneSnapshot, collector: DroneSnapshot):
    target_pos = np.array(target.position, dtype=float)
    collector_pos = np.array(collector.position, dtype=float)
    target_vel = np.array(target.velocity, dtype=float)

    forward_xy = np.array(target.velocity[:2], dtype=float)
    if float(np.linalg.norm(forward_xy)) < 1.0:
        forward_xy = collector_pos[:2] - target_pos[:2]
    if float(np.linalg.norm(forward_xy)) < 1e-3:
        forward_xy = np.array([1.0, 0.0], dtype=float)
    forward_xy = forward_xy / max(1e-6, float(np.linalg.norm(forward_xy)))
    side_xy = np.array([-forward_xy[1], forward_xy[0]], dtype=float)
    azimuth_rad = math.radians(composition.azimuth_deg)
    radial_xy = (-math.cos(azimuth_rad) * forward_xy) + (math.sin(azimuth_rad) * side_xy)

    desired_position = target_pos.copy()
    desired_position[0] += radial_xy[0] * composition.desired_distance
    desired_position[1] += radial_xy[1] * composition.desired_distance
    desired_position[2] += composition.relative_altitude

    velocity_cmd = (desired_position - collector_pos) * np.array([0.9, 0.9, 0.75], dtype=float) + target_vel * np.array([0.45, 0.45, 0.20], dtype=float)
    los_vector = target_pos - collector_pos
    los_distance = float(np.linalg.norm(los_vector))
    if los_distance < max(2.5, config.distance_min * 0.7):
        away = collector_pos - target_pos
        away_norm = float(np.linalg.norm(away))
        if away_norm > 1e-6:
            velocity_cmd += (away / away_norm) * (0.55 * config.interceptor_max_speed)

    speed = float(np.linalg.norm(velocity_cmd))
    if speed > config.interceptor_max_speed:
        velocity_cmd *= config.interceptor_max_speed / max(1e-6, speed)

    horiz = math.hypot(float(los_vector[0]), float(los_vector[1]))
    los_yaw = math.degrees(math.atan2(float(los_vector[1]), float(los_vector[0])))
    los_pitch = math.degrees(math.atan2(float(los_vector[2]), max(1e-6, horiz)))
    body_yaw = _wrap_deg(los_yaw + composition.body_yaw_bias_deg)
    camera_yaw = _clamp(_wrap_deg(los_yaw - body_yaw + composition.frame_yaw_bias_deg), -config.camera_yaw_limit_deg, config.camera_yaw_limit_deg)
    camera_pitch = _clamp(los_pitch + composition.frame_pitch_bias_deg, config.camera_pitch_min_deg, config.camera_pitch_max_deg)

    return {
        "velocity": (float(velocity_cmd[0]), float(velocity_cmd[1]), float(velocity_cmd[2])),
        "desired_position": (float(desired_position[0]), float(desired_position[1]), float(desired_position[2])),
        "distance_m": los_distance,
        "body_yaw_deg": body_yaw,
        "camera_yaw_deg": camera_yaw,
        "camera_pitch_deg": camera_pitch,
    }


def _auto_label(config: CollectionConfig, segmentation_frame, image_w: int, image_h: int):
    target_color = np.array(_segmentation_color_bgr(config.target_segmentation_id), dtype=np.int16)
    diff = np.abs(segmentation_frame.astype(np.int16) - target_color.reshape((1, 1, 3)))
    mask = np.all(diff <= np.array([10, 10, 10], dtype=np.int16).reshape((1, 1, 3)), axis=2)
    pixel_count = int(mask.sum())
    if pixel_count < int(config.min_mask_pixels):
        return False, None, None, pixel_count, 0.0
    ys, xs = np.where(mask)
    x1, y1, x2, y2 = int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())
    bbox_w = max(1, x2 - x1 + 1)
    bbox_h = max(1, y2 - y1 + 1)
    bbox_xyxy = [x1, y1, x2, y2]
    bbox_yolo = [
        (x1 + x2 + 1) * 0.5 / max(1, int(image_w)),
        (y1 + y2 + 1) * 0.5 / max(1, int(image_h)),
        bbox_w / max(1, int(image_w)),
        bbox_h / max(1, int(image_h)),
    ]
    return True, bbox_xyxy, bbox_yolo, pixel_count, float(pixel_count) / max(1.0, float(image_w * image_h))


def run_collection(config: CollectionConfig) -> CollectionResult:
    if np is None:
        raise RuntimeError("numpy is required for Collect.py")
    if config.show and cv2 is None:
        print("[WARN] OpenCV is not available, preview disabled")
        config.show = False

    project_root = Path(__file__).resolve().parents[2]
    output_root = _resolve_path(project_root, config.output_root)
    session_name = config.session_name.strip() if config.session_name else f"collect_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    session_dir = output_root / session_name
    images_dir = session_dir / "images"
    labels_dir = session_dir / "labels"
    segmentation_dir = session_dir / "segmentation"
    images_dir.mkdir(parents=True, exist_ok=False)
    labels_dir.mkdir(parents=True, exist_ok=False)
    if config.save_segmentation:
        segmentation_dir.mkdir(parents=True, exist_ok=False)
    session_dir.joinpath("session_config.json").write_text(json.dumps(config.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")

    manifest_path = session_dir / "manifest.jsonl"
    manifest = manifest_path.open("a", encoding="utf-8")
    client = None
    collector = None
    target = None
    created_collector = False
    created_target = False
    saved_frames = 0
    labeled_frames = 0
    empty_frames = 0
    exit_reason = "finished"

    try:
        client = TCPClient(
            host=config.connection.host,
            port=config.connection.port,
            timeout=config.connection.timeout,
            auto_connect=False,
            auto_reconnect=config.connection.auto_reconnect,
            reconnect_delay_s=config.connection.reconnect_delay_s,
        )
        if not client.connect() or not _ok(client.ping()):
            raise RuntimeError("failed to connect to simulator")

        if config.clean_existing:
            _ensure_removed(client, config.target.actor_id)
            _ensure_removed(client, config.interceptor.actor_id)
            time.sleep(0.2)

        collector = AgentDrone(client, actor_id=config.interceptor.actor_id, classname=config.interceptor.classname or DEFAULT_DRONE_CLASS, label=config.interceptor.label, mission_role=config.interceptor.role)
        target = AgentDrone(client, actor_id=config.target.actor_id, classname=config.target.classname or DEFAULT_DRONE_CLASS, label=config.target.label, mission_role=config.target.role)
        if not _spawn_drone(collector, config.interceptor):
            raise RuntimeError("collector spawn failed")
        created_collector = True
        if not _spawn_drone(target, config.target):
            raise RuntimeError("target spawn failed")
        created_target = True
        time.sleep(0.4)

        if not _ok(target.set_segmentation_id(config.target_segmentation_id)):
            raise RuntimeError("target segmentation id setup failed")
        if not _ok(collector.set_segmentation_id(config.interceptor_segmentation_id)):
            raise RuntimeError("collector segmentation id setup failed")

        for drone, drone_config in ((collector, config.interceptor), (target, config.target)):
            drone.enable_api_control(True)
            drone.takeoff(altitude=drone_config.altitude)
            if not _wait_for_altitude(drone, drone_config.altitude):
                raise RuntimeError(f"takeoff timeout for {drone.actor_id}")
        collector.set_camera_angles(config.interceptor.camera_pitch, config.interceptor.camera_yaw)

        rng = random.Random(config.seed)
        distance_bins = _distance_bins(config.distance_min, config.distance_max, count=5)
        pending_bins = []
        waypoint = None
        composition = None
        last_command = None
        last_target_snapshot = None
        last_collector_snapshot = None
        sample_index = 0
        transient_errors = 0
        start_s = time.monotonic()
        next_control_s = start_s
        next_sample_s = start_s
        next_log_s = start_s
        control_dt = max(0.02, 1.0 / max(1e-6, float(config.control_hz)))
        sample_dt = max(0.05, 1.0 / max(1e-6, float(config.sample_hz)))
        log_dt = max(0.2, float(config.telemetry_interval_s))

        while True:
            now_s = time.monotonic()
            elapsed_s = now_s - start_s
            if elapsed_s >= max(1.0, float(config.duration_s)):
                break
            if config.max_samples > 0 and saved_frames >= int(config.max_samples):
                exit_reason = "finished:max_samples"
                break

            if now_s >= next_control_s:
                last_target_snapshot = _snapshot(target)
                last_collector_snapshot = _snapshot(collector)
                if waypoint is None or _distance(last_target_snapshot.position, waypoint.position) <= max(0.5, float(config.waypoint_reach_radius)) or elapsed_s >= waypoint.expires_at:
                    waypoint = _sample_waypoint(config, rng, last_target_snapshot.position, elapsed_s)
                if composition is None or elapsed_s >= composition.expires_at:
                    composition = _sample_composition(config, rng, distance_bins, pending_bins, elapsed_s)
                last_command = _build_follow_command(config, composition, last_target_snapshot, last_collector_snapshot)

                target.move_to(*waypoint.position, speed=waypoint.speed, frame="ue", yaw_mode="angle", yaw=waypoint.yaw_deg, drivetrain="forward_only")
                collector.move_by_velocity(*last_command["velocity"], frame="ue", yaw_mode="angle", yaw=last_command["body_yaw_deg"], drivetrain="max_degree_of_freedom")
                collector.set_camera_angles(last_command["camera_pitch_deg"], last_command["camera_yaw_deg"])
                next_control_s = now_s + control_dt

            if now_s >= next_sample_s and last_command is not None and last_target_snapshot is not None and last_collector_snapshot is not None:
                try:
                    scene_packet = collector.get_image_packet(image_type="scene", quality=config.image_quality, max_depth_m=config.max_depth_m)
                    segmentation_packet = collector.get_image_packet(image_type="segmentation", quality=100, max_depth_m=config.max_depth_m)
                    if scene_packet is None or segmentation_packet is None:
                        raise RuntimeError("image packet missing")
                    segmentation_frame = segmentation_packet.decode_bgr()
                    if segmentation_frame is None:
                        raise RuntimeError("segmentation decode failed")
                    valid, bbox_xyxy, bbox_yolo, pixel_count, area_ratio = _auto_label(config, segmentation_frame, scene_packet.width or segmentation_packet.width, scene_packet.height or segmentation_packet.height)
                    if valid or config.save_empty_labels:
                        sample_index += 1
                        stem = f"{sample_index:06d}"
                        image_path = images_dir / f"{stem}.jpg"
                        label_path = labels_dir / f"{stem}.txt"
                        scene_bytes = base64.b64decode(scene_packet.data) if scene_packet.data else b""
                        if scene_bytes:
                            image_path.write_bytes(scene_bytes)
                            preview_frame = scene_packet.decode_bgr() if config.show else None
                        else:
                            preview_frame = scene_packet.decode_bgr()
                            if preview_frame is None:
                                raise RuntimeError("scene decode failed")
                            cv2.imwrite(str(image_path), preview_frame)
                        if config.save_segmentation:
                            cv2.imwrite(str(segmentation_dir / f"{stem}.png"), segmentation_frame)

                        if valid and bbox_yolo:
                            label_path.write_text(f"0 {bbox_yolo[0]:.6f} {bbox_yolo[1]:.6f} {bbox_yolo[2]:.6f} {bbox_yolo[3]:.6f}\n", encoding="utf-8")
                            labeled_frames += 1
                        else:
                            label_path.write_text("", encoding="utf-8")
                            empty_frames += 1

                        target_speed = math.sqrt(sum(float(v) * float(v) for v in last_target_snapshot.velocity))
                        metadata = {
                            "index": sample_index,
                            "timestamp_s": round(elapsed_s, 6),
                            "scene_image": f"images/{stem}.jpg",
                            "label_file": f"labels/{stem}.txt",
                            "segmentation_image": f"segmentation/{stem}.png" if config.save_segmentation else "",
                            "label_valid": valid,
                            "bbox_xyxy": bbox_xyxy,
                            "bbox_yolo": bbox_yolo,
                            "pixel_count": pixel_count,
                            "mask_area_ratio": area_ratio,
                            "distance_m": last_command["distance_m"],
                            "collector_position": list(last_collector_snapshot.position),
                            "collector_velocity": list(last_collector_snapshot.velocity),
                            "collector_body_yaw_deg": last_command["body_yaw_deg"],
                            "collector_camera_yaw_deg": last_command["camera_yaw_deg"],
                            "collector_camera_pitch_deg": last_command["camera_pitch_deg"],
                            "target_position": list(last_target_snapshot.position),
                            "target_velocity": list(last_target_snapshot.velocity),
                            "target_speed_mps": target_speed,
                            "waypoint": {"position": list(waypoint.position), "speed": waypoint.speed, "yaw_deg": waypoint.yaw_deg},
                            "composition": {
                                "desired_distance": composition.desired_distance,
                                "azimuth_deg": composition.azimuth_deg,
                                "relative_altitude": composition.relative_altitude,
                                "body_yaw_bias_deg": composition.body_yaw_bias_deg,
                                "frame_yaw_bias_deg": composition.frame_yaw_bias_deg,
                                "frame_pitch_bias_deg": composition.frame_pitch_bias_deg,
                            },
                        }
                        manifest.write(json.dumps(metadata, ensure_ascii=False) + "\n")
                        manifest.flush()
                        saved_frames += 1

                        if config.show and cv2 is not None and preview_frame is not None:
                            canvas = preview_frame.copy()
                            if valid and bbox_xyxy:
                                x1, y1, x2, y2 = [int(v) for v in bbox_xyxy]
                                cv2.rectangle(canvas, (x1, y1), (x2, y2), (40, 220, 60), 2, cv2.LINE_AA)
                            lines = [
                                f"COLLECT | saved={saved_frames} labeled={labeled_frames} empty={empty_frames}",
                                f"distance={last_command['distance_m']:.2f}m target_speed={target_speed:.2f}m/s pixels={pixel_count}",
                                f"body_yaw={last_command['body_yaw_deg']:+.1f} cam_yaw={last_command['camera_yaw_deg']:+.1f} cam_pitch={last_command['camera_pitch_deg']:+.1f}",
                                "keys: q=quit",
                            ]
                            for idx, text in enumerate(lines):
                                y = 24 + idx * 22
                                cv2.putText(canvas, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 0, 0), 3, cv2.LINE_AA)
                                cv2.putText(canvas, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (235, 235, 235), 1, cv2.LINE_AA)
                            cv2.imshow("graduation_collect", canvas)
                            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                                exit_reason = "user quit"
                                break
                    transient_errors = 0
                except Exception as exc:
                    transient_errors += 1
                    print(f"[WARN] sample capture failed ({transient_errors}): {exc}")
                    if transient_errors >= 5:
                        raise RuntimeError(f"sample capture failed repeatedly: {exc}") from exc
                next_sample_s = now_s + sample_dt

            if now_s >= next_log_s and last_command is not None and last_target_snapshot is not None:
                target_speed = math.sqrt(sum(float(v) * float(v) for v in last_target_snapshot.velocity))
                print(f"[COLLECT] t={elapsed_s:6.1f}s saved={saved_frames} labeled={labeled_frames} dist={last_command['distance_m']:6.2f}m target_speed={target_speed:5.2f}")
                next_log_s = now_s + log_dt

            time.sleep(0.005)

        return CollectionResult(session_dir=str(session_dir), exit_reason=exit_reason, saved_frames=saved_frames, labeled_frames=labeled_frames, empty_frames=empty_frames)
    finally:
        manifest.close()
        if cv2 is not None:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        try:
            if collector is not None and created_collector:
                collector.hover()
            if target is not None and created_target:
                target.hover()
        except Exception:
            pass
        should_cleanup = (not config.keep_actors) and (not str(exit_reason).startswith("error:"))
        if should_cleanup:
            for actor, created in ((target, created_target), (collector, created_collector)):
                if actor is not None and created:
                    try:
                        actor.remove_raw()
                    except Exception:
                        pass
        if client is not None:
            client.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Collect YOLO training data with GT view control and segmentation auto-labeling")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--no-auto-reconnect", dest="auto_reconnect", action="store_false")
    parser.set_defaults(auto_reconnect=True)

    parser.add_argument("--drone-class", default=None)
    parser.add_argument("--collector-class", default=None)
    parser.add_argument("--target-class", default=None)
    parser.add_argument("--collector-id", default="collect_drone_0")
    parser.add_argument("--target-id", default="collect_drone_1")

    parser.add_argument("--collector-spawn-x", type=float, default=0.0)
    parser.add_argument("--collector-spawn-y", type=float, default=0.0)
    parser.add_argument("--collector-spawn-z", type=float, default=1.0)
    parser.add_argument("--collector-yaw", type=float, default=0.0)
    parser.add_argument("--collector-altitude", type=float, default=12.0)
    parser.add_argument("--collector-camera-pitch", type=float, default=0.0)
    parser.add_argument("--collector-camera-yaw", type=float, default=0.0)

    parser.add_argument("--target-spawn-x", type=float, default=35.0)
    parser.add_argument("--target-spawn-y", type=float, default=12.0)
    parser.add_argument("--target-spawn-z", type=float, default=1.0)
    parser.add_argument("--target-yaw", type=float, default=180.0)
    parser.add_argument("--target-altitude", type=float, default=12.0)

    parser.add_argument("--output-root", default="PythonClient/Run/datasets/raw")
    parser.add_argument("--session-name", default="")
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--max-samples", type=int, default=0)
    parser.add_argument("--control-hz", type=float, default=12.0)
    parser.add_argument("--sample-hz", type=float, default=4.0)
    parser.add_argument("--telemetry-interval", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=1234)

    parser.add_argument("--x-min", type=float, default=-150.0)
    parser.add_argument("--x-max", type=float, default=150.0)
    parser.add_argument("--y-min", type=float, default=-150.0)
    parser.add_argument("--y-max", type=float, default=150.0)
    parser.add_argument("--z-min", type=float, default=6.0)
    parser.add_argument("--z-max", type=float, default=24.0)
    parser.add_argument("--target-speed-min", type=float, default=6.0)
    parser.add_argument("--target-speed-max", type=float, default=18.0)
    parser.add_argument("--waypoint-reach-radius", type=float, default=4.0)
    parser.add_argument("--waypoint-timeout", type=float, default=14.0)

    parser.add_argument("--distance-min", type=float, default=4.0)
    parser.add_argument("--distance-max", type=float, default=140.0)
    parser.add_argument("--relative-altitude-min", type=float, default=-12.0)
    parser.add_argument("--relative-altitude-max", type=float, default=20.0)
    parser.add_argument("--interceptor-max-speed", type=float, default=20.0)
    parser.add_argument("--composition-hold-min", type=float, default=2.0)
    parser.add_argument("--composition-hold-max", type=float, default=6.0)
    parser.add_argument("--body-yaw-bias-max", type=float, default=18.0)
    parser.add_argument("--frame-yaw-bias-max", type=float, default=20.0)
    parser.add_argument("--frame-pitch-bias-max", type=float, default=14.0)
    parser.add_argument("--camera-yaw-limit", type=float, default=35.0)
    parser.add_argument("--camera-pitch-min", type=float, default=-35.0)
    parser.add_argument("--camera-pitch-max", type=float, default=25.0)

    parser.add_argument("--target-segmentation-id", type=int, default=30)
    parser.add_argument("--collector-segmentation-id", type=int, default=20)
    parser.add_argument("--min-mask-pixels", type=int, default=20)
    parser.add_argument("--image-quality", type=int, default=95)
    parser.add_argument("--max-depth-m", type=float, default=200.0)
    parser.add_argument("--show", dest="show", action="store_true")
    parser.add_argument("--no-show", dest="show", action="store_false")
    parser.set_defaults(show=True)
    parser.add_argument("--save-segmentation", dest="save_segmentation", action="store_true")
    parser.add_argument("--no-save-segmentation", dest="save_segmentation", action="store_false")
    parser.set_defaults(save_segmentation=True)
    parser.add_argument("--save-empty-labels", action="store_true")
    parser.add_argument("--keep-actors", action="store_true")
    parser.add_argument("--clean-existing", dest="clean_existing", action="store_true")
    parser.add_argument("--no-clean-existing", dest="clean_existing", action="store_false")
    parser.set_defaults(clean_existing=True)
    return parser


def build_config(args: argparse.Namespace) -> CollectionConfig:
    default_drone_class = args.drone_class or DEFAULT_DRONE_CLASS
    return CollectionConfig(
        connection=ConnectionConfig(host=args.host, port=args.port, timeout=args.timeout, auto_reconnect=args.auto_reconnect),
        interceptor=DroneConfig(
            actor_id=args.collector_id,
            label="Collector",
            role="collector",
            classname=args.collector_class or default_drone_class,
            spawn_pose=Pose(x=args.collector_spawn_x, y=args.collector_spawn_y, z=args.collector_spawn_z, yaw=args.collector_yaw),
            altitude=args.collector_altitude,
            camera_pitch=args.collector_camera_pitch,
            camera_yaw=args.collector_camera_yaw,
        ),
        target=DroneConfig(
            actor_id=args.target_id,
            label="Target",
            role="target",
            classname=args.target_class or default_drone_class,
            spawn_pose=Pose(x=args.target_spawn_x, y=args.target_spawn_y, z=args.target_spawn_z, yaw=args.target_yaw),
            altitude=args.target_altitude,
        ),
        output_root=args.output_root,
        session_name=args.session_name,
        duration_s=args.duration,
        max_samples=args.max_samples,
        control_hz=args.control_hz,
        sample_hz=args.sample_hz,
        telemetry_interval_s=args.telemetry_interval,
        seed=args.seed,
        clean_existing=args.clean_existing,
        keep_actors=args.keep_actors,
        show=args.show,
        image_quality=args.image_quality,
        max_depth_m=args.max_depth_m,
        save_segmentation=args.save_segmentation,
        save_empty_labels=args.save_empty_labels,
        target_segmentation_id=args.target_segmentation_id,
        interceptor_segmentation_id=args.collector_segmentation_id,
        random_bounds=CollectionBounds(x_min=args.x_min, x_max=args.x_max, y_min=args.y_min, y_max=args.y_max, z_min=args.z_min, z_max=args.z_max),
        target_speed_min=args.target_speed_min,
        target_speed_max=args.target_speed_max,
        waypoint_reach_radius=args.waypoint_reach_radius,
        waypoint_timeout_s=args.waypoint_timeout,
        interceptor_max_speed=args.interceptor_max_speed,
        distance_min=args.distance_min,
        distance_max=args.distance_max,
        relative_altitude_min=args.relative_altitude_min,
        relative_altitude_max=args.relative_altitude_max,
        composition_hold_min_s=args.composition_hold_min,
        composition_hold_max_s=args.composition_hold_max,
        body_yaw_bias_max_deg=args.body_yaw_bias_max,
        frame_yaw_bias_max_deg=args.frame_yaw_bias_max,
        frame_pitch_bias_max_deg=args.frame_pitch_bias_max,
        camera_yaw_limit_deg=args.camera_yaw_limit,
        camera_pitch_min_deg=args.camera_pitch_min,
        camera_pitch_max_deg=args.camera_pitch_max,
        min_mask_pixels=args.min_mask_pixels,
    )


def main() -> int:
    config = build_config(build_parser().parse_args())
    result = run_collection(config)
    print(json.dumps(result.to_dict(), ensure_ascii=False, indent=2))
    return 0 if not result.exit_reason.startswith("error:") else 1


if __name__ == "__main__":
    raise SystemExit(main())
