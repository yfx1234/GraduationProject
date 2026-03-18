from __future__ import annotations

import base64
import json
import math
import random
import shutil
import time
from pathlib import Path

try:
    import cv2
except ImportError:
    cv2 = None

from GraduationSIM import DEFAULT_DRONE_CLASS, AgentBase, AgentDrone, AutoLabel, DroneSnapshot, ImagePacket, Pose, TCPClient


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def ok(resp):
    return AgentBase.is_ok(resp)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def wrap_deg(value):
    return (float(value) + 180.0) % 360.0 - 180.0


def snapshot(drone):
    return DroneSnapshot.from_state(drone.get_state(frame="ue"))


def wait_altitude(drone, altitude, timeout_s=20.0, tolerance_m=0.8):
    end = time.time() + timeout_s
    while time.time() < end:
        if snapshot(drone).position[2] >= float(altitude) - tolerance_m:
            return True
        time.sleep(0.25)
    return False


def clear_path(path, keep_dir=False):
    if not path.exists():
        return
    if path.is_dir():
        for child in list(path.iterdir()):
            clear_path(child, keep_dir=False)
        if not keep_dir:
            try:
                path.chmod(stat.S_IWRITE | stat.S_IREAD)
            except OSError:
                pass
            path.rmdir()
        return
    try:
        path.chmod(stat.S_IWRITE | stat.S_IREAD)
    except OSError:
        pass
    path.unlink()


def reset_collection_output(dataset_root, results_root):
    for rel in (
        ("images", "train"),
        ("images", "val"),
        ("labels", "train"),
        ("labels", "val"),
        ("metadata",),
        ("segmentation",),
    ):
        clear_path(dataset_root.joinpath(*rel), keep_dir=True)
    for cache_path in dataset_root.glob("labels/*.cache"):
        clear_path(cache_path)
    clear_path(results_root / "collect_summary.json")


def ensure_dataset_layout(root):
    for split in ("train", "val"):
        (root / "images" / split).mkdir(parents=True, exist_ok=True)
        (root / "labels" / split).mkdir(parents=True, exist_ok=True)
    (root / "data.yaml").write_text(
        "path: " + root.as_posix() + "\n"
        "train: images/train\n"
        "val: images/val\n\n"
        "nc: 1\n"
        "names:\n"
        "  0: drone\n",
        encoding="utf-8",
    )


def next_index(root):
    idx = -1
    for split in ("train", "val"):
        for path in (root / "images" / split).glob("*.*"):
            if path.suffix.lower() in IMAGE_EXTS and path.stem.isdigit():
                idx = max(idx, int(path.stem))
    return idx + 1


def count_images(root):
    counts = {"train": 0, "val": 0}
    for split in counts:
        counts[split] = sum(1 for p in (root / "images" / split).glob("*.*") if p.suffix.lower() in IMAGE_EXTS)
    return counts


def choose_split(counts, val_ratio):
    ratio = clamp(float(val_ratio), 0.0, 0.95)
    if ratio <= 0.0:
        split = "train"
    else:
        total_after_save = counts["train"] + counts["val"] + 1
        desired_val = int(math.floor(total_after_save * ratio + 0.5))
        if total_after_save > 1:
            desired_val = min(desired_val, total_after_save - 1)
        split = "val" if counts["val"] < desired_val else "train"
    counts[split] += 1
    return split


def dot3(a, b):
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])


def rotator_axes(pitch_deg, yaw_deg, roll_deg=0.0):
    pitch = math.radians(float(pitch_deg))
    yaw = math.radians(float(yaw_deg))
    roll = math.radians(float(roll_deg))
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cr, sr = math.cos(roll), math.sin(roll)
    forward = (cp * cy, cp * sy, sp)
    right = (sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp)
    up = (-cr * sp * cy - sr * sy, -cr * sp * sy + sr * cy, cr * cp)
    return forward, right, up


def project_world_point(camera_pos_cm, camera_rot_deg, fov_deg, image_w, image_h, world_pos_m):
    world_cm = (float(world_pos_m[0]) * 100.0, float(world_pos_m[1]) * 100.0, float(world_pos_m[2]) * 100.0)
    delta = (
        world_cm[0] - float(camera_pos_cm[0]),
        world_cm[1] - float(camera_pos_cm[1]),
        world_cm[2] - float(camera_pos_cm[2]),
    )
    forward, right, up = rotator_axes(camera_rot_deg[0], camera_rot_deg[1], camera_rot_deg[2])
    x_cam = dot3(delta, right)
    y_cam = dot3(delta, up)
    z_cam = dot3(delta, forward)
    if z_cam <= 1e-3:
        return None
    focal = float(image_w) / max(1e-6, 2.0 * math.tan(math.radians(clamp(float(fov_deg), 1.0, 179.0) * 0.5)))
    return (float(image_w) * 0.5 + focal * x_cam / z_cam, float(image_h) * 0.5 - focal * y_cam / z_cam)


def auto_label(scene_response, target_state, image_w, image_h):
    camera_pos = scene_response.get("camera_pos", [0.0, 0.0, 0.0])
    camera_rot = scene_response.get("camera_rot", [0.0, 0.0, 0.0])
    fov = float(scene_response.get("fov", 90.0) or 90.0)
    center = project_world_point(camera_pos, camera_rot, fov, image_w, image_h, target_state.position)
    if center is None:
        return AutoLabel(valid=False, image_w=image_w, image_h=image_h)

    forward, right, up = rotator_axes(target_state.pitch, target_state.yaw, target_state.roll)
    hx, hy, hz = 0.55, 0.55, 0.22
    pts = [center]
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                p = (
                    float(target_state.position[0]) + sx * hx * forward[0] + sy * hy * right[0] + sz * hz * up[0],
                    float(target_state.position[1]) + sx * hx * forward[1] + sy * hy * right[1] + sz * hz * up[1],
                    float(target_state.position[2]) + sx * hx * forward[2] + sy * hy * right[2] + sz * hz * up[2],
                )
                uv = project_world_point(camera_pos, camera_rot, fov, image_w, image_h, p)
                if uv is not None:
                    pts.append(uv)

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    if max(xs) < 0.0 or max(ys) < 0.0 or min(xs) >= float(image_w) or min(ys) >= float(image_h):
        return AutoLabel(valid=False, image_w=image_w, image_h=image_h)

    x1 = max(0, int(math.floor(min(xs) - 4.0)))
    y1 = max(0, int(math.floor(min(ys) - 4.0)))
    x2 = min(int(image_w) - 1, int(math.ceil(max(xs) + 4.0)))
    y2 = min(int(image_h) - 1, int(math.ceil(max(ys) + 4.0)))
    bw = max(1, x2 - x1 + 1)
    bh = max(1, y2 - y1 + 1)
    area = bw * bh
    return AutoLabel(
        valid=True,
        class_id=0,
        bbox_xyxy=[x1, y1, x2, y2],
        bbox_yolo=[(x1 + x2 + 1) * 0.5 / image_w, (y1 + y2 + 1) * 0.5 / image_h, bw / image_w, bh / image_h],
        pixel_count=area,
        area_ratio=float(area) / max(1.0, float(image_w * image_h)),
        image_w=image_w,
        image_h=image_h,
    )


def label_text(label):
    if not label.valid or not label.bbox_yolo:
        return ""
    x, y, w, h = label.bbox_yolo
    return f"{int(label.class_id)} {x:.6f} {y:.6f} {w:.6f} {h:.6f}\n"


def bbox_iou_xyxy(a, b):
    if not a or not b:
        return 0.0
    ax1, ay1, ax2, ay2 = [float(v) for v in a]
    bx1, by1, bx2, by2 = [float(v) for v in b]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw = max(0.0, ix2 - ix1 + 1.0)
    ih = max(0.0, iy2 - iy1 + 1.0)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, ax2 - ax1 + 1.0) * max(0.0, ay2 - ay1 + 1.0)
    area_b = max(0.0, bx2 - bx1 + 1.0) * max(0.0, by2 - by1 + 1.0)
    union = area_a + area_b - inter
    return 0.0 if union <= 0.0 else inter / union


def normalize_xy(x, y):
    mag = math.hypot(float(x), float(y))
    if mag <= 1e-6:
        return (0.0, 0.0), 0.0
    return (float(x) / mag, float(y) / mag), mag


def target_forward_xy(target_state, collector_state):
    forward, speed = normalize_xy(float(target_state.velocity[0]), float(target_state.velocity[1]))
    if speed >= 1.0:
        return forward

    yaw_rad = math.radians(float(target_state.yaw))
    forward, speed = normalize_xy(math.cos(yaw_rad), math.sin(yaw_rad))
    if speed >= 0.5:
        return forward

    fallback_x = float(target_state.position[0]) - float(collector_state.position[0])
    fallback_y = float(target_state.position[1]) - float(collector_state.position[1])
    forward, speed = normalize_xy(fallback_x, fallback_y)
    if speed >= 1e-3:
        return forward

    return (1.0, 0.0)


def pick_target_goal(rng, state):
    x = rng.uniform(-70.0, 70.0)
    y = rng.uniform(-70.0, 70.0)
    z = rng.uniform(8.0, 16.0)
    speed = rng.uniform(4.0, 7.5)
    yaw = math.degrees(math.atan2(y - float(state.position[1]), x - float(state.position[0])))
    return (x, y, z, speed, yaw)


def sample_view_distance(rng, recovery=False):
    roll = rng.random()
    if recovery:
        if roll < 0.75:
            return rng.uniform(12.0, 18.0)
        return rng.uniform(18.0, 22.0)
    if roll < 0.18:
        return rng.uniform(10.0, 14.0)
    if roll < 0.68:
        return rng.uniform(14.0, 22.0)
    return rng.uniform(22.0, 30.0)


def next_orbit_deg(rng, prev_view=None, orbit_direction=1.0):
    if prev_view is None:
        return rng.uniform(-180.0, 180.0)
    base = float(prev_view["orbit_deg"])
    step = rng.uniform(35.0, 80.0)
    if rng.random() < 0.25:
        step += rng.uniform(20.0, 55.0)
    return wrap_deg(base + float(orbit_direction) * step + rng.uniform(-18.0, 18.0))


def pick_view(rng, prev_view=None, orbit_direction=1.0):
    return {
        "distance": sample_view_distance(rng, recovery=False),
        "orbit_deg": next_orbit_deg(rng, prev_view=prev_view, orbit_direction=orbit_direction),
        "height_offset": rng.uniform(-3.0, 5.5),
        "body_yaw_bias_deg": rng.uniform(-8.0, 8.0),
        "camera_yaw_bias_deg": rng.uniform(-3.5, 3.5),
        "camera_pitch_bias_deg": rng.uniform(-4.0, 3.0),
    }


def pick_recovery_view(rng, anchor_orbit_deg=0.0):
    return {
        "distance": sample_view_distance(rng, recovery=True),
        "orbit_deg": wrap_deg(float(anchor_orbit_deg) + rng.uniform(-20.0, 20.0)),
        "height_offset": rng.uniform(-1.5, 3.0),
        "body_yaw_bias_deg": rng.uniform(-2.0, 2.0),
        "camera_yaw_bias_deg": 0.0,
        "camera_pitch_bias_deg": 0.0,
    }


def build_follow_command(
    view,
    target_state,
    collector_state,
    collector_z_min,
    collector_z_max,
    collector_speed_max,
    min_target_distance,
    camera_yaw_limit_deg,
    camera_pitch_min_deg,
    camera_pitch_max_deg,
):
    target_x = float(target_state.position[0])
    target_y = float(target_state.position[1])
    target_z = float(target_state.position[2])
    collector_x = float(collector_state.position[0])
    collector_y = float(collector_state.position[1])
    collector_z = float(collector_state.position[2])
    target_vx = float(target_state.velocity[0])
    target_vy = float(target_state.velocity[1])
    target_vz = float(target_state.velocity[2])

    forward = target_forward_xy(target_state, collector_state)
    right = (-forward[1], forward[0])
    orbit = math.radians(float(view["orbit_deg"]))
    distance = float(view["distance"])

    desired_x = target_x - math.cos(orbit) * distance * forward[0] + math.sin(orbit) * distance * right[0]
    desired_y = target_y - math.cos(orbit) * distance * forward[1] + math.sin(orbit) * distance * right[1]
    desired_z = clamp(target_z + float(view["height_offset"]), collector_z_min, collector_z_max)

    err_x = desired_x - collector_x
    err_y = desired_y - collector_y
    err_z = desired_z - collector_z
    vel_x = 0.90 * err_x + 0.50 * target_vx
    vel_y = 0.90 * err_y + 0.50 * target_vy
    vel_z = 0.75 * err_z + 0.25 * target_vz

    los_x = target_x - collector_x
    los_y = target_y - collector_y
    los_z = target_z - collector_z
    los_distance = math.sqrt(los_x * los_x + los_y * los_y + los_z * los_z)
    if los_distance < float(min_target_distance):
        away_x = collector_x - target_x
        away_y = collector_y - target_y
        away_z = collector_z - target_z
        away_mag = math.sqrt(away_x * away_x + away_y * away_y + away_z * away_z)
        if away_mag > 1e-6:
            push = (float(min_target_distance) - los_distance + 0.5) * 0.85
            vel_x += away_x / away_mag * push
            vel_y += away_y / away_mag * push
            vel_z += away_z / away_mag * push * 0.6

    speed = math.sqrt(vel_x * vel_x + vel_y * vel_y + vel_z * vel_z)
    if speed > float(collector_speed_max):
        scale = float(collector_speed_max) / max(1e-6, speed)
        vel_x *= scale
        vel_y *= scale
        vel_z *= scale

    horiz = math.hypot(los_x, los_y)
    los_yaw = math.degrees(math.atan2(los_y, los_x))
    los_pitch = math.degrees(math.atan2(los_z, max(1e-6, horiz)))
    body_yaw = wrap_deg(los_yaw + float(view["body_yaw_bias_deg"]))
    camera_yaw = clamp(
        wrap_deg(los_yaw - body_yaw + float(view["camera_yaw_bias_deg"])),
        -float(camera_yaw_limit_deg),
        float(camera_yaw_limit_deg),
    )
    camera_pitch = clamp(
        los_pitch + float(view["camera_pitch_bias_deg"]),
        float(camera_pitch_min_deg),
        float(camera_pitch_max_deg),
    )

    return {
        "velocity": (vel_x, vel_y, vel_z),
        "desired_position": (desired_x, desired_y, desired_z),
        "distance_m": los_distance,
        "body_yaw_deg": body_yaw,
        "camera_yaw_deg": camera_yaw,
        "camera_pitch_deg": camera_pitch,
    }


def run():
    host, port, timeout = "127.0.0.1", 9000, 10.0
    show = cv2 is not None
    keep_actors = False
    reset_existing = False
    seed = 1234

    collector_id = "collect_drone_0"
    target_id = "collect_drone_1"
    collector_pose = Pose(x=0.0, y=0.0, z=1.0, yaw=0.0)
    target_pose = Pose(x=-18.0, y=-12.0, z=1.0, yaw=180.0)
    collector_altitude = 10.0
    target_altitude = 10.0
    collector_fov = 120.0

    duration_s = 300.0
    max_samples = 0
    control_hz = 20.0
    sample_hz = 5.0
    image_quality = 95
    val_ratio = 0.1
    view_change_min_s = 2.4
    view_change_max_s = 4.0
    goal_reach_radius = 4.0
    goal_timeout_s = 12.0
    collector_z_min = 7.0
    collector_z_max = 22.0
    collector_speed_max = 20.0
    min_target_distance = 6.0
    camera_yaw_limit_deg = 35.0
    camera_pitch_min_deg = -30.0
    camera_pitch_max_deg = 22.0
    min_area_ratio = 0.00018
    max_area_ratio = 0.35
    edge_margin_px = 10
    duplicate_iou = 0.97

    project_root = Path(__file__).resolve().parents[2]
    dataset_root = project_root / "PythonClient" / "YOLO" / "dataset"
    results_root = project_root / "PythonClient" / "YOLO" / "results"
    results_root.mkdir(parents=True, exist_ok=True)
    if reset_existing:
        reset_collection_output(dataset_root, results_root)
    ensure_dataset_layout(dataset_root)

    client = TCPClient(host=host, port=port, timeout=timeout, auto_connect=False)
    collector = target = None
    created_collector = created_target = False

    split_counts = count_images(dataset_root)
    sample_index = next_index(dataset_root)
    saved_frames = 0
    labeled_frames = 0
    sample_attempts = 0
    skipped = {"invalid": 0, "tiny": 0, "huge": 0, "edge": 0, "duplicate": 0}
    last_saved_box = None
    last_follow = None
    invalid_streak = 0
    edge_streak = 0

    try:
        if not client.connect():
            return {"saved_frames": 0, "labeled_frames": 0, "sample_attempts": 0, "skipped": skipped}

        collector = AgentDrone(client, actor_id=collector_id, classname=DEFAULT_DRONE_CLASS, label="Collector", mission_role="collector")
        target = AgentDrone(client, actor_id=target_id, classname=DEFAULT_DRONE_CLASS, label="Target", mission_role="target")

        for actor_id in (collector_id, target_id):
            client.send_message({"remove_actor": {"actor_id": actor_id}})
        time.sleep(0.2)

        if not ok(collector.create(pose=collector_pose, classname=DEFAULT_DRONE_CLASS, label="Collector")):
            return {"saved_frames": 0, "labeled_frames": 0, "sample_attempts": 0, "skipped": skipped}
        created_collector = True
        if not ok(target.create(pose=target_pose, classname=DEFAULT_DRONE_CLASS, label="Target")):
            return {"saved_frames": 0, "labeled_frames": 0, "sample_attempts": 0, "skipped": skipped}
        created_target = True

        collector.enable_api_control(True)
        target.enable_api_control(True)
        collector.takeoff(collector_altitude)
        target.takeoff(target_altitude)
        wait_altitude(collector, collector_altitude)
        wait_altitude(target, target_altitude)
        collector.set_camera_fov(collector_fov)
        collector.set_camera_angles(0.0, 0.0)

        rng = random.Random(seed)
        orbit_direction = -1.0 if rng.random() < 0.5 else 1.0
        start = time.time()
        next_control = start
        next_sample = start + 1.0 / sample_hz
        target_goal = pick_target_goal(rng, snapshot(target))
        target_goal_deadline = start + goal_timeout_s
        view = pick_recovery_view(rng, anchor_orbit_deg=rng.uniform(-180.0, 180.0))
        view_deadline = start + 1.5

        while time.time() - start < duration_s:
            now = time.time()
            if max_samples > 0 and saved_frames >= max_samples:
                break

            if now >= next_control:
                target_state = snapshot(target)
                collector_state = snapshot(collector)

                dx = target_goal[0] - float(target_state.position[0])
                dy = target_goal[1] - float(target_state.position[1])
                dz = target_goal[2] - float(target_state.position[2])
                if math.sqrt(dx * dx + dy * dy + dz * dz) <= goal_reach_radius or now >= target_goal_deadline:
                    target_goal = pick_target_goal(rng, target_state)
                    target_goal_deadline = now + goal_timeout_s

                if now >= view_deadline:
                    if invalid_streak >= 2 or edge_streak >= 2:
                        view = pick_recovery_view(rng, anchor_orbit_deg=float(view["orbit_deg"]))
                        view_deadline = now + rng.uniform(1.2, 2.0)
                    else:
                        if rng.random() < 0.28:
                            orbit_direction *= -1.0
                        view = pick_view(rng, prev_view=view, orbit_direction=orbit_direction)
                        view_deadline = now + rng.uniform(view_change_min_s, view_change_max_s)

                target.move_to(
                    x=target_goal[0],
                    y=target_goal[1],
                    z=target_goal[2],
                    speed=target_goal[3],
                    frame="ue",
                    yaw_mode="angle",
                    yaw=target_goal[4],
                    move_pattern="forward_only",
                )

                last_follow = build_follow_command(
                    view=view,
                    target_state=target_state,
                    collector_state=collector_state,
                    collector_z_min=collector_z_min,
                    collector_z_max=collector_z_max,
                    collector_speed_max=collector_speed_max,
                    min_target_distance=min_target_distance,
                    camera_yaw_limit_deg=camera_yaw_limit_deg,
                    camera_pitch_min_deg=camera_pitch_min_deg,
                    camera_pitch_max_deg=camera_pitch_max_deg,
                )
                collector.move_by_velocity(
                    *last_follow["velocity"],
                    frame="ue",
                    yaw_mode="angle",
                    yaw=last_follow["body_yaw_deg"],
                    move_pattern="max_degree_of_freedom",
                )
                collector.set_camera_angles(last_follow["camera_pitch_deg"], last_follow["camera_yaw_deg"])
                next_control = now + 1.0 / control_hz

            if now >= next_sample:
                target_state = snapshot(target)
                scene_response = collector.get_image_response(image_type="scene", quality=image_quality)
                if ok(scene_response):
                    sample_attempts += 1
                    packet = ImagePacket.from_response(scene_response, image_type="scene")
                    frame = packet.decode_bgr()
                    image_w = int(packet.width or (frame.shape[1] if frame is not None else 0))
                    image_h = int(packet.height or (frame.shape[0] if frame is not None else 0))
                    label = auto_label(scene_response, target_state, image_w, image_h)

                    reason = ""
                    if not label.valid or not label.bbox_xyxy:
                        reason = "invalid"
                        invalid_streak += 1
                    elif float(label.area_ratio) < min_area_ratio:
                        reason = "tiny"
                        invalid_streak = 0
                    elif float(label.area_ratio) > max_area_ratio:
                        reason = "huge"
                        invalid_streak = 0
                    else:
                        invalid_streak = 0
                        x1, y1, x2, y2 = [int(v) for v in label.bbox_xyxy]
                        if x1 <= edge_margin_px or y1 <= edge_margin_px or x2 >= image_w - 1 - edge_margin_px or y2 >= image_h - 1 - edge_margin_px:
                            reason = "edge"
                            edge_streak += 1
                        elif last_saved_box is not None and bbox_iou_xyxy(last_saved_box, label.bbox_xyxy) >= duplicate_iou:
                            reason = "duplicate"
                            edge_streak = 0
                        else:
                            edge_streak = 0

                    if reason:
                        skipped[reason] += 1
                        if reason in ("invalid", "edge"):
                            view = pick_recovery_view(rng, anchor_orbit_deg=float(view["orbit_deg"]))
                            view_deadline = now + rng.uniform(1.0, 1.8)
                    else:
                        split = choose_split(split_counts, val_ratio)
                        stem = f"{sample_index:06d}"
                        sample_index += 1
                        image_path = dataset_root / "images" / split / f"{stem}.jpg"
                        label_path = dataset_root / "labels" / split / f"{stem}.txt"

                        raw = base64.b64decode(packet.data) if packet.data else b""
                        if raw:
                            image_path.write_bytes(raw)
                        elif frame is not None and cv2 is not None:
                            cv2.imwrite(str(image_path), frame)
                        else:
                            next_sample = now + 1.0 / sample_hz
                            time.sleep(0.005)
                            continue

                        label_path.write_text(label_text(label), encoding="utf-8")
                        saved_frames += 1
                        labeled_frames += 1
                        last_saved_box = list(label.bbox_xyxy)

                    if show and frame is not None and cv2 is not None:
                        preview = frame.copy()
                        if label.valid and label.bbox_xyxy:
                            x1, y1, x2, y2 = [int(v) for v in label.bbox_xyxy]
                            cv2.rectangle(preview, (x1, y1), (x2, y2), (40, 220, 60), 2, cv2.LINE_AA)

                        valid_ratio = float(saved_frames) / max(1.0, float(sample_attempts))
                        line1 = f"saved={saved_frames} attempts={sample_attempts} valid={valid_ratio:.1%} train={split_counts['train']} val={split_counts['val']}"
                        line2 = f"skip invalid={skipped['invalid']} tiny={skipped['tiny']} huge={skipped['huge']} edge={skipped['edge']} dup={skipped['duplicate']}"
                        if last_follow is not None:
                            line3 = (
                                f"orbit={view['orbit_deg']:+.0f}deg cmd={view['distance']:.1f}m los={last_follow['distance_m']:.1f}m "
                                f"cam(p={last_follow['camera_pitch_deg']:+.1f},y={last_follow['camera_yaw_deg']:+.1f}) "
                                f"streak(inv={invalid_streak},edge={edge_streak})"
                            )
                        else:
                            line3 = f"streak(inv={invalid_streak},edge={edge_streak})"
                        cv2.putText(preview, line1, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (235, 235, 235), 1, cv2.LINE_AA)
                        cv2.putText(preview, line2, (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (235, 235, 235), 1, cv2.LINE_AA)
                        cv2.putText(preview, line3, (10, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (235, 235, 235), 1, cv2.LINE_AA)
                        cv2.imshow("gt_collect", preview)
                        if (cv2.waitKey(1) & 0xFF) == ord("q"):
                            break

                next_sample = now + 1.0 / sample_hz
            time.sleep(0.005)

        summary = {
            "dataset_root": str(dataset_root),
            "saved_frames": saved_frames,
            "labeled_frames": labeled_frames,
            "sample_attempts": sample_attempts,
            "valid_ratio": 0.0 if sample_attempts <= 0 else float(saved_frames) / float(sample_attempts),
            "skipped": skipped,
            "split_counts": split_counts,
            "seed": seed,
            "reset_existing": reset_existing,
        }
        (results_root / "collect_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        return summary
    finally:
        try:
            if cv2 is not None:
                cv2.destroyAllWindows()
        except Exception:
            pass
        for drone, created in ((collector, created_collector), (target, created_target)):
            if drone is not None and created:
                try:
                    drone.hover()
                except Exception:
                    pass
        if not keep_actors:
            for actor, created in ((target, created_target), (collector, created_collector)):
                if actor is not None and created:
                    try:
                        actor.remove()
                    except Exception:
                        pass
        client.disconnect()


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=2))






