from __future__ import annotations

import base64
import json
import math
import random
import stat
import time

try:
    import cv2
except ImportError:
    cv2 = None

from params import net, sim

from GraduationSIM import (
    AgentBase,
    AgentDrone,
    DEFAULT_DRONE_CLASS,
    DroneSnapshot,
    ImagePacket,
    Pose,
    TCPClient,
    bbox_iou,
    build_auto_label,
    build_label_text,
    choose_split,
    count_images,
    ensure_dataset_layout,
    next_index,
    resolve_paths,
)

CLOSE_VIEW_PROFILES = {"close_follow", "side_slide", "near_pass", "scrape_pass", "face_fill"}
VERY_CLOSE_VIEW_PROFILES = {"near_pass", "scrape_pass", "face_fill"}



def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_deg(value):
    return (float(value) + 180.0) % 360.0 - 180.0


def sample_range(random_state, bounds):
    return random_state.uniform(float(bounds[0]), float(bounds[1]))


def increase_counter(counter, key, amount=1):
    counter[str(key)] = int(counter.get(str(key), 0)) + int(amount)


def get_snapshot(drone):
    return DroneSnapshot(drone.get_state(frame="ue"))


def wait_altitude(drone, altitude, timeout_s=20.0, tolerance_m=0.8):
    return AgentBase.wait_altitude(drone, altitude, timeout=timeout_s, tolerance=tolerance_m)


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
    for rel in (("images", "train"), ("images", "val"), ("labels", "train"), ("labels", "val"), ("metadata",), ("segmentation",)):
        clear_path(dataset_root.joinpath(*rel), keep_dir=True)
    for cache_path in dataset_root.glob("labels/*.cache"):
        clear_path(cache_path)
    clear_path(results_root / "collect_summary.json")


def normalize_xy(x, y):
    magnitude = math.hypot(float(x), float(y))
    if magnitude <= 1e-6:
        return (0.0, 0.0), 0.0
    return (float(x) / magnitude, float(y) / magnitude), magnitude


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
    return forward if speed >= 1e-3 else (1.0, 0.0)


def pick_target_goal(random_state, state, profile="close_follow"):
    position_x, position_y, _ = [float(value) for value in state.position]
    if profile == "face_fill":
        radius = random_state.uniform(2.4, 5.2)
        altitude = random_state.uniform(8.9, 10.8)
        speed = random_state.uniform(0.9, 1.8)
    elif profile == "scrape_pass":
        radius = random_state.uniform(2.8, 6.2)
        altitude = random_state.uniform(8.7, 11.0)
        speed = random_state.uniform(1.0, 2.1)
    elif profile == "near_pass":
        radius = random_state.uniform(3.5, 7.5)
        altitude = random_state.uniform(8.6, 11.4)
        speed = random_state.uniform(1.3, 2.6)
    elif profile in {"close_follow", "side_slide", "recovery"}:
        radius = random_state.uniform(4.5, 9.5)
        altitude = random_state.uniform(8.5, 11.8)
        speed = random_state.uniform(1.6, 3.0)
    else:
        radius = random_state.uniform(4.0, 8.0)
        altitude = random_state.uniform(8.6, 11.6)
        speed = random_state.uniform(1.4, 2.8)
    heading = random_state.uniform(-math.pi, math.pi)
    goal_x = clamp(position_x + radius * math.cos(heading), -70.0, 70.0)
    goal_y = clamp(position_y + radius * math.sin(heading), -70.0, 70.0)
    goal_yaw = math.degrees(math.atan2(goal_y - position_y, goal_x - position_x))
    return (goal_x, goal_y, altitude, speed, goal_yaw)


def choose_view_profile(random_state):
    roll = random_state.random()
    if roll < 0.10:
        return "close_follow"
    if roll < 0.24:
        return "side_slide"
    if roll < 0.54:
        return "near_pass"
    if roll < 0.82:
        return "scrape_pass"
    return "face_fill"


def smooth_orbit_deg(prev_view, target_orbit_deg, max_step_deg):
    if prev_view is None:
        return wrap_deg(target_orbit_deg)
    previous_orbit = float(prev_view.get("orbit_deg", 0.0))
    delta = wrap_deg(float(target_orbit_deg) - previous_orbit)
    return wrap_deg(previous_orbit + clamp(delta, -float(max_step_deg), float(max_step_deg)))


def pick_orbit_target(random_state, profile, orbit_direction):
    sign = float(orbit_direction) if random_state.random() < 0.68 else -float(orbit_direction)
    if profile == "face_fill":
        if random_state.random() < 0.80:
            return random_state.uniform(-14.0, 14.0)
        return sign * random_state.uniform(16.0, 34.0)
    if profile == "close_follow":
        if random_state.random() < 0.48:
            return random_state.uniform(-18.0, 18.0)
        return sign * random_state.uniform(18.0, 42.0)
    if profile == "side_slide":
        return sign * random_state.uniform(58.0, 98.0)
    if profile == "near_pass":
        return sign * random_state.uniform(82.0, 122.0)
    return sign * random_state.uniform(70.0, 102.0)


def sample_view_duration(random_state, profile):
    if profile == "close_follow":
        return random_state.uniform(1.2, 2.0)
    if profile == "side_slide":
        return random_state.uniform(1.0, 1.7)
    if profile == "near_pass":
        return random_state.uniform(0.8, 1.3)
    if profile == "scrape_pass":
        return random_state.uniform(0.7, 1.1)
    if profile == "face_fill":
        return random_state.uniform(0.7, 1.2)
    return random_state.uniform(0.9, 1.4)


def goal_timeout_for_profile(random_state, profile):
    if profile in VERY_CLOSE_VIEW_PROFILES:
        return random_state.uniform(3.8, 5.8)
    if profile in {"close_follow", "side_slide", "recovery"}:
        return random_state.uniform(4.5, 6.8)
    return random_state.uniform(4.0, 6.0)


def make_view(
    random_state,
    profile,
    orbit_deg,
    distance,
    height_offset,
    body_yaw_bias_deg,
    camera_yaw_bias_deg,
    camera_pitch_bias_deg,
    *,
    min_distance_m,
    guard_buffer_m,
    push_margin_m,
    speed_cap_mps,
    xy_gain,
    z_gain,
    target_vel_gain,
    target_vz_gain,
    push_gain,
    edgeMargin,
    maxAreaRatio,
    goal_profile=None,
):
    return {
        "profile": profile,
        "goal_profile": goal_profile or profile,
        "distance": sample_range(random_state, distance),
        "orbit_deg": orbit_deg,
        "height_offset": sample_range(random_state, height_offset),
        "body_yaw_bias_deg": sample_range(random_state, body_yaw_bias_deg),
        "camera_yaw_bias_deg": sample_range(random_state, camera_yaw_bias_deg),
        "camera_pitch_bias_deg": sample_range(random_state, camera_pitch_bias_deg),
        "min_distance_m": min_distance_m,
        "guard_buffer_m": guard_buffer_m,
        "push_margin_m": push_margin_m,
        "speed_cap_mps": speed_cap_mps,
        "xy_gain": xy_gain,
        "z_gain": z_gain,
        "target_vel_gain": target_vel_gain,
        "target_vz_gain": target_vz_gain,
        "push_gain": push_gain,
        "edgeMargin": edgeMargin,
        "maxAreaRatio": maxAreaRatio,
    }


def pick_view(random_state, prev_view=None, orbit_direction=1.0, force_profile=None):
    profile = str(force_profile or choose_view_profile(random_state))
    orbit_target = pick_orbit_target(random_state, profile, orbit_direction)
    if profile == "close_follow":
        return make_view(
            random_state, profile, smooth_orbit_deg(prev_view, orbit_target, 24.0), (1.35, 2.05), (-0.8, 1.2), (-3.0, 3.0), (-1.2, 1.2), (-1.8, 1.4),
            min_distance_m=1.05, guard_buffer_m=0.14, push_margin_m=0.08, speed_cap_mps=6.6, xy_gain=0.62, z_gain=0.54, target_vel_gain=0.28, target_vz_gain=0.14, push_gain=0.86, edgeMargin=3, maxAreaRatio=0.90,
        )
    if profile == "side_slide":
        return make_view(
            random_state, profile, smooth_orbit_deg(prev_view, orbit_target, 20.0), (1.10, 1.72), (-0.7, 1.0), (-2.2, 2.2), (-1.2, 1.2), (-1.6, 1.2),
            min_distance_m=0.88, guard_buffer_m=0.12, push_margin_m=0.07, speed_cap_mps=5.6, xy_gain=0.56, z_gain=0.50, target_vel_gain=0.24, target_vz_gain=0.12, push_gain=0.94, edgeMargin=2, maxAreaRatio=0.94,
        )
    if profile == "near_pass":
        return make_view(
            random_state, profile, smooth_orbit_deg(prev_view, orbit_target, 16.0), (0.90, 1.28), (-0.45, 0.7), (-1.5, 1.5), (-0.6, 0.6), (-1.2, 0.9),
            min_distance_m=0.74, guard_buffer_m=0.09, push_margin_m=0.05, speed_cap_mps=4.6, xy_gain=0.48, z_gain=0.44, target_vel_gain=0.20, target_vz_gain=0.10, push_gain=1.02, edgeMargin=1, maxAreaRatio=0.97,
        )
    if profile == "face_fill":
        return make_view(
            random_state, profile, smooth_orbit_deg(prev_view, orbit_target, 10.0), (0.68, 0.92), (-0.25, 0.35), (-1.0, 1.0), (-0.35, 0.35), (-0.8, 0.6),
            min_distance_m=0.56, guard_buffer_m=0.06, push_margin_m=0.04, speed_cap_mps=3.2, xy_gain=0.40, z_gain=0.36, target_vel_gain=0.16, target_vz_gain=0.08, push_gain=1.12, edgeMargin=1, maxAreaRatio=0.985,
        )
    return make_view(
        random_state, "scrape_pass", smooth_orbit_deg(prev_view, orbit_target, 14.0), (0.76, 1.04), (-0.35, 0.55), (-1.2, 1.2), (-0.5, 0.5), (-1.0, 0.8),
        min_distance_m=0.64, guard_buffer_m=0.08, push_margin_m=0.05, speed_cap_mps=3.9, xy_gain=0.44, z_gain=0.40, target_vel_gain=0.18, target_vz_gain=0.09, push_gain=1.08, edgeMargin=1, maxAreaRatio=0.98,
    )


def pick_recovery_view(random_state, anchor_orbit_deg=0.0):
    return make_view(
        random_state, "recovery", wrap_deg(float(anchor_orbit_deg) + random_state.uniform(-14.0, 14.0)), (1.25, 1.90), (-0.55, 0.9), (-1.5, 1.5), (0.0, 0.0), (0.0, 0.0),
        min_distance_m=0.92, guard_buffer_m=0.14, push_margin_m=0.08, speed_cap_mps=4.8, xy_gain=0.54, z_gain=0.48, target_vel_gain=0.22, target_vz_gain=0.11, push_gain=1.00, edgeMargin=2, maxAreaRatio=0.94, goal_profile="close_follow",
    )


def build_follow_command(
    view,
    target_state,
    collector_state,
    collectorMinAltitude,
    collectorMaxAltitude,
    collectorMaxSpeed,
    minTargetDistance,
    cameraYawLimit,
    cameraPitchMin,
    cameraPitchMax,
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

    xy_gain = float(view.get("xy_gain", 0.90))
    z_gain = float(view.get("z_gain", 0.75))
    target_vel_gain = float(view.get("target_vel_gain", 0.50))
    target_vz_gain = float(view.get("target_vz_gain", 0.25))
    speed_limit = min(float(collectorMaxSpeed), float(view.get("speed_cap_mps", collectorMaxSpeed)))
    effective_min_target_distance = max(float(minTargetDistance), float(view.get("min_distance_m", minTargetDistance)))
    push_gain = float(view.get("push_gain", 0.85))

    forward = target_forward_xy(target_state, collector_state)
    right = (-forward[1], forward[0])
    orbit = math.radians(float(view["orbit_deg"]))
    distance = float(view["distance"])

    desired_x = target_x - math.cos(orbit) * distance * forward[0] + math.sin(orbit) * distance * right[0]
    desired_y = target_y - math.cos(orbit) * distance * forward[1] + math.sin(orbit) * distance * right[1]
    desired_z = clamp(target_z + float(view["height_offset"]), collectorMinAltitude, collectorMaxAltitude)

    err_x = desired_x - collector_x
    err_y = desired_y - collector_y
    err_z = desired_z - collector_z
    vel_x = xy_gain * err_x + target_vel_gain * target_vx
    vel_y = xy_gain * err_y + target_vel_gain * target_vy
    vel_z = z_gain * err_z + target_vz_gain * target_vz

    los_x = target_x - collector_x
    los_y = target_y - collector_y
    los_z = target_z - collector_z
    los_distance = math.sqrt(los_x * los_x + los_y * los_y + los_z * los_z)
    guard_distance = effective_min_target_distance + float(view.get("guard_buffer_m", 0.20))
    if los_distance < guard_distance:
        away_x = collector_x - target_x
        away_y = collector_y - target_y
        away_z = collector_z - target_z
        away_mag = math.sqrt(away_x * away_x + away_y * away_y + away_z * away_z)
        if away_mag > 1e-6:
            push = (guard_distance - los_distance + float(view.get("push_margin_m", 0.12))) * push_gain
            vel_x += away_x / away_mag * push
            vel_y += away_y / away_mag * push
            vel_z += away_z / away_mag * push * 0.65

    speed = math.sqrt(vel_x * vel_x + vel_y * vel_y + vel_z * vel_z)
    if speed > speed_limit:
        scale = speed_limit / max(1e-6, speed)
        vel_x *= scale
        vel_y *= scale
        vel_z *= scale

    horiz = math.hypot(los_x, los_y)
    los_yaw = math.degrees(math.atan2(los_y, los_x))
    los_pitch = math.degrees(math.atan2(los_z, max(1e-6, horiz)))
    body_yaw = wrap_deg(los_yaw + float(view["body_yaw_bias_deg"]))
    camera_yaw = clamp(
        wrap_deg(los_yaw - body_yaw + float(view["camera_yaw_bias_deg"])),
        -float(cameraYawLimit),
        float(cameraYawLimit),
    )
    camera_pitch = clamp(
        los_pitch + float(view["camera_pitch_bias_deg"]),
        float(cameraPitchMin),
        float(cameraPitchMax),
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
    collectConfig = sim["collect"]
    host = str(net.get("host", "127.0.0.1"))
    port = int(net.get("port", 9000))
    timeout = 10.0
    showPreview = cv2 is not None
    keepActors = False
    resetOutput = False
    seed = 1234
    collectorId = str(collectConfig["collectorId"])
    targetId = str(collectConfig["targetId"])
    collectorPose = Pose.from_any(collectConfig["collectorPose"])
    targetPose = Pose.from_any(collectConfig["targetPose"])
    collectorAltitude = float(collectConfig.get("collectorAltitude", 10.0))
    targetAltitude = float(collectConfig.get("targetAltitude", 10.0))
    cameraFov = float(collectConfig.get("cameraFov", 90.0))

    durationSeconds, maxSamples = 300.0, 0
    controlHz, sampleHz, imageQuality = 24.0, 6.0, 95
    goalRadius, validationRatio = 1.8, 0.1
    collectorMinAltitude, collectorMaxAltitude, collectorMaxSpeed = 7.2, 18.0, 20.0
    minTargetDistance, cameraYawLimit = 0.55, 35.0
    cameraPitchMin, cameraPitchMax = -30.0, 22.0
    minAreaRatio, maxAreaRatio = 0.018, 0.98
    edgeMargin, duplicateIou = 3, 0.975

    paths = resolve_paths(__file__)
    dataset_root = paths.dataset_root
    results_root = paths.results_root
    results_root.mkdir(parents=True, exist_ok=True)
    if resetOutput:
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
    profile_attempts = {}
    profile_saved = {}
    close_saved_frames = 0
    very_close_saved_frames = 0

    try:
        if not client.connect():
            return {"saved_frames": 0, "labeled_frames": 0, "sample_attempts": 0, "skipped": skipped}

        collector = AgentDrone(client, actor_id=collectorId, classname=DEFAULT_DRONE_CLASS, label="Collector", mission_role="collector")
        target = AgentDrone(client, actor_id=targetId, classname=DEFAULT_DRONE_CLASS, label="Target", mission_role="target")

        AgentBase.remove_existing_actors(client, (collectorId, targetId))

        if not AgentBase.is_ok(collector.create(pose=collectorPose, classname=DEFAULT_DRONE_CLASS, label="Collector")):
            return {"saved_frames": 0, "labeled_frames": 0, "sample_attempts": 0, "skipped": skipped}
        created_collector = True
        if not AgentBase.is_ok(target.create(pose=targetPose, classname=DEFAULT_DRONE_CLASS, label="Target")):
            return {"saved_frames": 0, "labeled_frames": 0, "sample_attempts": 0, "skipped": skipped}
        created_target = True

        collector.enable_api_control(True)
        target.enable_api_control(True)
        collector.takeoff(collectorAltitude)
        target.takeoff(targetAltitude)
        wait_altitude(collector, collectorAltitude)
        wait_altitude(target, targetAltitude)
        collector.set_camera_fov(cameraFov)
        collector.set_camera_angles(0.0, 0.0)

        rng = random.Random(seed)
        orbit_direction = -1.0 if rng.random() < 0.5 else 1.0
        start = time.time()
        next_control = start
        next_sample = start + 1.0 / sampleHz
        view = pick_view(rng, prev_view=None, orbit_direction=orbit_direction, force_profile="close_follow")
        view_deadline = start + sample_view_duration(rng, view["profile"])
        target_goal = pick_target_goal(rng, get_snapshot(target), profile=view.get("goal_profile", view["profile"]))
        target_goal_deadline = start + goal_timeout_for_profile(rng, view.get("goal_profile", view["profile"]))

        while time.time() - start < durationSeconds:
            now = time.time()
            if maxSamples > 0 and saved_frames >= maxSamples:
                break

            if now >= next_control:
                target_state = get_snapshot(target)
                collector_state = get_snapshot(collector)
                active_profile = str(view.get("goal_profile", view.get("profile", "close_follow")))

                dx = target_goal[0] - float(target_state.position[0])
                dy = target_goal[1] - float(target_state.position[1])
                dz = target_goal[2] - float(target_state.position[2])
                if math.sqrt(dx * dx + dy * dy + dz * dz) <= goalRadius or now >= target_goal_deadline:
                    target_goal = pick_target_goal(rng, target_state, profile=active_profile)
                    target_goal_deadline = now + goal_timeout_for_profile(rng, active_profile)

                if now >= view_deadline:
                    if invalid_streak >= 2 or edge_streak >= 2:
                        view = pick_recovery_view(rng, anchor_orbit_deg=float(view["orbit_deg"]))
                    else:
                        if rng.random() < 0.34:
                            orbit_direction *= -1.0
                        view = pick_view(rng, prev_view=view, orbit_direction=orbit_direction)
                    view_deadline = now + sample_view_duration(rng, str(view.get("profile", "close_follow")))

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
                    collectorMinAltitude=collectorMinAltitude,
                    collectorMaxAltitude=collectorMaxAltitude,
                    collectorMaxSpeed=collectorMaxSpeed,
                    minTargetDistance=minTargetDistance,
                    cameraYawLimit=cameraYawLimit,
                    cameraPitchMin=cameraPitchMin,
                    cameraPitchMax=cameraPitchMax,
                )
                collector.move_by_velocity(
                    *last_follow["velocity"],
                    frame="ue",
                    yaw_mode="angle",
                    yaw=last_follow["body_yaw_deg"],
                    move_pattern="max_degree_of_freedom",
                )
                collector.set_camera_angles(last_follow["camera_pitch_deg"], last_follow["camera_yaw_deg"])
                next_control = now + 1.0 / controlHz

            if now >= next_sample:
                target_state = get_snapshot(target)
                active_profile = str(view.get("profile", "unknown"))
                increase_counter(profile_attempts, active_profile)
                scene_response = collector.get_image_response(image_type="scene", quality=imageQuality)
                if AgentBase.is_ok(scene_response):
                    sample_attempts += 1
                    packet = ImagePacket.from_response(scene_response, image_type="scene")
                    frame = packet.decode_bgr()
                    image_w = int(packet.width or (frame.shape[1] if frame is not None else 0))
                    image_h = int(packet.height or (frame.shape[0] if frame is not None else 0))
                    label = build_auto_label(scene_response, target_state, image_w, image_h)

                    active_edge_margin = int(view.get("edgeMargin", edgeMargin))
                    active_max_area_ratio = float(view.get("maxAreaRatio", maxAreaRatio))
                    reason = ""
                    if not label.valid or not label.bbox_xyxy:
                        reason = "invalid"
                        invalid_streak += 1
                    elif float(label.area_ratio) < minAreaRatio:
                        reason = "tiny"
                        invalid_streak = 0
                    elif float(label.area_ratio) > active_max_area_ratio:
                        reason = "huge"
                        invalid_streak = 0
                    else:
                        invalid_streak = 0
                        x1, y1, x2, y2 = [int(v) for v in label.bbox_xyxy]
                        if x1 <= active_edge_margin or y1 <= active_edge_margin or x2 >= image_w - 1 - active_edge_margin or y2 >= image_h - 1 - active_edge_margin:
                            reason = "edge"
                            edge_streak += 1
                        elif last_saved_box is not None and bbox_iou(last_saved_box, label.bbox_xyxy) >= duplicateIou:
                            reason = "duplicate"
                            edge_streak = 0
                        else:
                            edge_streak = 0

                    if reason:
                        skipped[reason] += 1
                        if reason in ("invalid", "edge"):
                            view = pick_recovery_view(rng, anchor_orbit_deg=float(view["orbit_deg"]))
                            view_deadline = now + sample_view_duration(rng, "recovery")
                    else:
                        split = choose_split(split_counts, validationRatio)
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
                            next_sample = now + 1.0 / sampleHz
                            time.sleep(0.005)
                            continue

                        label_path.write_text(build_label_text(label), encoding="utf-8")
                        saved_frames += 1
                        labeled_frames += 1
                        last_saved_box = list(label.bbox_xyxy)
                        increase_counter(profile_saved, active_profile)
                        if active_profile in CLOSE_VIEW_PROFILES:
                            close_saved_frames += 1
                        if active_profile in VERY_CLOSE_VIEW_PROFILES:
                            very_close_saved_frames += 1

                    if showPreview and frame is not None and cv2 is not None:
                        preview = frame.copy()
                        if label.valid and label.bbox_xyxy:
                            x1, y1, x2, y2 = [int(v) for v in label.bbox_xyxy]
                            cv2.rectangle(preview, (x1, y1), (x2, y2), (40, 220, 60), 2, cv2.LINE_AA)

                        valid_ratio = float(saved_frames) / max(1.0, float(sample_attempts))
                        line1 = f"saved={saved_frames} attempts={sample_attempts} valid={valid_ratio:.1%} train={split_counts['train']} val={split_counts['val']}"
                        line2 = f"profile={active_profile} close_saved={close_saved_frames} very_close={very_close_saved_frames} los={0.0 if last_follow is None else last_follow['distance_m']:.2f}m"
                        line3 = f"skip invalid={skipped['invalid']} tiny={skipped['tiny']} huge={skipped['huge']} edge={skipped['edge']} dup={skipped['duplicate']}"
                        if last_follow is not None:
                            line4 = (
                                f"orbit={view['orbit_deg']:+.0f}deg cmd={view['distance']:.2f}m cam(p={last_follow['camera_pitch_deg']:+.1f},y={last_follow['camera_yaw_deg']:+.1f}) "
                                f"streak(inv={invalid_streak},edge={edge_streak})"
                            )
                        else:
                            line4 = f"streak(inv={invalid_streak},edge={edge_streak})"
                        cv2.putText(preview, line1, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (235, 235, 235), 1, cv2.LINE_AA)
                        cv2.putText(preview, line2, (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (235, 235, 235), 1, cv2.LINE_AA)
                        cv2.putText(preview, line3, (10, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (235, 235, 235), 1, cv2.LINE_AA)
                        cv2.putText(preview, line4, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (235, 235, 235), 1, cv2.LINE_AA)
                        cv2.imshow("gt_collect", preview)
                        if (cv2.waitKey(1) & 0xFF) == ord("q"):
                            break

                next_sample = now + 1.0 / sampleHz
            time.sleep(0.005)

        summary = {
            "dataset_root": str(dataset_root),
            "saved_frames": saved_frames,
            "labeled_frames": labeled_frames,
            "sample_attempts": sample_attempts,
            "valid_ratio": 0.0 if sample_attempts <= 0 else float(saved_frames) / float(sample_attempts),
            "skipped": skipped,
            "split_counts": split_counts,
            "profile_attempts": profile_attempts,
            "profile_saved": profile_saved,
            "close_saved_frames": close_saved_frames,
            "very_close_saved_frames": very_close_saved_frames,
            "seed": seed,
            "resetOutput": resetOutput,
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
                AgentBase.safe_hover(drone)
        if not keepActors:
            for actor, created in ((target, created_target), (collector, created_collector)):
                if actor is not None and created:
                    AgentBase.safe_remove(actor)
        AgentBase.disconnect_client(client)


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=2))



