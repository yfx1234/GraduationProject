"""Spawn two drones and run the visual auto-intercept flow.

This script keeps the task-level orchestration, while TCPClient handles the
socket transport, AgentDrone wraps drone commands, and GuidanceAgent wraps the
guidance module.
"""

from __future__ import annotations

import argparse
import base64
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PYTHONCLIENT_DIR = os.path.dirname(SCRIPT_DIR)
if PYTHONCLIENT_DIR not in sys.path:
    sys.path.insert(0, PYTHONCLIENT_DIR)

from gradsim import AgentDrone, GuidanceAgent, TCPClient

BP_DRONE_CLASS = "/Game/Blueprints/BP_Drone.BP_Drone_C"
WINDOW_NAME = "auto_spawn_visual_intercept"
MAX_TRANSIENT_ERRORS = 3


@dataclass(frozen=True)
class OrbitPlan:
    center_m: np.ndarray
    radius_m: float
    omega_rad_s: float


@dataclass
class LoopResult:
    exit_reason: str
    captured: bool = False
    last_response: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RuntimeAgents:
    client: TCPClient
    interceptor: AgentDrone
    target: AgentDrone
    guidance: GuidanceAgent


def _ok(resp: Any) -> bool:
    return AgentDrone.is_ok(resp)


def _decode_bgr(data_b64: str):
    if cv2 is None or not data_b64:
        return None
    try:
        raw = base64.b64decode(data_b64)
    except Exception:
        return None
    arr = np.frombuffer(raw, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


def _ensure_removed(client: TCPClient, actor_id: str) -> None:
    response = AgentDrone.unwrap_response(client.request({"remove_actor": {"actor_id": actor_id}}), "remove_actor_return")
    if _ok(response):
        print(f"[INFO] removed existing actor: {actor_id}")


def _spawn_drone(
    drone: AgentDrone,
    classname: str,
    label: str,
    role: str,
    x: float,
    y: float,
    z: float,
    yaw: float = 0.0,
) -> bool:
    drone.mission_role = role
    drone.label = label
    response = drone.create_raw(
        spawn_x=x,
        spawn_y=y,
        spawn_z=z,
        yaw=yaw,
        classname=classname,
        label=label,
    )
    if _ok(response):
        drone.classname = classname
        drone.label = label
        drone.mission_role = role
        return True

    if classname != "DronePawn":
        print(f"[WARN] spawn with '{classname}' failed: {response}")
        fallback = drone.create_raw(
            spawn_x=x,
            spawn_y=y,
            spawn_z=z,
            yaw=yaw,
            classname="DronePawn",
            label=label,
        )
        if _ok(fallback):
            print("[WARN] fallback class 'DronePawn' used (may have no visible mesh).")
            drone.classname = "DronePawn"
            drone.label = label
            drone.mission_role = role
            return True
        print(f"[ERROR] fallback spawn failed: {fallback}")
        return False

    print(f"[ERROR] spawn failed: {response}")
    return False


def _drone_state(drone: AgentDrone) -> Dict[str, Any]:
    response = drone.get_state(frame="ue")
    return response if isinstance(response, dict) else {}


def _drone_pos(drone: AgentDrone) -> np.ndarray:
    state = _drone_state(drone)
    pos = state.get("position", [0.0, 0.0, 0.0]) if isinstance(state, dict) else [0.0, 0.0, 0.0]
    return np.array(pos[:3], dtype=np.float64)


def _wait_for_altitude(
    drone: AgentDrone,
    target_altitude: float,
    timeout_s: float = 20.0,
    tolerance_m: float = 0.8,
) -> bool:
    deadline = time.time() + max(1.0, timeout_s)
    while time.time() < deadline:
        state = _drone_state(drone)
        if _ok(state):
            pos = state.get("position", [0.0, 0.0, 0.0])
            if isinstance(pos, list) and len(pos) >= 3:
                altitude = float(pos[2])
                if altitude >= (target_altitude - tolerance_m):
                    return True
        time.sleep(0.25)
    return False


def _camera_basis_from_pitch_yaw(pitch_deg: float, yaw_deg: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    pitch = math.radians(float(pitch_deg))
    yaw = math.radians(float(yaw_deg))

    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    forward = np.array([cp * cy, cp * sy, sp], dtype=np.float64)
    right = np.array([-sy, cy, 0.0], dtype=np.float64)
    up = np.cross(forward, right)

    def _norm(vec: np.ndarray) -> np.ndarray:
        length = float(np.linalg.norm(vec))
        if length < 1e-8:
            return vec
        return vec / length

    return _norm(forward), _norm(right), _norm(up)


def _project_target_pixel(image_resp: Dict[str, Any], target_pos_m: np.ndarray) -> Optional[Tuple[int, int, float]]:
    cam_pos = image_resp.get("camera_pos", [0.0, 0.0, 0.0])
    cam_rot = image_resp.get("camera_rot", [0.0, 0.0, 0.0])
    fov = float(image_resp.get("fov", 90.0))
    width = int(image_resp.get("width", 0))
    height = int(image_resp.get("height", 0))

    if width <= 0 or height <= 0:
        return None
    if not (isinstance(cam_pos, list) and len(cam_pos) >= 3 and isinstance(cam_rot, list) and len(cam_rot) >= 2):
        return None

    cam_pos_cm = np.array(cam_pos[:3], dtype=np.float64)
    target_cm = target_pos_m * 100.0
    rel = target_cm - cam_pos_cm

    forward, right, up = _camera_basis_from_pitch_yaw(float(cam_rot[0]), float(cam_rot[1]))
    x_cam = float(np.dot(rel, forward))
    y_cam = float(np.dot(rel, right))
    z_cam = float(np.dot(rel, up))

    if x_cam <= 1e-3:
        return None

    focal = (0.5 * width) / max(math.tan(math.radians(max(1e-3, fov) * 0.5)), 1e-6)
    u = int(round(0.5 * width + focal * (y_cam / x_cam)))
    v = int(round(0.5 * height - focal * (z_cam / x_cam)))
    return u, v, x_cam


def _compute_target_reference(
    elapsed_s: float,
    orbit_center_m: np.ndarray,
    cruise_altitude_m: float,
    speed_mps: float,
    turn_rate_rad_s: float,
    vertical_amp_m: float,
) -> Tuple[np.ndarray, float]:
    speed = max(0.1, float(speed_mps))
    omega = max(0.01, abs(float(turn_rate_rad_s)))
    radius = max(6.0, speed / omega)
    theta = omega * float(elapsed_s)

    x = float(orbit_center_m[0]) + radius * math.sin(theta)
    y = float(orbit_center_m[1]) - radius * math.cos(theta)
    z = float(cruise_altitude_m) + float(vertical_amp_m) * math.sin(0.35 * theta)

    vx = radius * omega * math.cos(theta)
    vy = radius * omega * math.sin(theta)
    yaw_deg = math.degrees(math.atan2(vy, vx)) if (abs(vx) + abs(vy)) > 1e-6 else 0.0
    return np.array([x, y, z], dtype=np.float64), yaw_deg


def _send_target_reference(
    target: AgentDrone,
    elapsed_s: float,
    orbit_center_m: np.ndarray,
    target_altitude_m: float,
    target_speed: float,
    target_turn_rate: float,
    target_vertical_amp: float,
) -> Tuple[Dict[str, Any], np.ndarray, np.ndarray]:
    target_pos = _drone_pos(target)
    target_ref, yaw_deg = _compute_target_reference(
        elapsed_s=elapsed_s,
        orbit_center_m=orbit_center_m,
        cruise_altitude_m=target_altitude_m,
        speed_mps=target_speed,
        turn_rate_rad_s=target_turn_rate,
        vertical_amp_m=target_vertical_amp,
    )
    response = target.move_to(
        x=float(target_ref[0]),
        y=float(target_ref[1]),
        z=float(target_ref[2]),
        speed=float(target_speed),
        frame="ue",
        yaw_mode={"is_rate": False, "yaw_or_rate": yaw_deg},
        drivetrain="forward_only",
    )
    return response, target_pos, target_ref


def _draw_overlay(
    frame,
    image_resp: Dict[str, Any],
    interceptor_id: str,
    target_id: str,
    guidance_state: Dict[str, Any],
    interceptor_pos: np.ndarray,
    target_pos: np.ndarray,
    target_ref: np.ndarray,
    elapsed_s: float,
):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2

    cv2.drawMarker(frame, (cx, cy), (220, 220, 220), cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)

    proj = _project_target_pixel(image_resp, target_pos)
    if proj is not None:
        px, py, depth_cm = proj
        if 0 <= px < w and 0 <= py < h:
            cv2.circle(frame, (px, py), 10, (70, 220, 80), 2, cv2.LINE_AA)
            cv2.line(frame, (cx, cy), (px, py), (80, 180, 255), 1, cv2.LINE_AA)
            cv2.putText(
                frame,
                f"target_proj depth={depth_cm / 100.0:.1f}m",
                (max(8, px - 120), max(20, py - 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (70, 220, 80),
                1,
                cv2.LINE_AA,
            )

    intercept = guidance_state.get("intercept", {}) if isinstance(guidance_state, dict) else {}
    cmd_vel = intercept.get("cmd_velocity", [0.0, 0.0, 0.0]) if isinstance(intercept, dict) else [0.0, 0.0, 0.0]
    if not isinstance(cmd_vel, (list, tuple)) or len(cmd_vel) < 3:
        cmd_vel = [0.0, 0.0, 0.0]

    lines = [
        f"AUTO INTERCEPT | t={elapsed_s:.1f}s",
        f"interceptor={interceptor_id} target={target_id}",
        f"distance={float(intercept.get('distance', 0.0)):.2f}m closing={float(intercept.get('closing_speed', 0.0)):.2f}m/s",
        f"captured={bool(intercept.get('captured', False))} valid={bool(intercept.get('valid', False))}",
        f"cmd_vel=({float(cmd_vel[0]):+.2f}, {float(cmd_vel[1]):+.2f}, {float(cmd_vel[2]):+.2f}) m/s",
        f"intr_pos=({interceptor_pos[0]:+.1f}, {interceptor_pos[1]:+.1f}, {interceptor_pos[2]:+.1f})",
        f"tgt_pos =({target_pos[0]:+.1f}, {target_pos[1]:+.1f}, {target_pos[2]:+.1f})",
        f"tgt_ref =({target_ref[0]:+.1f}, {target_ref[1]:+.1f}, {target_ref[2]:+.1f})",
        "keys: q=quit",
    ]

    for idx, text in enumerate(lines):
        y = 22 + idx * 20
        cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.53, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.53, (235, 235, 235), 1, cv2.LINE_AA)

    return frame


def _build_orbit_plan(args: argparse.Namespace) -> OrbitPlan:
    omega = max(0.01, abs(float(args.target_turn_rate)))
    radius = max(6.0, float(args.target_speed) / omega)
    center = np.array(
        [
            float(args.target_spawn_x),
            float(args.target_spawn_y) + radius,
            float(args.target_altitude),
        ],
        dtype=np.float64,
    )
    return OrbitPlan(center_m=center, radius_m=radius, omega_rad_s=omega)


def _clean_existing_actors(runtime: RuntimeAgents, args: argparse.Namespace) -> None:
    if not args.clean_existing:
        return
    _ensure_removed(runtime.client, runtime.interceptor.actor_id)
    _ensure_removed(runtime.client, runtime.target.actor_id)
    time.sleep(0.2)


def _spawn_requested_drones(runtime: RuntimeAgents, args: argparse.Namespace) -> Tuple[bool, bool]:
    created_interceptor = _spawn_drone(
        drone=runtime.interceptor,
        classname=args.drone_class,
        label="Interceptor",
        role="interceptor",
        x=args.interceptor_spawn_x,
        y=args.interceptor_spawn_y,
        z=args.interceptor_spawn_z,
        yaw=0.0,
    )
    if not created_interceptor:
        return False, False

    created_target = _spawn_drone(
        drone=runtime.target,
        classname=args.drone_class,
        label="Target",
        role="target",
        x=args.target_spawn_x,
        y=args.target_spawn_y,
        z=args.target_spawn_z,
        yaw=180.0,
    )
    return created_interceptor, created_target


def _verify_spawned_agents(runtime: RuntimeAgents, args: argparse.Namespace) -> bool:
    time.sleep(0.3)
    current_ids = set(runtime.client.get_agents())
    missing = [actor_id for actor_id in (args.interceptor_id, args.target_id) if actor_id not in current_ids]
    if missing:
        print(f"[ERROR] spawned actors missing in registry: {missing}, current={sorted(current_ids)}")
        return False
    print(f"[INFO] interceptor={args.interceptor_id}, target={args.target_id}")
    return True


def _takeoff_pair(runtime: RuntimeAgents, args: argparse.Namespace) -> bool:
    print("[INFO] takeoff start")
    runtime.interceptor.takeoff(altitude=args.interceptor_altitude)
    runtime.target.takeoff(altitude=args.target_altitude)

    if not _wait_for_altitude(runtime.interceptor, args.interceptor_altitude):
        print(f"[ERROR] interceptor '{args.interceptor_id}' did not reach takeoff altitude")
        return False
    if not _wait_for_altitude(runtime.target, args.target_altitude):
        print(f"[ERROR] target '{args.target_id}' did not reach takeoff altitude")
        return False

    print("[INFO] both drones airborne")
    runtime.interceptor.set_camera_angles(0.0, 0.0)
    runtime.guidance.reset()
    return True


def _run_target_lead_phase(
    runtime: RuntimeAgents,
    args: argparse.Namespace,
    orbit_plan: OrbitPlan,
    loop_dt: float,
) -> Optional[str]:
    print(f"[INFO] target lead time: {args.target_lead_time:.1f}s")
    lead_start = time.time()
    lead_trace_t = -1.0

    while True:
        tick_start = time.time()
        lead_elapsed = tick_start - lead_start
        if lead_elapsed >= args.target_lead_time:
            return None

        move_resp, target_pos, target_ref = _send_target_reference(
            target=runtime.target,
            elapsed_s=lead_elapsed,
            orbit_center_m=orbit_plan.center_m,
            target_altitude_m=args.target_altitude,
            target_speed=args.target_speed,
            target_turn_rate=args.target_turn_rate,
            target_vertical_amp=args.target_vertical_amp,
        )
        if not _ok(move_resp):
            print(f"[ERROR] target lead reference failed: {move_resp}")
            return "error: target lead reference failed"

        if (lead_elapsed - lead_trace_t) >= max(0.1, float(args.telemetry_interval)):
            print(
                "[TRACE] "
                f"lead t={lead_elapsed:6.2f}s "
                f"tgt=({target_pos[0]:+.1f},{target_pos[1]:+.1f},{target_pos[2]:+.1f}) "
                f"ref=({target_ref[0]:+.1f},{target_ref[1]:+.1f},{target_ref[2]:+.1f})"
            )
            lead_trace_t = lead_elapsed

        sleep_s = loop_dt - (time.time() - tick_start)
        if sleep_s > 0:
            time.sleep(sleep_s)


def _open_visual_window() -> None:
    if cv2 is None:
        return
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1080, 620)


def _update_visualization(
    runtime: RuntimeAgents,
    args: argparse.Namespace,
    guidance_state: Dict[str, Any],
    interceptor_pos: np.ndarray,
    target_pos: np.ndarray,
    target_ref: np.ndarray,
    elapsed_s: float,
) -> Optional[str]:
    if not args.show or cv2 is None:
        return None

    image_resp = runtime.interceptor.get_image_raw(image_type="scene", quality=82)
    if not image_resp or not image_resp.get("data"):
        return None

    frame = _decode_bgr(image_resp.get("data", ""))
    if frame is None:
        return None

    vis = _draw_overlay(
        frame=frame,
        image_resp=image_resp,
        interceptor_id=runtime.interceptor.actor_id,
        target_id=runtime.target.actor_id,
        guidance_state=guidance_state,
        interceptor_pos=interceptor_pos,
        target_pos=target_pos,
        target_ref=target_ref,
        elapsed_s=elapsed_s,
    )
    cv2.imshow(WINDOW_NAME, vis)
    if (cv2.waitKey(1) & 0xFF) == ord("q"):
        print("[INFO] user quit visualization window")
        return "user quit"
    return None


def _run_intercept_loop(
    runtime: RuntimeAgents,
    args: argparse.Namespace,
    orbit_plan: OrbitPlan,
    loop_dt: float,
) -> LoopResult:
    t0 = time.time()
    last_resp: Dict[str, Any] = {}
    last_telemetry_t = -1.0
    move_error_count = 0
    intercept_error_count = 0

    print("[INFO] interception loop started (serial target reference + C++ auto_intercept)")
    while True:
        tick_start = time.time()
        elapsed = tick_start - t0
        if elapsed > args.max_time:
            print("[INFO] timeout reached")
            return LoopResult(exit_reason="timeout", last_response=last_resp)

        move_resp, target_pos, target_ref = _send_target_reference(
            target=runtime.target,
            elapsed_s=args.target_lead_time + elapsed,
            orbit_center_m=orbit_plan.center_m,
            target_altitude_m=args.target_altitude,
            target_speed=args.target_speed,
            target_turn_rate=args.target_turn_rate,
            target_vertical_amp=args.target_vertical_amp,
        )
        if not _ok(move_resp):
            move_error_count += 1
            print(f"[WARN] target reference failed ({move_error_count}/{MAX_TRANSIENT_ERRORS}): {move_resp}")
            if move_error_count >= MAX_TRANSIENT_ERRORS:
                return LoopResult(exit_reason="error: target reference failed", last_response=last_resp)
            time.sleep(0.2)
            continue
        move_error_count = 0

        intercept_resp = runtime.guidance.auto_intercept(
            interceptor_id=runtime.interceptor.actor_id,
            target_id=runtime.target.actor_id,
            method=args.method,
            speed=args.intercept_speed,
            nav_gain=args.nav_gain,
            lead_time=args.lead_time,
            capture_radius=args.capture_radius,
            stop_on_capture=True,
        )
        last_resp = intercept_resp
        if not _ok(intercept_resp):
            intercept_error_count += 1
            print(f"[WARN] auto_intercept failed ({intercept_error_count}/{MAX_TRANSIENT_ERRORS}): {intercept_resp}")
            if intercept_error_count >= MAX_TRANSIENT_ERRORS:
                return LoopResult(exit_reason="error: auto_intercept failed", last_response=last_resp)
            time.sleep(0.2)
            continue
        intercept_error_count = 0

        guidance_state = runtime.guidance.state()
        if not _ok(guidance_state):
            guidance_state = {}

        intercept_state = guidance_state.get("intercept", {}) if isinstance(guidance_state, dict) else {}
        interceptor_pos = _drone_pos(runtime.interceptor)
        distance = float(np.linalg.norm(target_pos - interceptor_pos))
        captured = bool(intercept_resp.get("captured", False)) or bool(intercept_state.get("captured", False))

        if (last_telemetry_t < 0.0) or ((elapsed - last_telemetry_t) >= max(0.1, float(args.telemetry_interval))):
            print(
                "[TRACE] "
                f"t={elapsed:6.2f}s dist={distance:6.2f}m "
                f"intr=({interceptor_pos[0]:+.1f},{interceptor_pos[1]:+.1f},{interceptor_pos[2]:+.1f}) "
                f"tgt=({target_pos[0]:+.1f},{target_pos[1]:+.1f},{target_pos[2]:+.1f}) "
                f"ref=({target_ref[0]:+.1f},{target_ref[1]:+.1f},{target_ref[2]:+.1f}) "
                f"cmd={intercept_resp.get('cmd_velocity', [0.0, 0.0, 0.0])}"
            )
            last_telemetry_t = elapsed

        capture_reason = None
        if captured:
            print(f"[INFO] captured target at t={elapsed:.2f}s")
            capture_reason = "captured"

        vis_exit = _update_visualization(
            runtime=runtime,
            args=args,
            guidance_state=guidance_state,
            interceptor_pos=interceptor_pos,
            target_pos=target_pos,
            target_ref=target_ref,
            elapsed_s=elapsed,
        )
        if vis_exit is not None:
            return LoopResult(exit_reason=vis_exit, captured=captured, last_response=last_resp)
        if capture_reason is not None:
            return LoopResult(exit_reason=capture_reason, captured=True, last_response=last_resp)

        sleep_s = loop_dt - (time.time() - tick_start)
        if sleep_s > 0:
            time.sleep(sleep_s)


def _cleanup_runtime(
    runtime: RuntimeAgents,
    args: argparse.Namespace,
    created_interceptor: bool,
    created_target: bool,
    exit_reason: str,
) -> None:
    for created, drone in (
        (created_target, runtime.target),
        (created_interceptor, runtime.interceptor),
    ):
        if not created:
            continue
        try:
            drone.hover()
        except Exception:
            pass

    if args.show and cv2 is not None:
        cv2.destroyAllWindows()

    should_cleanup = not args.keep_actors
    if should_cleanup and exit_reason.startswith("error:"):
        should_cleanup = False
        print("[WARN] keeping actors in scene for debugging because the script exited on error")

    if should_cleanup:
        if created_target:
            try:
                print("[INFO] remove target:", runtime.target.remove_raw())
            except Exception:
                pass
        if created_interceptor:
            try:
                print("[INFO] remove interceptor:", runtime.interceptor.remove_raw())
            except Exception:
                pass

    runtime.client.close()


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Spawn two drones and run C++ auto_intercept with visualization")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--timeout", type=float, default=10.0)

    parser.add_argument("--drone-class", default=BP_DRONE_CLASS)
    parser.add_argument("--interceptor-id", default="drone_0")
    parser.add_argument("--target-id", default="drone_1")

    parser.add_argument("--interceptor-spawn-x", type=float, default=0.0, help="meters")
    parser.add_argument("--interceptor-spawn-y", type=float, default=0.0, help="meters")
    parser.add_argument("--interceptor-spawn-z", type=float, default=1.0, help="meters")

    parser.add_argument("--target-spawn-x", type=float, default=35.0, help="meters")
    parser.add_argument("--target-spawn-y", type=float, default=12.0, help="meters")
    parser.add_argument("--target-spawn-z", type=float, default=1.0, help="meters")

    parser.add_argument("--interceptor-altitude", type=float, default=7.0)
    parser.add_argument("--target-altitude", type=float, default=8.0)

    parser.add_argument("--target-speed", type=float, default=6.0)
    parser.add_argument("--target-turn-rate", type=float, default=0.30)
    parser.add_argument("--target-vertical-amp", type=float, default=0.0)
    parser.add_argument("--target-lead-time", type=float, default=5.0)
    parser.add_argument("--telemetry-interval", type=float, default=0.5)

    parser.add_argument("--method", default="pure_pursuit", choices=["pure_pursuit", "proportional_nav", "smc"])
    parser.add_argument("--intercept-speed", type=float, default=8.0)
    parser.add_argument("--nav-gain", type=float, default=3.0)
    parser.add_argument("--lead-time", type=float, default=0.6)
    parser.add_argument("--capture-radius", type=float, default=1.5)

    parser.add_argument("--hz", type=float, default=12.0)
    parser.add_argument("--max-time", type=float, default=120.0)

    parser.add_argument("--show", dest="show", action="store_true")
    parser.add_argument("--no-show", dest="show", action="store_false")
    parser.set_defaults(show=True)

    parser.add_argument("--clean-existing", dest="clean_existing", action="store_true")
    parser.add_argument("--no-clean-existing", dest="clean_existing", action="store_false")
    parser.set_defaults(clean_existing=True)

    parser.add_argument("--keep-actors", action="store_true", help="do not remove spawned actors when script exits")
    return parser


def main() -> None:
    args = build_argparser().parse_args()

    if args.show and cv2 is None:
        print("[WARN] opencv-python not installed, visualization disabled.")
        args.show = False

    client = TCPClient(host=args.host, port=args.port, timeout=args.timeout, auto_connect=False)
    if not client.connect() or not _ok(client.ping()):
        print("[ERROR] cannot connect to simulator")
        client.close()
        return

    runtime = RuntimeAgents(
        client=client,
        interceptor=AgentDrone(
            client,
            args.interceptor_id,
            classname=args.drone_class,
            label="Interceptor",
            mission_role="interceptor",
        ),
        target=AgentDrone(
            client,
            args.target_id,
            classname=args.drone_class,
            label="Target",
            mission_role="target",
        ),
        guidance=GuidanceAgent(client),
    )

    created_interceptor = False
    created_target = False
    exit_reason = "finished"

    try:
        _clean_existing_actors(runtime, args)

        created_interceptor, created_target = _spawn_requested_drones(runtime, args)
        if not created_interceptor:
            exit_reason = "error: interceptor spawn failed"
            return
        if not created_target:
            exit_reason = "error: target spawn failed"
            return

        if not _verify_spawned_agents(runtime, args):
            exit_reason = "error: spawned actors missing in registry"
            return

        if not _takeoff_pair(runtime, args):
            exit_reason = "error: takeoff or altitude wait failed"
            return

        loop_dt = 1.0 / max(2.0, float(args.hz))
        orbit_plan = _build_orbit_plan(args)
        print(
            f"[INFO] target orbit center=({orbit_plan.center_m[0]:.1f}, {orbit_plan.center_m[1]:.1f}, "
            f"{orbit_plan.center_m[2]:.1f}) radius={orbit_plan.radius_m:.1f}m "
            f"omega={orbit_plan.omega_rad_s:.2f}rad/s"
        )

        lead_error = _run_target_lead_phase(runtime, args, orbit_plan, loop_dt)
        if lead_error is not None:
            exit_reason = lead_error
            return

        if args.show:
            _open_visual_window()

        result = _run_intercept_loop(runtime, args, orbit_plan, loop_dt)
        exit_reason = result.exit_reason
        print("[RESULT]", result.last_response)
        print(f"[INFO] exit reason: {exit_reason}")

    finally:
        _cleanup_runtime(
            runtime=runtime,
            args=args,
            created_interceptor=created_interceptor,
            created_target=created_target,
            exit_reason=exit_reason,
        )


if __name__ == "__main__":
    main()

