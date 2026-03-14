from __future__ import annotations

import argparse
from typing import Any

from GraduationSIM import DEFAULT_DRONE_CLASS, Pose, run_task


# 入口层只负责组装配置字典，运行细节交给 Task 处理。
def _add_toggle(parser: argparse.ArgumentParser, name: str, default: bool) -> None:
    dest = name.replace("-", "_")
    parser.add_argument(f"--{name}", dest=dest, action="store_true")
    parser.add_argument(f"--no-{name}", dest=dest, action="store_false")
    parser.set_defaults(**{dest: default})


def _add_pose_args(parser: argparse.ArgumentParser, prefix: str, defaults: dict[str, float]) -> None:
    for key, value in defaults.items():
        parser.add_argument(f"--{prefix}-{key}", type=float, default=value)


def _pose_from_args(args: argparse.Namespace, prefix: str) -> Pose:
    return Pose(
        x=getattr(args, f"{prefix}_spawn_x"),
        y=getattr(args, f"{prefix}_spawn_y"),
        z=getattr(args, f"{prefix}_spawn_z"),
        roll=getattr(args, f"{prefix}_roll"),
        pitch=getattr(args, f"{prefix}_pitch"),
        yaw=getattr(args, f"{prefix}_yaw"),
    )


def _drone_from_args(
    args: argparse.Namespace,
    prefix: str,
    label: str,
    role: str,
    classname: str | None,
    *,
    with_camera: bool = False,
) -> dict[str, Any]:
    config: dict[str, Any] = {
        "actor_id": getattr(args, f"{prefix}_id"),
        "label": label,
        "role": role,
        "classname": classname,
        "spawn_pose": _pose_from_args(args, prefix),
        "altitude": getattr(args, f"{prefix}_altitude"),
        "camera_pitch": 0.0,
        "camera_yaw": 0.0,
    }
    if with_camera:
        config["camera_pitch"] = args.interceptor_camera_pitch
        config["camera_yaw"] = args.interceptor_camera_yaw
    return config


def _pick(args: argparse.Namespace, mapping: dict[str, str]) -> dict[str, object]:
    return {target: getattr(args, source) for target, source in mapping.items()}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Spawn two drones and run YOLO-based visual intercept in GraduationSIM",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--timeout", type=float, default=10.0)

    parser.add_argument("--drone-class", default=None)
    parser.add_argument("--interceptor-class", default=None)
    parser.add_argument("--target-class", default=None)
    parser.add_argument("--guidance-class", default="GuidanceActor")

    parser.add_argument("--interceptor-id", default="drone_0")
    parser.add_argument("--target-id", default="drone_1")
    parser.add_argument("--guidance-id", default="guidance_0")

    _add_pose_args(
        parser,
        "interceptor",
        {
            "spawn-x": 0.0,
            "spawn-y": 0.0,
            "spawn-z": 1.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
    )
    parser.add_argument("--interceptor-altitude", type=float, default=7.0)
    parser.add_argument("--interceptor-camera-pitch", type=float, default=0.0)
    parser.add_argument("--interceptor-camera-yaw", type=float, default=0.0)

    _add_pose_args(
        parser,
        "target",
        {
            "spawn-x": 25.0,
            "spawn-y": -12.0,
            "spawn-z": 1.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 180.0,
        },
    )
    parser.add_argument("--target-altitude", type=float, default=8.0)

    parser.add_argument(
        "--trajectory-kind",
        default="circle",
        choices=["circle", "line", "waypoints", "figure8"],
        help="Only circle is implemented right now; the others are reserved for later.",
    )
    parser.add_argument("--target-speed", type=float, default=6.0)
    parser.add_argument("--target-turn-rate", type=float, default=0.30)
    parser.add_argument("--target-vertical-amp", type=float, default=0.0)
    parser.add_argument("--target-lead-time", type=float, default=5.0)
    parser.add_argument("--target-radius", type=float, default=None)

    parser.add_argument("--method", default="vision_pid_kalman")
    parser.add_argument("--model", default=None)
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default=None)
    parser.add_argument("--class-id", type=int, default=0, help="Set -1 for all classes")
    parser.add_argument("--max-det", type=int, default=5)
    parser.add_argument(
        "--selection-strategy",
        default="heuristic",
        choices=["heuristic", "highest_confidence"],
        help="Target selection policy when multiple YOLO boxes are present.",
    )
    parser.add_argument("--desired-area", type=float, default=None)
    parser.add_argument("--capture-area", type=float, default=None)
    parser.add_argument("--center-tol-x", type=float, default=None)
    parser.add_argument("--center-tol-y", type=float, default=None)
    parser.add_argument("--capture-hold-frames", type=int, default=None)
    parser.add_argument("--lost-to-search-frames", type=int, default=None)
    parser.add_argument("--intercept-speed", type=float, default=8.0, help="Legacy alias for max forward speed")
    parser.add_argument("--max-forward-speed", type=float, default=None)
    parser.add_argument("--max-reverse-speed", type=float, default=None)
    parser.add_argument("--max-vertical-speed", type=float, default=None)
    parser.add_argument("--max-yaw-rate-deg", type=float, default=None)
    parser.add_argument("--ram-area-target", type=float, default=None)
    parser.add_argument("--min-ram-speed", type=float, default=None)
    parser.add_argument("--intercept-distance", type=float, default=1.5)
    parser.add_argument("--search-cam-yaw-limit-deg", type=float, default=None)
    parser.add_argument("--search-cam-rate-deg", type=float, default=None)
    parser.add_argument("--search-body-yaw-rate-deg", type=float, default=None)
    parser.add_argument("--search-cam-pitch-deg", type=float, default=None)
    parser.add_argument("--search-vz-amp", type=float, default=None)
    _add_toggle(parser, "use-kalman", False)
    parser.set_defaults(use_kalman=None)
    _add_toggle(parser, "stop-on-capture", False)

    parser.add_argument("--hz", type=float, default=12.0)
    parser.add_argument("--max-time", type=float, default=120.0)
    parser.add_argument("--telemetry-interval", type=float, default=0.5)
    parser.add_argument("--image-quality", type=int, default=82)
    parser.add_argument("--window-name", default="graduation_visual_intercept")
    _add_toggle(parser, "show", True)
    _add_toggle(parser, "clean-existing", True)
    parser.add_argument("--keep-actors", action="store_true")
    return parser


def build_config(args: argparse.Namespace) -> dict[str, Any]:
    default_drone_class = args.drone_class or DEFAULT_DRONE_CLASS
    class_id = None if args.class_id is not None and args.class_id < 0 else args.class_id
    max_forward_speed = args.max_forward_speed if args.max_forward_speed is not None else args.intercept_speed

    return {
        "connection": {
            "host": args.host,
            "port": args.port,
            "timeout": args.timeout,
        },
        "interceptor": _drone_from_args(
            args,
            "interceptor",
            "Interceptor",
            "interceptor",
            args.interceptor_class or default_drone_class,
            with_camera=True,
        ),
        "target": _drone_from_args(
            args,
            "target",
            "Target",
            "target",
            args.target_class or default_drone_class,
        ),
        "guidance": {
            "actor_id": args.guidance_id,
            "classname": args.guidance_class,
            "label": "Guidance",
            "spawn_pose": Pose(),
        },
        "trajectory": {
            "kind": args.trajectory_kind,
            "speed": args.target_speed,
            "turn_rate": args.target_turn_rate,
            "vertical_amp": args.target_vertical_amp,
            "lead_time": args.target_lead_time,
            "radius": args.target_radius,
            "center": None,
            "line_endpoint": None,
            "figure8_radius": None,
            "waypoints": [],
        },
        "visual": {
            **_pick(
                args,
                {
                    "method": "method",
                    "conf": "conf",
                    "iou": "iou",
                    "imgsz": "imgsz",
                    "device": "device",
                    "max_det": "max_det",
                    "selection_strategy": "selection_strategy",
                    "desired_area": "desired_area",
                    "capture_area": "capture_area",
                    "center_tol_x": "center_tol_x",
                    "center_tol_y": "center_tol_y",
                    "capture_hold_frames": "capture_hold_frames",
                    "lost_to_search_frames": "lost_to_search_frames",
                    "max_reverse_speed": "max_reverse_speed",
                    "max_vertical_speed": "max_vertical_speed",
                    "max_yaw_rate_deg": "max_yaw_rate_deg",
                    "ram_area_target": "ram_area_target",
                    "min_ram_speed": "min_ram_speed",
                    "intercept_distance": "intercept_distance",
                    "search_cam_yaw_limit_deg": "search_cam_yaw_limit_deg",
                    "search_cam_rate_deg": "search_cam_rate_deg",
                    "search_body_yaw_rate_deg": "search_body_yaw_rate_deg",
                    "search_cam_pitch_deg": "search_cam_pitch_deg",
                    "search_vz_amp": "search_vz_amp",
                    "use_kalman": "use_kalman",
                    "stop_on_capture": "stop_on_capture",
                },
            ),
            "model_path": args.model,
            "class_id": class_id,
            "max_forward_speed": max_forward_speed,
        },
        "runtime": {
            "hz": args.hz,
            "max_time": args.max_time,
            "telemetry_interval": args.telemetry_interval,
            "show": args.show,
            "clean_existing": args.clean_existing,
            "keep_actors": args.keep_actors,
            "max_transient_errors": 3,
            "image_type": "scene",
            "image_quality": args.image_quality,
            "window_name": args.window_name,
        },
    }


def main() -> int:
    result = run_task(build_config(build_parser().parse_args()))
    return 1 if str(result.get("exit_reason", "")).startswith("error:") else 0


if __name__ == "__main__":
    raise SystemExit(main())

