"""Task script for the YOLO visual intercept demo."""

import argparse
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PYTHONCLIENT_DIR = os.path.dirname(SCRIPT_DIR)
if PYTHONCLIENT_DIR not in sys.path:
    sys.path.insert(0, PYTHONCLIENT_DIR)

from gradsim import run_visual_intercept


def build_parser():
    parser = argparse.ArgumentParser(
        description="Spawn two drones and run YOLO-based ram-style visual intercept with visualization"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--drone-class")

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

    parser.add_argument("--method", default="vision_pid_kalman", help="visual intercept method")
    parser.add_argument("--intercept-speed", type=float, default=8.0, help="legacy alias for max forward speed")
    parser.add_argument("--nav-gain", type=float, default=3.0, help=argparse.SUPPRESS)
    parser.add_argument("--lead-time", type=float, default=0.6, help=argparse.SUPPRESS)
    parser.add_argument("--capture-radius", type=float, default=1.5, help=argparse.SUPPRESS)

    parser.add_argument("--model", default=None, help="YOLO weights path; defaults to the newest local best.pt")
    parser.add_argument("--conf", type=float, default=0.35, help="YOLO confidence threshold")
    parser.add_argument("--iou", type=float, default=0.45, help="YOLO IoU threshold")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO inference size")
    parser.add_argument("--device", default=None, help="YOLO device, e.g. 0 or cpu")
    parser.add_argument("--class-id", type=int, default=0, help="filter detections by class id; set -1 for all")
    parser.add_argument("--max-det", type=int, default=5, help="maximum detections per frame")

    parser.add_argument("--desired-area", type=float, default=None)
    parser.add_argument("--capture-area", type=float, default=None)
    parser.add_argument("--center-tol-x", type=float, default=None)
    parser.add_argument("--center-tol-y", type=float, default=None)
    parser.add_argument("--capture-hold-frames", type=int, default=None)
    parser.add_argument("--lost-to-search-frames", type=int, default=None)
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

    parser.add_argument("--use-kalman", dest="use_kalman", action="store_true")
    parser.add_argument("--no-kalman", dest="use_kalman", action="store_false")
    parser.set_defaults(use_kalman=None)

    parser.add_argument("--stop-on-capture", dest="stop_on_capture", action="store_true")
    parser.add_argument("--no-stop-on-capture", dest="stop_on_capture", action="store_false")
    parser.set_defaults(stop_on_capture=False)

    parser.add_argument("--hz", type=float, default=12.0)
    parser.add_argument("--max-time", type=float, default=120.0)
    parser.add_argument("--telemetry-interval", type=float, default=0.5)

    parser.add_argument("--show", dest="show", action="store_true")
    parser.add_argument("--no-show", dest="show", action="store_false")
    parser.set_defaults(show=True)

    parser.add_argument("--clean-existing", dest="clean_existing", action="store_true")
    parser.add_argument("--no-clean-existing", dest="clean_existing", action="store_false")
    parser.set_defaults(clean_existing=True)
    parser.add_argument("--keep-actors", action="store_true")
    return parser


def build_target_trajectory(args):
    return {
        "kind": "circle",
        "speed": args.target_speed,
        "turn_rate": args.target_turn_rate,
        "vertical_amp": args.target_vertical_amp,
        "lead_time": args.target_lead_time,
    }


def build_task_config(args):
    visual_max_forward_speed = args.max_forward_speed
    if visual_max_forward_speed is None:
        visual_max_forward_speed = args.intercept_speed

    visual = {
        "method": args.method,
        "model_path": args.model,
        "conf": args.conf,
        "iou": args.iou,
        "imgsz": args.imgsz,
        "device": args.device,
        "class_id": None if args.class_id is not None and args.class_id < 0 else args.class_id,
        "max_det": args.max_det,
        "desired_area": args.desired_area,
        "capture_area": args.capture_area,
        "center_tol_x": args.center_tol_x,
        "center_tol_y": args.center_tol_y,
        "capture_hold_frames": args.capture_hold_frames,
        "lost_to_search_frames": args.lost_to_search_frames,
        "max_forward_speed": visual_max_forward_speed,
        "max_reverse_speed": args.max_reverse_speed,
        "max_vertical_speed": args.max_vertical_speed,
        "max_yaw_rate_deg": args.max_yaw_rate_deg,
        "ram_area_target": args.ram_area_target,
        "min_ram_speed": args.min_ram_speed,
        "intercept_distance": args.intercept_distance,
        "search_cam_yaw_limit_deg": args.search_cam_yaw_limit_deg,
        "search_cam_rate_deg": args.search_cam_rate_deg,
        "search_body_yaw_rate_deg": args.search_body_yaw_rate_deg,
        "search_cam_pitch_deg": args.search_cam_pitch_deg,
        "search_vz_amp": args.search_vz_amp,
        "use_kalman": args.use_kalman,
        "stop_on_capture": args.stop_on_capture,
    }

    config = {
        "connection": {
            "host": args.host,
            "port": args.port,
            "timeout": args.timeout,
        },
        "interceptor": {
            "id": args.interceptor_id,
            "label": "Interceptor",
            "role": "interceptor",
            "spawn": [args.interceptor_spawn_x, args.interceptor_spawn_y, args.interceptor_spawn_z],
            "yaw": 0.0,
            "altitude": args.interceptor_altitude,
        },
        "target": {
            "id": args.target_id,
            "label": "Target",
            "role": "target",
            "spawn": [args.target_spawn_x, args.target_spawn_y, args.target_spawn_z],
            "yaw": 180.0,
            "altitude": args.target_altitude,
            "trajectory": build_target_trajectory(args),
        },
        "guidance": {
            "id": "guidance_0",
            "classname": "GuidanceActor",
            "label": "Guidance",
        },
        "visual": visual,
        "runtime": {
            "hz": args.hz,
            "max_time": args.max_time,
            "telemetry_interval": args.telemetry_interval,
            "show": args.show,
            "clean_existing": args.clean_existing,
            "keep_actors": args.keep_actors,
            "max_transient_errors": 3,
        },
        "view": {
            "window_name": "auto_spawn_visual_intercept",
        },
    }
    if args.drone_class:
        config["drone_class"] = args.drone_class
    return config


def main():
    args = build_parser().parse_args()
    run_visual_intercept(build_task_config(args))


if __name__ == "__main__":
    main()
