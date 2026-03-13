from __future__ import annotations

import argparse

from GraduationSIM import (
    DEFAULT_DRONE_CLASS,
    ConnectionConfig,
    DroneConfig,
    GuidanceConfig,
    Pose,
    RuntimeConfig,
    TaskConfig,
    TrajectoryConfig,
    VisualConfig,
    run_task,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Spawn two drones and run YOLO-based visual intercept in GraduationSIM"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--no-auto-reconnect", dest="auto_reconnect", action="store_false")
    parser.set_defaults(auto_reconnect=True)

    parser.add_argument("--drone-class", default=None)
    parser.add_argument("--interceptor-class", default=None)
    parser.add_argument("--target-class", default=None)
    parser.add_argument("--guidance-class", default="GuidanceActor")

    parser.add_argument("--interceptor-id", default="drone_0")
    parser.add_argument("--target-id", default="drone_1")
    parser.add_argument("--guidance-id", default="guidance_0")

    parser.add_argument("--interceptor-spawn-x", type=float, default=0.0)
    parser.add_argument("--interceptor-spawn-y", type=float, default=0.0)
    parser.add_argument("--interceptor-spawn-z", type=float, default=1.0)
    parser.add_argument("--interceptor-roll", type=float, default=0.0)
    parser.add_argument("--interceptor-pitch", type=float, default=0.0)
    parser.add_argument("--interceptor-yaw", type=float, default=0.0)
    parser.add_argument("--interceptor-altitude", type=float, default=7.0)
    parser.add_argument("--interceptor-camera-pitch", type=float, default=0.0)
    parser.add_argument("--interceptor-camera-yaw", type=float, default=0.0)

    parser.add_argument("--target-spawn-x", type=float, default=15.0)
    parser.add_argument("--target-spawn-y", type=float, default=-12.0)
    parser.add_argument("--target-spawn-z", type=float, default=1.0)
    parser.add_argument("--target-roll", type=float, default=0.0)
    parser.add_argument("--target-pitch", type=float, default=0.0)
    parser.add_argument("--target-yaw", type=float, default=180.0)
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

    parser.add_argument("--use-kalman", dest="use_kalman", action="store_true")
    parser.add_argument("--no-kalman", dest="use_kalman", action="store_false")
    parser.set_defaults(use_kalman=None)

    parser.add_argument("--stop-on-capture", dest="stop_on_capture", action="store_true")
    parser.add_argument("--no-stop-on-capture", dest="stop_on_capture", action="store_false")
    parser.set_defaults(stop_on_capture=False)

    parser.add_argument("--hz", type=float, default=12.0)
    parser.add_argument("--max-time", type=float, default=120.0)
    parser.add_argument("--telemetry-interval", type=float, default=0.5)
    parser.add_argument("--image-quality", type=int, default=82)
    parser.add_argument("--max-depth-m", type=float, default=200.0)
    parser.add_argument("--window-name", default="graduation_visual_intercept")

    parser.add_argument("--show", dest="show", action="store_true")
    parser.add_argument("--no-show", dest="show", action="store_false")
    parser.set_defaults(show=True)

    parser.add_argument("--clean-existing", dest="clean_existing", action="store_true")
    parser.add_argument("--no-clean-existing", dest="clean_existing", action="store_false")
    parser.set_defaults(clean_existing=True)
    parser.add_argument("--keep-actors", action="store_true")
    return parser


def build_config(args: argparse.Namespace) -> TaskConfig:
    default_drone_class = args.drone_class or DEFAULT_DRONE_CLASS
    interceptor_class = args.interceptor_class or default_drone_class
    target_class = args.target_class or default_drone_class
    class_id = None if args.class_id is not None and args.class_id < 0 else args.class_id
    max_forward_speed = args.max_forward_speed if args.max_forward_speed is not None else args.intercept_speed

    connection = ConnectionConfig(
        host=args.host,
        port=args.port,
        timeout=args.timeout,
        auto_reconnect=args.auto_reconnect,
    )
    interceptor = DroneConfig(
        actor_id=args.interceptor_id,
        label="Interceptor",
        role="interceptor",
        classname=interceptor_class,
        spawn_pose=Pose(
            x=args.interceptor_spawn_x,
            y=args.interceptor_spawn_y,
            z=args.interceptor_spawn_z,
            roll=args.interceptor_roll,
            pitch=args.interceptor_pitch,
            yaw=args.interceptor_yaw,
        ),
        altitude=args.interceptor_altitude,
        camera_pitch=args.interceptor_camera_pitch,
        camera_yaw=args.interceptor_camera_yaw,
    )
    target = DroneConfig(
        actor_id=args.target_id,
        label="Target",
        role="target",
        classname=target_class,
        spawn_pose=Pose(
            x=args.target_spawn_x,
            y=args.target_spawn_y,
            z=args.target_spawn_z,
            roll=args.target_roll,
            pitch=args.target_pitch,
            yaw=args.target_yaw,
        ),
        altitude=args.target_altitude,
    )
    guidance = GuidanceConfig(
        actor_id=args.guidance_id,
        classname=args.guidance_class,
        label="Guidance",
    )
    trajectory = TrajectoryConfig(
        kind=args.trajectory_kind,
        speed=args.target_speed,
        turn_rate=args.target_turn_rate,
        vertical_amp=args.target_vertical_amp,
        lead_time=args.target_lead_time,
        radius=args.target_radius,
    )
    visual = VisualConfig(
        method=args.method,
        model_path=args.model,
        conf=args.conf,
        iou=args.iou,
        imgsz=args.imgsz,
        device=args.device,
        class_id=class_id,
        max_det=args.max_det,
        selection_strategy=args.selection_strategy,
        desired_area=args.desired_area,
        capture_area=args.capture_area,
        center_tol_x=args.center_tol_x,
        center_tol_y=args.center_tol_y,
        capture_hold_frames=args.capture_hold_frames,
        lost_to_search_frames=args.lost_to_search_frames,
        max_forward_speed=max_forward_speed,
        max_reverse_speed=args.max_reverse_speed,
        max_vertical_speed=args.max_vertical_speed,
        max_yaw_rate_deg=args.max_yaw_rate_deg,
        ram_area_target=args.ram_area_target,
        min_ram_speed=args.min_ram_speed,
        intercept_distance=args.intercept_distance,
        search_cam_yaw_limit_deg=args.search_cam_yaw_limit_deg,
        search_cam_rate_deg=args.search_cam_rate_deg,
        search_body_yaw_rate_deg=args.search_body_yaw_rate_deg,
        search_cam_pitch_deg=args.search_cam_pitch_deg,
        search_vz_amp=args.search_vz_amp,
        use_kalman=args.use_kalman,
        stop_on_capture=args.stop_on_capture,
    )
    runtime = RuntimeConfig(
        hz=args.hz,
        max_time=args.max_time,
        telemetry_interval=args.telemetry_interval,
        show=args.show,
        clean_existing=args.clean_existing,
        keep_actors=args.keep_actors,
        image_quality=args.image_quality,
        max_depth_m=args.max_depth_m,
        window_name=args.window_name,
    )
    return TaskConfig(
        connection=connection,
        interceptor=interceptor,
        target=target,
        guidance=guidance,
        trajectory=trajectory,
        visual=visual,
        runtime=runtime,
    )


def main() -> int:
    args = build_parser().parse_args()
    result = run_task(build_config(args))
    return 1 if result.exit_reason.startswith("error:") else 0


if __name__ == "__main__":
    raise SystemExit(main())
