from __future__ import annotations

import math
import time
from typing import Optional, Tuple

from .AgentBase import AgentBase
from .AgentDrone import DEFAULT_DRONE_CLASS, AgentDrone
from .AgentGuidance import AgentGuidance
from .DataTypes import DetectionResult, GuidanceState, TaskConfig, TaskResult, TrajectoryReference
from .TCPClient import TCPClient
from .Trajectory import TrajectoryFactory
from .UI import VisualUI
from .Yolo import YoloDetector


class Task:
    def __init__(self, config: TaskConfig, view: Optional[VisualUI] = None) -> None:
        self.config = config
        self.view = view or VisualUI(show=config.runtime.show, window_name=config.runtime.window_name)
        self.client: Optional[TCPClient] = None
        self.interceptor_agent: Optional[AgentDrone] = None
        self.target_agent: Optional[AgentDrone] = None
        self.guidance_agent: Optional[AgentGuidance] = None
        self.detector: Optional[YoloDetector] = None
        self.created_guidance = False
        self.created_interceptor = False
        self.created_target = False
        self.visual_session_started = False
        self.exit_reason = "finished"

    def _ok(self, response) -> bool:
        return AgentBase.is_ok(response)

    def _actor_ids(self):
        return {
            "guidance": self.config.guidance.actor_id,
            "interceptor": self.config.interceptor.actor_id,
            "target": self.config.target.actor_id,
        }

    def _iter_runtime_actors(self):
        return (
            ("target", self.created_target, self.target_agent),
            ("interceptor", self.created_interceptor, self.interceptor_agent),
            ("guidance", self.created_guidance, self.guidance_agent),
        )

    def _iter_runtime_drones(self):
        return self._iter_runtime_actors()[:2]

    def _ensure_removed(self, actor_id: str) -> None:
        if self.client is None:
            return
        response = AgentBase.unwrap_response(
            self.client.request({"remove_actor": {"actor_id": actor_id}}),
            "remove_actor_return",
        )
        if self._ok(response):
            print(f"[INFO] removed existing actor: {actor_id}")

    def _spawn_drone(self, drone: AgentDrone, drone_config) -> bool:
        drone.mission_role = drone_config.role
        drone.label = drone_config.label

        primary_class = drone_config.classname or DEFAULT_DRONE_CLASS
        response = drone.create_raw(
            pose=drone_config.spawn_pose,
            classname=primary_class,
            label=drone_config.label,
        )
        if self._ok(response):
            drone.classname = primary_class
            return True

        if primary_class != "DronePawn":
            print(f"[WARN] spawn with '{primary_class}' failed: {response}")
            fallback_response = drone.create_raw(
                pose=drone_config.spawn_pose,
                classname="DronePawn",
                label=drone_config.label,
            )
            if self._ok(fallback_response):
                print("[WARN] fallback class 'DronePawn' used (may have no visible mesh)")
                drone.classname = "DronePawn"
                return True
            print(f"[ERROR] fallback spawn failed: {fallback_response}")
            return False

        print(f"[ERROR] spawn failed: {response}")
        return False

    def _state_to_position(self, state) -> Tuple[float, float, float]:
        if isinstance(state, dict):
            position = state.get("position", [0.0, 0.0, 0.0])
            if isinstance(position, (list, tuple)) and len(position) >= 3:
                return (float(position[0]), float(position[1]), float(position[2]))
        return (0.0, 0.0, 0.0)

    def _drone_state(self, drone: AgentDrone):
        response = drone.get_state(frame="ue")
        return response if isinstance(response, dict) else {}

    def _drone_position(self, drone: AgentDrone) -> Tuple[float, float, float]:
        return self._state_to_position(self._drone_state(drone))

    def _wait_for_altitude(self, drone: AgentDrone, target_altitude: float, timeout_s: float = 20.0, tolerance_m: float = 0.8) -> bool:
        deadline = time.time() + max(1.0, timeout_s)
        while time.time() < deadline:
            state = self._drone_state(drone)
            if self._ok(state):
                altitude = self._state_to_position(state)[2]
                if altitude >= (float(target_altitude) - tolerance_m):
                    return True
            time.sleep(0.25)
        return False

    def _clean_existing_actors(self) -> None:
        if not self.config.runtime.clean_existing:
            return
        for actor_id in self._actor_ids().values():
            self._ensure_removed(actor_id)
        time.sleep(0.2)

    def _verify_spawned_agents(self) -> bool:
        if self.client is None:
            return False
        time.sleep(0.3)
        current_ids = set(self.client.get_agents())
        actor_ids = self._actor_ids()
        missing = [actor_id for actor_id in actor_ids.values() if actor_id not in current_ids]
        if missing:
            print(f"[ERROR] spawned actors missing in registry: {missing}, current={sorted(current_ids)}")
            return False
        print(
            f"[INFO] guidance={actor_ids['guidance']}, interceptor={actor_ids['interceptor']}, target={actor_ids['target']}"
        )
        return True

    def _takeoff_pair(self) -> bool:
        assert self.interceptor_agent is not None
        assert self.target_agent is not None
        assert self.guidance_agent is not None

        print("[INFO] takeoff start")
        for drone, drone_config in (
            (self.interceptor_agent, self.config.interceptor),
            (self.target_agent, self.config.target),
        ):
            enable_response = drone.enable_api_control(True)
            if not self._ok(enable_response):
                print(f"[WARN] enable_api_control failed for {drone.actor_id}: {enable_response}")
            drone.takeoff(altitude=drone_config.altitude)
            if self._wait_for_altitude(drone, drone_config.altitude):
                continue
            print(f"[ERROR] {drone_config.role} '{drone_config.actor_id}' did not reach takeoff altitude")
            return False

        print("[INFO] both drones airborne")
        self.interceptor_agent.set_camera_angles(self.config.interceptor.camera_pitch, self.config.interceptor.camera_yaw)
        guidance_reset = self.guidance_agent.reset()
        if not self._ok(guidance_reset):
            print(f"[ERROR] guidance reset failed: {guidance_reset}")
            return False
        return True

    @staticmethod
    def _try_call(action):
        try:
            return action()
        except Exception:
            return None

    def _cleanup_runtime(self) -> None:
        actor_ids = self._actor_ids()
        if self.guidance_agent is not None and self.created_guidance and self.visual_session_started:
            self._try_call(
                lambda: self.guidance_agent.visual_intercept_stop(
                    interceptor_id=actor_ids["interceptor"],
                    target_id=actor_ids["target"],
                )
            )

        for _label, created, drone in self._iter_runtime_drones():
            if created and drone is not None:
                self._try_call(drone.hover)

        self.view.close()
        should_cleanup = not self.config.runtime.keep_actors
        if should_cleanup and self.exit_reason.startswith("error:"):
            should_cleanup = False
            print("[WARN] keeping actors in scene for debugging because the script exited on error")

        if should_cleanup:
            for label, created, actor in self._iter_runtime_actors():
                if not created or actor is None:
                    continue
                response = self._try_call(actor.remove_raw)
                if response is not None:
                    print(f"[INFO] remove {label}: {response}")

        if self.client is not None:
            self.client.close()

    def _connect_runtime(self) -> bool:
        self.client = TCPClient(
            host=self.config.connection.host,
            port=self.config.connection.port,
            timeout=self.config.connection.timeout,
            auto_connect=False,
            auto_reconnect=self.config.connection.auto_reconnect,
            reconnect_delay_s=self.config.connection.reconnect_delay_s,
        )
        if not self.client.connect():
            print("[ERROR] cannot connect to simulator")
            return False
        ping_response = self.client.ping()
        if not self._ok(ping_response):
            print(f"[ERROR] ping failed: {ping_response}")
            self.client.close()
            return False

        self.interceptor_agent = AgentDrone(
            self.client,
            actor_id=self.config.interceptor.actor_id,
            classname=self.config.interceptor.classname or DEFAULT_DRONE_CLASS,
            label=self.config.interceptor.label,
            mission_role=self.config.interceptor.role,
        )
        self.target_agent = AgentDrone(
            self.client,
            actor_id=self.config.target.actor_id,
            classname=self.config.target.classname or DEFAULT_DRONE_CLASS,
            label=self.config.target.label,
            mission_role=self.config.target.role,
        )
        self.guidance_agent = AgentGuidance(
            self.client,
            actor_id=self.config.guidance.actor_id,
            classname=self.config.guidance.classname,
            label=self.config.guidance.label,
        )
        return True

    def _ensure_detector(self) -> YoloDetector:
        if self.detector is not None:
            return self.detector
        visual = self.config.visual
        self.detector = YoloDetector(
            model_path=visual.model_path,
            conf=visual.conf,
            iou=visual.iou,
            imgsz=visual.imgsz,
            device=visual.device,
            class_id=visual.class_id,
            max_det=visual.max_det,
            selection_strategy=visual.selection_strategy,
        )
        desc = self.detector.describe()
        print(
            f"[INFO] YOLO loaded: model={desc['model_path']} conf={desc['conf']:.2f} "
            f"iou={desc['iou']:.2f} imgsz={desc['imgsz']} device={desc['device']} "
            f"class={desc['class_id']} selection={desc['selection_strategy']}"
        )
        return self.detector

    @staticmethod
    def _normalize_visual_method(method: str) -> str:
        text = str(method or "vision_pid_kalman").strip().lower()
        if text in ("", "auto"):
            return "vision_pid_kalman"
        if text in ("pure_pursuit", "proportional_nav", "smc"):
            print(f"[WARN] legacy method '{text}' is not a visual method, using 'vision_pid_kalman'")
            return "vision_pid_kalman"
        return text

    def _build_trajectory(self):
        trajectory = TrajectoryFactory.create(
            config=self.config.trajectory,
            spawn_pose=self.config.target.spawn_pose,
            altitude=self.config.target.altitude,
        )
        trajectory.reference(0.0)
        return trajectory

    def _start_visual_intercept(self) -> bool:
        assert self.guidance_agent is not None
        assert self.interceptor_agent is not None
        assert self.target_agent is not None

        visual = self.config.visual
        payload = {
            "interceptor_id": self.interceptor_agent.actor_id,
            "target_id": self.target_agent.actor_id,
            "method": self._normalize_visual_method(visual.method),
            "stop_on_capture": bool(visual.stop_on_capture),
        }
        for key in (
            "desired_area",
            "capture_area",
            "center_tol_x",
            "center_tol_y",
            "capture_hold_frames",
            "lost_to_search_frames",
            "max_forward_speed",
            "max_reverse_speed",
            "max_vertical_speed",
            "max_yaw_rate_deg",
            "ram_area_target",
            "min_ram_speed",
            "intercept_distance",
            "search_cam_yaw_limit_deg",
            "search_cam_rate_deg",
            "search_body_yaw_rate_deg",
            "search_cam_pitch_deg",
            "search_vz_amp",
            "use_kalman",
        ):
            value = getattr(visual, key)
            if value is not None:
                payload[key] = value

        response = self.guidance_agent.visual_intercept_start(**payload)
        if not self._ok(response):
            print(f"[ERROR] visual intercept start failed: {response}")
            return False

        self.visual_session_started = True
        state = response.get("state", "UNKNOWN") if isinstance(response, dict) else "UNKNOWN"
        print(
            f"[INFO] visual intercept started: method={payload['method']} "
            f"selection={visual.selection_strategy} state={state}"
        )
        return True

    def _send_target_reference(self, elapsed_s: float, trajectory) -> Tuple[dict, Tuple[float, float, float], TrajectoryReference]:
        assert self.target_agent is not None
        target_pos = self._drone_position(self.target_agent)
        reference = trajectory.reference(elapsed_s)
        response = self.target_agent.move_to(
            x=reference.pose.x,
            y=reference.pose.y,
            z=reference.pose.z,
            speed=reference.speed,
            frame="ue",
            yaw_mode={"is_rate": False, "yaw_or_rate": reference.pose.yaw},
            drivetrain="forward_only",
        )
        return response, target_pos, reference

    def _run_target_lead_phase(self, trajectory, loop_dt: float) -> Optional[str]:
        lead_time = float(self.config.trajectory.lead_time)
        telemetry_interval = max(0.1, float(self.config.runtime.telemetry_interval))
        lead_start = time.time()
        last_trace = -1.0

        print(f"[INFO] target lead time: {lead_time:.1f}s")
        while True:
            tick_start = time.time()
            lead_elapsed = tick_start - lead_start
            if lead_elapsed >= lead_time:
                return None

            move_resp, target_pos, target_ref = self._send_target_reference(lead_elapsed, trajectory)
            if not self._ok(move_resp):
                print(f"[ERROR] target lead reference failed: {move_resp}")
                return "error: target lead reference failed"

            if (lead_elapsed - last_trace) >= telemetry_interval:
                ref_pos = target_ref.position_tuple()
                print(
                    "[TRACE] "
                    f"lead t={lead_elapsed:6.2f}s "
                    f"tgt=({target_pos[0]:+.1f},{target_pos[1]:+.1f},{target_pos[2]:+.1f}) "
                    f"ref=({ref_pos[0]:+.1f},{ref_pos[1]:+.1f},{ref_pos[2]:+.1f})"
                )
                last_trace = lead_elapsed

            sleep_s = loop_dt - (time.time() - tick_start)
            if sleep_s > 0:
                time.sleep(sleep_s)

    def _detect_from_image(self, image_packet) -> Tuple[object, DetectionResult]:
        if image_packet is None:
            return None, DetectionResult.empty()
        frame = image_packet.decode_bgr()
        if frame is None:
            return None, DetectionResult.empty(image_w=image_packet.width, image_h=image_packet.height)
        assert self.detector is not None
        detection = self.detector.detect_bgr(frame)
        return frame, detection

    def _run_intercept_loop(self, trajectory, loop_dt: float) -> TaskResult:
        assert self.interceptor_agent is not None
        assert self.target_agent is not None
        assert self.guidance_agent is not None

        telemetry_interval = max(0.1, float(self.config.runtime.telemetry_interval))
        max_time = float(self.config.runtime.max_time)
        max_transient_errors = max(1, int(self.config.runtime.max_transient_errors))

        t0 = time.time()
        last_response = {}
        last_telemetry_t = -1.0
        move_error_count = 0
        image_error_count = 0
        update_error_count = 0
        last_tick_time = None

        print("[INFO] visual intercept loop started (scene image -> YOLO -> C++ visual_intercept_update)")
        while True:
            tick_start = time.time()
            elapsed = tick_start - t0
            dt = loop_dt if last_tick_time is None else max(1e-3, tick_start - last_tick_time)
            last_tick_time = tick_start

            if elapsed > max_time:
                print("[INFO] timeout reached")
                return TaskResult(exit_reason="timeout", captured=False, last_response=last_response)

            move_resp, target_pos, target_ref = self._send_target_reference(self.config.trajectory.lead_time + elapsed, trajectory)
            if not self._ok(move_resp):
                move_error_count += 1
                print(f"[WARN] target reference failed ({move_error_count}/{max_transient_errors}): {move_resp}")
                if move_error_count >= max_transient_errors:
                    return TaskResult(exit_reason="error: target reference failed", captured=False, last_response=last_response)
                time.sleep(0.2)
                continue
            move_error_count = 0

            image_packet = self.interceptor_agent.get_image_packet(
                image_type=self.config.runtime.image_type,
                quality=self.config.runtime.image_quality,
                max_depth_m=self.config.runtime.max_depth_m,
            )
            if image_packet is None:
                image_error_count += 1
                print(f"[WARN] image fetch failed ({image_error_count}/{max_transient_errors})")
                if image_error_count >= max_transient_errors:
                    return TaskResult(exit_reason="error: image fetch failed", captured=False, last_response=last_response)
                time.sleep(0.2)
                continue
            image_error_count = 0

            frame, detection = self._detect_from_image(image_packet)
            update_response = self.guidance_agent.visual_intercept_update(
                has_detection=detection.has_detection,
                cx=detection.cx,
                cy=detection.cy,
                area=detection.area,
                area_ratio=detection.area_ratio,
                conf=detection.conf,
                dt=dt,
                image_w=detection.image_w or image_packet.width or 640,
                image_h=detection.image_h or image_packet.height or 480,
                interceptor_id=self.interceptor_agent.actor_id,
                target_id=self.target_agent.actor_id,
            )
            last_response = update_response if isinstance(update_response, dict) else {}
            if not self._ok(update_response):
                update_error_count += 1
                print(f"[WARN] visual_intercept_update failed ({update_error_count}/{max_transient_errors}): {update_response}")
                if update_error_count >= max_transient_errors:
                    return TaskResult(exit_reason="error: visual_intercept_update failed", captured=False, last_response=last_response)
                time.sleep(0.2)
                continue
            update_error_count = 0

            guidance_state = GuidanceState.from_response(update_response)
            interceptor_pos = self._drone_position(self.interceptor_agent)
            target_distance = guidance_state.target_distance
            if target_distance < 0.0:
                target_distance = math.dist(target_pos, interceptor_pos)
            captured = guidance_state.captured
            cmd_velocity = guidance_state.cmd_velocity
            control = guidance_state.control if isinstance(guidance_state.control, dict) else {}
            ex = float(control.get("ex", 0.0) or 0.0)
            ey = float(control.get("ey", 0.0) or 0.0)
            yaw_cmd = float(control.get("yaw_cmd_deg", 0.0) or 0.0)
            yaw_rate = float(control.get("yaw_rate_deg", 0.0) or 0.0)

            if (last_telemetry_t < 0.0) or ((elapsed - last_telemetry_t) >= telemetry_interval):
                print(
                    "[TRACE] "
                    f"t={elapsed:6.2f}s state={guidance_state.state} det={int(detection.has_detection)} "
                    f"conf={detection.conf:.2f} area={detection.area_ratio:.4f} dist={target_distance:6.2f}m "
                    f"sel={detection.selection_reason or '-'} score={detection.selection_score:.2f} "
                    f"cands={len(detection.candidates)} "
                    f"intr=({interceptor_pos[0]:+.1f},{interceptor_pos[1]:+.1f},{interceptor_pos[2]:+.1f}) "
                    f"tgt=({target_pos[0]:+.1f},{target_pos[1]:+.1f},{target_pos[2]:+.1f}) "
                    f"cmd={list(cmd_velocity)} ex={ex:+.3f} ey={ey:+.3f} yaw={yaw_cmd:+.1f} yr={yaw_rate:+.1f}"
                )
                last_telemetry_t = elapsed

            if captured:
                print(f"[INFO] intercepted target at t={elapsed:.2f}s distance={target_distance:.2f}m")

            vis_exit = self.view.render(
                image_packet,
                self.interceptor_agent.actor_id,
                self.target_agent.actor_id,
                guidance_state,
                detection,
                interceptor_pos,
                target_pos,
                target_ref,
                elapsed,
                frame=frame,
            )
            if vis_exit is not None:
                return TaskResult(exit_reason=vis_exit, captured=captured, last_response=last_response)
            if captured:
                return TaskResult(exit_reason="intercepted", captured=True, last_response=last_response)

            sleep_s = loop_dt - (time.time() - tick_start)
            if sleep_s > 0:
                time.sleep(sleep_s)

    def run(self) -> TaskResult:
        if self.view.show and not self.view.available():
            print("[WARN] opencv-python not installed, visualization disabled")
            self.view.show = False

        try:
            self._ensure_detector()
        except Exception as exc:
            self.exit_reason = f"error: {exc}"
            print(f"[ERROR] {exc}")
            return TaskResult(exit_reason=self.exit_reason)

        if not self._connect_runtime():
            self.exit_reason = "error: cannot connect"
            return TaskResult(exit_reason=self.exit_reason)

        assert self.guidance_agent is not None
        assert self.interceptor_agent is not None
        assert self.target_agent is not None

        try:
            self._clean_existing_actors()
            self.created_guidance = self.guidance_agent.create(
                pose=self.config.guidance.spawn_pose,
                classname=self.config.guidance.classname,
                label=self.config.guidance.label,
            )
            if not self.created_guidance:
                self.exit_reason = "error: guidance spawn failed"
                return TaskResult(exit_reason=self.exit_reason)

            self.created_interceptor = self._spawn_drone(self.interceptor_agent, self.config.interceptor)
            if not self.created_interceptor:
                self.exit_reason = "error: interceptor spawn failed"
                return TaskResult(exit_reason=self.exit_reason)

            self.created_target = self._spawn_drone(self.target_agent, self.config.target)
            if not self.created_target:
                self.exit_reason = "error: target spawn failed"
                return TaskResult(exit_reason=self.exit_reason)

            if not self._verify_spawned_agents():
                self.exit_reason = "error: spawned actors missing in registry"
                return TaskResult(exit_reason=self.exit_reason)
            if not self._takeoff_pair():
                self.exit_reason = "error: takeoff or altitude wait failed"
                return TaskResult(exit_reason=self.exit_reason)

            try:
                trajectory = self._build_trajectory()
            except Exception as exc:
                self.exit_reason = f"error: {exc}"
                print(f"[ERROR] {exc}")
                return TaskResult(exit_reason=self.exit_reason)

            loop_dt = 1.0 / max(2.0, float(self.config.runtime.hz))
            description = trajectory.describe()
            if description:
                print(
                    f"[INFO] target trajectory kind={self.config.trajectory.kind} "
                    f"center=({description.get('center_x', 0.0):.1f}, {description.get('center_y', 0.0):.1f}, {description.get('center_z', 0.0):.1f}) "
                    f"radius={description.get('radius', 0.0):.1f}m omega={description.get('omega', 0.0):.2f}rad/s"
                )
            else:
                print(f"[INFO] target trajectory kind={self.config.trajectory.kind}")

            lead_error = self._run_target_lead_phase(trajectory, loop_dt)
            if lead_error is not None:
                self.exit_reason = lead_error
                return TaskResult(exit_reason=self.exit_reason)

            if not self._start_visual_intercept():
                self.exit_reason = "error: visual intercept start failed"
                return TaskResult(exit_reason=self.exit_reason)

            self.view.open()
            result = self._run_intercept_loop(trajectory, loop_dt)
            self.exit_reason = result.exit_reason
            print(f"[RESULT] {result.to_dict()}")
            print(f"[INFO] exit reason: {self.exit_reason}")
            return result
        finally:
            self._cleanup_runtime()


def run_task(config: TaskConfig) -> TaskResult:
    return Task(config).run()


