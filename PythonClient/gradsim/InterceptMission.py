"""Mission runner for the YOLO-based visual intercept demo."""

import math
import time

import numpy as np

from .MissionRuntime import DroneMissionRuntime
from .VisionUtils import decode_bgr_base64
from .YoloDetector import YoloDetector


class VisualInterceptMission(DroneMissionRuntime):
    def __init__(self, config, view=None):
        super().__init__(config, view=view)
        self.detector = None

    def _trajectory(self):
        return self.config["target"]["trajectory"]

    def _visual(self):
        visual = self.config.setdefault("visual", {})
        visual.setdefault("method", "vision_pid_kalman")
        visual.setdefault("model_path", None)
        visual.setdefault("conf", 0.35)
        visual.setdefault("iou", 0.45)
        visual.setdefault("imgsz", 640)
        visual.setdefault("device", None)
        visual.setdefault("class_id", 0)
        visual.setdefault("max_det", 5)
        visual.setdefault("desired_area", None)
        visual.setdefault("capture_area", None)
        visual.setdefault("center_tol_x", None)
        visual.setdefault("center_tol_y", None)
        visual.setdefault("capture_hold_frames", None)
        visual.setdefault("lost_to_search_frames", None)
        visual.setdefault("max_forward_speed", None)
        visual.setdefault("max_reverse_speed", None)
        visual.setdefault("max_vertical_speed", None)
        visual.setdefault("max_yaw_rate_deg", None)
        visual.setdefault("ram_area_target", None)
        visual.setdefault("min_ram_speed", None)
        visual.setdefault("intercept_distance", 1.5)
        visual.setdefault("search_cam_yaw_limit_deg", None)
        visual.setdefault("search_cam_rate_deg", None)
        visual.setdefault("search_body_yaw_rate_deg", None)
        visual.setdefault("search_cam_pitch_deg", None)
        visual.setdefault("search_vz_amp", None)
        visual.setdefault("use_kalman", None)
        visual.setdefault("stop_on_capture", False)
        return visual

    @staticmethod
    def _normalize_visual_method(method):
        text = str(method or "vision_pid_kalman").strip().lower()
        if text in ("", "auto"):
            return "vision_pid_kalman"
        if text in ("pure_pursuit", "proportional_nav", "smc"):
            print(f"[WARN] legacy method '{text}' is not a visual method, using 'vision_pid_kalman'")
            return "vision_pid_kalman"
        return text

    def _build_orbit_plan(self):
        target = self.config["target"]
        trajectory = self._trajectory()
        speed = max(0.1, float(trajectory["speed"]))
        omega = max(0.01, abs(float(trajectory["turn_rate"])))
        radius = max(6.0, speed / omega)
        center = np.array(
            [
                float(target["spawn"][0]),
                float(target["spawn"][1]) + radius,
                float(target["altitude"]),
            ],
            dtype=np.float64,
        )
        return {"center": center, "radius": radius, "omega": omega}

    def _compute_target_reference(self, elapsed_s, orbit_plan):
        target = self.config["target"]
        trajectory = self._trajectory()
        if trajectory.get("kind") != "circle":
            raise ValueError(f"unsupported target trajectory: {trajectory.get('kind')}")

        omega = max(0.01, abs(float(trajectory["turn_rate"])))
        theta = omega * float(elapsed_s)
        radius = float(orbit_plan["radius"])

        x = float(orbit_plan["center"][0]) + radius * math.sin(theta)
        y = float(orbit_plan["center"][1]) - radius * math.cos(theta)
        z = float(target["altitude"]) + float(trajectory["vertical_amp"]) * math.sin(0.35 * theta)

        vx = radius * omega * math.cos(theta)
        vy = radius * omega * math.sin(theta)
        yaw_deg = math.degrees(math.atan2(vy, vx)) if (abs(vx) + abs(vy)) > 1e-6 else 0.0
        return np.array([x, y, z], dtype=np.float64), yaw_deg

    def _send_target_reference(self, elapsed_s, orbit_plan):
        target = self.runtime["target"]
        target_pos = self._drone_pos(target)
        target_ref, yaw_deg = self._compute_target_reference(elapsed_s, orbit_plan)
        response = target.move_to(
            x=float(target_ref[0]),
            y=float(target_ref[1]),
            z=float(target_ref[2]),
            speed=float(self._trajectory()["speed"]),
            frame="ue",
            yaw_mode={"is_rate": False, "yaw_or_rate": yaw_deg},
            drivetrain="forward_only",
        )
        return response, target_pos, target_ref

    def _run_target_lead_phase(self, orbit_plan, loop_dt):
        lead_time = float(self._trajectory()["lead_time"])
        telemetry_interval = max(0.1, float(self.config["runtime"]["telemetry_interval"]))
        lead_start = time.time()
        last_trace = -1.0

        print(f"[INFO] target lead time: {lead_time:.1f}s")
        while True:
            tick_start = time.time()
            lead_elapsed = tick_start - lead_start
            if lead_elapsed >= lead_time:
                return None

            move_resp, target_pos, target_ref = self._send_target_reference(lead_elapsed, orbit_plan)
            if not self._ok(move_resp):
                print(f"[ERROR] target lead reference failed: {move_resp}")
                return "error: target lead reference failed"

            if (lead_elapsed - last_trace) >= telemetry_interval:
                print(
                    "[TRACE] "
                    f"lead t={lead_elapsed:6.2f}s "
                    f"tgt=({target_pos[0]:+.1f},{target_pos[1]:+.1f},{target_pos[2]:+.1f}) "
                    f"ref=({target_ref[0]:+.1f},{target_ref[1]:+.1f},{target_ref[2]:+.1f})"
                )
                last_trace = lead_elapsed

            sleep_s = loop_dt - (time.time() - tick_start)
            if sleep_s > 0:
                time.sleep(sleep_s)

    def _ensure_detector(self):
        if self.detector is not None:
            return self.detector

        visual = self._visual()
        self.detector = YoloDetector(
            model_path=visual["model_path"],
            conf=visual["conf"],
            iou=visual["iou"],
            imgsz=visual["imgsz"],
            device=visual["device"],
            class_id=visual["class_id"],
            max_det=visual["max_det"],
        )
        desc = self.detector.describe()
        print(
            f"[INFO] YOLO loaded: model={desc['model_path']} conf={desc['conf']:.2f} "
            f"iou={desc['iou']:.2f} imgsz={desc['imgsz']} device={desc['device']} class={desc['class_id']}"
        )
        return self.detector

    def _start_visual_intercept(self):
        visual = self._visual()
        payload = {
            "interceptor_id": self.runtime["interceptor"].actor_id,
            "target_id": self.runtime["target"].actor_id,
            "method": self._normalize_visual_method(visual["method"]),
            "stop_on_capture": bool(visual["stop_on_capture"]),
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
            value = visual.get(key)
            if value is not None:
                payload[key] = value

        response = self.runtime["guidance"].visual_intercept_start(**payload)
        if not self._ok(response):
            print(f"[ERROR] visual intercept start failed: {response}")
            return False

        self.visual_session_started = True
        print(f"[INFO] visual intercept started: method={payload['method']}")
        return True

    @staticmethod
    def _empty_detection_from_image(image_resp):
        width = 0
        height = 0
        if isinstance(image_resp, dict):
            width = int(image_resp.get("width", 0) or 0)
            height = int(image_resp.get("height", 0) or 0)
        return {
            "has_detection": False,
            "bbox_xyxy": None,
            "cx": 0.0,
            "cy": 0.0,
            "area": 0.0,
            "area_ratio": 0.0,
            "conf": 0.0,
            "class_id": -1,
            "label": "",
            "image_w": width,
            "image_h": height,
        }

    def _detect_from_image(self, image_resp):
        if not image_resp or not image_resp.get("data"):
            return None, self._empty_detection_from_image(image_resp)

        frame = decode_bgr_base64(image_resp.get("data", ""))
        if frame is None:
            return None, self._empty_detection_from_image(image_resp)

        detection = self.detector.detect_bgr(frame)
        return frame, detection

    def _run_intercept_loop(self, orbit_plan, loop_dt):
        runtime = self.config["runtime"]
        telemetry_interval = max(0.1, float(runtime["telemetry_interval"]))
        max_time = float(runtime["max_time"])
        max_transient_errors = max(1, int(runtime["max_transient_errors"]))

        t0 = time.time()
        last_resp = {}
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
                return {"exit_reason": "timeout", "captured": False, "last_response": last_resp}

            move_resp, target_pos, target_ref = self._send_target_reference(self._trajectory()["lead_time"] + elapsed, orbit_plan)
            if not self._ok(move_resp):
                move_error_count += 1
                print(f"[WARN] target reference failed ({move_error_count}/{max_transient_errors}): {move_resp}")
                if move_error_count >= max_transient_errors:
                    return {"exit_reason": "error: target reference failed", "captured": False, "last_response": last_resp}
                time.sleep(0.2)
                continue
            move_error_count = 0

            image_resp = self.runtime["interceptor"].get_image_raw(image_type="scene", quality=82)
            if not image_resp:
                image_error_count += 1
                print(f"[WARN] image fetch failed ({image_error_count}/{max_transient_errors})")
                if image_error_count >= max_transient_errors:
                    return {"exit_reason": "error: image fetch failed", "captured": False, "last_response": last_resp}
                time.sleep(0.2)
                continue
            image_error_count = 0

            frame, detection = self._detect_from_image(image_resp)
            update_resp = self.runtime["guidance"].visual_intercept_update(
                has_detection=detection["has_detection"],
                cx=detection["cx"],
                cy=detection["cy"],
                area=detection["area"],
                area_ratio=detection["area_ratio"],
                conf=detection["conf"],
                dt=dt,
                image_w=detection["image_w"] or image_resp.get("width", 640),
                image_h=detection["image_h"] or image_resp.get("height", 480),
                interceptor_id=self.runtime["interceptor"].actor_id,
                target_id=self.runtime["target"].actor_id,
            )
            last_resp = update_resp
            if not self._ok(update_resp):
                update_error_count += 1
                print(f"[WARN] visual_intercept_update failed ({update_error_count}/{max_transient_errors}): {update_resp}")
                if update_error_count >= max_transient_errors:
                    return {"exit_reason": "error: visual_intercept_update failed", "captured": False, "last_response": last_resp}
                time.sleep(0.2)
                continue
            update_error_count = 0

            guidance_state = update_resp if isinstance(update_resp, dict) else {}
            interceptor_pos = self._drone_pos(self.runtime["interceptor"])
            distance = float(np.linalg.norm(target_pos - interceptor_pos))
            target_distance = float(guidance_state.get("target_distance", distance))
            captured = bool(guidance_state.get("captured", False))
            cmd_velocity = guidance_state.get("cmd_velocity", [0.0, 0.0, 0.0])

            if (last_telemetry_t < 0.0) or ((elapsed - last_telemetry_t) >= telemetry_interval):
                print(
                    "[TRACE] "
                    f"t={elapsed:6.2f}s state={guidance_state.get('state', 'UNKNOWN')} det={int(bool(detection['has_detection']))} "
                    f"conf={detection['conf']:.2f} area={detection['area_ratio']:.4f} dist={target_distance:6.2f}m "
                    f"intr=({interceptor_pos[0]:+.1f},{interceptor_pos[1]:+.1f},{interceptor_pos[2]:+.1f}) "
                    f"tgt=({target_pos[0]:+.1f},{target_pos[1]:+.1f},{target_pos[2]:+.1f}) "
                    f"cmd={cmd_velocity}"
                )
                last_telemetry_t = elapsed

            if captured:
                print(f"[INFO] intercepted target at t={elapsed:.2f}s distance={target_distance:.2f}m")

            vis_exit = self.view.render(
                image_resp,
                self.runtime["interceptor"].actor_id,
                self.runtime["target"].actor_id,
                guidance_state,
                detection,
                interceptor_pos,
                target_pos,
                target_ref,
                elapsed,
                frame=frame,
            )
            if vis_exit is not None:
                return {"exit_reason": vis_exit, "captured": captured, "last_response": last_resp}
            if captured:
                return {"exit_reason": "intercepted", "captured": True, "last_response": last_resp}

            sleep_s = loop_dt - (time.time() - tick_start)
            if sleep_s > 0:
                time.sleep(sleep_s)

    def run(self):
        if self.view.show and not self.view.available():
            print("[WARN] opencv-python not installed, visualization disabled.")
            self.view.show = False

        try:
            self._ensure_detector()
        except Exception as exc:
            self.exit_reason = f"error: {exc}"
            print(f"[ERROR] {exc}")
            return {"exit_reason": self.exit_reason, "last_response": {}}

        if self._create_runtime() is None:
            self.exit_reason = "error: cannot connect"
            return {"exit_reason": self.exit_reason, "last_response": {}}

        try:
            self._clean_existing_actors()
            if not self._spawn_requested_drones():
                if not self.created_guidance:
                    self.exit_reason = "error: guidance spawn failed"
                elif not self.created_interceptor:
                    self.exit_reason = "error: interceptor spawn failed"
                elif not self.created_target:
                    self.exit_reason = "error: target spawn failed"
                else:
                    self.exit_reason = "error: spawn failed"
                return {"exit_reason": self.exit_reason, "last_response": {}}

            if not self._verify_spawned_agents():
                self.exit_reason = "error: spawned actors missing in registry"
                return {"exit_reason": self.exit_reason, "last_response": {}}
            if not self._takeoff_pair():
                self.exit_reason = "error: takeoff or altitude wait failed"
                return {"exit_reason": self.exit_reason, "last_response": {}}

            loop_dt = 1.0 / max(2.0, float(self.config["runtime"]["hz"]))
            orbit_plan = self._build_orbit_plan()
            print(
                f"[INFO] target orbit center=({orbit_plan['center'][0]:.1f}, {orbit_plan['center'][1]:.1f}, "
                f"{orbit_plan['center'][2]:.1f}) radius={orbit_plan['radius']:.1f}m omega={orbit_plan['omega']:.2f}rad/s"
            )

            lead_error = self._run_target_lead_phase(orbit_plan, loop_dt)
            if lead_error is not None:
                self.exit_reason = lead_error
                return {"exit_reason": self.exit_reason, "last_response": {}}

            if not self._start_visual_intercept():
                self.exit_reason = "error: visual intercept start failed"
                return {"exit_reason": self.exit_reason, "last_response": {}}

            self.view.open()
            result = self._run_intercept_loop(orbit_plan, loop_dt)
            self.exit_reason = result["exit_reason"]
            print("[RESULT]", result["last_response"])
            print(f"[INFO] exit reason: {self.exit_reason}")
            return result
        finally:
            self._cleanup_runtime()


def run_visual_intercept(config):
    return VisualInterceptMission(config).run()
