from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

import cv2

from GraduationSIM import AgentBase, AgentDrone, AgentGuidance, DEFAULT_DRONE_CLASS, Pose, TCPClient
from GraduationSIM.DataTypes import to_bool, to_float, to_int, to_vec3


class DetectionCandidate:
    def __init__(self, x1, y1, x2, y2, cx, cy, area, area_ratio, conf, class_id, label) -> None:
        self.x1 = to_float(x1)
        self.y1 = to_float(y1)
        self.x2 = to_float(x2)
        self.y2 = to_float(y2)
        self.cx = to_float(cx)
        self.cy = to_float(cy)
        self.area = to_float(area)
        self.area_ratio = to_float(area_ratio)
        self.conf = to_float(conf)
        self.class_id = to_int(class_id, -1)
        self.label = str(label)
        self.selected = False


class DetectionResult:
    def __init__(
        self,
        has_detection=False,
        bbox_xyxy=None,
        cx=0.0,
        cy=0.0,
        area=0.0,
        area_ratio=0.0,
        conf=0.0,
        class_id=-1,
        label="",
        image_w=0,
        image_h=0,
        candidates=None,
    ) -> None:
        self.has_detection = bool(has_detection)
        self.bbox_xyxy = None if bbox_xyxy is None else list(bbox_xyxy)
        self.cx = to_float(cx)
        self.cy = to_float(cy)
        self.area = to_float(area)
        self.area_ratio = to_float(area_ratio)
        self.conf = to_float(conf)
        self.class_id = to_int(class_id, -1)
        self.label = str(label)
        self.image_w = to_int(image_w)
        self.image_h = to_int(image_h)
        self.candidates = list(candidates or [])

    @classmethod
    def empty(cls, image_w=0, image_h=0):
        return cls(image_w=image_w, image_h=image_h)

    def __bool__(self) -> bool:
        return self.has_detection or bool(self.candidates)


class GuidanceState:
    def __init__(
        self,
        state="UNKNOWN",
        valid=False,
        captured=False,
        detections=0,
        lost_count=0,
        target_distance=-1.0,
        cmd_velocity=(0.0, 0.0, 0.0),
        control=None,
        measurement=None,
        track=None,
        prediction=None,
        kalman=None,
    ) -> None:
        self.state = str(state)
        self.valid = bool(valid)
        self.captured = bool(captured)
        self.detections = to_int(detections)
        self.lost_count = to_int(lost_count)
        self.target_distance = to_float(target_distance, -1.0)
        self.cmd_velocity = to_vec3(cmd_velocity)
        self.control = dict(control or {})
        self.measurement = self.normalize_feature(measurement)
        self.track = self.normalize_feature(track)
        self.prediction = self.normalize_feature(prediction)
        self.kalman = self.normalize_kalman(kalman)

    @staticmethod
    def normalize_feature(payload):
        payload = payload if isinstance(payload, dict) else {}
        return {
            "valid": to_bool(payload.get("valid")),
            "used_prediction": to_bool(payload.get("used_prediction")),
            "cx": to_float(payload.get("cx")),
            "cy": to_float(payload.get("cy")),
            "area": to_float(payload.get("area")),
            "lead_time": to_float(payload.get("lead_time")),
        }

    @staticmethod
    def normalize_kalman(payload):
        payload = payload if isinstance(payload, dict) else {}
        return {
            "valid": to_bool(payload.get("valid")),
            "pos": to_vec3(payload.get("pos")),
            "vel": to_vec3(payload.get("vel")),
            "acc": to_vec3(payload.get("acc")),
            "uncertainty": to_float(payload.get("uncertainty")),
            "q": to_float(payload.get("q")),
        }

    @classmethod
    def from_response(cls, payload):
        payload = payload or {}
        return cls(
            state=payload.get("state", "UNKNOWN"),
            valid=to_bool(payload.get("valid")),
            captured=to_bool(payload.get("captured")),
            detections=payload.get("detections", 0),
            lost_count=payload.get("lost_count", 0),
            target_distance=payload.get("target_distance", -1.0),
            cmd_velocity=payload.get("cmd_velocity"),
            control=payload.get("control") if isinstance(payload.get("control"), dict) else {},
            measurement=payload.get("measurement"),
            track=payload.get("track"),
            prediction=payload.get("prediction"),
            kalman=payload.get("kalman"),
        )
class YoloDetector:
    def __init__(self, model_path=None, conf=0.35, iou=0.45, imgsz=640, device=None, class_id=0, max_det=5) -> None:
        self.project_root = Path(__file__).resolve().parents[2]
        self.model_path = Path(model_path) if model_path else Path(
            r"D:\Xstarlab\UEProjects\GraduationProject\GraduationProject\PythonClient\YOLO\runs\detect\drone_detect2\weights\best.pt"
        )
        self.conf = float(conf)
        self.iou = float(iou)
        self.imgsz = int(imgsz)
        self.device = None if device in (None, "", "auto") else str(device)
        self.class_id = None if class_id is None else int(class_id)
        self.max_det = max(1, int(max_det))

        source_root = self.project_root / "PythonClient" / "YOLO" / "ultralytics"
        settings_root = self.project_root / "PythonClient" / ".ultralytics_config"
        settings_root.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault("YOLO_CONFIG_DIR", str(settings_root))
        os.environ.setdefault("YOLO_AUTOINSTALL", "False")
        os.environ.setdefault("YOLO_VERBOSE", "False")

        source_text = str(source_root)
        if source_text not in sys.path:
            sys.path.insert(0, source_text)

        from ultralytics import YOLO

        self.model = YOLO(str(self.model_path))
        names = getattr(self.model, "names", {})
        self.names = names if isinstance(names, dict) else {}

    def detect_bgr(self, frame) -> DetectionResult:
        if frame is None:
            return DetectionResult.empty()

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
            return DetectionResult.empty(image_w=frame.shape[1], image_h=frame.shape[0])

        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) <= 0:
            return DetectionResult.empty(image_w=frame.shape[1], image_h=frame.shape[0])

        image_h, image_w = frame.shape[:2]
        candidates = self.build_candidates(boxes, image_w, image_h)
        selected = self.select_candidate(candidates)
        if selected is None:
            return DetectionResult.empty(image_w=image_w, image_h=image_h)

        for candidate in candidates:
            candidate.selected = candidate is selected

        return DetectionResult(
            has_detection=True,
            bbox_xyxy=[selected.x1, selected.y1, selected.x2, selected.y2],
            cx=selected.cx,
            cy=selected.cy,
            area=selected.area,
            area_ratio=selected.area_ratio,
            conf=selected.conf,
            class_id=selected.class_id,
            label=selected.label,
            image_w=image_w,
            image_h=image_h,
            candidates=candidates,
        )

    def build_candidates(self, boxes, image_w, image_h):
        confidences = boxes.conf.detach().cpu().numpy()
        boxes_xyxy = boxes.xyxy.detach().cpu().numpy()
        classes = None if getattr(boxes, "cls", None) is None else boxes.cls.detach().cpu().numpy()
        image_area = max(1.0, float(image_w * image_h))
        candidates = []
        for index, xyxy in enumerate(boxes_xyxy):
            x1, y1, x2, y2 = [float(value) for value in xyxy.tolist()]
            width = max(0.0, x2 - x1)
            height = max(0.0, y2 - y1)
            class_value = int(classes[index]) if classes is not None else -1
            candidates.append(
                DetectionCandidate(
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                    cx=0.5 * (x1 + x2),
                    cy=0.5 * (y1 + y2),
                    area=width * height,
                    area_ratio=(width * height) / image_area,
                    conf=float(confidences[index]),
                    class_id=class_value,
                    label=self.names.get(class_value, str(class_value)),
                )
            )
        return candidates

    def select_candidate(self, candidates):
        if not candidates:
            return None
        return max(candidates, key=lambda item: (item.conf, item.area))


class VisualUI:
    def __init__(self, show=True, window_name="visual_intercept") -> None:
        self.show = bool(show)
        self.window_name = window_name
        self.opened = False

    def open(self) -> None:
        if not self.show or self.opened:
            return
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1080, 620)
        self.opened = True

    def close(self) -> None:
        if not self.opened:
            return
        cv2.destroyAllWindows()
        self.opened = False

    def render(self, image_packet, interceptor_id, target_id, guidance_state, detection, interceptor_pos, target_pos, frame=None):
        if not self.show or image_packet is None:
            return None

        image_data = frame if frame is not None else image_packet.decode_bgr()
        if image_data is None:
            return None

        self.draw_detection(image_data, detection)
        self.draw_guidance_track(image_data, guidance_state)
        control = guidance_state.control or {}
        cmd_vel = guidance_state.cmd_velocity
        track = guidance_state.track
        prediction = guidance_state.prediction
        kalman = guidance_state.kalman
        track_mode = "prediction" if track.get("used_prediction") else "measurement"
        track_area_ratio = track.get("area", 0.0) / max(1.0, float(image_data.shape[0] * image_data.shape[1]))
        prediction_area_ratio = prediction.get("area", 0.0) / max(1.0, float(image_data.shape[0] * image_data.shape[1]))
        prediction_lead = prediction.get("lead_time", 0.0)
        kalman_vel = kalman.get("vel", (0.0, 0.0, 0.0))
        kalman_acc = kalman.get("acc", (0.0, 0.0, 0.0))
        lines = [
            "YOLO VISUAL INTERCEPT",
            f"interceptor={interceptor_id} target={target_id}",
            f"target_distance={guidance_state.target_distance:+.2f}m ex={float(control.get('ex', 0.0)):+.3f} ey={float(control.get('ey', 0.0)):+.3f} area_norm={float(control.get('area_norm', 0.0)):.4f}",
            f"cmd_vel=({cmd_vel[0]:+.2f}, {cmd_vel[1]:+.2f}, {cmd_vel[2]:+.2f}) m/s",
            f"track={track_mode} valid={int(bool(track.get('valid')))} px=({track.get('cx', 0.0):.1f}, {track.get('cy', 0.0):.1f}) area_ratio={track_area_ratio:.4f}",
            f"prediction valid={int(bool(prediction.get('valid')))} lead={prediction_lead:.2f}s px=({prediction.get('cx', 0.0):.1f}, {prediction.get('cy', 0.0):.1f}) area_ratio={prediction_area_ratio:.4f}",
            f"kalman_valid={int(bool(kalman.get('valid')))} q={kalman.get('q', 0.0):.3f} uncertainty={kalman.get('uncertainty', 0.0):.3f}",
            f"kalman_vel=({kalman_vel[0]:+.2f}, {kalman_vel[1]:+.2f}, {kalman_vel[2]:+.2f})",
            f"kalman_acc=({kalman_acc[0]:+.2f}, {kalman_acc[1]:+.2f}, {kalman_acc[2]:+.2f})",
            f"intr_pos=({interceptor_pos[0]:+.1f}, {interceptor_pos[1]:+.1f}, {interceptor_pos[2]:+.1f})",
            f"tgt_pos =({target_pos[0]:+.1f}, {target_pos[1]:+.1f}, {target_pos[2]:+.1f})",
        ]

        for index, text in enumerate(lines):
            y = 22 + index * 20
            cv2.putText(image_data, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.53, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(image_data, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.53, (235, 235, 235), 1, cv2.LINE_AA)

        cv2.imshow(self.window_name, image_data)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            return "quit"
        return None

    def draw_detection(self, image_data, detection) -> None:
        if not detection:
            return

        for candidate in detection.candidates:
            x1, y1, x2, y2 = [int(round(v)) for v in (candidate.x1, candidate.y1, candidate.x2, candidate.y2)]
            color = (60, 230, 90) if candidate.selected else (0, 200, 255)
            thickness = 2 if candidate.selected else 1
            cv2.rectangle(image_data, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)
            if emphasize:
                cv2.rectangle(
                    image_data,
                    (max(0, x1 - 3), max(0, y1 - 3)),
                    (min(image_w - 1, x2 + 3), min(image_h - 1, y2 + 3)),
                    color,
                    1,
                    cv2.LINE_AA,
                )
                cv2.circle(image_data, (cx, cy), max(10, half_size // 2), color, 1, cv2.LINE_AA)
            if candidate.selected:
                cv2.circle(image_data, (int(round(candidate.cx)), int(round(candidate.cy))), 4, color, -1, cv2.LINE_AA)
                cv2.putText(
                    image_data,
                    f"YOLO {candidate.label} conf={candidate.conf:.2f}",
                    (max(8, x1), max(22, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1,
                    cv2.LINE_AA,
                )

    def draw_guidance_track(self, image_data, guidance_state) -> None:
        image_h, image_w = image_data.shape[:2]
        center = (image_w // 2, image_h // 2)
        cv2.drawMarker(image_data, center, (230, 230, 230), cv2.MARKER_CROSS, 18, 1, cv2.LINE_AA)

        def draw_feature(feature, color, label, *, fallback_area=0.0, box_pad=0, thickness=2, emphasize=False):
            if not feature.get("valid"):
                return
            cx = int(round(feature.get("cx", 0.0)))
            cy = int(round(feature.get("cy", 0.0)))
            area = max(float(feature.get("area", 0.0)), float(fallback_area), 1.0)
            cv2.line(image_data, center, (cx, cy), color, 1, cv2.LINE_AA)
            cv2.drawMarker(image_data, (cx, cy), color, cv2.MARKER_TILTED_CROSS, 22, 2, cv2.LINE_AA)
            half_size = max(8, int(round(0.5 * math.sqrt(area)))) + int(box_pad)
            x1 = max(0, cx - half_size)
            y1 = max(0, cy - half_size)
            x2 = min(image_w - 1, cx + half_size)
            y2 = min(image_h - 1, cy + half_size)
            cv2.rectangle(image_data, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)
            if emphasize:
                cv2.rectangle(
                    image_data,
                    (max(0, x1 - 3), max(0, y1 - 3)),
                    (min(image_w - 1, x2 + 3), min(image_h - 1, y2 + 3)),
                    color,
                    1,
                    cv2.LINE_AA,
                )
                cv2.circle(image_data, (cx, cy), max(10, half_size // 2), color, 1, cv2.LINE_AA)
            cv2.putText(
                image_data,
                label,
                (max(8, cx + 10), max(24, cy - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

        measurement = guidance_state.measurement or {}
        track = guidance_state.track or {}
        prediction = guidance_state.prediction or {}
        measurement_area = float(measurement.get("area", 0.0))
        track_area = float(track.get("area", 0.0))
        prediction_area_fallback = max(measurement_area, track_area, 64.0)

        if track.get("valid") and not track.get("used_prediction"):
            draw_feature(track, (255, 220, 0), "TRACK", fallback_area=measurement_area)
        if prediction.get("valid"):
            label = f"KALMAN PRED {float(prediction.get('lead_time', 0.0)):.2f}s"
            draw_feature(prediction, (0, 0, 255), label, fallback_area=prediction_area_fallback, box_pad=10, thickness=3, emphasize=True)
        elif track.get("valid") and track.get("used_prediction"):
            draw_feature(track, (0, 0, 255), "KALMAN PRED", fallback_area=prediction_area_fallback, box_pad=10, thickness=3, emphasize=True)

def run():
    host = "127.0.0.1"
    port = 9000
    timeout = 10.0
    show = True
    interceptor_pose = Pose(x=0.0, y=0.0, z=1.0, yaw=0.0)
    target_pose = Pose(x=25.0, y=-12.0, z=1.0, yaw=180.0)
    interceptor_altitude = 7.0
    target_altitude = 8.0
    target_speed = 6.0
    target_radius = 6.0
    target_turn_rate = 0.30
    target_vertical_amp = 0.0
    image_quality = 82
    hz = 25.0
    max_time = 500.0
    interceptor_id = "drone_0"
    target_id = "target_0"
    guidance_id = "guidance_0"
    client = None
    interceptor = None
    target = None
    guidance = None
    ui = VisualUI(show=show)
    created_interceptor = False
    created_target = False
    created_guidance = False
    visual_started = False

    try:
        detector = YoloDetector(conf=0.35, iou=0.45, imgsz=640, device=None, class_id=0, max_det=5)
        client = TCPClient(host=host, port=port, timeout=timeout, auto_connect=False)
        if not client.connect():
            return {"captured": False}

        interceptor = AgentDrone(client, actor_id=interceptor_id, classname=DEFAULT_DRONE_CLASS, label="Interceptor", mission_role="interceptor")
        target = AgentDrone(client, actor_id=target_id, classname=DEFAULT_DRONE_CLASS, label="Target", mission_role="target")
        guidance = AgentGuidance(client, actor_id=guidance_id, classname="GuidanceActor", label="Guidance")

        for actor_id in (guidance_id, interceptor_id, target_id):
            client.send_message({"remove_actor": {"actor_id": actor_id}})
        time.sleep(0.2)

        response = guidance.create(pose=Pose(), classname="GuidanceActor", label="Guidance")
        if not AgentBase.is_ok(response):
            return {"captured": False}
        created_guidance = True

        response = interceptor.create(pose=interceptor_pose, classname=DEFAULT_DRONE_CLASS, label="Interceptor")
        if not AgentBase.is_ok(response):
            return {"captured": False}
        created_interceptor = True

        response = target.create(pose=target_pose, classname=DEFAULT_DRONE_CLASS, label="Target")
        if not AgentBase.is_ok(response):
            return {"captured": False}
        created_target = True

        deadline = time.time() + 5.0
        while time.time() < deadline:
            state = guidance.get_state()
            if AgentBase.is_ok(state) or isinstance(state, dict):
                break
            time.sleep(0.2)
        else:
            return {"captured": False}

        for drone in (interceptor, target):
            deadline = time.time() + 5.0
            while time.time() < deadline:
                state = drone.get_state(frame="ue")
                if AgentBase.is_ok(state) or isinstance(state, dict):
                    break
                time.sleep(0.2)
            else:
                return {"captured": False}

        for drone, altitude in ((interceptor, interceptor_altitude), (target, target_altitude)):
            drone.enable_api_control(True)
            drone.takeoff(altitude=altitude)
            deadline = time.time() + 20.0
            while time.time() < deadline:
                position = drone.get_state(frame="ue").get("position", [0.0, 0.0, 0.0])
                if len(position) >= 3 and float(position[2]) >= altitude - 0.8:
                    break
                time.sleep(0.25)
            else:
                return {"captured": False}

        interceptor.set_camera_angles(0.0, 0.0)
        response = guidance.reset()
        if not AgentBase.is_ok(response):
            return {"captured": False}

        response = guidance.visual_intercept_start(
            InterceptorId=interceptor.actor_id,
            TargetId=target.actor_id,
            Method="vision_pid_kalman",
            StopOnCaptureFlag=0,
            MaxForwardSpeed=11.6,
            MaxVerticalSpeed=2.6,
            MaxYawRateDeg=124.0,
            CaptureArea=0.08,
            CenterTolX=0.045,
            CenterTolY=0.060,
            MinRamSpeed=5.8,
            InterceptDistance=1.5,
            TrackLeadTime=0.28,
            RamLeadTime=0.62,
            LostToSearchFrames=24,
            SearchCamYawLimitDeg=0.0,
            SearchCamRateDeg=0.0,
            SearchBodyYawRateDeg=48.0,
            UseKalmanFlag=1,
        )
        if not AgentBase.is_ok(response):
            return {"captured": False}
        visual_started = True

        ui.open()
        center_x = target_pose.x
        center_y = target_pose.y + target_radius
        omega = abs(target_turn_rate)
        loop_dt = 1.0 / hz
        start_time = time.time()
        last_tick = start_time

        while True:
            now = time.time()
            elapsed = now - start_time
            if elapsed > max_time:
                return {"captured": False}

            theta = omega * elapsed
            target_x = center_x + target_radius * math.sin(theta)
            target_y = center_y - target_radius * math.cos(theta)
            target_z = target_altitude + target_vertical_amp * math.sin(0.35 * theta)
            target_yaw = math.degrees(theta) if omega > 1e-6 else target_pose.yaw
            response = target.move_to(
                x=target_x,
                y=target_y,
                z=target_z,
                speed=target_speed,
                frame="ue",
                yaw_mode="angle",
                yaw=target_yaw,
                move_pattern="forward_only",
            )
            if not AgentBase.is_ok(response):
                return {"captured": False}

            image_packet = interceptor.get_image_packet(image_type="scene", quality=image_quality)
            if image_packet is None:
                return {"captured": False}

            frame = image_packet.decode_bgr()
            detection = detector.detect_bgr(frame) if frame is not None else DetectionResult.empty(image_w=image_packet.width, image_h=image_packet.height)
            dt = max(1e-3, now - last_tick)
            last_tick = now
            response = guidance.visual_intercept_update(
                has_detection=detection.has_detection,
                cx=detection.cx,
                cy=detection.cy,
                area=detection.area,
                area_ratio=detection.area_ratio,
                conf=detection.conf,
                dt=dt,
                image_w=detection.image_w or image_packet.width or 640,
                image_h=detection.image_h or image_packet.height or 480,
                interceptor_id=interceptor.actor_id,
                target_id=target.actor_id,
            )
            if not AgentBase.is_ok(response):
                return {"captured": False}

            state = GuidanceState.from_response(response)
            interceptor_pos = interceptor.get_state(frame="ue").get("position", [0.0, 0.0, 0.0])
            target_pos = [target_x, target_y, target_z]
            if ui.render(image_packet, interceptor.actor_id, target.actor_id, state, detection, interceptor_pos, target_pos, frame=frame):
                return {"captured": state.captured}
            if state.captured:
                return {"captured": True}

            sleep_s = loop_dt - (time.time() - now)
            if sleep_s > 0:
                time.sleep(sleep_s)
    finally:
        if guidance is not None and created_guidance and visual_started:
            try:
                guidance.visual_intercept_stop(interceptor_id=interceptor_id, target_id=target_id)
            except Exception:
                pass
        for drone, created in ((interceptor, created_interceptor), (target, created_target)):
            if drone is not None and created:
                try:
                    drone.hover()
                except Exception:
                    pass
        for actor, created in ((target, created_target), (interceptor, created_interceptor), (guidance, created_guidance)):
            if actor is not None and created:
                try:
                    actor.remove()
                except Exception:
                    pass
        ui.close()
        if client is not None:
            client.disconnect()


if __name__ == "__main__":
    run()











