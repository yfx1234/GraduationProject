from __future__ import annotations

import math
import time
from pathlib import Path

import cv2

from GraduationSIM import AgentBase, AgentDrone, AgentGuidance, DEFAULT_DRONE_CLASS, Pose, TCPClient
from run import GuidanceState, YoloDetector


def ok(resp):
    return AgentBase.is_ok(resp)


def wait_ok(fn, timeout=5.0, dt=0.2):
    end = time.time() + timeout
    while time.time() < end:
        resp = fn()
        if isinstance(resp, dict):
            return resp
        time.sleep(dt)
    return {}


def wait_alt(drone, alt, timeout=20.0):
    end = time.time() + timeout
    while time.time() < end:
        pos = drone.get_state(frame="ue").get("position", [0.0, 0.0, 0.0])
        if len(pos) >= 3 and float(pos[2]) >= alt - 0.8:
            return True
        time.sleep(0.25)
    return False


class BBoxPredictor:
    # 中文注释：这里单独做一个轻量级框预测，只负责把 YOLO 框预测到下一时刻。
    def __init__(self, alpha=0.74, beta=0.18):
        self.beta = float(beta)
        self.alpha = float(alpha)
        self.box = None
        self.vel = [0.0, 0.0, 0.0, 0.0]

    def update(self, box, dt):
        if not box or len(box) != 4:
            return
        box = [float(v) for v in box]
        dt = max(1e-3, float(dt))
        if self.box is None:
            self.box = box
            self.vel = [0.0, 0.0, 0.0, 0.0]
            return
        pred = [self.box[i] + self.vel[i] * dt for i in range(4)]
        residual = [box[i] - pred[i] for i in range(4)]
        self.box = [pred[i] + self.alpha * residual[i] for i in range(4)]
        gain = self.beta / dt
        self.vel = [self.vel[i] + gain * residual[i] for i in range(4)]

    def predict(self, lead_time, image_w, image_h):
        if self.box is None:
            return None
        lead = max(0.0, float(lead_time))
        box = [self.box[i] + self.vel[i] * lead for i in range(4)]
        x1, y1, x2, y2 = box
        x1 = max(0.0, min(float(image_w - 2), x1))
        y1 = max(0.0, min(float(image_h - 2), y1))
        x2 = max(x1 + 1.0, min(float(image_w - 1), x2))
        y2 = max(y1 + 1.0, min(float(image_h - 1), y2))
        return [x1, y1, x2, y2]


def draw_box(frame, box, color, label, thickness=2):
    if not box or len(box) != 4:
        return
    h, w = frame.shape[:2]
    x1 = max(0, min(w - 2, int(round(box[0]))))
    y1 = max(0, min(h - 2, int(round(box[1]))))
    x2 = max(x1 + 1, min(w - 1, int(round(box[2]))))
    y2 = max(y1 + 1, min(h - 1, int(round(box[3]))))
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)
    cv2.putText(frame, label, (max(8, x1), max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA)


def draw(frame, detection, resp, bbox_predictor):
    h, w = frame.shape[:2]
    center = (w // 2, h // 2)
    cv2.drawMarker(frame, center, (240, 240, 240), cv2.MARKER_CROSS, 16, 1, cv2.LINE_AA)

    if detection and detection.bbox_xyxy:
        draw_box(frame, detection.bbox_xyxy, (60, 230, 90), f"YOLO {detection.conf:.2f}", thickness=2)

    state = GuidanceState.from_response(resp if isinstance(resp, dict) else {})
    track = state.track or {}
    prediction = state.prediction or {}
    control = state.control or {}
    kalman = state.kalman or {}

    lead_time = float(prediction.get("lead_time", 0.0))
    if not prediction.get("valid") and track.get("used_prediction"):
        lead_time = float(track.get("lead_time", lead_time))

    if (prediction.get("valid") or track.get("used_prediction")) and bbox_predictor.box is not None:
        pred_box = bbox_predictor.predict(lead_time, w, h)
        if pred_box is not None:
            draw_box(frame, pred_box, (0, 0, 255), f"KALMAN {lead_time:.2f}s", thickness=2)

    track_mode = "prediction" if track.get("used_prediction") else "measurement"
    text = (
        f"state={state.state} lost={state.lost_count} track={track_mode} pred={int(bool(prediction.get('valid')))} "
        f"ex={float(control.get('ex', 0.0)):+.3f} ey={float(control.get('ey', 0.0)):+.3f} "
        f"yaw={float(control.get('yaw_cmd_deg', 0.0)):+.1f} q={float(kalman.get('q', 0.0)):.2f} "
        f"u={float(kalman.get('uncertainty', 0.0)):.2f}"
    )
    cv2.putText(frame, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (235, 235, 235), 1, cv2.LINE_AA)
    cv2.imshow("fixed_cam_intercept", frame)
    return (cv2.waitKey(1) & 0xFF) == ord("q")


def main():
    host, port, timeout = "127.0.0.1", 9000, 10.0
    interceptor_id, target_id, guidance_id = "drone_0", "drone_1", "guidance_0"
    interceptor_pose = Pose(x=0.0, y=0.0, z=1.0, yaw=0.0)
    target_pose = Pose(x=12.0, y=-3.5, z=1.0, yaw=180.0)
    interceptor_alt, target_alt = 8.0, 8.0
    target_speed, target_radius, target_turn_rate = 4.5, 3.5, 0.18
    hz, image_quality, max_time = 25.0, 82, 500.0
    model_path = (
        Path(__file__).resolve().parents[1]
        / "YOLO"
        / "runs"
        / "detect"
        / "drone_detect2_collect_finetune_w0"
        / "weights"
        / "best.pt"
    )

    client = TCPClient(host=host, port=port, timeout=timeout, auto_connect=False)
    interceptor = target = guidance = detector = None
    bbox_predictor = BBoxPredictor()
    created_interceptor = created_target = created_guidance = visual_started = False

    try:
        if not client.connect():
            return
        interceptor = AgentDrone(client, actor_id=interceptor_id, classname=DEFAULT_DRONE_CLASS, label="Interceptor", mission_role="interceptor")
        target = AgentDrone(client, actor_id=target_id, classname=DEFAULT_DRONE_CLASS, label="Target", mission_role="target")
        guidance = AgentGuidance(client, actor_id=guidance_id, classname="GuidanceActor", label="Guidance")

        for actor_id in (guidance_id, interceptor_id, target_id):
            client.send_message({"remove_actor": {"actor_id": actor_id}})
        time.sleep(0.2)

        if not ok(guidance.create(pose=Pose(), classname="GuidanceActor", label="Guidance")):
            return
        created_guidance = True
        if not ok(interceptor.create(pose=interceptor_pose, classname=DEFAULT_DRONE_CLASS, label="Interceptor")):
            return
        created_interceptor = True
        if not ok(target.create(pose=target_pose, classname=DEFAULT_DRONE_CLASS, label="Target")):
            return
        created_target = True

        wait_ok(lambda: guidance.get_state())
        wait_ok(lambda: interceptor.get_state(frame="ue"))
        wait_ok(lambda: target.get_state(frame="ue"))

        for drone, alt in ((interceptor, interceptor_alt), (target, target_alt)):
            drone.enable_api_control(True)
            drone.takeoff(altitude=alt)
            if not wait_alt(drone, alt):
                return

        interceptor.hover()
        target.hover()
        time.sleep(1.0)
        if not ok(interceptor.set_camera_fov(128.0)):
            return
        interceptor.set_camera_angles(0.0, 0.0)
        if not ok(guidance.reset()):
            return

        # 中文注释：把 YOLO 放到连接和起飞之后再加载，避免在连接仿真前阻塞。
        detector = YoloDetector(model_path=str(model_path), conf=0.35, iou=0.45, imgsz=640, class_id=0, max_det=5)

        resp = guidance.visual_intercept_start(
            InterceptorId=interceptor.actor_id,
            TargetId=target.actor_id,
            Method="vision_pid_kalman",
            StopOnCaptureFlag=0,
            MaxForwardSpeed=10.8,
            MaxVerticalSpeed=2.6,
            MaxYawRateDeg=138.0,
            CaptureArea=0.10,
            CenterTolX=0.032,
            CenterTolY=0.045,
            RamAreaTarget=0.24,
            MinRamSpeed=6.0,
            InterceptDistance=1.2,
            TrackLeadTime=0.34,
            RamLeadTime=0.95,
            LostToSearchFrames=28,
            SearchCamPitchDeg=0.0,
            SearchCamYawLimitDeg=0.0,
            SearchCamRateDeg=0.0,
            SearchBodyYawRateDeg=52.0,
            UseKalmanFlag=1,
        )
        if not ok(resp):
            return
        visual_started = True

        cv2.namedWindow("fixed_cam_intercept", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("fixed_cam_intercept", 1080, 620)
        center_x = target_pose.x
        center_y = target_pose.y + target_radius
        omega = abs(target_turn_rate)
        loop_dt = 1.0 / hz
        start = last = time.time()

        while True:
            now = time.time()
            if now - start > max_time:
                break

            theta = omega * (now - start)
            tx = center_x + target_radius * math.sin(theta)
            ty = center_y - target_radius * math.cos(theta)
            tz = target_alt
            yaw = math.degrees(theta) if omega > 1e-6 else target_pose.yaw
            if not ok(target.move_to(x=tx, y=ty, z=tz, speed=target_speed, frame="ue", yaw_mode="angle", yaw=yaw, move_pattern="forward_only")):
                break

            packet = interceptor.get_image_packet(image_type="scene", quality=image_quality)
            if packet is None:
                break
            frame = packet.decode_bgr()
            if frame is None:
                break

            detection = detector.detect_bgr(frame)
            dt = max(1e-3, now - last)
            last = now
            if detection and detection.bbox_xyxy:
                bbox_predictor.update(detection.bbox_xyxy, dt)

            resp = guidance.visual_intercept_update(
                has_detection=detection.has_detection,
                cx=detection.cx,
                cy=detection.cy,
                area=detection.area,
                area_ratio=detection.area_ratio,
                conf=detection.conf,
                dt=dt,
                image_w=detection.image_w or packet.width or 640,
                image_h=detection.image_h or packet.height or 480,
                interceptor_id=interceptor.actor_id,
                target_id=target.actor_id,
            )
            if not ok(resp):
                break
            if draw(frame, detection, resp, bbox_predictor):
                break
            if resp.get("captured"):
                break

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
        client.disconnect()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()



