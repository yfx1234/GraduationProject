"""Visualization helpers for the visual intercept demo."""

try:
    import cv2
except ImportError:
    cv2 = None

from .VisionUtils import decode_bgr_base64


class VisualInterceptView:
    def __init__(self, show=True, window_name="auto_spawn_visual_intercept"):
        self.show = bool(show)
        self.window_name = window_name
        self.opened = False

    def available(self):
        return self.show and cv2 is not None

    def open(self):
        if not self.available() or self.opened:
            return
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1080, 620)
        self.opened = True

    def close(self):
        if cv2 is None:
            return
        cv2.destroyAllWindows()
        self.opened = False

    def render(
        self,
        image_resp,
        interceptor_id,
        target_id,
        guidance_state,
        detection,
        interceptor_pos,
        target_pos,
        target_ref,
        elapsed_s,
        frame=None,
    ):
        if not self.available() or not image_resp:
            return None

        if frame is None:
            frame = decode_bgr_base64(image_resp.get("data", ""))
        if frame is None:
            return None

        vis = self._draw_overlay(
            frame,
            interceptor_id,
            target_id,
            guidance_state,
            detection,
            interceptor_pos,
            target_pos,
            target_ref,
            elapsed_s,
        )
        cv2.imshow(self.window_name, vis)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            print("[INFO] user quit visualization window")
            return "user quit"
        return None

    def _draw_detection(self, frame, detection):
        if not detection:
            return

        for candidate in detection.get("candidates", []):
            x1, y1, x2, y2 = [int(round(v)) for v in (candidate["x1"], candidate["y1"], candidate["x2"], candidate["y2"])]
            selected = bool(candidate.get("selected", False))
            color = (60, 230, 90) if selected else (0, 200, 255)
            thickness = 2 if selected else 1
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)
            if selected:
                cx = int(round(candidate["cx"]))
                cy = int(round(candidate["cy"]))
                cv2.circle(frame, (cx, cy), 4, color, -1, cv2.LINE_AA)
                cv2.putText(
                    frame,
                    (
                        f"YOLO {candidate.get('label', '')} conf={float(candidate.get('conf', 0.0)):.2f} "
                        f"score={float(candidate.get('selection_score', 0.0)):.2f}"
                    ),
                    (max(8, x1), max(22, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1,
                    cv2.LINE_AA,
                )

        if detection.get("has_detection") and not detection.get("candidates"):
            x1, y1, x2, y2 = [int(round(v)) for v in detection["bbox_xyxy"]]
            color = (60, 230, 90)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
            cx = int(round(detection["cx"]))
            cy = int(round(detection["cy"]))
            cv2.circle(frame, (cx, cy), 4, color, -1, cv2.LINE_AA)

    def _draw_overlay(
        self,
        frame,
        interceptor_id,
        target_id,
        guidance_state,
        detection,
        interceptor_pos,
        target_pos,
        target_ref,
        elapsed_s,
    ):
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.drawMarker(frame, (cx, cy), (220, 220, 220), cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)
        self._draw_detection(frame, detection)

        cmd_vel = guidance_state.get("cmd_velocity", [0.0, 0.0, 0.0]) if isinstance(guidance_state, dict) else [0.0, 0.0, 0.0]
        if not isinstance(cmd_vel, (list, tuple)) or len(cmd_vel) < 3:
            cmd_vel = [0.0, 0.0, 0.0]

        control = guidance_state.get("control", {}) if isinstance(guidance_state, dict) else {}
        state = str(guidance_state.get("state", "UNKNOWN")) if isinstance(guidance_state, dict) else "UNKNOWN"
        detections = int(guidance_state.get("detections", 0)) if isinstance(guidance_state, dict) else 0
        lost_count = int(guidance_state.get("lost_count", 0)) if isinstance(guidance_state, dict) else 0
        target_distance = float(guidance_state.get("target_distance", -1.0)) if isinstance(guidance_state, dict) else -1.0
        candidate_count = len((detection or {}).get("candidates", []))

        lines = [
            f"YOLO VISUAL INTERCEPT | t={elapsed_s:.1f}s state={state}",
            f"interceptor={interceptor_id} target={target_id}",
            (
                f"detected={bool(detection and detection.get('has_detection'))} candidates={candidate_count} "
                f"conf={float((detection or {}).get('conf', 0.0)):.2f} area_ratio={float((detection or {}).get('area_ratio', 0.0)):.4f}"
            ),
            (
                f"select={str((detection or {}).get('selection_reason', ''))} "
                f"score={float((detection or {}).get('selection_score', 0.0)):.2f} "
                f"captured={bool(guidance_state.get('captured', False)) if isinstance(guidance_state, dict) else False}"
            ),
            f"valid={bool(guidance_state.get('valid', False)) if isinstance(guidance_state, dict) else False} detections={detections} lost={lost_count}",
            f"target_distance={target_distance:+.2f}m ex={float(control.get('ex', 0.0)):+.3f} ey={float(control.get('ey', 0.0)):+.3f} area_norm={float(control.get('area_norm', 0.0)):.4f}",
            f"yaw_cmd={float(control.get('yaw_cmd_deg', 0.0)):+.2f} yaw_rate={float(control.get('yaw_rate_deg', 0.0)):+.2f}",
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
