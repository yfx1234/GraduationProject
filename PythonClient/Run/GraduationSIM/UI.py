from __future__ import annotations

try:
    import cv2
except ImportError:
    cv2 = None

from .DataTypes import DetectionResult, GuidanceState, ImagePacket, TrajectoryReference


class VisualUI:
    def __init__(self, show: bool = True, window_name: str = "graduation_visual_intercept") -> None:
        self.show = bool(show)
        self.window_name = window_name
        self.opened = False

    def available(self) -> bool:
        return self.show and cv2 is not None

    def open(self) -> None:
        if not self.available() or self.opened:
            return
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1080, 620)
        except Exception as exc:
            print(f"[WARN] failed to open visualization window: {exc}")
            self.show = False
            self.opened = False
            return
        self.opened = True

    def close(self) -> None:
        if cv2 is None:
            return
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        self.opened = False

    def render(
        self,
        image_packet: ImagePacket,
        interceptor_id: str,
        target_id: str,
        guidance_state: GuidanceState,
        detection: DetectionResult,
        interceptor_pos,
        target_pos,
        target_ref: TrajectoryReference,
        elapsed_s: float,
        frame=None,
    ):
        if not self.available() or image_packet is None:
            return None

        if frame is None:
            frame = image_packet.decode_bgr()
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

    def _draw_detection(self, frame, detection: DetectionResult) -> None:
        if not detection:
            return

        for candidate in detection.candidates:
            x1, y1, x2, y2 = [int(round(v)) for v in (candidate.x1, candidate.y1, candidate.x2, candidate.y2)]
            color = (60, 230, 90) if candidate.selected else (0, 200, 255)
            thickness = 2 if candidate.selected else 1
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)
            if candidate.selected:
                cx = int(round(candidate.cx))
                cy = int(round(candidate.cy))
                cv2.circle(frame, (cx, cy), 4, color, -1, cv2.LINE_AA)
                cv2.putText(
                    frame,
                    f"YOLO {candidate.label} conf={candidate.conf:.2f}",
                    (max(8, x1), max(22, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1,
                    cv2.LINE_AA,
                )

        if detection.has_detection and not detection.candidates and detection.bbox_xyxy:
            x1, y1, x2, y2 = [int(round(v)) for v in detection.bbox_xyxy]
            color = (60, 230, 90)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
            cv2.circle(frame, (int(round(detection.cx)), int(round(detection.cy))), 4, color, -1, cv2.LINE_AA)

    def _draw_overlay(
        self,
        frame,
        interceptor_id: str,
        target_id: str,
        guidance_state: GuidanceState,
        detection: DetectionResult,
        interceptor_pos,
        target_pos,
        target_ref: TrajectoryReference,
        elapsed_s: float,
    ):
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.drawMarker(frame, (cx, cy), (220, 220, 220), cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)
        self._draw_detection(frame, detection)

        control = guidance_state.control or {}
        cmd_vel = guidance_state.cmd_velocity
        ref_pos = target_ref.position_tuple()

        lines = [
            f"YOLO VISUAL INTERCEPT | t={elapsed_s:.1f}s state={guidance_state.state}",
            f"interceptor={interceptor_id} target={target_id}",
            f"detected={detection.has_detection} candidates={len(detection.candidates)} conf={detection.conf:.2f} area_ratio={detection.area_ratio:.4f}",
            f"select={detection.selection_reason} score={detection.selection_score:.2f} captured={guidance_state.captured}",
            f"valid={guidance_state.valid} detections={guidance_state.detections} lost={guidance_state.lost_count}",
            f"target_distance={guidance_state.target_distance:+.2f}m ex={float(control.get('ex', 0.0)):+.3f} ey={float(control.get('ey', 0.0)):+.3f} area_norm={float(control.get('area_norm', 0.0)):.4f}",
            f"yaw_cmd={float(control.get('yaw_cmd_deg', 0.0)):+.2f} yaw_rate={float(control.get('yaw_rate_deg', 0.0)):+.2f}",
            f"cmd_vel=({cmd_vel[0]:+.2f}, {cmd_vel[1]:+.2f}, {cmd_vel[2]:+.2f}) m/s",
            f"intr_pos=({interceptor_pos[0]:+.1f}, {interceptor_pos[1]:+.1f}, {interceptor_pos[2]:+.1f})",
            f"tgt_pos =({target_pos[0]:+.1f}, {target_pos[1]:+.1f}, {target_pos[2]:+.1f})",
            f"tgt_ref =({ref_pos[0]:+.1f}, {ref_pos[1]:+.1f}, {ref_pos[2]:+.1f})",
            "keys: q=quit",
        ]

        for index, text in enumerate(lines):
            y = 22 + index * 20
            cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.53, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.53, (235, 235, 235), 1, cv2.LINE_AA)
        return frame
