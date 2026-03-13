from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import List, Optional

from .DataTypes import DetectionCandidate, DetectionResult


class YoloDetector:
    def __init__(
        self,
        model_path: Optional[str] = None,
        conf: float = 0.35,
        iou: float = 0.45,
        imgsz: int = 640,
        device: Optional[str] = None,
        class_id: Optional[int] = 0,
        max_det: int = 5,
        selection_strategy: str = "heuristic",
    ) -> None:
        self.project_root = Path(__file__).resolve().parents[3]
        self.model_path = self._resolve_model_path(model_path)
        self.conf = float(conf)
        self.iou = float(iou)
        self.imgsz = int(imgsz)
        self.device = None if device in (None, "", "auto") else str(device)
        self.class_id = None if class_id is None else int(class_id)
        self.max_det = max(1, int(max_det))
        self.selection_strategy = self._normalize_selection_strategy(selection_strategy)
        self.shadow_reject_cy_ratio = 0.82
        self.shadow_pair_dx_ratio = 0.18
        self.shadow_pair_dy_ratio = 0.06
        self.vertical_preference = 0.65
        self.shadow_pair_penalty = 0.55
        self._ensure_ultralytics_import_path()

        try:
            from ultralytics import YOLO
        except Exception as exc:
            raise RuntimeError(f"failed to import ultralytics from local source: {exc}") from exc

        try:
            self.model = YOLO(str(self.model_path))
        except Exception as exc:
            raise RuntimeError(f"failed to load YOLO model '{self.model_path}': {exc}") from exc

        names = getattr(self.model, "names", {})
        self.names = names if isinstance(names, dict) else {}

    @staticmethod
    def _normalize_selection_strategy(selection_strategy: Optional[str]) -> str:
        text = str(selection_strategy or "heuristic").strip().lower()
        if text in ("highest_confidence", "confidence", "raw"):
            return "highest_confidence"
        return "heuristic"

    def _ensure_ultralytics_import_path(self) -> None:
        source_root = self.project_root / "PythonClient" / "YOLO" / "ultralytics"
        if not source_root.exists():
            raise RuntimeError(f"local ultralytics source not found: {source_root}")

        settings_root = self.project_root / "PythonClient" / ".ultralytics_config"
        settings_root.mkdir(parents=True, exist_ok=True)

        os.environ.setdefault("YOLO_CONFIG_DIR", str(settings_root))
        os.environ.setdefault("YOLO_AUTOINSTALL", "False")
        os.environ.setdefault("YOLO_VERBOSE", "False")

        source_text = str(source_root)
        if source_text not in sys.path:
            sys.path.insert(0, source_text)

    def _resolve_model_path(self, model_path: Optional[str]) -> Path:
        if model_path:
            candidate = Path(model_path)
            if not candidate.is_absolute():
                candidate = self.project_root / candidate
            if candidate.exists():
                return candidate.resolve()
            raise RuntimeError(f"YOLO model path not found: {candidate}")

        candidates = [
            self.project_root / "PythonClient" / "YOLO" / "runs" / "detect" / "drone_detect2" / "weights" / "best.pt",
            self.project_root / "PythonClient" / "YOLO" / "runs" / "detect" / "drone_detect" / "weights" / "best.pt",
            self.project_root / "PythonClient" / "YOLO" / "weights" / "yolo26n-objv1-150.pt",
            self.project_root / "PythonClient" / "YOLO" / "weights" / "yolo26n.pt",
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate.resolve()
        raise RuntimeError("no YOLO model found under PythonClient/YOLO")

    def describe(self):
        class_desc = "all" if self.class_id is None else str(self.class_id)
        return {
            "model_path": str(self.model_path),
            "conf": self.conf,
            "iou": self.iou,
            "imgsz": self.imgsz,
            "device": self.device or "auto",
            "class_id": class_desc,
            "max_det": self.max_det,
            "selection_strategy": self.selection_strategy,
            "names": self.names,
        }

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

        frame_h, frame_w = frame.shape[:2]
        candidates = self._build_candidates(boxes, frame_w, frame_h)
        selected = self._select_candidate(candidates, frame_w, frame_h)
        if selected is None:
            return DetectionResult.empty(image_w=frame_w, image_h=frame_h)

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
            image_w=int(frame_w),
            image_h=int(frame_h),
            selection_score=selected.selection_score,
            selection_reason=selected.selection_reason,
            candidates=candidates,
        )

    def _build_candidates(self, boxes, frame_w: int, frame_h: int) -> List[DetectionCandidate]:
        confidences = boxes.conf.detach().cpu().numpy()
        xyxy_all = boxes.xyxy.detach().cpu().numpy()
        classes = None
        if getattr(boxes, "cls", None) is not None:
            classes = boxes.cls.detach().cpu().numpy()

        frame_area = max(1.0, float(frame_w * frame_h))
        candidates: List[DetectionCandidate] = []
        for index, xyxy in enumerate(xyxy_all):
            x1, y1, x2, y2 = [float(v) for v in xyxy.tolist()]
            width = max(0.0, x2 - x1)
            height = max(0.0, y2 - y1)
            area = width * height
            class_value = int(classes[index]) if classes is not None else -1
            cx = 0.5 * (x1 + x2)
            cy = 0.5 * (y1 + y2)
            candidates.append(
                DetectionCandidate(
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                    cx=cx,
                    cy=cy,
                    area=area,
                    area_ratio=area / frame_area,
                    conf=float(confidences[index]),
                    class_id=class_value,
                    label=self.names.get(class_value, str(class_value)),
                    cx_norm=cx / max(1.0, float(frame_w)),
                    cy_norm=cy / max(1.0, float(frame_h)),
                )
            )
        return candidates

    def _apply_shadow_penalty(self, candidates: List[DetectionCandidate], frame_w: int, frame_h: int) -> None:
        dx_limit = self.shadow_pair_dx_ratio * max(1.0, float(frame_w))
        dy_limit = self.shadow_pair_dy_ratio * max(1.0, float(frame_h))
        for upper in candidates:
            for lower in candidates:
                if upper is lower:
                    continue
                if upper.cy >= lower.cy:
                    continue
                if abs(upper.cx - lower.cx) > dx_limit:
                    continue
                if (lower.cy - upper.cy) < dy_limit:
                    continue
                lower.shadow_penalty = max(lower.shadow_penalty, self.shadow_pair_penalty)

    def _score_candidate(self, candidate: DetectionCandidate) -> float:
        vertical_bonus = self.vertical_preference * (1.0 - candidate.cy_norm)
        return candidate.conf + vertical_bonus - candidate.shadow_penalty

    def _select_candidate(self, candidates: List[DetectionCandidate], frame_w: int, frame_h: int) -> Optional[DetectionCandidate]:
        if not candidates:
            return None

        if self.selection_strategy == "highest_confidence":
            selected = max(candidates, key=lambda item: (item.conf, item.area))
            selected.selection_score = selected.conf
            selected.selection_reason = "highest_confidence"
            for candidate in candidates:
                if candidate is not selected:
                    candidate.selection_score = candidate.conf
                    candidate.selection_reason = "candidate"
            return selected

        self._apply_shadow_penalty(candidates, frame_w, frame_h)
        eligible = [
            candidate
            for candidate in candidates
            if candidate.cy_norm <= self.shadow_reject_cy_ratio or candidate.shadow_penalty < 0.4
        ]
        pool = eligible if eligible else candidates

        for candidate in candidates:
            candidate.selection_score = self._score_candidate(candidate)
            if candidate in pool:
                candidate.selection_reason = "primary"
            elif candidate.cy_norm > self.shadow_reject_cy_ratio:
                candidate.selection_reason = "bottom_reject"
            else:
                candidate.selection_reason = "shadow_penalty"

        return max(pool, key=lambda item: (item.selection_score, item.conf, -item.cy_norm))
