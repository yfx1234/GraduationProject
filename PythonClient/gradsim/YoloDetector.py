"""Ultralytics detector wrapper for the visual intercept mission."""

import os
import sys
from pathlib import Path


class YoloDetector:
    def __init__(
        self,
        model_path=None,
        conf=0.35,
        iou=0.45,
        imgsz=640,
        device=None,
        class_id=0,
        max_det=5,
    ):
        self.project_root = Path(__file__).resolve().parents[2]
        self.model_path = self._resolve_model_path(model_path)
        self.conf = float(conf)
        self.iou = float(iou)
        self.imgsz = int(imgsz)
        self.device = None if device in (None, "", "auto") else str(device)
        self.class_id = None if class_id is None else int(class_id)
        self.max_det = max(1, int(max_det))
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

    def _ensure_ultralytics_import_path(self):
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

    def _resolve_model_path(self, model_path):
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
            "names": self.names,
        }

    def detect_bgr(self, frame):
        if frame is None:
            return self.empty_detection()

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
            return self.empty_detection(frame)

        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) <= 0:
            return self.empty_detection(frame)

        frame_h, frame_w = frame.shape[:2]
        candidates = self._build_candidates(boxes, frame_w, frame_h)
        selected = self._select_candidate(candidates, frame_w, frame_h)
        if selected is None:
            return self.empty_detection(frame)

        for candidate in candidates:
            candidate["selected"] = candidate["index"] == selected["index"]

        return {
            "has_detection": True,
            "bbox_xyxy": [selected["x1"], selected["y1"], selected["x2"], selected["y2"]],
            "cx": selected["cx"],
            "cy": selected["cy"],
            "area": selected["area"],
            "area_ratio": selected["area_ratio"],
            "conf": selected["conf"],
            "class_id": selected["class_id"],
            "label": selected["label"],
            "image_w": int(frame_w),
            "image_h": int(frame_h),
            "selection_score": selected["selection_score"],
            "selection_reason": selected["selection_reason"],
            "candidates": candidates,
        }

    def _build_candidates(self, boxes, frame_w, frame_h):
        confidences = boxes.conf.detach().cpu().numpy()
        xyxy_all = boxes.xyxy.detach().cpu().numpy()
        classes = None
        if getattr(boxes, "cls", None) is not None:
            classes = boxes.cls.detach().cpu().numpy()

        frame_area = max(1.0, float(frame_w * frame_h))
        candidates = []
        for idx, xyxy in enumerate(xyxy_all):
            x1, y1, x2, y2 = [float(v) for v in xyxy.tolist()]
            width = max(0.0, x2 - x1)
            height = max(0.0, y2 - y1)
            area = width * height
            class_id = int(classes[idx]) if classes is not None else -1
            cx = 0.5 * (x1 + x2)
            cy = 0.5 * (y1 + y2)
            candidates.append(
                {
                    "index": idx,
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "width": width,
                    "height": height,
                    "area": area,
                    "area_ratio": area / frame_area,
                    "cx": cx,
                    "cy": cy,
                    "cx_norm": cx / max(1.0, float(frame_w)),
                    "cy_norm": cy / max(1.0, float(frame_h)),
                    "conf": float(confidences[idx]),
                    "class_id": class_id,
                    "label": self.names.get(class_id, str(class_id)),
                    "shadow_penalty": 0.0,
                    "selection_score": 0.0,
                    "selection_reason": "",
                    "selected": False,
                }
            )
        return candidates

    def _apply_shadow_penalty(self, candidates, frame_w, frame_h):
        dx_limit = self.shadow_pair_dx_ratio * max(1.0, float(frame_w))
        dy_limit = self.shadow_pair_dy_ratio * max(1.0, float(frame_h))
        for upper in candidates:
            for lower in candidates:
                if upper["index"] == lower["index"]:
                    continue
                if upper["cy"] >= lower["cy"]:
                    continue
                if abs(upper["cx"] - lower["cx"]) > dx_limit:
                    continue
                if (lower["cy"] - upper["cy"]) < dy_limit:
                    continue
                lower["shadow_penalty"] = max(lower["shadow_penalty"], self.shadow_pair_penalty)

    def _score_candidate(self, candidate):
        vertical_bonus = self.vertical_preference * (1.0 - candidate["cy_norm"])
        return candidate["conf"] + vertical_bonus - candidate["shadow_penalty"]

    def _select_candidate(self, candidates, frame_w, frame_h):
        if not candidates:
            return None

        self._apply_shadow_penalty(candidates, frame_w, frame_h)
        eligible = [
            candidate
            for candidate in candidates
            if candidate["cy_norm"] <= self.shadow_reject_cy_ratio or candidate["shadow_penalty"] < 0.4
        ]
        pool = eligible if eligible else candidates

        for candidate in candidates:
            candidate["selection_score"] = self._score_candidate(candidate)
            if candidate in pool:
                candidate["selection_reason"] = "primary"
            elif candidate["cy_norm"] > self.shadow_reject_cy_ratio:
                candidate["selection_reason"] = "bottom_reject"
            else:
                candidate["selection_reason"] = "shadow_penalty"

        return max(pool, key=lambda item: (item["selection_score"], item["conf"], -item["cy_norm"]))

    @staticmethod
    def empty_detection(frame=None):
        image_h = 0
        image_w = 0
        if frame is not None:
            image_h, image_w = frame.shape[:2]
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
            "image_w": int(image_w),
            "image_h": int(image_h),
            "selection_score": 0.0,
            "selection_reason": "",
            "candidates": [],
        }
