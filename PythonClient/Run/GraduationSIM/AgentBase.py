from __future__ import annotations

import json
from typing import Any, Dict, Optional

from .DataTypes import ImagePacket, Pose


JsonDict = Dict[str, Any]


class AgentBase:
    _IMAGE_TYPE_NAME_MAP = {
        0: "scene",
        1: "depth_planar",
        3: "depth_vis",
        5: "segmentation",
        7: "infrared",
    }

    def __init__(
        self,
        client,
        actor_id: str,
        classname: Optional[str] = None,
        label: str = "Agent",
        spawn_unit: str = "m",
    ) -> None:
        self.client = client
        self.actor_id = actor_id
        self.classname = classname
        self.label = label
        self.spawn_unit = spawn_unit
        self.last_response: JsonDict = {}

    def _create_extra(self) -> JsonDict:
        extra: JsonDict = {}
        if self.spawn_unit:
            extra["unit"] = self.spawn_unit
        return extra

    @staticmethod
    def is_ok(response: Any) -> bool:
        if not isinstance(response, dict):
            return False
        status = response.get("status")
        return status == "ok" or status is True

    @staticmethod
    def unwrap_response(response: JsonDict, field_name: str) -> JsonDict:
        if isinstance(response, dict):
            value = response.get(field_name)
            if isinstance(value, dict):
                return value
        return response

    @staticmethod
    def _load_json_text(value: Any) -> Any:
        if isinstance(value, (dict, list)):
            return value
        if not isinstance(value, str):
            return value
        text = value.strip()
        if not text:
            return {}
        if text[:1] not in "[{":
            return value
        try:
            return json.loads(text)
        except Exception:
            return value

    @staticmethod
    def normalize_frame(frame: str) -> str:
        if isinstance(frame, str) and frame.lower() == "ned":
            return "ned"
        return "ue"

    @classmethod
    def normalize_image_type(cls, image_type: Any) -> str:
        if isinstance(image_type, int):
            return cls._IMAGE_TYPE_NAME_MAP.get(image_type, "scene")
        text = str(image_type).strip().lower()
        if text.isdigit():
            return cls._IMAGE_TYPE_NAME_MAP.get(int(text), "scene")
        if text in ("depth", "depthplanar", "depth_planar"):
            return "depth_planar"
        if text in ("depthvis", "depth_vis"):
            return "depth_vis"
        if text in ("seg", "segment"):
            return "segmentation"
        if text == "ir":
            return "infrared"
        return text or "scene"

    @staticmethod
    def _normalize_param(value: Any) -> Any:
        if isinstance(value, (int, float, bool, str)):
            return value
        if isinstance(value, Pose):
            return value.to_dict()
        if isinstance(value, tuple):
            value = list(value)
        if isinstance(value, list):
            return [AgentBase._normalize_param(item) for item in value]
        if isinstance(value, dict):
            return {str(key): AgentBase._normalize_param(item) for key, item in value.items()}
        return str(value)

    def create_raw(
        self,
        pose: Optional[Pose] = None,
        classname: Optional[str] = None,
        label: Optional[str] = None,
        extra: Optional[JsonDict] = None,
    ) -> JsonDict:
        actor_classname = classname or self.classname
        if not actor_classname:
            raise ValueError("classname must be provided before creating actor")

        spawn_pose = pose or Pose()
        payload: JsonDict = {
            "classname": actor_classname,
            "expected_id": self.actor_id,
            "label": label or self.label or "Agent",
            "pose": spawn_pose.to_spawn_dict(),
        }

        merged_extra = self._create_extra()
        if extra:
            merged_extra.update(extra)
        if merged_extra:
            payload.update(merged_extra)

        response = self.client.request({"add_actor": payload})
        self.last_response = self.unwrap_response(response, "add_actor_return")
        self.classname = actor_classname
        self.label = payload["label"]
        return self.last_response

    def create(
        self,
        pose: Optional[Pose] = None,
        classname: Optional[str] = None,
        label: Optional[str] = None,
        extra: Optional[JsonDict] = None,
    ) -> bool:
        return self.is_ok(self.create_raw(pose=pose, classname=classname, label=label, extra=extra))

    def remove_raw(self) -> JsonDict:
        response = self.client.request({"remove_actor": {"actor_id": self.actor_id}})
        self.last_response = self.unwrap_response(response, "remove_actor_return")
        return self.last_response

    def remove(self) -> bool:
        return self.is_ok(self.remove_raw())

    def call_function(
        self,
        function_name: str,
        *args: Any,
        expect_return: bool = False,
        named_parameters: Optional[JsonDict] = None,
    ) -> JsonDict:
        payload: JsonDict = {
            "actor_id": self.actor_id,
            "function": function_name,
            "return": bool(expect_return),
        }

        if args and named_parameters is not None:
            raise ValueError("use positional parameters or named_parameters, not both")

        if named_parameters is not None:
            payload["parameters"] = self._normalize_param(named_parameters)
        elif args:
            payload["parameters"] = [self._normalize_param(arg) for arg in args]

        response = self.client.request({"call_actor": payload})
        self.last_response = self.unwrap_response(response, "call_actor_return")
        return self.last_response

    def call_json(self, function_name: str, named_parameters: Optional[JsonDict] = None) -> JsonDict:
        response = self.call_function(function_name, expect_return=True, named_parameters=named_parameters)
        if not self.is_ok(response):
            return response

        parsed = self._load_json_text(response.get("return", ""))
        if isinstance(parsed, dict):
            self.last_response = parsed
            return parsed
        if parsed == {}:
            self.last_response = {"status": "ok"}
            return self.last_response
        self.last_response = {"status": "ok", "return": parsed}
        return self.last_response

    def get_state(self, frame: str = "ue") -> JsonDict:
        return self.call_json("GetState", named_parameters={"Frame": self.normalize_frame(frame)})

    state = get_state

    def get_image_response(self, image_type: Any = "scene", quality: int = 90, max_depth_m: float = 200.0) -> JsonDict:
        params = {
            "ImageType": self.normalize_image_type(image_type),
            "Quality": int(quality),
            "MaxDepthMeters": float(max_depth_m),
        }
        return self.call_json("GetImage", named_parameters=params)

    def get_image_packet(self, image_type: Any = "scene", quality: int = 90, max_depth_m: float = 200.0) -> Optional[ImagePacket]:
        response = self.get_image_response(image_type=image_type, quality=quality, max_depth_m=max_depth_m)
        if not self.is_ok(response):
            return None
        return ImagePacket.from_response(response, image_type=self.normalize_image_type(image_type))

    def get_image_base64(self, image_type: Any = "scene", quality: int = 90, max_depth_m: float = 200.0) -> Optional[str]:
        packet = self.get_image_packet(image_type=image_type, quality=quality, max_depth_m=max_depth_m)
        return packet.data if packet else None
