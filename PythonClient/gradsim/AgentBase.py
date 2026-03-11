"""Shared actor wrapper for GradSim."""

import base64


class AgentBase:
    """Base wrapper for common actor operations."""

    _IMAGE_TYPE_NAME_MAP = {
        0: "scene",
        1: "depth_planar",
        3: "depth_vis",
        5: "segmentation",
        7: "infrared",
    }

    def __init__(self, client, actor_id, classname=None, label="Agent", spawn_unit="m"):
        self.client = client
        self.actor_id = actor_id
        self.classname = classname
        self.label = label
        self.spawn_unit = spawn_unit
        self.last_response = {}

    def _create_extra(self):
        extra = {}
        if self.spawn_unit:
            extra["unit"] = self.spawn_unit
        return extra

    @staticmethod
    def is_ok(response):
        if not isinstance(response, dict):
            return False
        status = response.get("status")
        return status == "ok" or status is True

    @staticmethod
    def unwrap_response(response, field_name):
        if isinstance(response, dict):
            field = response.get(field_name)
            if isinstance(field, dict):
                return field
        return response

    @staticmethod
    def normalize_frame(frame):
        if isinstance(frame, str) and frame.lower() == "ned":
            return "ned"
        return "ue"

    @classmethod
    def normalize_image_type(cls, image_type):
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

    def create_raw(
        self,
        spawn_x=0.0,
        spawn_y=0.0,
        spawn_z=0.0,
        yaw=0.0,
        roll=0.0,
        pitch=0.0,
        classname=None,
        label=None,
        extra=None,
    ):
        actor_classname = classname or self.classname
        if not actor_classname:
            raise ValueError("classname must be provided before creating actor")

        actor_label = label or self.label or "Agent"
        payload = {
            "classname": actor_classname,
            "expected_id": self.actor_id,
            "label": actor_label,
            "pose": {
                "x": float(spawn_x),
                "y": float(spawn_y),
                "z": float(spawn_z),
                "roll": float(roll),
                "pitch": float(pitch),
                "yaw": float(yaw),
            },
        }

        merged_extra = self._create_extra()
        if extra:
            merged_extra.update(extra)
        if merged_extra:
            payload.update(merged_extra)

        response = self.client.request({"add_actor": payload})
        self.last_response = self.unwrap_response(response, "add_actor_return")
        self.classname = actor_classname
        self.label = actor_label
        return self.last_response

    def create(
        self,
        spawn_x=0.0,
        spawn_y=0.0,
        spawn_z=0.0,
        yaw=0.0,
        roll=0.0,
        pitch=0.0,
        classname=None,
        label=None,
        extra=None,
    ):
        return self.is_ok(
            self.create_raw(
                spawn_x=spawn_x,
                spawn_y=spawn_y,
                spawn_z=spawn_z,
                yaw=yaw,
                roll=roll,
                pitch=pitch,
                classname=classname,
                label=label,
                extra=extra,
            )
        )

    def remove_raw(self):
        response = self.client.request({"remove_actor": {"actor_id": self.actor_id}})
        self.last_response = self.unwrap_response(response, "remove_actor_return")
        return self.last_response

    def remove(self):
        return self.is_ok(self.remove_raw())

    @staticmethod
    def _normalize_param(value):
        if isinstance(value, (int, float, bool, str)):
            return value
        if isinstance(value, tuple):
            value = list(value)
        if isinstance(value, list):
            return [AgentBase._normalize_param(item) for item in value]
        if isinstance(value, dict):
            return {str(key): AgentBase._normalize_param(item) for key, item in value.items()}
        return str(value)

    def call_function(self, function_name, *args, expect_return=True, named_parameters=None):
        payload = {
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

    def get_image(self, image_type="scene", quality=90, max_depth_m=200.0):
        self.last_response = self.client.request(
            {
                "get_image": {
                    "id": self.actor_id,
                    "image_type": self.normalize_image_type(image_type),
                    "quality": int(quality),
                    "max_depth_m": float(max_depth_m),
                }
            }
        )
        return self.last_response

    def get_image_raw(self, image_type="scene", quality=90, max_depth_m=200.0):
        response = self.get_image(image_type=image_type, quality=quality, max_depth_m=max_depth_m)
        if self.is_ok(response):
            return response
        return None

    def get_image_base64(self, image_type="scene", quality=90, max_depth_m=200.0):
        response = self.get_image_raw(image_type=image_type, quality=quality, max_depth_m=max_depth_m)
        if response:
            return response.get("data")
        return None

    def get_image_numpy(self, image_type="scene", quality=90, max_depth_m=200.0):
        data_b64 = self.get_image_base64(
            image_type=image_type,
            quality=quality,
            max_depth_m=max_depth_m,
        )
        if not data_b64:
            return None

        try:
            import cv2
            import numpy as np
        except ImportError:
            return None

        try:
            raw = base64.b64decode(data_b64)
        except Exception:
            return None

        array = np.frombuffer(raw, dtype=np.uint8)
        return cv2.imdecode(array, cv2.IMREAD_COLOR)