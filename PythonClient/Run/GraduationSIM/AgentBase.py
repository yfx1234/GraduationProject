from __future__ import annotations

import json
from typing import Any, Dict, Optional

from .DataTypes import ImagePacket, Pose

JsonDict = Dict[str, Any]


class AgentBase:
    def __init__(
        self,
        client,
        actor_id,
        classname=None,
        label="Agent",
        unit="m",
    ) -> None:
        self.client = client
        self.actor_id = actor_id
        self.classname = classname
        self.label = label
        self.unit = unit

    @staticmethod
    def is_ok(response) -> bool:
        if not isinstance(response, dict):
            return False
        status = response.get("status")
        return status == "ok" or status is True

    @staticmethod
    def unwrap_response(response, field_name) -> JsonDict:
        if isinstance(response, dict):
            value = response.get(field_name)
            if isinstance(value, dict):
                return value
        return response

    @staticmethod
    def parse_json_text(value) -> Any:
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

    @staticmethod
    def normalize_param(value) -> Any:
        if isinstance(value, (int, float, bool, str)):
            return value
        if isinstance(value, Pose):
            return value.to_dict()
        if isinstance(value, tuple):
            value = list(value)
        if isinstance(value, list):
            return [AgentBase.normalize_param(item) for item in value]
        if isinstance(value, dict):
            return {str(key): AgentBase.normalize_param(item) for key, item in value.items()}
        return str(value)

    def create(
        self,
        pose=None,
        classname=None,
        label=None,
        extra=None,
    ) -> JsonDict:
        actor_classname = classname or self.classname
        if not actor_classname:
            raise ValueError("Type must be set before creating actor")

        payload: JsonDict = {
            "classname": actor_classname,
            "expected_id": self.actor_id,
            "label": label or self.label or "Agent",
            "pose": (pose or Pose()).to_initial_dict(),
        }

        extra_payload = {"unit": self.unit}
        if extra:
            extra_payload.update(extra)
        if extra_payload:
            payload.update(extra_payload)

        response = self.client.send_message({"add_actor": payload})
        response = self.unwrap_response(response, "add_actor_return")
        self.classname = actor_classname
        self.label = str(payload["label"])
        return response

    def remove(self) -> JsonDict:
        response = self.client.send_message({"remove_actor": {"actor_id": self.actor_id}})
        return self.unwrap_response(response, "remove_actor_return")

    def call_function(
        self,
        function_name,
        *args,
        expect_return=False,
        dict_args=None,
        parse_return_json=False,
    ) -> JsonDict:
        payload: JsonDict = {
            "actor_id": self.actor_id,
            "function": function_name,
            "return": bool(expect_return),
        }

        if dict_args is not None:
            payload["parameters"] = self.normalize_param(dict_args)
        elif args:
            payload["parameters"] = [self.normalize_param(arg) for arg in args]

        response = self.client.send_message({"call_actor": payload})
        response = self.unwrap_response(response, "call_actor_return")
        if not parse_return_json:
            return response

        if not self.is_ok(response):
            return response

        parsed = self.parse_json_text(response.get("return", ""))
        if isinstance(parsed, dict):
            return parsed
        if parsed == {}:
            return {"status": "ok"}
        return {"status": "ok", "return": parsed}

    def get_state(self, frame: str = "ue") -> JsonDict:
        return self.call_function(
            "GetState",
            expect_return=True,
            dict_args={"Frame": self.normalize_frame(frame)},
            parse_return_json=True,
        )

    state = get_state

    def get_image_response(
        self,
        image_type="scene",
        quality=90,
    ) -> JsonDict:
        return self.call_function(
            "GetImage",
            expect_return=True,
            dict_args={
                "ImageType": image_type,
                "Quality": int(quality),
            },
            parse_return_json=True,
        )

    def get_image_packet(
        self,
        image_type="scene",
        quality=90,
    ) -> Optional[ImagePacket]:
        response = self.get_image_response(
            image_type=image_type,
            quality=quality,
        )
        if not self.is_ok(response):
            return None
        return ImagePacket.from_response(
            response,
            image_type=image_type,
        )

    def get_image_base64(
        self,
        image_type="scene",
        quality=90,
    ) -> Optional[str]:
        packet = self.get_image_packet(
            image_type=image_type,
            quality=quality,
        )
        return packet.data if packet else None
