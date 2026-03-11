from typing import Any, List

from .client import GradSimClient


class AgentBase:
    """Generic actor wrapper based on add/remove/call_actor messages."""

    def __init__(self, client: GradSimClient, actor_id: str):
        self.client = client
        self.actor_id = actor_id

    def create(self, classname: str, x=0.0, y=0.0, z=0.0,
               roll=0.0, pitch=0.0, yaw=0.0, label="Agent") -> dict:
        return self.client.add_actor(
            classname=classname,
            expected_id=self.actor_id,
            label=label,
            x=x,
            y=y,
            z=z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )

    def remove(self) -> dict:
        return self.client.remove_actor(self.actor_id)

    @staticmethod
    def _normalize_param(value: Any) -> Any:
        if isinstance(value, (int, float, bool, str)):
            return value
        if isinstance(value, tuple):
            value = list(value)
        if isinstance(value, list):
            if len(value) == 3 and all(isinstance(v, (int, float)) for v in value):
                return [float(value[0]), float(value[1]), float(value[2])]
            return value
        if isinstance(value, dict):
            return value
        return str(value)

    def call_function(self, function_name: str, *args, expect_return: bool = True) -> dict:
        params: List[Any] = [self._normalize_param(arg) for arg in args] if args else None
        return self.client.call_actor(
            actor_id=self.actor_id,
            function_name=function_name,
            parameters=params,
            expect_return=expect_return,
        )