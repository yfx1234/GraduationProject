from __future__ import annotations

from .AgentBase import AgentBase


class AgentDrone(AgentBase):
    def __init__(
        self,
        client,
        actor_id: str = "drone_0",
        classname: str = "/Game/Blueprints/BP_Drone.BP_Drone_C",
        label: str = "Drone",
        mission_role: str = "unknown",
    ) -> None:
        super().__init__(
            client=client,
            actor_id=actor_id,
            classname=classname,
            label=label,
            unit="m",
        )
        self.mission_role = mission_role

    def create(self, pose=None, classname=None, label=None, extra=None):
        extraPayload = {}
        if self.mission_role and self.mission_role != "unknown":
            extraPayload["role"] = self.mission_role
        if extra:
            extraPayload.update(extra)
        return super().create(
            pose=pose,
            classname=classname,
            label=label,
            extra=extraPayload,
        )

    @staticmethod
    def yaw_mode(yaw_mode):
        if isinstance(yaw_mode, dict):
            yawValue = float(yaw_mode.get("yaw_or_rate", 0.0))
            if yaw_mode.get("is_rate"):
                return "Rate", yawValue
            return "Angle", yawValue

        text = str(yaw_mode or "auto").strip().lower()
        if text == "hold":
            return "Hold", 0.0
        if text == "angle":
            return "Angle", 0.0
        if text == "rate":
            return "Rate", 0.0
        return "Auto", 0.0

    @staticmethod
    def move_pattern(move_pattern) -> str:
        text = str(move_pattern or "max_degree_of_freedom").strip().lower()
        if text in ("forward_only", "forward"):
            return "ForwardOnly"
        return "MaxDegreeOfFreedom"

    def set_heading_control(self, yaw_mode=None, yaw=None, move_pattern=None):
        if yaw_mode is None and yaw is None and move_pattern is None:
            return None

        yawModeName, yawFromMode = self.yaw_mode(yaw_mode)
        yawDegrees = float(yaw if yaw is not None else yawFromMode)
        movePatternName = self.move_pattern(move_pattern)
        return self.call_function(
            "SetHeadingControl",
            dict_args={
                "NewYawMode": yawModeName,
                "NewMovePattern": movePatternName,
                "YawDeg": yawDegrees,
            },
        )

    def enable_api_control(self, enable=True):
        return self.call_function(
            "EnableApiControl",
            dict_args={"bEnable": bool(enable)},
        )

    def takeoff(self, altitude=3.0):
        return self.call_function(
            "Takeoff",
            dict_args={"Altitude": float(altitude)},
        )

    def hover(self):
        return self.call_function("Hover")

    def move_to(self, x, y, z, speed=2.0, frame="ue", yaw_mode=None, yaw=None, move_pattern=None):
        self.set_heading_control(yaw_mode=yaw_mode, yaw=yaw, move_pattern=move_pattern)
        return self.call_function(
            "SetTargetPosition",
            dict_args={
                "NewTargetPosition": [float(x), float(y), float(z)],
                "Speed": float(speed),
                "Frame": self.normalize_frame(frame),
            },
        )

    def move_by_velocity(self, vx, vy, vz, frame="ue", yaw_mode=None, yaw=None, move_pattern=None):
        self.set_heading_control(yaw_mode=yaw_mode, yaw=yaw, move_pattern=move_pattern)
        return self.call_function(
            "MoveByVelocity",
            dict_args={
                "Vx": float(vx),
                "Vy": float(vy),
                "Vz": float(vz),
                "Frame": self.normalize_frame(frame),
            },
        )

    def set_camera_angles(self, pitch, yaw):
        return self.call_function(
            "SetCameraAngles",
            dict_args={"TargetPitch": float(pitch), "TargetYaw": float(yaw)},
        )

    def set_camera_fov(self, fov_deg):
        return self.call_function(
            "SetCameraFOV",
            dict_args={"NewFOV": float(fov_deg)},
        )
