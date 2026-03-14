from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

from .AgentBase import AgentBase

DEFAULT_DRONE_CLASS = "/Game/Blueprints/BP_Drone.BP_Drone_C"


class AgentDrone(AgentBase):
    def __init__(
        self,
        client,
        actor_id: str = "drone_0",
        classname: Optional[str] = DEFAULT_DRONE_CLASS,
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
        extra_payload: Dict[str, Any] = {}
        if self.mission_role and self.mission_role != "unknown":
            extra_payload["role"] = self.mission_role
        if extra:
            extra_payload.update(extra)
        return super().create(
            pose=pose,
            classname=classname,
            label=label,
            extra=extra_payload,
        )

    @staticmethod
    def _yaw_mode_name(yaw_mode: Any) -> Tuple[str, float]:
        if isinstance(yaw_mode, dict):
            yaw_value = float(yaw_mode.get("yaw_or_rate", 0.0))
            if yaw_mode.get("is_rate"):
                return "Rate", yaw_value
            return "Angle", yaw_value

        text = str(yaw_mode or "auto").strip().lower()
        if text == "hold":
            return "Hold", 0.0
        if text == "angle":
            return "Angle", 0.0
        if text == "rate":
            return "Rate", 0.0
        return "Auto", 0.0

    @staticmethod
    def _drivetrain_name(drivetrain: Optional[str]) -> str:
        text = str(drivetrain or "max_degree_of_freedom").strip().lower()
        if text in ("forward_only", "forward"):
            return "ForwardOnly"
        return "MaxDegreeOfFreedom"

    def _apply_heading(self, yaw_mode=None, yaw=None, drivetrain=None):
        if yaw_mode is None and yaw is None and drivetrain is None:
            return None

        yaw_mode_name, yaw_from_mode = self._yaw_mode_name(yaw_mode)
        yaw_deg = float(yaw if yaw is not None else yaw_from_mode)
        drive_name = self._drivetrain_name(drivetrain)
        return self.call_function(
            "SetHeadingControl",
            dict_args={
                "NewYawMode": yaw_mode_name,
                "NewDrivetrain": drive_name,
                "YawDeg": yaw_deg,
            },
        )

    def enable_api_control(self, enable: bool = True):
        return self.call_function("EnableApiControl", dict_args={"bEnable": bool(enable)})

    def takeoff(self, altitude: float = 3.0):
        return self.call_function("Takeoff", dict_args={"Altitude": float(altitude)})

    def land(self):
        return self.call_function("Land")

    def hover(self):
        return self.call_function("Hover")

    def move_to(
        self,
        x,
        y,
        z,
        speed: float = 2.0,
        frame: str = "ue",
        yaw_mode=None,
        yaw=None,
        drivetrain=None,
    ):
        self._apply_heading(yaw_mode=yaw_mode, yaw=yaw, drivetrain=drivetrain)
        return self.call_function(
            "SetTargetPosition",
            dict_args={
                "NewTargetPosition": [float(x), float(y), float(z)],
                "Speed": float(speed),
                "Frame": self.normalize_frame(frame),
            },
        )

    move_to_position = move_to

    def move_by_velocity(
        self,
        vx,
        vy,
        vz,
        frame: str = "ue",
        yaw_mode=None,
        yaw=None,
        drivetrain=None,
    ):
        self._apply_heading(yaw_mode=yaw_mode, yaw=yaw, drivetrain=drivetrain)
        return self.call_function(
            "MoveByVelocity",
            dict_args={
                "Vx": float(vx),
                "Vy": float(vy),
                "Vz": float(vz),
                "Frame": self.normalize_frame(frame),
            },
        )

    def set_camera_angles(self, pitch: float, yaw: float):
        return self.call_function(
            "SetCameraAngles",
            dict_args={"TargetPitch": float(pitch), "TargetYaw": float(yaw)},
        )

    def set_pid_position(self, kp: float, kd: float = 0.0):
        return self.call_function(
            "SetPositionControllerGains",
            dict_args={"Kp": float(kp), "Kd": float(kd)},
        )

    def set_pid_velocity(self, kp: float, ki: float = 0.0, kd: float = 0.0):
        return self.call_function(
            "SetVelocityControllerGains",
            dict_args={"Kp": float(kp), "Ki": float(ki), "Kd": float(kd)},
        )

    def set_pid_attitude(self, kp: float, kd: float = 0.0):
        return self.call_function(
            "SetAttitudeControllerGains",
            dict_args={"Kp": float(kp), "Kd": float(kd)},
        )

    def set_pid_angle_rate(self, kp: float):
        return self.call_function("SetAngleRateControllerGains", dict_args={"Kp": float(kp)})

    def set_target_attitude(
        self,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        thrust: float = 9.81,
        frame: str = "ned",
    ):
        return self.call_function(
            "SetTargetAttitude",
            dict_args={
                "RollDeg": float(roll),
                "PitchDeg": float(pitch),
                "YawDeg": float(yaw),
                "Thrust": float(thrust),
                "Frame": self.normalize_frame(frame),
            },
        )

    def set_motor_speeds(self, m0: float, m1: float, m2: float, m3: float):
        return self.call_function(
            "SetMotorSpeeds",
            dict_args={
                "M0": float(m0),
                "M1": float(m1),
                "M2": float(m2),
                "M3": float(m3),
            },
        )

    def reset(self):
        return self.call_function("ResetActorState")

    def reset_to(
        self,
        x=0.0,
        y=0.0,
        z=0.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        frame: str = "ue",
    ):
        return self.call_function(
            "ResetDrone",
            dict_args={
                "NewLocation": [float(x), float(y), float(z)],
                "NewRotation": {"roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)},
                "Frame": self.normalize_frame(frame),
            },
        )
