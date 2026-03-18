from __future__ import annotations
from typing import Any, Dict, Optional, Tuple
from .AgentBase import AgentBase

class AgentDrone(AgentBase):
    def __init__(
        self,
        client,
        actor_id = "drone_0",
        classname = "/Game/Blueprints/BP_Drone.BP_Drone_C",
        label = "Drone",
        mission_role = "unknown",
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
        extra_payload = {}
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
    def yaw_mode(yaw_mode):
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
    def move_pattern(move_pattern) -> str:
        text = str(move_pattern or "max_degree_of_freedom").strip().lower()
        if text in ("forward_only", "forward"):
            return "ForwardOnly"
        return "MaxDegreeOfFreedom"

    def set_heading_control(self, yaw_mode=None, yaw=None, move_pattern=None):
        if yaw_mode is None and yaw is None and move_pattern is None:
            return None

        yaw_mode, yaw_from_mode = self.yaw_mode(yaw_mode)
        yaw_deg = float(yaw if yaw is not None else yaw_from_mode)
        move_pattern_name = self.move_pattern(move_pattern)
        return self.call_function(
            "SetHeadingControl",
            dict_args={
                "NewYawMode": yaw_mode,
                "NewMovePattern": move_pattern_name,
                "YawDeg": yaw_deg,
            },
        )

    def enable_api_control(self, enable = True):
        return self.call_function(
            "EnableApiControl", 
            dict_args={"bEnable": bool(enable)}
        )

    def takeoff(self, altitude = 3.0):
        return self.call_function(
            "Takeoff", 
            dict_args={"Altitude": float(altitude)}
        )

    def land(self):
        return self.call_function("Land")

    def hover(self):
        return self.call_function("Hover")

    def move_to(self,x,y,z,speed = 2.0,frame = "ue",yaw_mode=None,yaw=None,move_pattern=None,):
        self.set_heading_control(yaw_mode=yaw_mode, yaw=yaw, move_pattern=move_pattern)
        return self.call_function(
            "SetTargetPosition",
            dict_args={
                "NewTargetPosition": [float(x), float(y), float(z)],
                "Speed": float(speed),
                "Frame": self.normalize_frame(frame),
            },
        )

    def move_by_velocity(self,vx,vy,vz,frame = "ue",yaw_mode=None,yaw=None,move_pattern=None,):
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

    def set_pid_position(self, kp, kd = 0.0):
        return self.call_function(
            "SetPositionControllerGains",
            dict_args={"Kp": float(kp), "Kd": float(kd)},
        )

    def set_pid_velocity(self, kp, ki = 0.0, kd = 0.0):
        return self.call_function(
            "SetVelocityControllerGains",
            dict_args={"Kp": float(kp), "Ki": float(ki), "Kd": float(kd)},
        )

    def set_pid_attitude(self, kp, kd = 0.0):
        return self.call_function(
            "SetAttitudeControllerGains",
            dict_args={"Kp": float(kp), "Kd": float(kd)},
        )

    def set_pid_angle_rate(self, kp):
        return self.call_function(
            "SetAngleRateControllerGains", 
            dict_args={"Kp": float(kp)}
        )

    def set_target_attitude(self,roll = 0.0,pitch = 0.0,yaw = 0.0,thrust = 9.81,frame = "ned",):
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

    def set_motor_speeds(self, m0, m1, m2, m3):
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
