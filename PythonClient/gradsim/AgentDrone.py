from .AgentBase import AgentBase

DEFAULT_DRONE_CLASS = "/Game/Blueprints/BP_Drone.BP_Drone_C"


class AgentDrone(AgentBase):
    def __init__(
        self,
        client,
        actor_id="drone_0",
        classname=DEFAULT_DRONE_CLASS,
        label="Drone",
        mission_role="unknown",
    ):
        super().__init__(client=client, actor_id=actor_id, classname=classname, label=label, spawn_unit="m")
        self.mission_role = mission_role

    def _create_extra(self):
        extra = super()._create_extra()
        if self.mission_role and self.mission_role != "unknown":
            extra["role"] = self.mission_role
        return extra

    @staticmethod
    def _yaw_mode_name(yaw_mode):
        if isinstance(yaw_mode, dict):
            if yaw_mode.get("is_rate"):
                return "Rate", float(yaw_mode.get("yaw_or_rate", 0.0))
            return "Angle", float(yaw_mode.get("yaw_or_rate", 0.0))

        text = str(yaw_mode or "auto").strip().lower()
        if text == "hold":
            return "Hold", 0.0
        if text == "angle":
            return "Angle", 0.0
        if text == "rate":
            return "Rate", 0.0
        return "Auto", 0.0

    @staticmethod
    def _drivetrain_name(drivetrain):
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
            named_parameters={
                "NewYawMode": yaw_mode_name,
                "NewDrivetrain": drive_name,
                "YawDeg": yaw_deg,
            },
        )

    def enable_api_control(self, enable=True):
        return self.call_function("EnableApiControl", named_parameters={"bEnable": bool(enable)})

    def takeoff(self, altitude=3.0):
        return self.call_function("Takeoff", named_parameters={"Altitude": float(altitude)})

    def land(self):
        return self.call_function("Land")

    def hover(self):
        return self.call_function("Hover")

    def move_to(self, x, y, z, speed=2.0, frame="ue", yaw_mode=None, yaw=None, drivetrain=None):
        self._apply_heading(yaw_mode=yaw_mode, yaw=yaw, drivetrain=drivetrain)
        return self.call_function(
            "SetTargetPosition",
            named_parameters={
                "NewTargetPosition": [float(x), float(y), float(z)],
                "Speed": float(speed),
                "Frame": self.normalize_frame(frame),
            },
        )

    move_to_position = move_to

    def move_by_velocity(self, vx, vy, vz, frame="ue", yaw_mode=None, yaw=None, drivetrain=None):
        self._apply_heading(yaw_mode=yaw_mode, yaw=yaw, drivetrain=drivetrain)
        return self.call_function(
            "MoveByVelocity",
            named_parameters={
                "Vx": float(vx),
                "Vy": float(vy),
                "Vz": float(vz),
                "Frame": self.normalize_frame(frame),
            },
        )

    def set_camera_angles(self, pitch, yaw):
        return self.call_function(
            "SetCameraAngles",
            named_parameters={"TargetPitch": float(pitch), "TargetYaw": float(yaw)},
        )

    def set_pid_position(self, kp, kd=0.0):
        return self.call_function(
            "SetPositionControllerGains",
            named_parameters={"Kp": float(kp), "Kd": float(kd)},
        )

    def set_pid_velocity(self, kp, ki=0.0, kd=0.0):
        return self.call_function(
            "SetVelocityControllerGains",
            named_parameters={"Kp": float(kp), "Ki": float(ki), "Kd": float(kd)},
        )

    def set_pid_attitude(self, kp, kd=0.0):
        return self.call_function(
            "SetAttitudeControllerGains",
            named_parameters={"Kp": float(kp), "Kd": float(kd)},
        )

    def set_pid_angle_rate(self, kp):
        return self.call_function("SetAngleRateControllerGains", named_parameters={"Kp": float(kp)})

    def set_target_attitude(self, roll=0.0, pitch=0.0, yaw=0.0, thrust=9.81, frame="ned"):
        return self.call_function(
            "SetTargetAttitude",
            named_parameters={
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
            named_parameters={
                "M0": float(m0),
                "M1": float(m1),
                "M2": float(m2),
                "M3": float(m3),
            },
        )

    def reset(self):
        return self.call_function("ResetActorState")

    def reset_to(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, frame="ue"):
        return self.call_function(
            "ResetDrone",
            named_parameters={
                "NewLocation": [float(x), float(y), float(z)],
                "NewRotation": {"roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)},
                "Frame": self.normalize_frame(frame),
            },
        )


DroneAgent = AgentDrone
