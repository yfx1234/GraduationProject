"""Drone actor wrapper."""

from .AgentBase import AgentBase

DEFAULT_DRONE_CLASS = "/Game/Blueprints/BP_Drone.BP_Drone_C"


class AgentDrone(AgentBase):
    """Object wrapper for common drone commands."""

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

    def call_command(self, function_name, **payload):
        request_payload = {"function": function_name, "id": self.actor_id}
        request_payload.update(payload)
        self.last_response = self.client.request({"call_drone": request_payload})
        return self.last_response

    def takeoff(self, altitude=3.0):
        return self.call_command("takeoff", altitude=float(altitude))

    def land(self):
        return self.call_command("land")

    def hover(self):
        return self.call_command("hover")

    def move_to(self, x, y, z, speed=2.0, frame="ue", yaw_mode=None, yaw=None, drivetrain=None):
        payload = {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "speed": float(speed),
            "frame": self.normalize_frame(frame),
        }
        if yaw_mode is not None:
            payload["yaw_mode"] = yaw_mode
        if yaw is not None:
            payload["yaw"] = float(yaw)
        if drivetrain is not None:
            payload["drivetrain"] = str(drivetrain)
        return self.call_command("move_to_position", **payload)

    move_to_position = move_to

    def move_by_velocity(self, vx, vy, vz, frame="ue", yaw_mode=None, yaw=None, drivetrain=None):
        payload = {
            "vx": float(vx),
            "vy": float(vy),
            "vz": float(vz),
            "frame": self.normalize_frame(frame),
        }
        if yaw_mode is not None:
            payload["yaw_mode"] = yaw_mode
        if yaw is not None:
            payload["yaw"] = float(yaw)
        if drivetrain is not None:
            payload["drivetrain"] = str(drivetrain)
        return self.call_command("move_by_velocity", **payload)

    def set_camera_angles(self, pitch, yaw):
        return self.call_command("set_camera_angles", pitch=float(pitch), yaw=float(yaw))

    def set_pid_position(self, kp, kd=0.0):
        return self.call_command("set_pid_position", kp=float(kp), kd=float(kd))

    def set_pid_velocity(self, kp, ki=0.0, kd=0.0):
        return self.call_command("set_pid_velocity", kp=float(kp), ki=float(ki), kd=float(kd))

    def set_pid_attitude(self, kp, kd=0.0):
        return self.call_command("set_pid_attitude", kp=float(kp), kd=float(kd))

    def set_pid_angle_rate(self, kp):
        return self.call_command("set_pid_angle_rate", kp=float(kp))

    def set_target_attitude(self, roll=0.0, pitch=0.0, yaw=0.0, thrust=9.81, frame="ned"):
        return self.call_command(
            "set_target_attitude",
            roll=float(roll),
            pitch=float(pitch),
            yaw=float(yaw),
            thrust=float(thrust),
            frame=self.normalize_frame(frame),
        )

    def set_motor_speeds(self, m0, m1, m2, m3):
        return self.call_command("set_motor_speeds", m0=float(m0), m1=float(m1), m2=float(m2), m3=float(m3))

    def reset(self):
        return self.call_command("reset")

    def reset_to(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        return self.call_function(
            "ResetDrone",
            [float(x), float(y), float(z)],
            {"roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)},
        )

    def get_state(self, frame="ue"):
        self.last_response = self.client.request(
            {"get_drone_state": {"id": self.actor_id, "frame": self.normalize_frame(frame)}}
        )
        return self.last_response

    state = get_state


DroneAgent = AgentDrone