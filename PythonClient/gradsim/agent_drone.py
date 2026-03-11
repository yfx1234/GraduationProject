from .agent_base import AgentBase


class DroneAgent(AgentBase):
    """Convenience wrapper around drone-specific commands."""

    def takeoff(self, altitude: float = 3.0) -> dict:
        return self.client.drone_takeoff(altitude=float(altitude), drone_id=self.actor_id)

    def land(self) -> dict:
        return self.client.drone_land(drone_id=self.actor_id)

    def hover(self) -> dict:
        return self.client.drone_hover(drone_id=self.actor_id)

    def move_to(self, x: float, y: float, z: float, speed: float = 2.0) -> dict:
        return self.client.drone_move_to(x=float(x), y=float(y), z=float(z), speed=float(speed), drone_id=self.actor_id)

    def move_by_velocity(self, vx: float, vy: float, vz: float) -> dict:
        return self.client.drone_move_by_velocity(vx=float(vx), vy=float(vy), vz=float(vz), drone_id=self.actor_id)

    def set_camera_angles(self, pitch: float, yaw: float) -> dict:
        return self.client.drone_set_camera_angles(pitch=float(pitch), yaw=float(yaw), drone_id=self.actor_id)

    def reset(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0) -> dict:
        return self.call_function(
            "ResetDrone",
            [float(x), float(y), float(z)],
            {"roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)},
        )

    def get_state(self, frame: str = "ue") -> dict:
        return self.client.drone_state(drone_id=self.actor_id, frame=frame)
