from .agent_base import AgentBase


class TurretAgent(AgentBase):
    """Convenience wrapper around turret-specific commands."""

    def set_angles(self, pitch: float, yaw: float) -> dict:
        return self.client.turret_set_angles(pitch=float(pitch), yaw=float(yaw), turret_id=self.actor_id)

    def fire(self, speed: float = 400.0) -> dict:
        return self.client.turret_fire(speed=float(speed), turret_id=self.actor_id)

    def start_tracking(self, target_id: str) -> dict:
        return self.client.turret_start_tracking(target_id=str(target_id), turret_id=self.actor_id)

    def stop_tracking(self) -> dict:
        return self.client.turret_stop_tracking(turret_id=self.actor_id)

    def show_prediction_line(self) -> dict:
        return self.call_function("ShowPredictionLine")

    def hide_prediction_line(self) -> dict:
        return self.call_function("HidePredictionLine")

    def reset(self) -> dict:
        return self.client.turret_reset(turret_id=self.actor_id)

    def get_state(self) -> dict:
        return self.client.turret_state(turret_id=self.actor_id)
