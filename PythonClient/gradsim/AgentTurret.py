from .AgentBase import AgentBase


class AgentTurret(AgentBase):
    def __init__(self, client, actor_id="turret_0", classname="TurretPawn", label="Turret"):
        super().__init__(client=client, actor_id=actor_id, classname=classname, label=label, spawn_unit="m")

    def enable_api_control(self, enable=True):
        return self.call_function("EnableApiControl", named_parameters={"bEnable": bool(enable)})

    def set_angles(self, pitch, yaw):
        return self.call_function(
            "SetTargetAngles",
            named_parameters={"TargetPitch": float(pitch), "TargetYaw": float(yaw)},
        )

    def fire(self, speed=400.0):
        return self.call_function("FireX", named_parameters={"InitialSpeed": float(speed)})

    def start_tracking(self, target_id):
        return self.call_function("StartTracking", named_parameters={"TargetActorID": str(target_id)})

    def stop_tracking(self):
        return self.call_function("StopTracking")

    def show_prediction_line(self):
        return self.call_function("ShowPredictionLine")

    show_prediction = show_prediction_line

    def hide_prediction_line(self):
        return self.call_function("HidePredictionLine")

    hide_prediction = hide_prediction_line

    def reset(self):
        return self.call_function("ResetTurret")


TurretAgent = AgentTurret
