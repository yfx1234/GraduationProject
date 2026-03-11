"""Guidance module wrapper."""


class AgentGuidance:
    """Object wrapper for guidance commands."""

    def __init__(self, client):
        self.client = client

    def call_command(self, function_name, **payload):
        request_payload = {"function": function_name}
        request_payload.update(payload)
        return self.client.request({"call_guidance": request_payload})

    def reset(self):
        return self.call_command("reset")

    def state(self):
        return self.client.request({"get_guidance_state": {}})

    def set_intercept_method(
        self,
        method=None,
        speed=None,
        nav_gain=None,
        lead_time=None,
        capture_radius=None,
    ):
        payload = {}
        if method:
            payload["method"] = method
        if speed is not None:
            payload["speed"] = float(speed)
        if nav_gain is not None:
            payload["nav_gain"] = float(nav_gain)
        if lead_time is not None:
            payload["lead_time"] = float(lead_time)
        if capture_radius is not None:
            payload["capture_radius"] = float(capture_radius)
        return self.call_command("set_intercept_method", **payload)

    def list_intercept_agents(self):
        return self.call_command("list_intercept_agents")

    def auto_intercept(
        self,
        interceptor_id,
        target_id,
        method=None,
        speed=None,
        nav_gain=None,
        lead_time=None,
        capture_radius=None,
        stop_on_capture=True,
    ):
        payload = {
            "interceptor_id": interceptor_id,
            "target_id": target_id,
            "stop_on_capture": bool(stop_on_capture),
        }
        if method:
            payload["method"] = method
        if speed is not None:
            payload["speed"] = float(speed)
        if nav_gain is not None:
            payload["nav_gain"] = float(nav_gain)
        if lead_time is not None:
            payload["lead_time"] = float(lead_time)
        if capture_radius is not None:
            payload["capture_radius"] = float(capture_radius)
        return self.call_command("auto_intercept", **payload)

    def update_target(self, x, y, z, dt=0.1):
        return self.call_command("update_target", x=float(x), y=float(y), z=float(z), dt=float(dt))

    def compute_aim(self, turret_id="turret_0", muzzle_speed=400.0):
        return self.call_command("compute_aim", turret_id=turret_id, muzzle_speed=float(muzzle_speed))

    def auto_engage(
        self,
        turret_id="turret_0",
        target_id="drone_0",
        muzzle_speed=400.0,
        dt=0.05,
        latency=None,
        fire=False,
    ):
        payload = {
            "turret_id": turret_id,
            "target_id": target_id,
            "muzzle_speed": float(muzzle_speed),
            "dt": float(dt),
            "fire": bool(fire),
        }
        if latency is not None:
            payload["latency"] = float(latency)
        return self.call_command("auto_engage", **payload)

    def set_kalman_params(self, process_noise=1.0, measurement_noise=0.5):
        return self.call_command(
            "set_kalman_params",
            process_noise=float(process_noise),
            measurement_noise=float(measurement_noise),
        )

    def visual_intercept_start(self, **payload):
        return self.call_command("visual_intercept_start", **payload)

    def visual_intercept_update(self, **payload):
        return self.call_command("visual_intercept_update", **payload)

    def visual_intercept_stop(self):
        return self.call_command("visual_intercept_stop")

    def visual_intercept_state(self):
        return self.call_command("visual_intercept_state")


GuidanceAgent = AgentGuidance