from .AgentBase import AgentBase


class AgentGuidance(AgentBase):
    def __init__(self, client, actor_id="guidance_0", classname="GuidanceActor", label="Guidance"):
        super().__init__(client=client, actor_id=actor_id, classname=classname, label=label, spawn_unit="m")

    def state(self):
        return self.call_json("GetState")

    get_state = state

    def reset(self):
        return self.call_json("ResetGuidance")

    def set_method(self, method, nav_constant=4.0, iterations=3):
        return self.call_json(
            "SetMethod",
            named_parameters={
                "Method": str(method),
                "NavConstant": float(nav_constant),
                "Iterations": int(iterations),
            },
        )

    def set_intercept_method(
        self,
        method=None,
        speed=None,
        nav_gain=None,
        lead_time=None,
        capture_radius=None,
    ):
        params = {}
        if method:
            params["Method"] = str(method)
        if speed is not None:
            params["Speed"] = float(speed)
        if nav_gain is not None:
            params["NavGain"] = float(nav_gain)
        if lead_time is not None:
            params["LeadTime"] = float(lead_time)
        if capture_radius is not None:
            params["CaptureRadiusValue"] = float(capture_radius)
        return self.call_json("SetInterceptMethod", named_parameters=params)

    def list_intercept_agents(self):
        return self.call_json("ListInterceptAgents")

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
        params = {
            "InterceptorId": str(interceptor_id),
            "TargetId": str(target_id),
            "bStopOnCapture": bool(stop_on_capture),
        }
        if method:
            params["Method"] = str(method)
        if speed is not None:
            params["Speed"] = float(speed)
        if nav_gain is not None:
            params["NavGain"] = float(nav_gain)
        if lead_time is not None:
            params["LeadTime"] = float(lead_time)
        if capture_radius is not None:
            params["CaptureRadiusValue"] = float(capture_radius)
        return self.call_json("AutoIntercept", named_parameters=params)

    def update_target(self, x, y, z, dt=0.1):
        return self.call_json(
            "UpdateTarget",
            named_parameters={"X": float(x), "Y": float(y), "Z": float(z), "Dt": float(dt)},
        )

    def compute_aim(self, turret_id="turret_0", muzzle_speed=400.0):
        return self.call_json(
            "ComputeAim",
            named_parameters={"TurretId": str(turret_id), "MuzzleSpeed": float(muzzle_speed)},
        )

    def auto_engage(
        self,
        turret_id="turret_0",
        target_id="drone_0",
        muzzle_speed=400.0,
        dt=0.05,
        latency=None,
        fire=False,
    ):
        params = {
            "TurretId": str(turret_id),
            "TargetId": str(target_id),
            "MuzzleSpeed": float(muzzle_speed),
            "Dt": float(dt),
            "bFire": bool(fire),
        }
        if latency is not None:
            params["Latency"] = float(latency)
        return self.call_json("AutoEngage", named_parameters=params)

    def set_kalman_params(self, process_noise=1.0, measurement_noise=0.5):
        return self.call_json(
            "SetKalmanParams",
            named_parameters={
                "ProcessNoise": float(process_noise),
                "MeasurementNoise": float(measurement_noise),
            },
        )

    def visual_intercept_start(self, **payload):
        params = {
            "InterceptorId": str(payload.get("interceptor_id", "")),
            "TargetId": str(payload.get("target_id", "")),
            "Method": str(payload.get("method", "vision_pid_kalman")),
        }
        mapping = {
            "desired_area": "DesiredArea",
            "capture_area": "CaptureArea",
            "center_tol_x": "CenterTolX",
            "center_tol_y": "CenterTolY",
            "capture_hold_frames": "CaptureHoldFrames",
            "lost_to_search_frames": "LostToSearchFrames",
            "max_forward_speed": "MaxForwardSpeed",
            "max_reverse_speed": "MaxReverseSpeed",
            "max_vertical_speed": "MaxVerticalSpeed",
            "max_yaw_rate_deg": "MaxYawRateDeg",
            "search_cam_yaw_limit_deg": "SearchCamYawLimitDeg",
            "search_cam_rate_deg": "SearchCamRateDeg",
            "search_body_yaw_rate_deg": "SearchBodyYawRateDeg",
            "search_cam_pitch_deg": "SearchCamPitchDeg",
            "search_vz_amp": "SearchVzAmp",
        }
        for key, target_key in mapping.items():
            if key in payload and payload[key] is not None:
                params[target_key] = payload[key]
        if "stop_on_capture" in payload:
            params["StopOnCaptureFlag"] = 1 if payload["stop_on_capture"] else 0
        if "use_kalman" in payload:
            params["UseKalmanFlag"] = 1 if payload["use_kalman"] else 0
        return self.call_json("VisualInterceptStart", named_parameters=params)

    def visual_intercept_update(self, **payload):
        params = {
            "HasDetection": 1 if payload.get("has_detection") else 0,
            "Cx": float(payload.get("cx", 0.0)),
            "Cy": float(payload.get("cy", 0.0)),
            "Area": float(payload.get("area", 0.0)),
            "AreaRatio": float(payload.get("area_ratio", -1.0)),
            "Conf": float(payload.get("conf", 1.0)),
            "Dt": float(payload.get("dt", 0.08)),
            "ImageW": float(payload.get("image_w", 640.0)),
            "ImageH": float(payload.get("image_h", 480.0)),
            "InterceptorId": str(payload.get("interceptor_id", "")),
            "TargetId": str(payload.get("target_id", "")),
        }
        return self.call_json("VisualInterceptUpdate", named_parameters=params)

    def visual_intercept_stop(self, interceptor_id="", target_id=""):
        return self.call_json(
            "VisualInterceptStop",
            named_parameters={"InterceptorId": str(interceptor_id), "TargetId": str(target_id)},
        )

    def visual_intercept_state(self):
        return self.call_json("VisualInterceptState")


GuidanceAgent = AgentGuidance
