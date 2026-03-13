from __future__ import annotations

from typing import Optional

from .AgentBase import AgentBase


class AgentGuidance(AgentBase):
    def __init__(self, client, actor_id: str = "guidance_0", classname: str = "GuidanceActor", label: str = "Guidance") -> None:
        super().__init__(client=client, actor_id=actor_id, classname=classname, label=label, spawn_unit="m")

    def get_state(self):
        return self.call_json("GetState")

    state = get_state

    def reset(self):
        return self.call_json("ResetGuidance")

    def set_method(self, method: str, nav_constant: float = 4.0, iterations: int = 3):
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
        method: Optional[str] = None,
        speed: Optional[float] = None,
        nav_gain: Optional[float] = None,
        lead_time: Optional[float] = None,
        capture_radius: Optional[float] = None,
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
        interceptor_id: str,
        target_id: str,
        method: Optional[str] = None,
        speed: Optional[float] = None,
        nav_gain: Optional[float] = None,
        lead_time: Optional[float] = None,
        capture_radius: Optional[float] = None,
        stop_on_capture: bool = True,
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

    def set_kalman_params(self, process_noise: float = 1.0, measurement_noise: float = 0.5):
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
            "ram_area_target": "RamAreaTarget",
            "min_ram_speed": "MinRamSpeed",
            "intercept_distance": "InterceptDistance",
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
        if "use_kalman" in payload and payload["use_kalman"] is not None:
            params["UseKalmanFlag"] = 1 if payload["use_kalman"] else 0
        return self.call_json("VisualInterceptStart", named_parameters=params)

    def visual_intercept_update(
        self,
        *,
        has_detection: bool,
        cx: float,
        cy: float,
        area: float,
        area_ratio: float,
        conf: float,
        dt: float,
        image_w: float,
        image_h: float,
        interceptor_id: str,
        target_id: str,
    ):
        return self.call_json(
            "VisualInterceptUpdate",
            named_parameters={
                "HasDetection": 1 if has_detection else 0,
                "Cx": float(cx),
                "Cy": float(cy),
                "Area": float(area),
                "AreaRatio": float(area_ratio),
                "Conf": float(conf),
                "Dt": float(dt),
                "ImageW": float(image_w),
                "ImageH": float(image_h),
                "InterceptorId": str(interceptor_id),
                "TargetId": str(target_id),
            },
        )

    def visual_intercept_stop(self, interceptor_id: str = "", target_id: str = ""):
        return self.call_json(
            "VisualInterceptStop",
            named_parameters={"InterceptorId": str(interceptor_id), "TargetId": str(target_id)},
        )

    def visual_intercept_state(self):
        return self.call_json("VisualInterceptState")
