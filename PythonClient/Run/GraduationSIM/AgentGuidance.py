from __future__ import annotations

from typing import Any, Dict, Optional

from .AgentBase import AgentBase


class AgentGuidance(AgentBase):
    def __init__(
        self,
        client,
        actor_id: str = "guidance_0",
        classname: str = "GuidanceActor",
        label: str = "Guidance",
    ) -> None:
        super().__init__(
            client=client,
            actor_id=actor_id,
            classname=classname,
            label=label,
            unit="m",
        )

    @staticmethod
    def _set_if_not_none(params: Dict[str, Any], key: str, value: Any) -> None:
        if value is not None:
            params[key] = value

    def _call_json(self, function_name: str, dict_args: Optional[Dict[str, Any]] = None):
        return self.call_function(
            function_name,
            expect_return=True,
            dict_args=dict_args,
            parse_return_json=True,
        )

    def get_state(self):
        return self._call_json("GetState")

    state = get_state

    def reset(self):
        return self._call_json("ResetGuidance")

    def set_method(self, method: str, nav_constant: float = 4.0, iterations: int = 3):
        return self._call_json(
            "SetMethod",
            dict_args={
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
        params: Dict[str, Any] = {}
        self._set_if_not_none(params, "Method", str(method) if method else None)
        self._set_if_not_none(params, "Speed", float(speed) if speed is not None else None)
        self._set_if_not_none(params, "NavGain", float(nav_gain) if nav_gain is not None else None)
        self._set_if_not_none(params, "LeadTime", float(lead_time) if lead_time is not None else None)
        self._set_if_not_none(params, "CaptureRadiusValue", float(capture_radius) if capture_radius is not None else None)
        return self._call_json("SetInterceptMethod", dict_args=params)

    def list_intercept_agents(self):
        return self._call_json("ListInterceptAgents")

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
        params: Dict[str, Any] = {
            "InterceptorId": str(interceptor_id),
            "TargetId": str(target_id),
            "bStopOnCapture": bool(stop_on_capture),
        }
        self._set_if_not_none(params, "Method", str(method) if method else None)
        self._set_if_not_none(params, "Speed", float(speed) if speed is not None else None)
        self._set_if_not_none(params, "NavGain", float(nav_gain) if nav_gain is not None else None)
        self._set_if_not_none(params, "LeadTime", float(lead_time) if lead_time is not None else None)
        self._set_if_not_none(params, "CaptureRadiusValue", float(capture_radius) if capture_radius is not None else None)
        return self._call_json("AutoIntercept", dict_args=params)

    def set_kalman_params(self, process_noise: float = 1.0, measurement_noise: float = 0.5):
        return self._call_json(
            "SetKalmanParams",
            dict_args={
                "ProcessNoise": float(process_noise),
                "MeasurementNoise": float(measurement_noise),
            },
        )

    def visual_intercept_start(self, **payload):
        params: Dict[str, Any] = {
            "InterceptorId": str(payload.get("interceptor_id", "")),
            "TargetId": str(payload.get("target_id", "")),
            "Method": str(payload.get("method", "vision_pid_kalman")),
        }

        field_map = {
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
        for source_key, target_key in field_map.items():
            self._set_if_not_none(params, target_key, payload.get(source_key))

        if "stop_on_capture" in payload:
            params["StopOnCaptureFlag"] = 1 if payload["stop_on_capture"] else 0
        if "use_kalman" in payload and payload["use_kalman"] is not None:
            params["UseKalmanFlag"] = 1 if payload["use_kalman"] else 0

        return self._call_json("VisualInterceptStart", dict_args=params)

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
        return self._call_json(
            "VisualInterceptUpdate",
            dict_args={
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
        return self._call_json(
            "VisualInterceptStop",
            dict_args={
                "InterceptorId": str(interceptor_id),
                "TargetId": str(target_id),
            },
        )

    def visual_intercept_state(self):
        return self._call_json("VisualInterceptState")
