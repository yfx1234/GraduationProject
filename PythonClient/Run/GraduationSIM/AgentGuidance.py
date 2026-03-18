from __future__ import annotations
from typing import Any, Dict, Optional
from .AgentBase import AgentBase

class AgentGuidance(AgentBase):
    def __init__(
        self,
        client,
        actor_id = "guidance_0",
        classname = "GuidanceActor",
        label = "Guidance",
    ) -> None:
        super().__init__(
            client=client,
            actor_id=actor_id,
            classname=classname,
            label=label,
            unit="m",
        )

    def get_state(self):
        return self.call_function(
            "GetState",
            expect_return=True,
            parse_return_json=True,
        )

    state = get_state

    def reset(self):
        return self.call_function(
            "ResetGuidance",
            expect_return=True,
            parse_return_json=True,
        )

    def set_method(self, method, nav_constant=4.0, iterations=3):
        return self.call_function(
            "SetMethod",
            expect_return=True,
        dict_args={
            "Method": str(method),
            "NavConstant": float(nav_constant),
            "Iterations": int(iterations),
        },
        parse_return_json=True,
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
        return self.call_function(
            "SetInterceptMethod",
            expect_return=True,
            dict_args=params,
            parse_return_json=True,
        )

    def auto_intercept(
        self,
        interceptor_id,
        target_id,
        method=None,
        speed=None,
        nav_gain=None,
        lead_time=None,
        capture_radius=None,
    ):
        params = {
            "InterceptorId": str(interceptor_id),
            "TargetId": str(target_id),
            "bStopOnCapture": True,
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
        return self.call_function(
            "AutoIntercept",
            expect_return=True,
            dict_args=params,
            parse_return_json=True,
        )

    def set_kalman_params(self, process_noise = 1.0, measurement_noise = 0.5):
        return self.call_function(
            "SetKalmanParams",
            expect_return=True,
            dict_args={
                "ProcessNoise": float(process_noise),
                "MeasurementNoise": float(measurement_noise),
            },
            parse_return_json=True,
        )

    def visual_intercept_start(self, **payload):
        params = {}
        for key, value in payload.items():
            if value is not None:
                params[key] = value
                
        return self.call_function(
            "VisualInterceptStart",
            expect_return=True,
            dict_args=params,
            parse_return_json=True,
        )

    def visual_intercept_update(
        self,
        *,
        has_detection,
        cx,
        cy,
        area,
        area_ratio,
        conf,
        dt,
        image_w,
        image_h,
        interceptor_id,
        target_id,
    ):
        return self.call_function(
            "VisualInterceptUpdate",
            expect_return=True,
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
            parse_return_json=True,
        )

    def visual_intercept_stop(self, interceptor_id = "", target_id = ""):
        return self.call_function(
            "VisualInterceptStop",
            expect_return=True,
            dict_args={
                "InterceptorId": str(interceptor_id),
                "TargetId": str(target_id),
            },
            parse_return_json=True,
        )

    def visual_intercept_state(self):
        return self.call_function(
            "VisualInterceptState",
            expect_return=True,
            parse_return_json=True,
        )
