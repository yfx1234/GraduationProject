from __future__ import annotations

from .AgentBase import AgentBase


class AgentGuidance(AgentBase):
    def __init__(
        self,
        client,
        actor_id="guidance_0",
        classname="GuidanceActor",
        label="Guidance",
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

    def reset(self):
        return self.call_function(
            "ResetGuidance",
            expect_return=True,
            parse_return_json=True,
        )

    def visual_intercept_start(self, **payload):
        payloadArgs = {}
        for key, value in payload.items():
            if value is not None:
                payloadArgs[key] = value
        return self.call_function(
            "VisualInterceptStart",
            expect_return=True,
            dict_args=payloadArgs,
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
        processing_latency=0.0,
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
                "ProcessingLatency": float(processing_latency),
                "ImageW": float(image_w),
                "ImageH": float(image_h),
                "InterceptorId": str(interceptor_id),
                "TargetId": str(target_id),
            },
            parse_return_json=True,
        )

    def visual_intercept_stop(self, interceptor_id="", target_id=""):
        return self.call_function(
            "VisualInterceptStop",
            expect_return=True,
            dict_args={
                "InterceptorId": str(interceptor_id),
                "TargetId": str(target_id),
            },
            parse_return_json=True,
        )
