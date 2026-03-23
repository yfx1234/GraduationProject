from __future__ import annotations


DEFAULT_VISUAL_INTERCEPT_ARGS = {
    "Method": "vision_pid_kalman",
    "StopOnCaptureFlag": 0,
    "DesiredArea": 0.06,
    "CaptureArea": 0.09,
    "CenterTolX": 0.028,
    "CenterTolY": 0.040,
    "MaxVerticalSpeed": 2.6,
    "MaxYawRateDeg": 120.0,
    "RamAreaTarget": 0.20,
    "MinRamSpeed": 9.0,
    "InterceptDistance": 1.00,
    "TrackLeadTime": 0.14,
    "RamLeadTime": 0.22,
    "LostToSearchFrames": 14,
    "SearchCamPitchDeg": 0.0,
    "SearchCamYawLimitDeg": 0.0,
    "SearchCamRateDeg": 0.0,
    "SearchBodyYawRateDeg": 34.0,
    "UseKalmanFlag": 1,
}


def _to_float(value, default):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _clamp(value, low, high):
    return max(low, min(high, value))


def build_visual_intercept_args(run_config, interceptor_speed):
    payload = dict(DEFAULT_VISUAL_INTERCEPT_ARGS)
    configured_args = run_config.get("arg") if isinstance(run_config, dict) else None
    if isinstance(configured_args, dict):
        payload.update({key: value for key, value in configured_args.items() if value is not None})

    max_forward_speed = _clamp(
        _to_float(interceptor_speed, payload.get("MaxForwardSpeed", 14.0)),
        0.5,
        60.0,
    )
    payload["MaxForwardSpeed"] = max_forward_speed

    configured_min_ram = payload.get("MinRamSpeed", 9.0)
    min_ram_speed = _to_float(configured_min_ram, 9.0)
    if not min_ram_speed:
        min_ram_speed = 9.0
    payload["MinRamSpeed"] = _clamp(min(min_ram_speed, max_forward_speed), 0.0, max_forward_speed)
    return payload