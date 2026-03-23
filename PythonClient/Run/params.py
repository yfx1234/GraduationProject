from __future__ import annotations

from guidance_params import DEFAULT_VISUAL_INTERCEPT_ARGS


def pose(x=0.0, y=0.0, z=1.0, yaw=0.0, pitch=0.0, roll=0.0):
    return {
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "yaw": float(yaw),
        "pitch": float(pitch),
        "roll": float(roll),
    }


net = {
    "host": "127.0.0.1",
    "port": 9000,
}


sim = {
    "run": {
        "interceptorId": "drone_0",
        "targetId": "drone_1",
        "guidanceId": "guidance_0",
        "interceptorPose": pose(0.0, 0.0, 1.0, 0.0),
        "targetPose": pose(12.0, 0.0, 1.0, 0.0),
        "interceptorAltitude": 8.0,
        "interceptorSpeed": 4.0,
        "targetAltitude": 8.2,
        "targetSpeed": 3.0,
        "targetPath": "line",
        "arg": dict(DEFAULT_VISUAL_INTERCEPT_ARGS),
    },
    "collect": {
        "collectorId": "collect_drone_0",
        "targetId": "collect_drone_1",
        "collectorPose": pose(0.0, 0.0, 1.0, 0.0),
        "targetPose": pose(-18.0, -12.0, 1.0, 180.0),
        "collectorAltitude": 10.0,
        "targetAltitude": 10.0,
        "cameraFov": 90.0,
    },
}


yolo = {
    "model": "",
    "baseModel": "",
}


train = {
    "epochs": 100,
    "batch": 16,
}
