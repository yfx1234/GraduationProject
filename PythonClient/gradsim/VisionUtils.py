"""Shared image helpers for GradSim Python tools."""

import base64

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None


def has_opencv():
    return cv2 is not None


def decode_bgr_base64(data_b64):
    if cv2 is None or not data_b64:
        return None
    try:
        raw = base64.b64decode(data_b64)
    except Exception:
        return None
    array = np.frombuffer(raw, dtype=np.uint8)
    return cv2.imdecode(array, cv2.IMREAD_COLOR)
