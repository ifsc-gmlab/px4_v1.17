"""Finite-safe coordinate and attitude transformations."""

import math
from typing import Tuple


def _finite(*values: float) -> Tuple[float, ...]:
    converted = []
    for value in values:
        if isinstance(value, bool):
            raise ValueError("geometry inputs must be finite numbers")
        try:
            number = float(value)
        except (TypeError, ValueError):
            raise ValueError("geometry inputs must be finite numbers")
        if not math.isfinite(number):
            raise ValueError("geometry inputs must be finite numbers")
        converted.append(number)
    return tuple(converted)


def body_offset_to_enu(forward: float, left: float, yaw: float) -> Tuple[float, float]:
    """Convert a forward/left body displacement to east/north coordinates."""
    forward, left, yaw = _finite(forward, left, yaw)
    east = forward * math.cos(yaw) - left * math.sin(yaw)
    north = forward * math.sin(yaw) + left * math.cos(yaw)
    return east, north


def enu_to_longitudinal_lateral(east: float, north: float, yaw: float) -> Tuple[float, float]:
    """Project an east/north displacement onto forward and left body axes."""
    east, north, yaw = _finite(east, north, yaw)
    longitudinal = east * math.cos(yaw) + north * math.sin(yaw)
    lateral = -east * math.sin(yaw) + north * math.cos(yaw)
    return longitudinal, lateral


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Convert an x/y/z/w quaternion to roll, pitch and yaw radians."""
    x, y, z, w = _finite(x, y, z, w)
    magnitude = math.sqrt(x * x + y * y + z * z + w * w)
    if magnitude == 0.0:
        raise ValueError("quaternion must not be zero")
    x, y, z, w = (x / magnitude, y / magnitude, z / magnitude, w / magnitude)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


quaternion_to_roll_pitch_yaw = quaternion_to_rpy
project_enu_to_body = enu_to_longitudinal_lateral
