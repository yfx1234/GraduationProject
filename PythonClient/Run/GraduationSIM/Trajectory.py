from __future__ import annotations

import math
from typing import Any, Dict

from .DataTypes import Pose, TrajectoryReference


class BaseTrajectory:
    def reference(self, elapsed_s: float) -> TrajectoryReference:
        raise NotImplementedError

    def describe(self) -> Dict[str, float]:
        return {}


class CircleTrajectory(BaseTrajectory):
    def __init__(self, config: Any, spawn_pose: Pose, altitude: float) -> None:
        self.config = config
        self.spawn_pose = spawn_pose
        self.altitude = float(altitude)
        self.speed = max(0.1, float(config.speed))
        self.omega = max(0.01, abs(float(config.turn_rate)))
        self.radius = (
            max(6.0, float(config.radius))
            if config.radius is not None
            else max(6.0, self.speed / self.omega)
        )
        if config.center is not None:
            self.center = (
                float(config.center[0]),
                float(config.center[1]),
                float(config.center[2]),
            )
        else:
            self.center = (
                float(spawn_pose.x),
                float(spawn_pose.y) + self.radius,
                self.altitude,
            )

    def reference(self, elapsed_s: float) -> TrajectoryReference:
        theta = self.omega * float(elapsed_s)
        x = self.center[0] + self.radius * math.sin(theta)
        y = self.center[1] - self.radius * math.cos(theta)
        z = self.altitude + float(self.config.vertical_amp) * math.sin(0.35 * theta)
        vx = self.radius * self.omega * math.cos(theta)
        vy = self.radius * self.omega * math.sin(theta)
        yaw_deg = (
            math.degrees(math.atan2(vy, vx))
            if (abs(vx) + abs(vy)) > 1e-6
            else float(self.spawn_pose.yaw)
        )
        return TrajectoryReference(
            pose=Pose(x=x, y=y, z=z, yaw=yaw_deg),
            speed=self.speed,
        )

    def describe(self) -> Dict[str, float]:
        return {
            "radius": self.radius,
            "omega": self.omega,
            "center_x": self.center[0],
            "center_y": self.center[1],
            "center_z": self.center[2],
        }


class LineTrajectory(BaseTrajectory):
    def __init__(self, *_args, **_kwargs) -> None:
        pass

    def reference(self, elapsed_s: float) -> TrajectoryReference:
        raise NotImplementedError("trajectory kind 'line' is reserved but not implemented yet")


class WaypointTrajectory(BaseTrajectory):
    def __init__(self, *_args, **_kwargs) -> None:
        pass

    def reference(self, elapsed_s: float) -> TrajectoryReference:
        raise NotImplementedError("trajectory kind 'waypoints' is reserved but not implemented yet")


class FigureEightTrajectory(BaseTrajectory):
    def __init__(self, *_args, **_kwargs) -> None:
        pass

    def reference(self, elapsed_s: float) -> TrajectoryReference:
        raise NotImplementedError("trajectory kind 'figure8' is reserved but not implemented yet")


class TrajectoryFactory:
    @staticmethod
    def create(config: Any, spawn_pose: Pose, altitude: float) -> BaseTrajectory:
        kind = str(config.kind or "circle").strip().lower()
        if kind == "circle":
            return CircleTrajectory(config=config, spawn_pose=spawn_pose, altitude=altitude)
        if kind == "line":
            return LineTrajectory(config=config, spawn_pose=spawn_pose, altitude=altitude)
        if kind == "waypoints":
            return WaypointTrajectory(config=config, spawn_pose=spawn_pose, altitude=altitude)
        if kind == "figure8":
            return FigureEightTrajectory(config=config, spawn_pose=spawn_pose, altitude=altitude)
        raise ValueError(f"unsupported trajectory kind: {config.kind}")
