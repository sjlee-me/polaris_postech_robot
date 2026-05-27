#!/usr/bin/env python3
"""Reference helpers for robot local-to-global coordinate conversion."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass(frozen=True)
class RobotPose2D:
    """Simple 2D robot pose container in the global map frame."""

    x: float
    y: float
    theta: float


def pose_from_robot_infos(payload: Dict[str, object]) -> RobotPose2D:
    """Build a RobotPose2D from a robot_infos-style payload.

    Input:
    - payload: dict containing a nested "pose" object with x, y, theta fields

    Output:
    - RobotPose2D(x, y, theta)

    Behavior:
    - Reads pose values from payload["pose"]
    - Converts missing values to 0.0 by default
    - Raises ValueError if payload["pose"] is not a dict
    """

    pose = payload.get("pose", {})
    if not isinstance(pose, dict):
        raise ValueError("payload['pose'] must be a dict")
    return RobotPose2D(
        x=float(pose.get("x", 0.0)),
        y=float(pose.get("y", 0.0)),
        theta=float(pose.get("theta", 0.0)),
    )


def local_to_global_xy(pose: RobotPose2D, local_x: float, local_y: float) -> Tuple[float, float]:
    """Convert a local 2D point into a global 2D point.

    Input:
    - pose: robot pose in the global map frame
    - local_x: local forward-axis offset in the robot frame
    - local_y: local left-axis offset in the robot frame

    Output:
    - (global_x, global_y)

    Behavior:
    - Applies a 2D rotation by pose.theta
    - Applies a 2D translation by pose.x and pose.y
    - Returns the transformed point in the global map frame
    """

    global_x = pose.x + local_x * math.cos(pose.theta) - local_y * math.sin(pose.theta)
    global_y = pose.y + local_x * math.sin(pose.theta) + local_y * math.cos(pose.theta)
    return global_x, global_y


def local_to_global_from_robot_infos(payload: Dict[str, object], local_x: float, local_y: float) -> Tuple[float, float]:
    """Convert a local 2D point using a robot_infos-style payload.

    Input:
    - payload: dict containing robot_infos pose data
    - local_x: local forward-axis offset in the robot frame
    - local_y: local left-axis offset in the robot frame

    Output:
    - (global_x, global_y)

    Behavior:
    - Extracts pose information from the payload
    - Reuses the common local-to-global transform function
    - Returns the final global point
    """

    pose = pose_from_robot_infos(payload)
    return local_to_global_xy(pose, local_x, local_y)
