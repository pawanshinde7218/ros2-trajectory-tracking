"""Shared geometry and math helpers for the trajectory_tracking package.

Kept dependency-free (pure Python + numpy) so it can be unit tested without
a ROS2 runtime.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np

Point2D = Tuple[float, float]


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the range (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def distance(p1: Point2D, p2: Point2D) -> float:
    """Euclidean distance between two 2D points."""
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def heading_between(p1: Point2D, p2: Point2D) -> float:
    """Heading (yaw) of the vector p1 -> p2, in radians."""
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])


def cumulative_distances(points: Sequence[Point2D]) -> List[float]:
    """Cumulative arc length along a polyline, starting at 0.0 for points[0]."""
    if not points:
        return []
    cum = [0.0]
    for i in range(1, len(points)):
        cum.append(cum[-1] + distance(points[i - 1], points[i]))
    return cum


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Convert a yaw angle to a quaternion (x, y, z, w) for planar motion."""
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw from a quaternion, assuming planar (z-axis only) rotation."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def interpolate_point(p1: Point2D, p2: Point2D, ratio: float) -> Point2D:
    """Linearly interpolate between p1 and p2. ratio is clamped to [0, 1]."""
    ratio = max(0.0, min(1.0, ratio))
    x = p1[0] + ratio * (p2[0] - p1[0])
    y = p1[1] + ratio * (p2[1] - p1[1])
    return (x, y)


def resample_polyline_by_arclength(points: Sequence[Point2D], spacing: float) -> List[Point2D]:
    """Resample a dense polyline so consecutive samples are ~`spacing` apart.

    Used as a fallback / sanity resampler; the primary smoothing path uses
    scipy's CubicSpline directly on arc length in path_smoother.py.
    """
    if len(points) < 2 or spacing <= 0.0:
        return list(points)

    cum = cumulative_distances(points)
    total_length = cum[-1]
    if total_length <= 0.0:
        return list(points)

    n_samples = max(2, int(np.floor(total_length / spacing)) + 1)
    sample_targets = np.linspace(0.0, total_length, n_samples)

    resampled: List[Point2D] = []
    seg_idx = 0
    for target in sample_targets:
        while seg_idx < len(cum) - 2 and cum[seg_idx + 1] < target:
            seg_idx += 1
        seg_len = cum[seg_idx + 1] - cum[seg_idx]
        ratio = 0.0 if seg_len == 0.0 else (target - cum[seg_idx]) / seg_len
        resampled.append(interpolate_point(points[seg_idx], points[seg_idx + 1], ratio))
    return resampled
