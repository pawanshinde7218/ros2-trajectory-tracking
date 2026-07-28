"""Unit tests for trajectory_tracking.utils.

These tests exercise only the pure-Python/numpy geometry helpers, so they
can be run with plain pytest even outside a sourced ROS2 workspace:

    cd trajectory_tracking
    python3 -m pytest test/test_utils.py -v
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

# Allow running this test file directly (without an ament build) by adding
# the package source directory to the path.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from trajectory_tracking.utils import (  # noqa: E402
    cumulative_distances,
    distance,
    heading_between,
    interpolate_point,
    normalize_angle,
    quaternion_from_yaw,
    resample_polyline_by_arclength,
    yaw_from_quaternion,
)


class TestNormalizeAngle:
    def test_already_in_range(self):
        assert normalize_angle(0.5) == pytest.approx(0.5)

    def test_wraps_positive_overflow(self):
        assert normalize_angle(2 * math.pi + 0.1) == pytest.approx(0.1, abs=1e-9)

    def test_wraps_negative_overflow(self):
        assert normalize_angle(-2 * math.pi - 0.1) == pytest.approx(-0.1, abs=1e-9)

    def test_pi_boundary(self):
        result = normalize_angle(3 * math.pi)
        assert -math.pi <= result <= math.pi


class TestDistanceAndHeading:
    def test_distance_simple(self):
        assert distance((0, 0), (3, 4)) == pytest.approx(5.0)

    def test_distance_zero(self):
        assert distance((1, 1), (1, 1)) == pytest.approx(0.0)

    def test_heading_east(self):
        assert heading_between((0, 0), (1, 0)) == pytest.approx(0.0)

    def test_heading_north(self):
        assert heading_between((0, 0), (0, 1)) == pytest.approx(math.pi / 2)


class TestCumulativeDistances:
    def test_empty(self):
        assert cumulative_distances([]) == []

    def test_single_point(self):
        assert cumulative_distances([(0, 0)]) == [0.0]

    def test_straight_line(self):
        points = [(0, 0), (1, 0), (2, 0), (3, 0)]
        assert cumulative_distances(points) == pytest.approx([0.0, 1.0, 2.0, 3.0])

    def test_monotonic_increasing(self):
        points = [(0, 0), (1, 1), (2, 0), (3, 3)]
        cum = cumulative_distances(points)
        assert all(cum[i] <= cum[i + 1] for i in range(len(cum) - 1))


class TestQuaternionConversions:
    def test_round_trip_zero(self):
        q = quaternion_from_yaw(0.0)
        assert yaw_from_quaternion(*q) == pytest.approx(0.0, abs=1e-9)

    def test_round_trip_arbitrary(self):
        for yaw in [0.3, -1.2, math.pi / 2, -math.pi / 2, 2.5]:
            q = quaternion_from_yaw(yaw)
            recovered = yaw_from_quaternion(*q)
            assert recovered == pytest.approx(yaw, abs=1e-9)


class TestInterpolatePoint:
    def test_midpoint(self):
        result = interpolate_point((0, 0), (2, 2), 0.5)
        assert result == pytest.approx((1.0, 1.0))

    def test_clamped_below_zero(self):
        result = interpolate_point((0, 0), (2, 2), -1.0)
        assert result == pytest.approx((0.0, 0.0))

    def test_clamped_above_one(self):
        result = interpolate_point((0, 0), (2, 2), 2.0)
        assert result == pytest.approx((2.0, 2.0))


class TestResamplePolyline:
    def test_too_few_points_returned_unchanged(self):
        points = [(0, 0)]
        assert resample_polyline_by_arclength(points, 0.1) == points

    def test_spacing_roughly_respected(self):
        points = [(0, 0), (10, 0)]
        resampled = resample_polyline_by_arclength(points, 1.0)
        assert len(resampled) >= 10
        gaps = [
            distance(resampled[i], resampled[i + 1]) for i in range(len(resampled) - 1)
        ]
        assert all(g <= 1.0 + 1e-6 for g in gaps)

    def test_endpoints_preserved(self):
        points = [(0, 0), (5, 5), (10, 0)]
        resampled = resample_polyline_by_arclength(points, 0.5)
        assert resampled[0] == pytest.approx(points[0])
        assert resampled[-1] == pytest.approx(points[-1])


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
