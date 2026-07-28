#!/usr/bin/env python3
"""path_smoother.py

Subscribes to /raw_waypoints (nav_msgs/Path, the discrete points collected
by waypoint_collector), fits a natural cubic spline x(s), y(s) parameterized
by cumulative chord-length s, samples it at a fixed spatial resolution
(default 5 cm), and republishes the dense result as nav_msgs/Path on
/smooth_path for the trajectory_generator node.

Algorithm
---------
Given waypoints P0..Pn, let s_i be the cumulative Euclidean distance from
P0 to Pi. We fit two independent cubic splines:

    x(s) = CubicSpline(s_i, x_i)
    y(s) = CubicSpline(s_i, y_i)

then evaluate x(s), y(s) at s = 0, ds, 2*ds, ..., s_n, where ds is the
requested sample spacing (default 0.05 m). This is the standard
"parametric cubic spline over chord length" approach to path smoothing —
no Nav2 smoother server is used.
"""

from __future__ import annotations

from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from scipy.interpolate import CubicSpline

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path

from script.utils import cumulative_distances

Point2D = Tuple[float, float]


class PathSmoother(Node):
    """Fits a cubic spline through discrete waypoints and resamples it densely."""

    def __init__(self) -> None:
        super().__init__("path_smoother")

        self.declare_parameter("sample_spacing", 0.05)  # metres, 5 cm default
        self.declare_parameter("frame_id", "odom")

        self._sample_spacing: float = self.get_parameter("sample_spacing").value
        self._frame_id: str = self.get_parameter("frame_id").value

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._raw_waypoints_sub = self.create_subscription(
            Path, "/raw_waypoints", self._on_raw_waypoints, latched_qos
        )
        self._smooth_path_pub = self.create_publisher(Path, "/smooth_path", latched_qos)

        self.get_logger().info(
            f"path_smoother ready — will resample every {self._sample_spacing:.3f} m."
        )

    def _on_raw_waypoints(self, msg: Path) -> None:
        waypoints: List[Point2D] = [
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
        ]

        if len(waypoints) < 3:
            self.get_logger().error(
                f"Need at least 3 waypoints to fit a cubic spline, got {len(waypoints)}."
            )
            return

        smooth_points = self._fit_and_sample_spline(waypoints)
        self._publish_smooth_path(smooth_points, msg.header.frame_id or self._frame_id)

        self.get_logger().info(
            f"Smoothed {len(waypoints)} waypoints into {len(smooth_points)} "
            f"points on /smooth_path."
        )

    def _fit_and_sample_spline(self, waypoints: List[Point2D]) -> List[Point2D]:
        s = np.array(cumulative_distances(waypoints))
        xs = np.array([p[0] for p in waypoints])
        ys = np.array([p[1] for p in waypoints])

        # Natural boundary conditions: zero curvature at path endpoints.
        cs_x = CubicSpline(s, xs, bc_type="natural")
        cs_y = CubicSpline(s, ys, bc_type="natural")

        total_length = float(s[-1])
        num_samples = max(2, int(np.floor(total_length / self._sample_spacing)) + 1)
        s_samples = np.linspace(0.0, total_length, num_samples)

        smooth_points = [(float(cs_x(si)), float(cs_y(si))) for si in s_samples]
        return smooth_points

    def _publish_smooth_path(self, points: List[Point2D], frame_id: str) -> None:
        path_msg = Path()
        path_msg.header.frame_id = frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose = Pose()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self._smooth_path_pub.publish(path_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
