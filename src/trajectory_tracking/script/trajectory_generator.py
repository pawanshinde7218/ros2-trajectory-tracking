#!/usr/bin/env python3
"""trajectory_generator.py

Subscribes to /smooth_path (nav_msgs/Path from path_smoother) and produces a
time-parameterized trajectory using constant-velocity time parameterization:

    v = constant (default 0.5 m/s)
    d_i = cumulative Euclidean distance from point 0 to point i
    t_i = d_i / v

Heading at each point is estimated from the vector to the next point
(the final point reuses the previous heading). The result is published as a
custom trajectory_tracking/Trajectory message (array of TrajectoryPoint) on
the /trajectory topic.
"""

from __future__ import annotations

from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import Path

from trajectory_tracking.msg import Trajectory, TrajectoryPoint
from script.utils import cumulative_distances, heading_between

Point2D = Tuple[float, float]


class TrajectoryGenerator(Node):
    """Converts a smoothed geometric path into a time-parameterized trajectory."""

    def __init__(self) -> None:
        super().__init__("trajectory_generator")

        self.declare_parameter("cruise_velocity", 0.5)  # m/s
        self._velocity: float = self.get_parameter("cruise_velocity").value

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._smooth_path_sub = self.create_subscription(
            Path, "/smooth_path", self._on_smooth_path, latched_qos
        )
        self._trajectory_pub = self.create_publisher(Trajectory, "/trajectory", latched_qos)

        self.get_logger().info(
            f"trajectory_generator ready — constant velocity = {self._velocity:.2f} m/s."
        )

    def _on_smooth_path(self, msg: Path) -> None:
        points: List[Point2D] = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

        if len(points) < 2:
            self.get_logger().error("Need at least 2 path points to generate a trajectory.")
            return

        trajectory_msg = self._build_trajectory(points, msg.header.frame_id)
        self._trajectory_pub.publish(trajectory_msg)

        total_time = trajectory_msg.points[-1].time_from_start if trajectory_msg.points else 0.0
        self.get_logger().info(
            f"Published trajectory with {len(trajectory_msg.points)} points, "
            f"total duration {total_time:.2f} s."
        )

    def _build_trajectory(self, points: List[Point2D], frame_id: str) -> Trajectory:
        cum_dist = cumulative_distances(points)

        headings: List[float] = []
        for i in range(len(points) - 1):
            headings.append(heading_between(points[i], points[i + 1]))
        headings.append(headings[-1])  # final point keeps last heading

        traj_msg = Trajectory()
        traj_msg.header.frame_id = frame_id
        traj_msg.header.stamp = self.get_clock().now().to_msg()

        for (x, y), d, theta in zip(points, cum_dist, headings):
            tp = TrajectoryPoint()
            tp.x = float(x)
            tp.y = float(y)
            tp.theta = float(theta)
            tp.velocity = float(self._velocity)
            tp.time_from_start = float(d / self._velocity) if self._velocity > 0.0 else 0.0
            traj_msg.points.append(tp)

        # Final point: command zero velocity so the controller knows to stop.
        if traj_msg.points:
            traj_msg.points[-1].velocity = 0.0

        return traj_msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
