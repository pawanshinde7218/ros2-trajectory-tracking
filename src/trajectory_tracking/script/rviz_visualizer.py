#!/usr/bin/env python3
"""rviz_visualizer.py

Subscribes to /odom and accumulates the robot's actually-travelled path,
republishing it as a nav_msgs/Path on /robot_path (displayed as a green
line in RViz, configured via config/trajectory_tracking.rviz).

Waypoint markers (red spheres) are published directly by waypoint_collector
and the smoothed path (blue line) is published directly by path_smoother —
this node's sole responsibility is the third and final piece of the required
visualization: the robot's actual trajectory as it drives.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, Path


class RvizVisualizer(Node):
    """Accumulates and republishes the robot's travelled path for RViz."""

    def __init__(self) -> None:
        super().__init__("rviz_visualizer")

        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("min_point_spacing", 0.02)  # metres, avoid flooding Path
        self.declare_parameter("max_path_points", 5000)

        self._frame_id: str = self.get_parameter("frame_id").value
        self._min_spacing: float = self.get_parameter("min_point_spacing").value
        self._max_points: int = self.get_parameter("max_path_points").value

        self._robot_path = Path()
        self._robot_path.header.frame_id = self._frame_id

        self._odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._robot_path_pub = self.create_publisher(Path, "/robot_path", 10)

        self.get_logger().info("rviz_visualizer ready — tracking travelled path on /robot_path.")

    def _on_odom(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = msg.header.stamp
        pose.pose = Pose()
        pose.pose.position.x = msg.pose.pose.position.x
        pose.pose.position.y = msg.pose.pose.position.y
        pose.pose.orientation = msg.pose.pose.orientation

        if self._robot_path.poses:
            last = self._robot_path.poses[-1].pose.position
            dx = pose.pose.position.x - last.x
            dy = pose.pose.position.y - last.y
            if (dx * dx + dy * dy) ** 0.5 < self._min_spacing:
                return

        self._robot_path.poses.append(pose)
        if len(self._robot_path.poses) > self._max_points:
            self._robot_path.poses.pop(0)

        self._robot_path.header.stamp = self.get_clock().now().to_msg()
        self._robot_path_pub.publish(self._robot_path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RvizVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
