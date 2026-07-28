#!/usr/bin/env python3
"""waypoint_collector.py

Collects exactly N (default 7) waypoints published by RViz's "Publish Point"
tool on /clicked_point, visualizes them as red sphere markers, and once N
points have been collected, publishes them once (latched) as a
nav_msgs/Path on /raw_waypoints for the path_smoother node to consume.

This node intentionally implements NO Nav2 components. It is a minimal,
single-responsibility ROS2 node following rclpy best practices.
"""

from __future__ import annotations

from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

Point2D = Tuple[float, float]


class WaypointCollector(Node):
    """Collects clicked waypoints from RViz and publishes them once full."""

    def __init__(self) -> None:
        super().__init__("waypoint_collector")

        self.declare_parameter("num_waypoints", 7)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("marker_scale", 0.15)

        self._num_waypoints: int = self.get_parameter("num_waypoints").value
        self._frame_id: str = self.get_parameter("frame_id").value
        self._marker_scale: float = self.get_parameter("marker_scale").value

        self._waypoints: List[Point2D] = []
        self._published_once = False

        # Latched (transient local) QoS so late-joining subscribers
        # (path_smoother, RViz) still receive the final waypoint set.
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._clicked_point_sub = self.create_subscription(
            PointStamped, "/clicked_point", self._on_clicked_point, 10
        )

        self._raw_waypoints_pub = self.create_publisher(Path, "/raw_waypoints", latched_qos)
        self._marker_pub = self.create_publisher(MarkerArray, "/waypoint_markers", latched_qos)

        self.get_logger().info(
            f"waypoint_collector ready — click {self._num_waypoints} points "
            f"in RViz using 'Publish Point' to trigger smoothing."
        )

    def _on_clicked_point(self, msg: PointStamped) -> None:
        if self._published_once:
            self.get_logger().warn(
                "Waypoint set already published; ignoring further clicks. "
                "Restart the node to collect a new set."
            )
            return

        point = (msg.point.x, msg.point.y)
        self._waypoints.append(point)
        self.get_logger().info(
            f"Collected waypoint {len(self._waypoints)}/{self._num_waypoints}: "
            f"({point[0]:.3f}, {point[1]:.3f})"
        )

        self._publish_markers()

        if len(self._waypoints) >= self._num_waypoints:
            self._publish_waypoint_path()
            self._published_once = True
            self.get_logger().info(
                "All waypoints collected — published to /raw_waypoints."
            )

    def _publish_markers(self) -> None:
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(self._waypoints):
            marker = Marker()
            marker.header.frame_id = self._frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = self._marker_scale
            marker.scale.y = self._marker_scale
            marker.scale.z = self._marker_scale
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self._marker_pub.publish(marker_array)

    def _publish_waypoint_path(self) -> None:
        path_msg = Path()
        path_msg.header.frame_id = self._frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in self._waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose = Pose()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self._raw_waypoints_pub.publish(path_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
