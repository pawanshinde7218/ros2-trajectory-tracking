#!/usr/bin/env python3
"""trajectory_controller.py

Time-indexed Pure Pursuit trajectory tracking controller.

Subscribes to /odom for the current robot pose and to /trajectory (the
time-parameterized trajectory from trajectory_generator). At each control
tick (default 20 Hz) it:

  1. Computes elapsed time since the trajectory started executing.
  2. Looks up the reference point on the trajectory at that elapsed time
     (interpolating between the two bracketing points) — the desired point
     is chosen by ELAPSED TIME, not by nearest-waypoint search.
  3. Computes the distance error and heading error between the robot's
     current pose and that reference point.
  4. Derives linear and angular velocity commands using a pure-pursuit-style
     curvature law, and publishes geometry_msgs/Twist on /cmd_vel.
  5. Stops the robot once the final trajectory point has been reached (both
     in time and in remaining distance).

No Nav2 controller server, DWB, RPP or MPPI is used — this is a from-scratch
implementation.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from trajectory_tracking.msg import Trajectory, TrajectoryPoint
from script.utils import normalize_angle, yaw_from_quaternion

ReferencePoint = Tuple[float, float, float, float]  # x, y, theta, velocity


class TrajectoryController(Node):
    """Time-indexed pure-pursuit-style trajectory tracking controller."""

    def __init__(self) -> None:
        super().__init__("trajectory_controller")

        self.declare_parameter("control_frequency", 20.0)  # Hz
        self.declare_parameter("max_linear_velocity", 1.0)  # m/s
        self.declare_parameter("max_angular_velocity", 2.0)  # rad/s
        self.declare_parameter("goal_position_tolerance", 0.05)  # m
        self.declare_parameter("goal_heading_tolerance", 0.05)  # rad
        self.declare_parameter("distance_gain", 0.6)  # proportional gain on distance error

        self._control_freq: float = self.get_parameter("control_frequency").value
        self._max_v: float = self.get_parameter("max_linear_velocity").value
        self._max_w: float = self.get_parameter("max_angular_velocity").value
        self._goal_pos_tol: float = self.get_parameter("goal_position_tolerance").value
        self._goal_heading_tol: float = self.get_parameter("goal_heading_tolerance").value
        self._k_dist: float = self.get_parameter("distance_gain").value

        self._trajectory: Optional[List[TrajectoryPoint]] = None
        self._trajectory_start_time: Optional[rclpy.time.Time] = None
        self._current_pose: Optional[Tuple[float, float, float]] = None  # x, y, theta
        self._goal_reached = False

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._trajectory_sub = self.create_subscription(
            Trajectory, "/trajectory", self._on_trajectory, latched_qos
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        period = 1.0 / self._control_freq
        self._control_timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f"trajectory_controller ready — running at {self._control_freq:.1f} Hz."
        )

    # ------------------------------------------------------------------ #
    # Subscriptions
    # ------------------------------------------------------------------ #
    def _on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._current_pose = (x, y, theta)

    def _on_trajectory(self, msg: Trajectory) -> None:
        if not msg.points:
            self.get_logger().error("Received empty trajectory; ignoring.")
            return
        self._trajectory = list(msg.points)
        self._trajectory_start_time = self.get_clock().now()
        self._goal_reached = False
        self.get_logger().info(
            f"New trajectory received with {len(self._trajectory)} points. "
            f"Starting tracking."
        )

    # ------------------------------------------------------------------ #
    # Control loop
    # ------------------------------------------------------------------ #
    def _control_loop(self) -> None:
        if self._trajectory is None or self._current_pose is None or self._goal_reached:
            return

        elapsed = (self.get_clock().now() - self._trajectory_start_time).nanoseconds * 1e-9
        ref = self._reference_at_time(elapsed)
        if ref is None:
            return
        ref_x, ref_y, ref_theta, ref_v = ref

        robot_x, robot_y, robot_theta = self._current_pose

        dx = ref_x - robot_x
        dy = ref_y - robot_y
        distance_error = math.hypot(dx, dy)
        heading_to_target = math.atan2(dy, dx)
        heading_error = normalize_angle(heading_to_target - robot_theta)

        final_point = self._trajectory[-1]
        final_distance = math.hypot(final_point.x - robot_x, final_point.y - robot_y)
        trajectory_complete = elapsed >= final_point.time_from_start

        if trajectory_complete and final_distance <= self._goal_pos_tol:
            self._publish_stop()
            self._goal_reached = True
            self.get_logger().info("Final waypoint reached — stopping robot.")
            return

        linear_velocity, angular_velocity = self._compute_control(
            ref_v, distance_error, heading_error
        )

        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self._cmd_vel_pub.publish(twist)

    def _compute_control(
        self, ref_velocity: float, distance_error: float, heading_error: float
    ) -> Tuple[float, float]:
        """Pure-pursuit-style curvature law using the time-indexed reference point.

        curvature = 2 * sin(heading_error) / max(distance_error, eps)
        angular_velocity = curvature * linear_velocity

        Linear velocity is the trajectory's reference speed, boosted by a
        small proportional term on remaining distance error and tapered
        down when the heading error is large (so the robot turns in place
        rather than overshooting on sharp corrections).
        """
        eps = 1e-3
        heading_taper = max(0.0, math.cos(heading_error))
        linear_velocity = (ref_velocity + self._k_dist * distance_error) * heading_taper
        linear_velocity = max(0.0, min(linear_velocity, self._max_v))

        curvature = 2.0 * math.sin(heading_error) / max(distance_error, eps)
        angular_velocity = curvature * max(linear_velocity, 0.05)
        angular_velocity = max(-self._max_w, min(angular_velocity, self._max_w))

        return linear_velocity, angular_velocity

    def _reference_at_time(self, elapsed: float) -> Optional[ReferencePoint]:
        """Interpolate the trajectory at the given elapsed time."""
        traj = self._trajectory
        if traj is None:
            return None

        if elapsed <= traj[0].time_from_start:
            p = traj[0]
            return (p.x, p.y, p.theta, p.velocity)

        if elapsed >= traj[-1].time_from_start:
            p = traj[-1]
            return (p.x, p.y, p.theta, p.velocity)

        # Linear scan is fine: trajectories here are a few hundred points.
        for i in range(len(traj) - 1):
            t0 = traj[i].time_from_start
            t1 = traj[i + 1].time_from_start
            if t0 <= elapsed <= t1:
                ratio = 0.0 if t1 == t0 else (elapsed - t0) / (t1 - t0)
                x = traj[i].x + ratio * (traj[i + 1].x - traj[i].x)
                y = traj[i].y + ratio * (traj[i + 1].y - traj[i].y)
                theta = traj[i].theta + ratio * normalize_angle(
                    traj[i + 1].theta - traj[i].theta
                )
                velocity = traj[i].velocity + ratio * (traj[i + 1].velocity - traj[i].velocity)
                return (x, y, normalize_angle(theta), velocity)

        p = traj[-1]
        return (p.x, p.y, p.theta, p.velocity)

    def _publish_stop(self) -> None:
        self._cmd_vel_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
