"""trajectory_tracking.launch.py

Launches the full waypoint -> smoothing -> trajectory -> control pipeline.

Does NOT launch Gazebo and does NOT spawn a robot — this assumes the
differential-drive robot, its Gazebo simulation, and its ros2_control /
diff_drive_controller stack are already running (per assignment scope).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    num_waypoints_arg = DeclareLaunchArgument(
        "num_waypoints", default_value="7", description="Number of RViz-clicked waypoints to collect."
    )
    sample_spacing_arg = DeclareLaunchArgument(
        "sample_spacing", default_value="0.05", description="Spline resampling spacing (m)."
    )
    cruise_velocity_arg = DeclareLaunchArgument(
        "cruise_velocity", default_value="0.5", description="Constant trajectory velocity (m/s)."
    )
    control_frequency_arg = DeclareLaunchArgument(
        "control_frequency", default_value="20.0", description="Controller loop rate (Hz)."
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id", default_value="odom", description="Common frame for path/marker publishing."
    )

    waypoint_collector_node = Node(
        package="trajectory_tracking",
        executable="waypoint_collector.py",
        name="waypoint_collector",
        output="screen",
        parameters=[{
            "num_waypoints": LaunchConfiguration("num_waypoints"),
            "frame_id": LaunchConfiguration("frame_id"),
        }],
    )

    path_smoother_node = Node(
        package="trajectory_tracking",
        executable="path_smoother.py",
        name="path_smoother",
        output="screen",
        parameters=[{
            "sample_spacing": LaunchConfiguration("sample_spacing"),
            "frame_id": LaunchConfiguration("frame_id"),
        }],
    )

    trajectory_generator_node = Node(
        package="trajectory_tracking",
        executable="trajectory_generator.py",
        name="trajectory_generator",
        output="screen",
        parameters=[{
            "cruise_velocity": LaunchConfiguration("cruise_velocity"),
        }],
    )

    trajectory_controller_node = Node(
        package="trajectory_tracking",
        executable="trajectory_controller.py",
        name="trajectory_controller",
        output="screen",
        parameters=[{
            "control_frequency": LaunchConfiguration("control_frequency"),
        }],
    )

    # Not part of the four explicitly-listed pipeline nodes, but required to
    # satisfy the visualization requirement (green "robot travelled path").
    rviz_visualizer_node = Node(
        package="trajectory_tracking",
        executable="rviz_visualizer.py",
        name="rviz_visualizer",
        output="screen",
        parameters=[{
            "frame_id": LaunchConfiguration("frame_id"),
        }],
    )

    return LaunchDescription([
        num_waypoints_arg,
        sample_spacing_arg,
        cruise_velocity_arg,
        control_frequency_arg,
        frame_id_arg,
        waypoint_collector_node,
        path_smoother_node,
        trajectory_generator_node,
        trajectory_controller_node,
        rviz_visualizer_node,
    ])
