from launch import LaunchDescription
from ament_index_python import get_package_share_path
from ament_index_python.packages import get_package_share_directory

from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os


def generate_launch_description():
    urdf_path=os.path.join(get_package_share_path("trajectory_tracking"),"urdf","robot_model.xacro")
    world_path=os.path.join(get_package_share_path("trajectory_tracking"),"world","warehouse.sdf")
    rviz_config = os.path.join(get_package_share_path("trajectory_tracking"),'config','trajectory_tracking.rviz')
    robot_description=ParameterValue(Command(['xacro ', urdf_path]),value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description, 'use_sim_time':True}]
    )

    rviz_node=Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d',rviz_config,],
        parameters=[{'use_sim_time': True}]
    )

    start_gazebo=ExecuteProcess(
        # cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py', 'gz_args:=-r empty.sdf'],
        # cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',f"gz_args:=-r {world_path}"],
        # output="screen"
        cmd=['ign', 'gazebo', '-r', 'default.sdf'],
        output='screen'
    )
    spawan_robot=TimerAction(
         period=6.0,
        actions=[ 
            ExecuteProcess(
                cmd=['ros2', 'run', 'ros_gz_sim', 'create', '-topic', 'robot_description','-name','Three_wheel_Robot'],
                output="screen"
            )
        ]
    )

    bridge_topics = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
        
            # Clock (GZ -> ROS)
            '/world/warehouse/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    
            # cmd_vel (ROS -> GZ)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
    
            # Odom (GZ -> ROS)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
    
            # TF (GZ -> ROS)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    
            # Laser
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    
            # Camera
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
    
            # Joint states
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
    
            # IMU
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
    
            # Remapping
            '--ros-args',
            '-r',
            '/world/warehouse/clock:=/clock',
        ]
    )
    return LaunchDescription([
        robot_state_publisher_node,
        start_gazebo,
        spawan_robot,
        bridge_topics,
        rviz_node,
        
    ])