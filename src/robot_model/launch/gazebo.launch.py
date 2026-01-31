from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_name = 'robot_model'
    pkg_share = get_package_share_directory(pkg_name)

    # Path to xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # Process xacro the ROS 2 way
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # --- Gazebo Harmonic (empty world, no CLI prompt) ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items(),
    )

    return LaunchDescription([

        # 🟢 Start Gazebo Harmonic
        gazebo,

        # 🟢 Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
            output='screen'
        ),

        # 🟢 Spawn robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'soccer_bot',
                '-topic', 'robot_description',
                '-x', '0',
                '-y', '0',
                '-z', '0.1'
            ],
            output='screen'
        ),

        # 🟢 Bridge Gazebo ↔ ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            ],
            output='screen'
        ),
    ])
