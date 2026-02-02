from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    TextSubstitution
)
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # -------------------------
    # Robot description (XACRO)
    # -------------------------
    xacro_file = PathJoinSubstitution([
        FindPackageShare('robot_model'),
        'urdf',
        'robot.urdf.xacro'
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            TextSubstitution(text=' '),   # 🔴 THIS SPACE IS CRITICAL
            xacro_file
        ]),
        value_type=str
    )

    # -------------------------
    # Gazebo Fortress
    # -------------------------
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

    # -------------------------
    # Nodes
    # -------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'soccer_bot',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.2'
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
        ]
    )

    spawn_ball = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'soccer_ball',
            '-file',
            os.path.join(
                get_package_share_directory('gazebo_world'),
                'models',
                'soccer_ball.sdf'
            ),

            # --- Pose ---
            '-x', '1.0',     # forward (meters)
            '-y', '0.0',     # left (meters)
            '-z', '0.05',    # height (meters)

            # Optional orientation (radians)
            '-R', '0.0',     # roll
            '-P', '0.0',     # pitch
            '-Y', '0.0',     # yaw
        ]
    )

    # -------------------------
    # Launch description
    # -------------------------
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        spawn_ball
    ])
