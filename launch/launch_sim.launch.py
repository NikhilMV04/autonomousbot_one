#!/usr/bin/env python3
import os

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('autonomousbot_one')

    # 1) Only the world is a launch argument
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'obstacles.world'),
        description='Path to the Gazebo world file'
    )

    # 2) Parse the Xacro right here in Python
    xacro_path = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_desc_config = xacro.process_file(xacro_path)
    robot_description = robot_desc_config.toxml()

    # 3) robot_state_publisher node (publishes /robot_description & TF)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # 4) Start Gazebo, passing in the world arg
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # 5) Spawn your robot in Gazebo from /robot_description
    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # 6) Controller manager (ros2_control_node)
    ros2_control_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    output='screen',
    parameters=[
        robot_description,  # <- this is now correct
        os.path.join(pkg_share, 'config', 'my_controllers.yaml'),
        {'use_sim_time': True}
    ]
)

    # 7) Spawners for your two controllers (must match names in YAML)
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    diff_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen'
    )

    return LaunchDescription([
        declare_world,
        rsp_node,
        gazebo,
        spawn_node,
        ros2_control_node,
        joint_state_spawner,
        diff_cont_spawner,
    ])
