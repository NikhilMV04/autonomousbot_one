import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Run xacro at launch-time to generate the URDF:
    robot_description = Command([
        FindExecutable('xacro'), ' ',
        PathJoinSubstitution([
            get_package_share_directory('autonomousbot_one'),
            'description',
            'robot.urdf.xacro'
        ])
    ])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([rsp_node])