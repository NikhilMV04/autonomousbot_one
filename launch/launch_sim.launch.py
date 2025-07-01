#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='autonomousbot_one' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )


    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner



    # Launch them all!
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner
    ])







# import os

# import xacro
# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.substitutions import LaunchConfiguration
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node


# def generate_launch_description():
#     pkg_share = get_package_share_directory('autonomousbot_one')

#     # 1) Only the world is a launch argument
#     declare_world = DeclareLaunchArgument(
#         'world',
#         default_value=os.path.join(pkg_share, 'worlds', 'obstacles.world'),
#         description='Path to the Gazebo world file'
#     )

#     # 2) Parse the Xacro right here in Python
#     xacro_path = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
#     robot_desc_config = xacro.process_file(xacro_path)
#     robot_description = robot_desc_config.toxml()

#     # 3) robot_state_publisher node (publishes /robot_description & TF)
#     rsp_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{
#             'robot_description': robot_description,
#             'use_sim_time': True
#         }]
#     )

#     # 4) Start Gazebo, passing in the world arg
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 get_package_share_directory('gazebo_ros'),
#                 'launch', 'gazebo.launch.py'
#             )
#         ),
#         launch_arguments={'world': LaunchConfiguration('world')}.items()
#     )

#     # 5) Spawn your robot in Gazebo from /robot_description
#     spawn_node = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
#         output='screen'
#     )

#     # 6) Controller manager (ros2_control_node)
#     ros2_control_node = Node(
#     package='controller_manager',
#     executable='ros2_control_node',
#     output='screen',
#     parameters=[
#         robot_description,  # <- this is now correct
#         os.path.join(pkg_share, 'config', 'my_controllers.yaml'),
#         {'use_sim_time': True}
#     ]
# )

#     # 7) Spawners for your two controllers (must match names in YAML)
#     joint_state_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['joint_state_broadcaster'],
#         output='screen'
#     )
#     diff_cont_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['diff_cont'],
#         output='screen'
#     )

#     return LaunchDescription([
#         declare_world,
#         rsp_node,
#         gazebo,
#         spawn_node,
#         ros2_control_node,
#         joint_state_spawner,
#         diff_cont_spawner,
#     ])
