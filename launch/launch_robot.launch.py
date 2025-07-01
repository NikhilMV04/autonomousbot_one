import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

def generate_launch_description():

    package_name='autonomousbot_one'  # CHANGE ME if needed

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )
    

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # 👇 ADD THIS BLOCK to launch your serial_comm_node
    # serial_comm_node = Node(
    #     package='serial_comm',
    #     executable='serial_comm_node',
    #     name='serial_comm_node',
    #     output='screen',
    #     parameters=[{
    #         'port': '/dev/ttyACM0',         # Update if different
    #         'baudrate': 115200,
    #         'wheel_diameter': 0.066,
    #         'encoder_ticks_per_rev': 20,
    #         'wheel_base': 0.15,
    #     }],
    # )

    # Launch everything
    return LaunchDescription([
        rsp,
        # twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        # serial_comm_node  # 👈 Include it in the launch sequence
    ])




















# from launch import LaunchDescription
# from launch.actions import TimerAction
# from launch_ros.actions import Node
# import os
# import xacro
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     pkg_share = get_package_share_directory('autonomousbot_one')
#     xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
#     robot_description = xacro.process_file(xacro_file).toxml()
#     control_yaml = os.path.join(pkg_share, 'config', 'my_controllers.yaml')

#     # Robot State Publisher - this node publishes robot_description
#     rsp_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{
#             'robot_description': robot_description
#         }],
#         output='screen',
#     )

#     # Controller Manager - receives robot_description via topic
#     control_node = Node(
#         package='controller_manager',
#         executable='ros2_control_node',
#         name='controller_manager',
#         parameters=[
#             control_yaml,
#             {'use_sim_time': True}
#         ],
#         remappings=[
#             ('/robot_description', '/controller_manager/robot_description')
#         ],
#         output='screen',
#     )

#     # Spawner nodes
#     spawn_joint = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['joint_state_broadcaster'],
#         output='screen'
#     )
#     spawn_diff = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['diff_cont'],
#         output='screen'
#     )

#     return LaunchDescription([
#         rsp_node,
#         control_node,
#         TimerAction(period=5.0, actions=[spawn_joint]),
#         TimerAction(period=7.0, actions=[spawn_diff])
#     ])




# import os
# import xacro

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import TimerAction
# from launch_ros.actions import Node
# import yaml


# def generate_launch_description():
#     pkg_share = get_package_share_directory('autonomousbot_one')

#     # --- 1) robot_state_publisher ---
#     xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
#     robot_description = xacro.process_file(xacro_file).toxml()

#     rsp_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{
#             'robot_description': robot_description,
#             'use_sim_time': False
#         }]
#     )

#     # --- 2) ros2_control_node (controller_manager) ---
#     control_yaml_path = os.path.join(pkg_share, 'config', 'my_controllers.yaml')
#     with open(control_yaml_path, 'r') as f:
#         raw_yaml = yaml.safe_load(f)

#     control_yaml = raw_yaml["controller_manager"]["ros__parameters"]


#     control_node = Node(
#         package='controller_manager',
#         executable='ros2_control_node',
#         name='controller_manager',
#         output='screen',
#         parameters=[
#             {'robot_description': robot_description},
#             control_yaml,
#             {'use_sim_time': False}
#         ],
#     )
#     print(control_yaml)

#     # --- 3) spawners, after a small delay ---
#     spawn_joint = Node(
#     package='controller_manager',
#     executable='spawner',
#     arguments=['joint_broad'],
#     output='screen',
# )

#     spawn_diff = Node(
#     package='controller_manager',
#     executable='spawner',
#     arguments=['diff_cont'],
#     output='screen',
# )


#     serial_comm_node = Node(
#         package='serial_comm',
#         executable='serial_comm_node',
#         name='serial_comm_node',
#         output='screen',
#         parameters=[{
#             'port': '/dev/ttyACM0',         # Update if different
#             'baudrate': 115200,
#             'wheel_diameter': 0.066,
#             'encoder_ticks_per_rev': 20,
#             'wheel_base': 0.15,
#         }],
#     )
#     return LaunchDescription([
#         rsp_node,
#         control_node,
#         # give the controller manager 5s to come up before loading the broadcasters
#         TimerAction(period=5.0, actions=[spawn_joint]),
#         # give another 2s before loading the diff drive controller
#         TimerAction(period=7.0, actions=[spawn_diff]),
#     ])
