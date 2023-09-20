import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    package_name='openmower'
    
    xacro_file = os.path.join(get_package_share_directory('openmower'), 'description/robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={
        'use_ros2_control': '1',
        'use_sim_time': '0'
        }).toxml()

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': False}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel_out','/diff_drive_base_controller/cmd_vel_unstamped')]
        )
    
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description_config},
                    controller_params_file]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )

    # load_mower_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'mower_controller'],
    #     output='screen'
    # )

    # Launch them all!
    return LaunchDescription([
        node_robot_state_publisher,
        twist_mux,
        controller_manager,

        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[load_joint_state_controller],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_controller],
            )
        ),

        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_mower_controller],
        #     )
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory("openmower"), '/launch/gps.launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory("openmower"), '/launch/localization.launch.py']),
            launch_arguments={
                'use_sim_time': 'false',
                # 'map': os.path.join(get_package_share_directory("openmower"), 'maps', 'world.yaml'),
                'autostart': 'true',
                'params_file': os.path.join(get_package_share_directory("openmower"), 'config', 'nav2_params.yaml'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory("openmower"), '/launch/nav2.launch.py']),
            launch_arguments={
                'use_sim_time': 'false',
                # 'map': os.path.join(get_package_share_directory("openmower"), 'maps', 'world.yaml'),
                'autostart': 'true',
                'params_file': os.path.join(get_package_share_directory("openmower"), 'config', 'nav2_params.yaml'),
            }.items(),
        ),

        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([get_package_share_directory("foxglove_bridge"), '/launch/foxglove_bridge_launch.xml']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory("openmower"), '/launch/micro_ros_agent.launch.py']),
        ),
    ])