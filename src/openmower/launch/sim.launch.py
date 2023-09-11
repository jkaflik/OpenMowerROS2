import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    package_name='openmower'
    
    xacro_file = os.path.join(get_package_share_directory('openmower'), 'description/robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={
        'use_ros2_control': '0',
        'use_sim_time': '1'
        }).toxml()

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': True}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
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
            remappings=[('/cmd_vel_out','/diff_drive_base_controller/cmd_vel_unstamped')]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description_config,
                   '-name', 'openmower',
                   '-allow_renaming', 'true'],
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

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )



    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("openmower"), '/launch/localization.launch.py']),
        launch_arguments={
            'use_sim_time': 'true',
            # 'map': os.path.join(get_package_share_directory("openmower"), 'maps', 'world.yaml'),
            'autostart': 'true',
            'params_file': os.path.join(get_package_share_directory("openmower"), 'config', 'nav2_params.yaml'),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("openmower"), '/launch/nav2.launch.py']),
        launch_arguments={
            'use_sim_time': 'true',
            # 'map': os.path.join(get_package_share_directory("openmower"), 'maps', 'world.yaml'),
            'autostart': 'true',
            'params_file': os.path.join(get_package_share_directory("openmower"), 'config', 'nav2_params.yaml'),
        }.items(),
    )

    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([get_package_share_directory("foxglove_bridge"), '/launch/foxglove_bridge_launch.xml']),
    )

    # Launch them all!
    return LaunchDescription([
        bridge,
        node_robot_state_publisher,
        twist_mux,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 {}'.format(os.path.join(get_package_share_directory(package_name),'worlds','empty.world'))])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_mower_controller],
            )
        ),
        gz_spawn_entity,
        foxglove_bridge,
    ])