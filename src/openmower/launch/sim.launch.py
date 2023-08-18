import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    package_name='openmower'
    
    use_sim_time = True
    use_ros2_control = False
    
    xacro_file = os.path.join(get_package_share_directory('openmower'), 'description/robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file, use_ros2_control=use_ros2_control, use_sim_time=use_sim_time).toxml()

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
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
    
    rqtRobotSteering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('openmower'), 'config', 'view_bot.rviz')]
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

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        bridge,
        node_robot_state_publisher,
        joystick,
        twist_mux,
        rqtRobotSteering,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        gz_spawn_entity,
    ])