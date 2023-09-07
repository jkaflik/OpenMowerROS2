import os
import yaml
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    default_device = '/dev/ttyAMA0'
    default_baudrate = '115200'

    # Fetch package path and set the default YAML file location
    default_yaml_path = os.path.join(
        get_package_share_directory('openmower'),
        'config', 'hardware', 'openmower.yaml'
    )

    device = default_device
    baudrate = default_baudrate

    # Launch Description
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'yaml_file',
            default_value=default_yaml_path,
            description='Full path to the YAML file to use for this launch session'
        ),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            output='screen',
            parameters=[LaunchConfiguration('yaml_file')],
            arguments=['serial', '--dev', device, '--baud', str(baudrate)],
        ),
    ])

    return ld