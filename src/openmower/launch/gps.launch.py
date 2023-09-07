import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory
def generate_launch_description():

    default_config_path = os.path.join(
        get_package_share_directory('openmower'),
        'config', 'gps_config.yaml'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to the configuration YAML file.'
    )

    ntrip_client_args = {
        'host': 'system.asgeupos.pl',
        'port': '8082',
        'mountpoint': 'BOGI_RTCM_3_1',
        'username': 'jkaflik',
        'password': '7*i4ODRctXfpqkr#',
    }

    return LaunchDescription([
        declare_config_file_cmd,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory("ntrip_client"), '/ntrip_client_launch.py']),
            launch_arguments=ntrip_client_args.items()
        ),
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            output='both',
            parameters=[os.path.join(get_package_share_directory("openmower"), 'config', 'ublox_gps.yaml')]
        ),
    ])