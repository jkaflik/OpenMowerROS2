import os
import sys

from threading import Event
from threading import Thread
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
from launch_testing.io_handler import ActiveIoHandler
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from open_mower_map_server.msg import Map

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            # executable=sys.executable,
            package='open_mower_map_server',
            executable='map_server_node',
            name='open_mower_map_server',
            output='screen',
            parameters=[{
                'type': 'geojson',
                'path': os.path.join(os.getcwd(), 'src', 'open_mower_map_server', 'test', 'test_map.geojson'),
            }],
        ),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[{
                'datum': [-22.9, -43.2, 0.0],
            }],
        ),
        launch_testing.actions.ReadyToTest()
    ])

class TestOpenMowerMapServer(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        rclpy.init()

    @classmethod
    def tearDownClass(self):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_open_mower_map_server')

    def tearDown(self):
        self.node.destroy_node()

    def test_map(self):
        self.node.get_logger().info('Waiting for map...')
        self.event = Event()

        self.node.create_subscription(
            Map,
            'map',
            lambda msg: self.event.set(),
            1
        )
        self.event.wait(10.0)
        self.assertTrue(self.event.is_set())