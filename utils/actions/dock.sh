#!/bin/bash

# Script to trigger docking procedure using the ROS2 action CLI

echo "Sending dock action command..."

# Send the goal (empty goal since our dock action doesn't require any parameters)
ros2 action send_goal /dock open_mower_next/action/Dock "{}" --feedback

echo "Dock action completed."