#!/usr/bin/env bash

set -e

echo "Updating package list..."
sudo apt update
rosdep update

echo "Installing ROS packages..."
make custom-deps deps

echo "Sourcing ROS workspace..."
echo "source /opt/ws/install/setup.bash" >> ~/.bashrc
echo "source /opt/ws/.devcontainer/openmower_config.bash" >> ~/.bashrc
source ~/.bashrc
