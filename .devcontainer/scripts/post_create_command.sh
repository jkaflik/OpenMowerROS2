#!/usr/bin/env bash

set -e

echo "Installing ROS packages..."

sudo apt update
rosdep update
make custom-deps deps
sudo chown -R $(whoami) /opt/ws/

echo "source /opt/ws/install/setup.bash" >> ~/.bashrc
echo "source /opt/ws/.devcontainer/openmower_config.env" >> ~/.bashrc
