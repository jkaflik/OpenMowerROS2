#!/usr/bin/env bash

set -e

sudo apt update
rosdep update
make custom-deps deps
sudo chown -R $(whoami) /opt/ws/

echo "source /opt/ws/install/setup.bash" >> ~/.bashrc
