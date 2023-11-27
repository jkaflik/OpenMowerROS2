#!/usr/bin/env bash

set -e

SCRIPT_PATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
cd "$SCRIPT_PATH/.."

# Install custom dependencies using vcstool

mkdir -p src/lib
vcs import src/lib --force --shallow < custom_deps.yaml

# check if arch is arm64
if [ "$(uname -m)" = "aarch64" ]; then
  # check if src/nav2_bringup does not exist
  if [ ! -d "src/nav2_bringup" ]; then
    curl -L https://api.github.com/repos/ros-planning/navigation2/tarball/1.2.2 \
      | tar xz -C src/lib --wildcards "*/nav2_bringup" --strip-components=1

    # remove "turtlebot3_gazebo" dependency from nav2_bringup/package.xml since it has not arm64 build
    sed -i '/turtlebot3_gazebo/d' src/lib/nav2_bringup/package.xml
  fi
fi
