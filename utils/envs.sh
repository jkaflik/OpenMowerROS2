#!/usr/bin/env bash

envs=(
  "ROS_DISTRO"
  "ROS_VERSION"
  "ROS_PYTHON_VERSION"
  "ROS_AUTOMATIC_DISCOVERY_RANGE"
  "AMENT_PREFIX_PATH"
  "CMAKE_PREFIX_PATH"
  "COLCON_PREFIX_PATH"
  "LD_LIBRARY_PATH"
  "PYTHONPATH"
  "PATH"
)

for env in "${envs[@]}"; do
  echo -n "$env=${!env};"
done
