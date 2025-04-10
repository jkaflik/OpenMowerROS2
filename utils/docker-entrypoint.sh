#!/bin/bash
set -e

# Source ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f "${WORKSPACE}/install/local_setup.bash" ]; then
  source ${WORKSPACE}/install/local_setup.bash
fi

# Execute the command passed to the container
exec "$@"
