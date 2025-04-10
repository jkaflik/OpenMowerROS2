#!/bin/bash
set -e

if [ -z "${OM_DATUM_LAT}" ]; then
  export OM_DATUM_LAT=30.0
fi

if [ -z "${OM_DATUM_LONG}" ]; then
  export OM_DATUM_LONG=0.5
fi

if [ -z "${OM_MAP_PATH}" ]; then
  export OM_MAP_PATH=${WORKSPACE}/map/json
fi

if [ ! -f "${OM_MAP_PATH}" ]; then
  touch ${OM_MAP_PATH}
fi

# Source ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f "${WORKSPACE}/install/local_setup.bash" ]; then
  source ${WORKSPACE}/install/local_setup.bash
fi

# Execute the command passed to the container
exec "$@"
