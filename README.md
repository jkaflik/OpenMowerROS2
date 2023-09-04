# OpenMower ROS 2

## Overview

This is a ROS 2 port of the [OpenMower ROS](https://github.com/ClemensElflein/open_mower_ros/) software. The OpenMower project is a robotic lawn mower that is open source and open hardware.

The initial goal of this project is to get learn about robot development using ROS2. The long term goal is to build a robotic lawn mower using the OpenMower hardware, but with longer term goals of adding features such as pairing robots, obstacle avoidance etc.

Inspired by [articubot](https://github.com/joshnewans/articubot_one).

## Dependencies

This project depends on the following ROS 2 packages:
- [control2]
- [nav2]
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization)
- [vesc](https://github.com/f1tenth/vesc/)
- [ublox](https://index.ros.org/p/ublox/github-KumarRobotics-ublox/#iron)

## Hardware upgrades

List of potential upgrades for myself:

- [battery](https://amelectronics.pl/produkt/akumulator-pakiet-7s4p-28v-14000mah-14ah-bms-10a/)

## Development

Contributions are welcome.

### Dev container

The project is set up to use a dev container. This is a docker container that has all the dependencies installed. This makes it easy to get started with the project. To use the dev container, you need to have [Docker](https://www.docker.com/) installed.

#### Visual Studio Code

The most straightforward IDE is [Visual Studio Code](https://code.visualstudio.com/). The project is set up to use the [Remote Development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) extension. This extension allows you to open the project in a dev container.

As a side container, Xserver is run with a VNC server and client exposed via a web browser. This allows you to run the Gazebo simulator and view the GUI in a web browser. The VNC server is exposed on port 12345 to the host.

#### CLion

Developing with CLion is also possible, but requires a minimum more effort to get the things running smoothly.

Steps:
- run `make dev-containers` to get the workspace and xserver containers running
- configure [remote toolchain](https://www.jetbrains.com/help/clion/remote-projects-support.html#remote-toolchain) for your workspace
  this repository is shipped with preconfigured CLion project files, so you should be able to just open the project
  __NOTE__: you need to use `localhost` as the host, port `2222` and `openmower` as the username (password is `openmower`)
- make sure remote development has correct mapping. Your project path has to be mapped to `/home/ws` in the container

### Simulator

The project is setup to use the [Gazebo Garden](http://gazebosim.org/) simulator. Mower is loaded from URDF.

Currently supported features:
- control - [ros2 control hardware system interface](src/openmower/description/gazebo_control.xacro) `gz_ros2_control/GazeboSimSystem`
- [GPS](src/openmower/description/gps.xacro) - `navsat` sensor
    currently it's broken due to [OSM bug in Qt](https://github.com/gazebosim/gz-gui/issues/482)

TODO:
- [Bridge topics from Gazebo to ROS 2](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge)
- Depth camera - [plugin example](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_point_cloud/examples/depth_camera.sdf)
- Custom sensors: e.g. power source - [instructions](https://github.com/gazebosim/gz-sensors/blob/main/tutorials/custom_sensors.md)

### Forwarding hardware devices to your dev container

It's possible to interact with real hardware without need to run entire stack on OpenMower robot. To do so, you need to forward serial devices to your dev container. To do so, you need to run following command on your host machine:

```bash
OPENMOWER_REMOTE_IP=192.168.100.100 make remote-devices
```

## Notes (unstructured)

- installing librealsense from ROS repositories does not setup udev rules, so you need to do it manually: https://github.com/IntelRealSense/librealsense/blob/master/config/99-realsense-libusb.rules