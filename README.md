# OpenMower ROS 2

## Overview

This is a ROS 2 port of the [OpenMower ROS](https://github.com/ClemensElflein/open_mower_ros/) software. The OpenMower project is a robotic lawn mower that is open source and open hardware.

The initial goal of this project is to get learn about robot development using ROS2. The long term goal is to build a robotic lawn mower using the OpenMower hardware, but with longer term goals of adding features such as pairing robots, obstacle avoidance etc.

Inspired by [articubot](https://github.com/joshnewans/articubot_one).

## Dependencies

This project depends on the following ROS 2 packages:
- [vesc](https://github.com/f1tenth/vesc/)
- [ublox](https://index.ros.org/p/ublox/github-KumarRobotics-ublox/#iron)

## Hardware upgrades

- [battery](https://amelectronics.pl/produkt/akumulator-pakiet-7s4p-28v-14000mah-14ah-bms-10a/)

## Development

Contributions are welcome.

### Dev container

The project is setup to use a dev container. This is a docker container that has all the dependencies installed. This makes it easy to get started with the project. To use the dev container, you need to have [Docker](https://www.docker.com/) installed.

The most prefered IDE is [Visual Studio Code](https://code.visualstudio.com/). The project is setup to use the [Remote Development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) extension. This extension allows you to open the project in a dev container.

As a side container, Xserver is run with a VNC server and client exposed via a web browser. This allows you to run the Gazebo simulator and view the GUI in a web browser. The VNC server is exposed on port 12345 to the host.


### Forwarding hardware devices to your dev container

It's possible to interact with real hardware without need to run entire stack on OpenMower robot. To do so, you need to forward serial devices to your dev container. To do so, you need to run following command on your host machine:

```bash
OPENMOWER_REMOTE_IP=blabla make remote-devices
