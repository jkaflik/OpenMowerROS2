# Simulator

## Overview

OpenMowerROS2 incorporates a [Gazebo](http://gazebosim.org/) simulator and [ros_gz](https://github.com/gazebosim/ros_gz) integration.

::: tip
If you are not familiar with Gazebo, please refer to [Gazebo tutorials](http://gazebosim.org/tutorials).
:::

::: warning
This project uses new version of Gazebo formerly known as Ignition Gazebo.
:::

## Getting started

::: info
Simulator is not yet fully functional. It is still under development.
:::

Run the following command to start the simulator:

```bash
ros2 launch openmower sim.launch.py
```


If run inside [devcontainer](devcontainer), Gazebo GUI will be displayed in a VNC web client. You can access it by opening [`http://localhost:12345`](http://localhost:12345) in your browser.

::: tip
Learn more about [VNC client](devcontainer#detailed).
:::
