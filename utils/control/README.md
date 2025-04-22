# OpenMower Terminal Controller

::: warning
This is a work in progress. Written by LLM.
:::

A terminal-based UI controller for OpenMower. This utility provides a simple terminal interface for controlling the robot and viewing its status.

## Features

- Control the robot using arrow keys
- Monitor battery status with colored indicators
- View position and orientation information
- Check charging status

## Prerequisites

- ROS2 environment setup
- Python 3.6+
- Python packages: `curses`, `rclpy`

## Usage

The controller can be launched using the Makefile rule:

```bash
make control
```

Or run directly:

```bash
cd utils/control
python3 cli_controller.py
```

## Controls

- **↑** (Up Arrow): Increase forward speed
- **↓** (Down Arrow): Increase backward speed
- **←** (Left Arrow): Turn left
- **→** (Right Arrow): Turn right
- **Space**: Stop all movement
- **Q**: Quit the application

## ROS2 Topics

This controller interfaces with the following ROS2 topics:

- **Subscribes to**:
  - `/odometry/filtered/map`: For position and orientation information
  - `/power`: For battery state information
  - `/power/charger_present`: For charging status

- **Publishes to**:
  - `/cmd_vel_joy`: TwistStamped messages for robot control