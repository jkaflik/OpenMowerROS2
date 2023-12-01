# OpenMowerROS2 firmware

## Overview

Original OpenMower firmware is communicating with ROS via serial port using simple packet protocol.
With ROS2 we can use more advanced communication protocols, like [DDS](https://en.wikipedia.org/wiki/Data_Distribution_Service), using [micro-ROS](https://micro.ros.org/).
No need to run a dedicated node that translates serial packets to ROS messages.
Micro-ROS takes care of that.

This firmware supports only the recent OpenMower mainboard v0.13.x.

## Features

* :white_check_mark: DDS communication
* :white_check_mark: `sensor_msgs/Imu` message published on `/imu/data_raw` topic
* :white_check_mark: `sensor_msgs/BatteryState` message published on `/power` topic
  * :white_check_mark: `std_msgs/Float32` message published on `/power/charge_voltage` topic
  * :white_check_mark: `std_msgs/Bool` message published on `/power/charger_present` topic
* :white_check_mark: OpenMower charging logic
* :white_check_mark: ping micro-ROS agent, if not responding for some time, reboot
* :red_circle: safety features

## Flash

Follow instructions in [omros2-firmware](https://github.com/jkaflik/omros2-firmware#build) repository.
