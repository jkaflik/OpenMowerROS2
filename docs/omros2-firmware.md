# OpenMowerROS2 firmware

## Overview

Original OpenMower firmware is communicating with ROS via serial port using simple packet protocol.
With ROS2 we can use more advanced communication protocols, like [DDS](https://en.wikipedia.org/wiki/Data_Distribution_Service), using [micro-ROS](https://micro.ros.org/).
No need to run a dedicated node that translates serial packets to ROS messages.
Micro-ROS takes care of that.

## Features

* [x] DDS communication
* [x] `sensor_msgs/Imu` message published on `/imu/data_raw` topic
* [x] `sensor_msgs/BatteryState` message published on `/power` topic
  * [x] `std_msgs/Float32` message published on `/power/charge_voltage` topic
  * [x] `std_msgs/Bool` message published on `/power/charger_present` topic
* [x] OpenMower charging logic
* [x] ping micro-ROS agent, if not responding for some time, reboot

## Flash

Follow instructions in [omros2-firmware](https://github.com/jkaflik/omros2-firmware#build) repository.
