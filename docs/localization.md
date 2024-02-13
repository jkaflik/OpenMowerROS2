---
title: Robot localization
---
# {{ $frontmatter.title }}

## Overview

Robot pose is based on an absolute position from GPS and relative readings from wheel odometry and IMU.
Orientation is not known on startup and defaults to 0. 

As soon as robot starts moving, orientation is assumed based on robot motion and sensors reading.
It might take a while to get an accurate orientation. Best to start moving in a straight line and do a few circles.

Currently, there is no fallback scenario if had its position changed externally. For example, if you move the robot manually, it will not be able to recover position itself. It will require same procedure as on startup. 

Later on, when undocking behavior is implemented, it will be possible to recover orientation knowing base station position.

## Sensors

[robot_localization documentation](http://docs.ros.org/en/melodic/api/robot_localization/html/integrating_gps.html) nicely describes
how to get wheel odometry, GPS and IMU sensors data fusion to get an accurate localization.

![Senor data flow](http://docs.ros.org/en/melodic/api/robot_localization/html/_images/navsat_transform_workflow.png)

### Wheel odometry

Default motor controller VESC reports wheel odometry.

### IMU

Accelerometer and gyroscope is required. Magnetometer is not fused.

### GPU

It's expected GPS is RTK capable. Otherwise, localization will be inaccurate.
More on this in [GPS](gps.md) section.

## Configuration

<<< ../config/robot_localization.yaml{yaml}
