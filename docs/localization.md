---
title: Robot localization
---
# {{ $frontmatter.title }}

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

<<< ../src/openmower/config/robot_localization.yaml{yaml}
