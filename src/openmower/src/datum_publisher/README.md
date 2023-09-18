# openmower_datum_publisher_node

## overview

This node has two responsibilities:

- call [robot_localization](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)'s `/datum` service to set the datum of the robot
  this is done, so there is no need to fill this info in robot_localization config files
- publish the datum as a [sensor_msgs/NavSatFix](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html) message on the topic `/datum/fix`
  mostly to use with Foxglove Studio to see the datum on the map component

## parameters

- `datum.latitude` (double, required): latitude of the datum
- `datum.longitude` (double, required): longitude of the datum
- `datum.publish_as_fix` (bool, default: false): publish the datum as a [sensor_msgs/NavSatFix](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html) message on the topic `/datum/fix`
- `datum.fix_topic` (string, default: "/datum/fix"): topic to publish the datum as a [sensor_msgs/NavSatFix](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html) message on