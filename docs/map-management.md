---
title: Map management
---
# {{ $frontmatter.title }}

## Overview

The map management is a ROS package responsible for managing map. It is a drop-in replacement for [nav2 map server](https://navigation.ros.org/configuration/packages/configuring-map-server.html).

Map provides a static 2D environment representation used for robot navigation.
The main concept is to have a map that consists of a set of areas. Map is created by combining multiple areas together. Obstacle areas are decoupled from navigation areas and can be added or removed at any time.

The second concept is a pose of docking station. It consists of a position and orientation. Position is a middle of the charging connectors. Orientation heading towards the robot's connectors.

### ROS message definition

<<< ../src/open_mower_map_server/msg/Map.msg

## Area

Each area is a polygon that can be either free to navigate or occupied by an obstacle. It's uniquely identified by an ID.
Area can be added, removed or updated at any time.

Area name is used to identify the area. It's not required to be unique, but it's recommended to use unique names to avoid confusion.

### Types

- **Exclusion area** is an area that is not allowed to be occupied by the robot. It is used to exclude areas that are not safe for the robot to drive on. For example, a pond or a flower bed.
- **Navigation area** is an area that is allowed to be occupied by the robot. It is used to define the area where the robot is allowed to drive on. For example, a driveway.
- **Lawn area** is an area that is allowed to be occupied by the robot. It is used to define the area where the robot is allowed to drive on and execute mowing pattern.

### ROS message definition

<<< ../src/open_mower_map_server/msg/Area.msg


## Docking station

Docking station is a position where the robot can charge its batteries. It's uniquely identified by an ID.
Docking station can be added, removed or updated at any time.

Pose is a position and orientation of the docking station. Position is a middle of the charging connectors. Orientation heading towards the robot's connectors.

### ROS message definition

<<< ../src/open_mower_map_server/msg/DockingStation.msg
