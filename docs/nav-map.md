---
title: Navigation map
---
# {{ $frontmatter.title }}

## Overview

Map provides a static 2D environment representation used for robot navigation.
The main concept is to have a map that consists of a set of areas. Map is created by combining multiple areas together. Obstacle areas are decoupled from navigation areas and can be added or removed at any time.

The second concept is a pose of docking station. It consists of a position and orientation. Position is a middle of the charging connectors. Orientation heading towards the robot's connectors.

### ROS message definition

<<< ../src/open_mower_map_server/msg/Map.msg

## Area

Each area is a polygon that can be either free to navigate or occupied by an obstacle. It's uniquely identified by an ID.


Area can be added, removed or updated at any time.

### ROS message definition

<<< ../src/open_mower_map_server/msg/Area.msg


### Obstacle area

