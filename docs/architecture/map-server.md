---
title: Map server
---
# {{ $frontmatter.title }}

## Overview

Map server is a ROS node responsible for managing map and providing map-related services to navigation stack.

Such as:
- map persistence as a GeoJSON file
- adding and removing areas or docking stations
- publishing map
- publishing costmap grid for navigation stack
- publishing helper topics for visualization

Map provides a static 2D environment representation used for robot navigation.
The main concept is to have a map that consists of a set of areas. Map is created by combining multiple areas together. Obstacle areas are decoupled from navigation areas and can be added or removed at any time.

The second concept is a pose of docking station. It consists of a position and orientation. Position is a middle of the charging connectors. Orientation heading towards the robot's connectors.

It is a drop-in replacement for [nav2 map server](https://navigation.ros.org/configuration/packages/configuring-map-server.html).

### ROS message definition

<<< ../../src/msg/Map.msg

## Area

Each area is a polygon that can be either free to navigate or occupied by an obstacle. It's uniquely identified by an ID.
Area can be added, removed or updated at any time.

Area name is used to identify the area. It's not required to be unique, but it's recommended to use unique names to avoid confusion.

### Types

- **Exclusion area** is an area that is not allowed to be occupied by the robot. It is used to exclude areas that are not safe for the robot to drive on. For example, a pond or a flower bed.
- **Navigation area** is an area that is allowed to be occupied by the robot. It is used to define the area where the robot is allowed to drive on. For example, a driveway.
- **Operation area** is an area that is allowed to be occupied by the robot. It is used to define the area where the robot is allowed to drive on and execute its operation - mowing a lawn.

### ROS message definition

<<< ../../src/msg/Area.msg


## Docking station

Docking station is a position where the robot can charge its batteries. It's uniquely identified by an ID.
Docking station can be added, removed or updated at any time.

Pose is a position and orientation of the docking station. Position is a middle of the charging connectors. Orientation heading towards the robot's connectors.

### ROS message definition

<<< ../../src/msg/DockingStation.msg

## Map management API

Area and docking station can be added, removed or updated at any time.
All operations are performed using ROS services. After each operation, the map is published to the `/map` topic and persisted.

### Services

#### Save area `/save_area`

<<< ../../src/srv/SaveArea.srv

#### Remove area `/remove_area`

<<< ../../src/srv/RemoveArea.srv

#### Save docking station `/save_docking_station`

<<< ../../src/srv/SaveDockingStation.srv

#### Remove docking station `/remove_docking_station`

<<< ../../src/srv/RemoveDockingStation.srv

## Supported map types

### GeoJSON

Currently the only supported map type is GeoJSON. It's a format for encoding a variety of geographic data structures. It's a human-readable and easy to parse format.
OpenMowerNext uses a `Feature` of `polygon` type objects to represent areas and docking stations.

Map server will automatically load the map from the file specified in the `OM_MAP_PATH` environment variable:
```bash
OM_MAP_PATH=$HOME/.openmower/map.geojson
```

#### Schema Definition

OpenMowerNext extends the standard GeoJSON format with specific properties and conventions. A formal JSONSchema definition is available at [geojson-schema.json](assets/geojson-schema.json) that documents:

- Area features (Polygon type) with navigation, operation, or exclusion types
- Docking station features (LineString type) with position and orientation information

#### Examples

- [test_map.geojson](https://github.com/jkaflik/OpenMowerROS2/src/map_server/test/test_map.geojson)
