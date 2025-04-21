---
title: Docking helper
---
# {{ $frontmatter.title }}

## Overview

Dock Helper is a ROS node providing services and actions to assist with docking the robot to charging stations. It manages:

- Finding the nearest docking station
- Docking to a specific docking station by ID
- Docking to the nearest available docking station

The node obtains docking station information from the map and uses the Nav2 docking capability to execute the actual docking operation.

> [!IMPORTANT]
> The node requires transform information between the robot's `base_link` and `charging_port` to calculate the correct approach position for docking.

## Docking Station Information

Docking stations are received from the `/mowing_map` topic and stored internally. Each docking station includes:
- ID
- Name
- Pose (position and orientation)

## Services

### Find Nearest Docking Station `/find_nearest_docking_station`

This service finds the docking station closest to the robot's current position and returns its details.

<<< ../../src/srv/FindNearestDockingStation.srv

## Actions

### Dock Robot to Nearest Station `/dock_robot_nearest`

<<< ../../src/action/DockRobotNearest.action

### Dock Robot to Specific Station `/dock_robot_to`

<<< ../../src/action/DockRobotTo.action

## Implementation Details

The docking operation utilizes Nav2's `/dock_robot` action to perform the actual docking. The Docking Helper node:

1. Identifies the target docking station (nearest or specific)
2. Calculates the appropriate docking pose by accounting for the offset between the robot's `base_link` and `charging_port`
3. Monitors docking progress and reports status through action feedback
4. Reports success or failure when docking completes

The docking pose is calculated by:
- Taking the docking station's pose
- Rotating it 180 degrees (to face the docking station)
- Adding an offset based on the distance between `base_link` and `charging_port`