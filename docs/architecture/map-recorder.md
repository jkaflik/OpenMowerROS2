---
title: Map recorder
---
# {{ $frontmatter.title }}

## Overview

Map recorder is a ROS node exposing an API to trigger map recording:

- record area boundary
- record docking station

Mower operator is responsible of controlling the robot. Publish twist commands to `/cmd_vel_joy` topic to control the robot.

> [!WARNING]
> The robot must be in a known position. GPS should be using RTK corrections. Otherwise, the robot will not be able to record the area correctly. Expected accuracy is <2cm.
> Map recorder will not deny recording if GPS is not using RTK corrections, but the recorded area will be inaccurate.

> [!IMPORTANT]
> Since robot's orientation is not known on startup, it is required to drive the robot around, spin it around and do a few circles to get an accurate orientation. Best to start moving in a straight line and do a few circles. 

## Area boundary recording

User initiates area boundary recording by calling `/record_area_boundary` action. It can be started in two modes:
| Recording Mode | Description |
|---------------|-------------|
| `auto_recording = false` | User manually adds points to the area boundary |
| `auto_recording = true` | User drives the robot around the area and the area boundary is recorded automatically |

Auto recording can be enabled/disabled at any time using `/set_recording_mode` action.

Auto recording is useful for areas that are not bounded with a straight line with a lot of curves. If your area is a rectangle or a simple shape, it's better to use manual recording.

User can add points to the area boundary by calling `/add_boundary_point` action. The point is added to the area boundary and the area is updated. The area is not saved to the map server yet. It is only saved when the recording is finished.

Recorded polygon is published to `/map_recorder/record_boundaries_polygon` topic. It can be used for visualization purposes.

Recording is finished by calling `/finish_area_recording` action. The area is saved to the map server and immidiately available for use.

## Docking station recording

Drive the robot in a front of the docking station and call `/record_docking_station` action.
It will start docking station recording. Mower will automatically drive to the docking station.

If charging is detected, the docking station will be recorded with it's pose. (mower's `charging_port` rotated by 180 degrees)

If charging is not detected, action will be aborted and the docking station will not be recorded.

## Actions

### Record area boundary `/record_area_boundary`

<<< ../../src/action/RecordAreaBoundary.action

#### Set recording mode `/set_recording_mode`

<<< ../assets/std_srvs_setbool.srv

#### Add boundary point `/add_boundary_point`

<<< ../assets/std_srvs_trigger.srv

#### Finish area recording `/finish_area_recording`

<<< ../assets/std_srvs_trigger.srv

---

### Record docking station `/record_docking_station`

<<< ../../src/action/RecordDockingStation.action