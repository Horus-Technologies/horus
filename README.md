## Horus Software Stack
This project provides autonomy for drones and is powered by ROS2 Humble. In its current state, it integrates with Ardupilot SITL.

Only the motion control software is currently implemented, and is divided between three subsystems:
- ssGlobalPlanner
- ssLocalPlanner
- ssTrajectoryController

These subsystems are realized as ROS2 nodes, and are found under src/nodes.

gtest is used for unit testing.
