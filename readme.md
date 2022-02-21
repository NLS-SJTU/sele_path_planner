# Path planner for semantic-elevation map
## Introduction
* this is a DWA-based path planner for semantic-elevation map.
## Requirement
* grid map
* elevation map
## Subscribe topic
* elevation_map(grid_map) - with a layer called "elevation".
* semantic_map(grid_map) - with two layer called "label" and "prob", optional.
* targetP_odom(geometry_msgs::Pose) - local target in odom frame.
* targetP(geometry_msgs::Pose) - local target in body frame.
## Publish topic
* RosAria/cmd_vel(geometry_msgs::Twist) - command velocity.
## Maintained by
* Xue Wuyang
