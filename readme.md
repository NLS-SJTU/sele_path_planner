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
## Citing
You are welcomed to use this code. If you use this work in an academic context, please cite the following publication:
* Y. Zhao, P. Liu, W. Xue, R. Miao, Z. Gong and R. Ying, "Semantic Probabilistic Traversable Map Generation For Robot Path Planning," 2019 IEEE International Conference on Robotics and Biomimetics (ROBIO), 2019, pp. 2576-2582, doi: 10.1109/ROBIO49542.2019.8961533.

    @INPROCEEDINGS{8961533,
      author={Zhao, Yimo and Liu, Peilin and Xue, Wuyang and Miao, Ruihang and Gong, Zheng and Ying, Rendong},
      booktitle={2019 IEEE International Conference on Robotics and Biomimetics (ROBIO)}, 
      title={Semantic Probabilistic Traversable Map Generation For Robot Path Planning}, 
      year={2019},
      pages={2576-2582},
      doi={10.1109/ROBIO49542.2019.8961533}
    }

## Maintained by
* Xue Wuyang
