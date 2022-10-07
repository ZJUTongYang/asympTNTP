# asympTNTP

# Asymptotically Optimal Task-Space Non-Revisiting Tracking with a Robot Arm

This repository contains the supplementary material (code and video) for a submission to IEEE International Conference on Robotics and Automation (ICRA) 2023, under review. 

## Video Link

The video is publicly shared. Should increased viewing anonymity be desired, browsing in private mode is recommended.

[Supplementary Video](https://drive.google.com/file/d/1oohsD15Of06QJ7IiqRk7zmZbpSx9lh5h/view?usp=sharing)

## Code

(The code is being improved and updated whilst the manuscript is under review.)

### Prerequisites

(1) Ubuntu 18.04 (with ROS melodic) or 20.04 (with ROS noetic)
(2) Boost (for graph library)
(3) MoveIt! 
(4) Universal Robotic kinematic support (with UR5e manipulator support)
```cpp
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git 
```

### Explanation

This repo consists of the C++ implementation of the proposed asymptotically optimal task-space non-revisiting tracking (TNTP) algorithm as well as a demonstrative testing scene. Following modules are intergrated into this repo: 

(1) Gazebo-based UR5e manipulator simulation.
(2) Interfaces to MoveIt! planningScene.
(3) Low-level PRM* implementation for the manipulator planning of reconfiguration motions. 
(4) High-level directed graph-based asymptotic optimal TNTP framework. 

