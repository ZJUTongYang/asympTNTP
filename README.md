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

### Applications

In order to use the code in this repo into your own project, you may need to do the following changes: 

(1) The environmental scene should be input. We set this by an internal function "addObstacles_case_study_1". You might need to create a subscriber that receives the obstacle data given by a sensor. 

(2) The task-space curve (aka EE poses) should be input. We set this by reading a .txt file which stores the homogeneous matrix of each EE pose. You might need to create a subscriber that receives the task-space curve by another ROS node. Note that the orientation that the EE is pointing at its x-axis. 

(3) More detailed instructions will be added in the future. 


 
