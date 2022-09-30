#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

void addObstacles(const moveit::planning_interface::PlanningSceneInterface& planning_scene_interface_, 
    const moveit::planning_interface::MoveGroupInterface& ur5_);

void addObstacles_case_study_1(const moveit::planning_interface::PlanningSceneInterface& planning_scene_interface_, 
    const moveit::planning_interface::MoveGroupInterface& ur5_);

void addObstacles_case_study_2(const moveit::planning_interface::PlanningSceneInterface& planning_scene_interface_, 
    const moveit::planning_interface::MoveGroupInterface& ur5_);

void addObstacles_case_study_real_world(const moveit::planning_interface::PlanningSceneInterface& planning_scene_interface_, 
    const moveit::planning_interface::MoveGroupInterface& ur5_);

