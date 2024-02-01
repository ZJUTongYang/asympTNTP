#pragma once

#include <vector>
#include <geometry_msgs/Pose.h>

#include "prmstar.h"

#include "definitions.h"

std::vector<geometry_msgs::Pose> task_space_discrete_waypoints_for_test();

std::vector<geometry_msgs::Pose> case_study_1(std::string file_name);
std::vector<TaskSpaceWaypoint> case_study_1_matrix44(std::string file_name);


std::vector<TaskSpaceWaypoint> input_data(std::string filename);

std::vector<std::vector<double>> input_data_joint_space(std::string file_name);

void output_data(std::string file_name, const std::vector<optimal_tntp::RobotState>& robot_state);
